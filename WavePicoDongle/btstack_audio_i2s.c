/*
 * BTstack audio sink for the Pico, driving an I2S DAC (PCM5100A on the
 * Pimoroni Pico Audio Pack) through PIO + DMA.
 *
 * Design
 * ------
 * - A PIO state machine clocks out 16-bit stereo I2S continuously once the
 *   sink is first initialised. When no audio is available it plays silence,
 *   so the DAC's PLL stays locked and stream starts/stops do not pop.
 * - Two DMA channels are chained to each other (A -> B -> A ...) so the
 *   hardware switches buffers with no gap. When a channel completes, the DMA
 *   IRQ re-arms it with the next filled buffer (or the silence buffer on
 *   underrun) and asks the BTstack run loop to poll its data sources.
 * - Buffers are filled on the BTstack thread (a polled data source), never
 *   in the IRQ, because the A2DP playback callback decodes SBC from ring
 *   buffers that are not IRQ-safe.
 */

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/sync.h"

#include "btstack_run_loop.h"
#include "btstack_audio_i2s.h"
#include "audio_i2s.pio.h"

#define NUM_BUFFERS        6      // pool of buffers between BTstack and DMA
#define FRAMES_PER_BUFFER  256    // ~5.8 ms at 44.1 kHz; total queue ~35 ms
#define AUDIO_DMA_IRQ      1      // use DMA_IRQ_1 (DMA_IRQ_0 is the SDK default for others)

// One uint32_t per stereo frame: low half = left, high half = right.
static uint32_t audio_buffers[NUM_BUFFERS][FRAMES_PER_BUFFER];
static uint32_t silence_buffer[FRAMES_PER_BUFFER];

// Single-producer / single-consumer counters (free-running, wrap-safe).
//   filled_count   : buffers filled by the BTstack thread
//   queued_count   : buffers handed to a DMA channel (IRQ)
//   released_count : buffers finished playing (IRQ)
// Slot (n % NUM_BUFFERS) is free for filling while filled - released < NUM_BUFFERS.
static volatile uint32_t filled_count;
static volatile uint32_t queued_count;
static volatile uint32_t released_count;

static int  dma_channels[2] = { -1, -1 };
static volatile bool dma_channel_has_pool_buffer[2];

static PIO  audio_pio;
static uint audio_sm;
static uint audio_pio_offset;
static bool hardware_initialized;

static btstack_data_source_t fill_data_source;
// The I2S stream can start (silence) before the BTstack run loop exists; the
// data source is registered once the sink is initialised from BTstack.
static volatile bool fill_data_source_added;

static void (*playback_callback)(int16_t * buffer, uint16_t num_samples, const btstack_audio_context_t * context);
static uint8_t  num_channels;
static uint32_t sample_rate;
static volatile bool stream_started;
static int32_t  volume_gain_q15 = 32768;   // 1.0

// ---------------------------------------------------------------------------
// DMA

static void dma_arm_channel(int index){
    const uint32_t * buffer;
    if (filled_count != queued_count){
        buffer = audio_buffers[queued_count % NUM_BUFFERS];
        queued_count++;
        dma_channel_has_pool_buffer[index] = true;
    } else {
        buffer = silence_buffer;
        dma_channel_has_pool_buffer[index] = false;
    }
    // Transfer count auto-reloads on trigger; only the read address must be reset.
    dma_channel_set_read_addr(dma_channels[index], buffer, false);
}

static void __isr audio_dma_irq_handler(void){
    bool serviced = false;
    for (int i = 0; i < 2; i++){
        uint ch = (uint) dma_channels[i];
        if (dma_irqn_get_channel_status(AUDIO_DMA_IRQ, ch)){
            dma_irqn_acknowledge_channel(AUDIO_DMA_IRQ, ch);
            // Channels complete in the same order they were armed, so the
            // finished pool buffer is always the oldest one outstanding.
            if (dma_channel_has_pool_buffer[i]){
                released_count++;
            }
            dma_arm_channel(i);
            serviced = true;
        }
    }
    if (serviced && fill_data_source_added){
        btstack_run_loop_poll_data_sources_from_irq();
    }
}

// ---------------------------------------------------------------------------
// Buffer filling (BTstack thread)

static void apply_volume(int16_t * samples, int count){
    int32_t gain = volume_gain_q15;
    if (gain >= 32768) return;
    for (int i = 0; i < count; i++){
        samples[i] = (int16_t) ((samples[i] * gain) >> 15);
    }
}

// The PCM510xA "zero data detect" goes into full analog mute after 1024
// consecutive all-zero frames (datasheet 9.3.2.3; not configurable in hardware
// mode), which shows up as static on the output. To keep the DAC out of that
// state we never send exact digital zero: a +/-1 LSB tone at fs/2 is added to
// every frame. fs/2 is removed by the DAC's interpolation filter, so it is
// inaudible, but the zero detector always sees non-zero data.
#define ANTI_MUTE_LSB 1

static inline int16_t add_saturate(int16_t sample, int16_t delta){
    int32_t v = (int32_t) sample + delta;
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    return (int16_t) v;
}

static void add_anti_mute_tone(int16_t * samples, int frames){
    for (int i = 0; i < frames; i++){
        int16_t delta = (i & 1) ? -ANTI_MUTE_LSB : ANTI_MUTE_LSB;
        samples[2 * i]     = add_saturate(samples[2 * i],     delta);
        samples[2 * i + 1] = add_saturate(samples[2 * i + 1], delta);
    }
}

static void fill_buffers(void){
    while (stream_started && playback_callback && (filled_count - released_count) < NUM_BUFFERS){
        int16_t * samples = (int16_t *) audio_buffers[filled_count % NUM_BUFFERS];
        btstack_audio_context_t context = { .timestamp = time_us_32() };

        playback_callback(samples, FRAMES_PER_BUFFER, &context);

        if (num_channels == 1){
            // Expand mono to stereo in place, back to front.
            for (int i = FRAMES_PER_BUFFER - 1; i >= 0; i--){
                int16_t s = samples[i];
                samples[2 * i]     = s;
                samples[2 * i + 1] = s;
            }
        }
        apply_volume(samples, FRAMES_PER_BUFFER * 2);
        add_anti_mute_tone(samples, FRAMES_PER_BUFFER);

        __compiler_memory_barrier();
        filled_count++;
    }
}

static void fill_data_source_process(btstack_data_source_t * ds, btstack_data_source_callback_type_t callback_type){
    (void) ds;
    if (callback_type == DATA_SOURCE_CALLBACK_POLL){
        fill_buffers();
    }
}

// Drop filled buffers that have not yet been handed to DMA.
static void discard_pending_buffers(void){
    uint32_t irq_state = save_and_disable_interrupts();
    filled_count = queued_count;
    restore_interrupts(irq_state);
}

// ---------------------------------------------------------------------------
// Hardware setup

static void set_pio_sample_rate(uint32_t rate){
    // PIO runs at 2 cycles per BCK, 32 BCK per frame.
    float div = (float) clock_get_hz(clk_sys) / (float) (rate * 64u);
    pio_sm_set_clkdiv(audio_pio, audio_sm, div);
}

static void hardware_init(uint32_t rate){
    // DAC soft mute only until the I2S clocks are running (avoids a power-up pop).
    gpio_init(AUDIO_MUTE_PIN);
    gpio_set_dir(AUDIO_MUTE_PIN, GPIO_OUT);
    gpio_put(AUDIO_MUTE_PIN, 0);

    // "Silence" is the anti-mute tone alone (see add_anti_mute_tone), not exact zero.
    memset(silence_buffer, 0, sizeof(silence_buffer));
    add_anti_mute_tone((int16_t *) silence_buffer, FRAMES_PER_BUFFER);

    // PIO
    bool ok = pio_claim_free_sm_and_add_program_for_gpio_range(&audio_i2s_program, &audio_pio, &audio_sm, &audio_pio_offset,
                                                                AUDIO_I2S_DATA_PIN, 3, true);
    hard_assert(ok);
    pio_gpio_init(audio_pio, AUDIO_I2S_DATA_PIN);
    pio_gpio_init(audio_pio, AUDIO_I2S_CLOCK_PIN_BASE);
    pio_gpio_init(audio_pio, AUDIO_I2S_CLOCK_PIN_BASE + 1);
    audio_i2s_program_init(audio_pio, audio_sm, audio_pio_offset, AUDIO_I2S_DATA_PIN, AUDIO_I2S_CLOCK_PIN_BASE);
    set_pio_sample_rate(rate);

    // DMA: two channels chained to each other
    dma_channels[0] = dma_claim_unused_channel(true);
    dma_channels[1] = dma_claim_unused_channel(true);
    for (int i = 0; i < 2; i++){
        dma_channel_config cfg = dma_channel_get_default_config(dma_channels[i]);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
        channel_config_set_read_increment(&cfg, true);
        channel_config_set_write_increment(&cfg, false);
        channel_config_set_dreq(&cfg, pio_get_dreq(audio_pio, audio_sm, true));
        channel_config_set_chain_to(&cfg, dma_channels[i ^ 1]);
        dma_channel_configure(dma_channels[i], &cfg, &audio_pio->txf[audio_sm], silence_buffer, FRAMES_PER_BUFFER, false);
        dma_channel_has_pool_buffer[i] = false;
        dma_irqn_set_channel_enabled(AUDIO_DMA_IRQ, dma_channels[i], true);
    }
    irq_add_shared_handler(dma_get_irq_num(AUDIO_DMA_IRQ), audio_dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(dma_get_irq_num(AUDIO_DMA_IRQ), true);

    // Start streaming silence; it runs forever from here on.
    dma_channel_start(dma_channels[0]);
    pio_sm_set_enabled(audio_pio, audio_sm, true);

    // Unmute now that clocks and (silent) data are running, and stay unmuted.
    // Holding XSMT low while idle left the output producing static on the
    // line/headphone jack; digital silence on an unmuted DAC is clean.
    gpio_put(AUDIO_MUTE_PIN, 1);

    hardware_initialized = true;
}

// BTstack data source used to refill buffers on the BTstack thread.
// Requires the BTstack run loop, so it is added on the first sink init.
static void add_fill_data_source(void){
    if (fill_data_source_added) return;
    btstack_run_loop_set_data_source_handler(&fill_data_source, &fill_data_source_process);
    btstack_run_loop_enable_data_source_callbacks(&fill_data_source, DATA_SOURCE_CALLBACK_POLL);
    btstack_run_loop_add_data_source(&fill_data_source);
    fill_data_source_added = true;
}

void btstack_audio_i2s_start_silence(uint32_t samplerate){
    if (hardware_initialized) return;
    sample_rate = samplerate;
    hardware_init(samplerate);
}

// ---------------------------------------------------------------------------
// btstack_audio_sink_t

static int i2s_sink_init(uint8_t channels, uint32_t samplerate,
                         void (*playback)(int16_t * buffer, uint16_t num_samples, const btstack_audio_context_t * context)){
    if (channels < 1 || channels > 2) return -1;

    stream_started    = false;
    num_channels      = channels;
    sample_rate       = samplerate;
    playback_callback = playback;

    if (!hardware_initialized){
        hardware_init(samplerate);
    } else {
        set_pio_sample_rate(samplerate);
    }
    add_fill_data_source();
    return 0;
}

static uint32_t i2s_sink_get_samplerate(void){
    return sample_rate;
}

static void i2s_sink_set_volume(uint8_t volume){
    if (volume > 127) volume = 127;
    // Square-law curve: closer to perceived loudness than linear.
    volume_gain_q15 = (int32_t) (((uint32_t) volume * volume * 32768u) / (127u * 127u));
}

static void i2s_sink_start_stream(void){
    if (!hardware_initialized) return;
    stream_started = true;
    fill_buffers();                 // prime the queue right away
}

static void i2s_sink_stop_stream(void){
    if (!hardware_initialized) return;
    stream_started = false;         // DMA falls back to the silence buffer
    discard_pending_buffers();
}

static void i2s_sink_close(void){
    i2s_sink_stop_stream();
    playback_callback = NULL;
}

static const btstack_audio_sink_t btstack_audio_i2s_sink = {
    .init           = &i2s_sink_init,
    .get_samplerate = &i2s_sink_get_samplerate,
    .set_volume     = &i2s_sink_set_volume,
    .start_stream   = &i2s_sink_start_stream,
    .stop_stream    = &i2s_sink_stop_stream,
    .close          = &i2s_sink_close,
};

const btstack_audio_sink_t * btstack_audio_i2s_sink_get_instance(void){
    return &btstack_audio_i2s_sink;
}
