#ifndef BTSTACK_AUDIO_I2S_H
#define BTSTACK_AUDIO_I2S_H

#include "btstack_audio.h"

// Pimoroni Pico Audio Pack (PIM544) wiring, see Docs/pico_audio_pack_schematic.pdf
#ifndef AUDIO_I2S_DATA_PIN
#define AUDIO_I2S_DATA_PIN       9   // PCM5100A DIN
#endif
#ifndef AUDIO_I2S_CLOCK_PIN_BASE
#define AUDIO_I2S_CLOCK_PIN_BASE 10  // PCM5100A BCK (GP10) and LRCK (GP11)
#endif
#ifndef AUDIO_MUTE_PIN
#define AUDIO_MUTE_PIN           22  // PCM5100A XSMT: low = soft mute, high = play (kept high once I2S runs)
#endif

// BTstack audio sink that plays 16-bit stereo through a PIO I2S DAC.
// Whenever no audio is queued (idle, buffering, paused, underrun) it clocks out silence.
const btstack_audio_sink_t * btstack_audio_i2s_sink_get_instance(void);

// Start clocking silence to the DAC right away, before BTstack is up.
// Safe to call before cyw43_arch_init(); the sink's init() later takes over.
void btstack_audio_i2s_start_silence(uint32_t samplerate);

#endif
