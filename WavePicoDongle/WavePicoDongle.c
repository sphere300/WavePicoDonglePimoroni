/*
 * WavePicoDongle - Bluetooth A2DP audio receiver for Raspberry Pi Pico 2 W
 * with a Pimoroni Pico Audio Pack (PCM5100A I2S DAC, line out + headphone amp).
 *
 * - Discoverable/connectable as a Bluetooth Classic headphone/speaker.
 * - Pairing window: for PAIRING_WINDOW_MS after power-up the dongle is
 *   discoverable and accepts one new device (Secure Simple Pairing with Secure
 *   Connections). The window closes early once a device pairs or connects.
 *   Afterwards it is hidden and only already-paired devices can reconnect.
 * - Receives SBC-encoded A2DP audio, decodes it and plays it via I2S.
 * - AVRCP target: the source's absolute volume is applied in software.
 * - Media buttons (buttons.c): play/pause, next/fast forward, previous/rewind,
 *   volume up/down; polled and debounced only while a source is connected.
 *
 * Audio pipeline (based on BTstack's example/a2dp_sink_demo.c):
 *   L2CAP media packet -> SBC frame ring buffer -> SBC decode (on demand,
 *   when the I2S sink asks for samples) -> resampler (small drift
 *   correction based on buffer level) -> I2S sink.
 *
 * Status LED (CYW43 LED): off = idle, on = source connected.
 */

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"

#include "btstack.h"
#include "btstack_resample.h"
#include "btstack_ring_buffer.h"

#include "btstack_audio_i2s.h"
#include "buttons.h"

#define DEVICE_NAME "WavePicoDongle"

// How long after power-up new devices may pair.
#define PAIRING_WINDOW_MS 30000

// Volume change per press (or per auto-repeat while held) of a volume button.
#define BUTTON_VOLUME_STEP_PERCENT 5

#define NUM_CHANNELS                   2
#define BYTES_PER_FRAME                (2 * NUM_CHANNELS)
#define MAX_SBC_FRAME_SIZE             512   // dual-channel / high-bitpool SBC frames exceed 120 bytes
#define SBC_MAX_AUDIO_FRAMES_PER_BLOCK 128
#define RESAMPLE_ADDITIONAL_FRAMES     16
#define RESAMPLE_OUTPUT_FRAMES         (SBC_MAX_AUDIO_FRAMES_PER_BLOCK + RESAMPLE_ADDITIONAL_FRAMES)

// SBC frame ring buffer: below MIN stretch samples, above MAX compress samples.
#define OPTIMAL_FRAMES_MIN 60
#define OPTIMAL_FRAMES_MAX 80
#define ADDITIONAL_FRAMES  30

// ---------------------------------------------------------------------------
// State

static btstack_packet_callback_registration_t hci_event_callback_registration;

static bool pairing_window_open;
static btstack_timer_source_t pairing_window_timer;

static uint8_t sdp_avdtp_sink_service_buffer[150];
static uint8_t sdp_avrcp_target_service_buffer[150];
static uint8_t sdp_avrcp_controller_service_buffer[200];
static uint8_t device_id_sdp_service_buffer[100];

// Support all SBC configurations with bitpool 2-53.
static uint8_t media_sbc_codec_capabilities[] = {
    0xFF,   // all sampling frequencies, all channel modes
    0xFF,   // all block lengths, subbands, allocation methods
    2, 53
};

static const btstack_sbc_decoder_t *   sbc_decoder_instance;
static btstack_sbc_decoder_bluedroid_t sbc_decoder_context;

static uint8_t sbc_frame_storage[(OPTIMAL_FRAMES_MAX + ADDITIONAL_FRAMES) * MAX_SBC_FRAME_SIZE];
static btstack_ring_buffer_t sbc_frame_ring_buffer;
static unsigned int sbc_frame_size;

// Overflow for partially consumed decoded SBC frames (+ resampling headroom).
static uint8_t decoded_audio_storage[RESAMPLE_OUTPUT_FRAMES * BYTES_PER_FRAME];
static btstack_ring_buffer_t decoded_audio_ring_buffer;

static int media_initialized;
static int audio_stream_started;
static btstack_resample_t resample_instance;
static uint32_t resampling_min_factor;

// Lower-layer request for audio samples, filled by handle_pcm_data().
static int16_t * request_buffer;
static int       request_frames;

static int volume_percentage = 100;
static const avrcp_battery_status_t battery_status = AVRCP_BATTERY_STATUS_EXTERNAL;

typedef struct {
    uint8_t  reconfigure;
    uint8_t  num_channels;
    uint16_t sampling_frequency;
    uint8_t  block_length;
    uint8_t  subbands;
    uint8_t  min_bitpool_value;
    uint8_t  max_bitpool_value;
    btstack_sbc_channel_mode_t      channel_mode;
    btstack_sbc_allocation_method_t allocation_method;
} media_codec_configuration_sbc_t;

typedef enum {
    STREAM_STATE_CLOSED,
    STREAM_STATE_OPEN,
    STREAM_STATE_PLAYING,
    STREAM_STATE_PAUSED,
} stream_state_t;

static struct {
    uint8_t a2dp_local_seid;
    uint8_t media_sbc_codec_configuration[4];
} stream_endpoint;

static struct {
    bd_addr_t      addr;
    uint16_t       a2dp_cid;
    uint8_t        a2dp_local_seid;
    stream_state_t stream_state;
    media_codec_configuration_sbc_t sbc_configuration;
} a2dp_connection;

static struct {
    uint16_t avrcp_cid;
    bool     play_status_known;     // player has reported its playback status
    bool     playing;
} avrcp_connection;

static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size);
static void a2dp_sink_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size);
static void handle_l2cap_media_data_packet(uint8_t seid, uint8_t * packet, uint16_t size);
static void avrcp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size);
static void avrcp_controller_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size);
static void avrcp_target_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size);
static void update_buttons_enabled(void);
static void handle_button(button_id_t button, button_event_t event, bool held);

static void set_led(bool on){
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
}

// ---------------------------------------------------------------------------
// Setup

static void setup_bluetooth(void){
    l2cap_init();
    sdp_init();

    a2dp_sink_init();
    avrcp_init();
    avrcp_controller_init();
    avrcp_target_init();

    // A2DP sink stream endpoint (SBC)
    a2dp_sink_register_packet_handler(&a2dp_sink_packet_handler);
    a2dp_sink_register_media_handler(&handle_l2cap_media_data_packet);
    avdtp_stream_endpoint_t * local_stream_endpoint = a2dp_sink_create_stream_endpoint(
            AVDTP_AUDIO, AVDTP_CODEC_SBC,
            media_sbc_codec_capabilities, sizeof(media_sbc_codec_capabilities),
            stream_endpoint.media_sbc_codec_configuration, sizeof(stream_endpoint.media_sbc_codec_configuration));
    btstack_assert(local_stream_endpoint != NULL);
    stream_endpoint.a2dp_local_seid = avdtp_local_seid(local_stream_endpoint);

    // AVRCP controller + target
    avrcp_register_packet_handler(&avrcp_packet_handler);
    avrcp_controller_register_packet_handler(&avrcp_controller_packet_handler);
    avrcp_target_register_packet_handler(&avrcp_target_packet_handler);

    // SDP records
    memset(sdp_avdtp_sink_service_buffer, 0, sizeof(sdp_avdtp_sink_service_buffer));
    a2dp_sink_create_sdp_record(sdp_avdtp_sink_service_buffer, sdp_create_service_record_handle(),
                                AVDTP_SINK_FEATURE_MASK_HEADPHONE | AVDTP_SINK_FEATURE_MASK_SPEAKER, NULL, NULL);
    btstack_assert(de_get_len(sdp_avdtp_sink_service_buffer) <= sizeof(sdp_avdtp_sink_service_buffer));
    sdp_register_service(sdp_avdtp_sink_service_buffer);

    // We send Category 1 commands (play/pause) to the media player.
    memset(sdp_avrcp_controller_service_buffer, 0, sizeof(sdp_avrcp_controller_service_buffer));
    uint16_t controller_supported_features = 1 << AVRCP_CONTROLLER_SUPPORTED_FEATURE_CATEGORY_PLAYER_OR_RECORDER;
    avrcp_controller_create_sdp_record(sdp_avrcp_controller_service_buffer, sdp_create_service_record_handle(),
                                       controller_supported_features, NULL, NULL);
    btstack_assert(de_get_len(sdp_avrcp_controller_service_buffer) <= sizeof(sdp_avrcp_controller_service_buffer));
    sdp_register_service(sdp_avrcp_controller_service_buffer);

    // We receive Category 2 commands (absolute volume) from the media player.
    memset(sdp_avrcp_target_service_buffer, 0, sizeof(sdp_avrcp_target_service_buffer));
    uint16_t target_supported_features = 1 << AVRCP_TARGET_SUPPORTED_FEATURE_CATEGORY_MONITOR_OR_AMPLIFIER;
    avrcp_target_create_sdp_record(sdp_avrcp_target_service_buffer, sdp_create_service_record_handle(),
                                   target_supported_features, NULL, NULL);
    btstack_assert(de_get_len(sdp_avrcp_target_service_buffer) <= sizeof(sdp_avrcp_target_service_buffer));
    sdp_register_service(sdp_avrcp_target_service_buffer);

    memset(device_id_sdp_service_buffer, 0, sizeof(device_id_sdp_service_buffer));
    device_id_create_sdp_record(device_id_sdp_service_buffer, sdp_create_service_record_handle(),
                                DEVICE_ID_VENDOR_ID_SOURCE_BLUETOOTH, BLUETOOTH_COMPANY_ID_BLUEKITCHEN_GMBH, 1, 1);
    btstack_assert(de_get_len(device_id_sdp_service_buffer) <= sizeof(device_id_sdp_service_buffer));
    sdp_register_service(device_id_sdp_service_buffer);

    // GAP: always connectable (so paired devices can reconnect); discoverable
    // and pairable only during the pairing window (opened on HCI_STATE_WORKING).
    gap_set_local_name(DEVICE_NAME);
    gap_discoverable_control(0);
    gap_set_bondable_mode(0);
    // Class of Device: Service Class Audio + Rendering, Major Audio/Video, Minor Loudspeaker
    gap_set_class_of_device(0x240414);
    // Secure Simple Pairing with Secure Connections (P-256 ECDH). No display or
    // buttons, so pairing is "just works"; it is accepted manually in
    // hci_packet_handler, and only while the pairing window is open.
    gap_ssp_set_enable(1);
    gap_secure_connections_enable(true);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    gap_ssp_set_auto_accept(0);
    // Require an encrypted link from a completed pairing for A2DP/AVRCP.
    // Only SDP is reachable without pairing.
    gap_set_security_level(LEVEL_2);
    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE);
    gap_set_allow_role_switch(true);

    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Media buttons (polled only while a source is connected)
    buttons_init(&handle_button);

    // Audio output
    const btstack_audio_sink_t * sink = btstack_audio_i2s_sink_get_instance();
    btstack_audio_sink_set_instance(sink);
    // Bring up I2S now (plays silence, DAC muted) so the DAC is ready before the first stream.
    sink->init(NUM_CHANNELS, 44100, NULL);
    sink->set_volume((uint8_t) (volume_percentage * 127 / 100));
}

// ---------------------------------------------------------------------------
// Media processing

// Called by the I2S sink (on the BTstack thread) when it needs samples.
static void playback_handler(int16_t * buffer, uint16_t num_audio_frames, const btstack_audio_context_t * context){
    UNUSED(context);

    if (sbc_frame_size == 0){
        memset(buffer, 0, num_audio_frames * BYTES_PER_FRAME);
        return;
    }

    // First, leftovers from previously decoded/resampled audio.
    uint32_t bytes_read;
    btstack_ring_buffer_read(&decoded_audio_ring_buffer, (uint8_t *) buffer, num_audio_frames * BYTES_PER_FRAME, &bytes_read);
    buffer           += bytes_read / 2;
    num_audio_frames -= bytes_read / BYTES_PER_FRAME;

    // Then decode SBC frames until the request is satisfied (handle_pcm_data writes into request_*).
    request_buffer = buffer;
    request_frames = num_audio_frames;
    while (request_frames && btstack_ring_buffer_bytes_available(&sbc_frame_ring_buffer) >= sbc_frame_size){
        uint8_t sbc_frame[MAX_SBC_FRAME_SIZE];
        btstack_ring_buffer_read(&sbc_frame_ring_buffer, sbc_frame, sbc_frame_size, &bytes_read);
        sbc_decoder_instance->decode_signed_16(&sbc_decoder_context, 0, sbc_frame, sbc_frame_size);
    }

    // Underrun: pad with silence rather than leave stale data.
    if (request_frames > 0){
        memset(request_buffer, 0, request_frames * BYTES_PER_FRAME);
        request_frames = 0;
    }
}

// SBC decoder output.
static void handle_pcm_data(int16_t * data, int num_audio_frames, int num_channels, int sample_rate, void * context){
    UNUSED(sample_rate);
    UNUSED(context);
    UNUSED(num_channels);   // decoder always delivers stereo here

    int16_t  output_buffer[RESAMPLE_OUTPUT_FRAMES * NUM_CHANNELS];
    uint32_t resampled_frames = btstack_resample_block(&resample_instance, data, num_audio_frames, output_buffer);

    int frames_to_copy = btstack_min(resampled_frames, request_frames);
    memcpy(request_buffer, output_buffer, frames_to_copy * BYTES_PER_FRAME);
    request_frames -= frames_to_copy;
    request_buffer += frames_to_copy * NUM_CHANNELS;

    int frames_to_store = resampled_frames - frames_to_copy;
    if (frames_to_store){
        int status = btstack_ring_buffer_write(&decoded_audio_ring_buffer,
                                               (uint8_t *) &output_buffer[frames_to_copy * NUM_CHANNELS],
                                               frames_to_store * BYTES_PER_FRAME);
        if (status){
            printf("PCM ring buffer overflow\n");
        }
    }
}

static int media_processing_init(media_codec_configuration_sbc_t * configuration){
    if (media_initialized) return 0;

    sbc_decoder_instance = btstack_sbc_decoder_bluedroid_init_instance(&sbc_decoder_context);
    sbc_decoder_instance->configure(&sbc_decoder_context, SBC_MODE_STANDARD, handle_pcm_data, NULL);

    btstack_ring_buffer_init(&sbc_frame_ring_buffer, sbc_frame_storage, sizeof(sbc_frame_storage));
    btstack_ring_buffer_init(&decoded_audio_ring_buffer, decoded_audio_storage, sizeof(decoded_audio_storage));
    btstack_resample_init(&resample_instance, NUM_CHANNELS);
    resampling_min_factor = btstack_resample_get_min_factor_for_output_capacity(SBC_MAX_AUDIO_FRAMES_PER_BLOCK, RESAMPLE_OUTPUT_FRAMES);

    const btstack_audio_sink_t * audio = btstack_audio_sink_get_instance();
    audio->init(NUM_CHANNELS, configuration->sampling_frequency, &playback_handler);

    audio_stream_started = 0;
    media_initialized = 1;
    return 0;
}

static void media_processing_start(void){
    if (!media_initialized) return;
    btstack_audio_sink_get_instance()->start_stream();
    audio_stream_started = 1;
}

static void media_processing_pause(void){
    if (!media_initialized) return;
    audio_stream_started = 0;
    btstack_audio_sink_get_instance()->stop_stream();
    btstack_ring_buffer_reset(&decoded_audio_ring_buffer);
    btstack_ring_buffer_reset(&sbc_frame_ring_buffer);
}

static void media_processing_close(void){
    if (!media_initialized) return;
    media_initialized = 0;
    audio_stream_started = 0;
    sbc_frame_size = 0;
    btstack_audio_sink_get_instance()->close();
}

// ---------------------------------------------------------------------------
// Media packets

static int read_media_data_header(uint8_t * packet, int size, int * offset, avdtp_media_packet_header_t * media_header){
    const int media_header_len = 12;
    int pos = *offset;
    if (size - pos < media_header_len) return 0;

    media_header->version    = packet[pos] & 0x03;
    media_header->padding    = get_bit16(packet[pos], 2);
    media_header->extension  = get_bit16(packet[pos], 3);
    media_header->csrc_count = (packet[pos] >> 4) & 0x0F;
    pos++;
    media_header->marker       = get_bit16(packet[pos], 0);
    media_header->payload_type = (packet[pos] >> 1) & 0x7F;
    pos++;
    media_header->sequence_number = big_endian_read_16(packet, pos);
    pos += 2;
    media_header->timestamp = big_endian_read_32(packet, pos);
    pos += 4;
    media_header->synchronization_source = big_endian_read_32(packet, pos);
    pos += 4;
    *offset = pos;
    return 1;
}

static int read_sbc_header(uint8_t * packet, int size, int * offset, avdtp_sbc_codec_header_t * sbc_header){
    int pos = *offset;
    if (size - pos < 1) return 0;
    sbc_header->fragmentation   = get_bit16(packet[pos], 7);
    sbc_header->starting_packet = get_bit16(packet[pos], 6);
    sbc_header->last_packet     = get_bit16(packet[pos], 5);
    sbc_header->num_frames      = packet[pos] & 0x0f;
    pos++;
    *offset = pos;
    return 1;
}

static void handle_l2cap_media_data_packet(uint8_t seid, uint8_t * packet, uint16_t size){
    UNUSED(seid);
    if (!media_initialized) return;

    int pos = 0;
    avdtp_media_packet_header_t media_header;
    if (!read_media_data_header(packet, size, &pos, &media_header)) return;

    avdtp_sbc_codec_header_t sbc_header;
    if (!read_sbc_header(packet, size, &pos, &sbc_header)) return;
    if (sbc_header.num_frames == 0) return;

    int      packet_length = size - pos;
    uint8_t *packet_begin  = packet + pos;

    sbc_frame_size = packet_length / sbc_header.num_frames;
    if (sbc_frame_size == 0 || sbc_frame_size > MAX_SBC_FRAME_SIZE){
        printf("Unsupported SBC frame size %u\n", sbc_frame_size);
        sbc_frame_size = 0;
        return;
    }
    if (btstack_ring_buffer_write(&sbc_frame_ring_buffer, packet_begin, packet_length) != ERROR_CODE_SUCCESS){
        printf("SBC ring buffer overflow\n");
    }

    // Clock drift compensation: nudge the resampler based on buffer fill level.
    int sbc_frames_in_buffer = btstack_ring_buffer_bytes_available(&sbc_frame_ring_buffer) / sbc_frame_size;
    const uint32_t nominal_factor = 0x10000;
    const uint32_t compensation   = 0x00100;
    uint32_t resampling_factor;
    if (sbc_frames_in_buffer < OPTIMAL_FRAMES_MIN){
        resampling_factor = nominal_factor - compensation;  // stretch
    } else if (sbc_frames_in_buffer <= OPTIMAL_FRAMES_MAX){
        resampling_factor = nominal_factor;
    } else {
        resampling_factor = nominal_factor + compensation;  // compress
    }
    btstack_assert(resampling_factor >= resampling_min_factor);
    btstack_resample_set_factor(&resample_instance, resampling_factor);

    // Start playback once enough frames are buffered.
    if (!audio_stream_started && sbc_frames_in_buffer >= OPTIMAL_FRAMES_MIN){
        media_processing_start();
    }
}

// ---------------------------------------------------------------------------
// HCI / GAP

static void close_pairing_window(const char * reason){
    if (!pairing_window_open) return;
    pairing_window_open = false;
    btstack_run_loop_remove_timer(&pairing_window_timer);
    gap_discoverable_control(0);
    gap_set_bondable_mode(0);
    printf("Pairing window closed (%s) - only paired devices can connect\n", reason);
}

static void pairing_window_timeout_handler(btstack_timer_source_t * ts){
    UNUSED(ts);
    close_pairing_window("timeout");
}

static void open_pairing_window(void){
    pairing_window_open = true;
    gap_set_bondable_mode(1);
    gap_discoverable_control(1);
    btstack_run_loop_set_timer_handler(&pairing_window_timer, &pairing_window_timeout_handler);
    btstack_run_loop_set_timer(&pairing_window_timer, PAIRING_WINDOW_MS);
    btstack_run_loop_add_timer(&pairing_window_timer);
    printf("Pairing window open for %u s\n", (unsigned) (PAIRING_WINDOW_MS / 1000));
}

static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;

    bd_addr_t address;
    switch (hci_event_packet_get_type(packet)){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
                gap_local_bd_addr(address);
                printf("Bluetooth ready: '%s' %s\n", DEVICE_NAME, bd_addr_to_str(address));
                open_pairing_window();
            }
            break;
        case HCI_EVENT_PIN_CODE_REQUEST:
            // Legacy PIN pairing (pre-2.1 devices, or an SSP downgrade): never allowed.
            hci_event_pin_code_request_get_bd_addr(packet, address);
            printf("Rejecting legacy PIN pairing from %s\n", bd_addr_to_str(address));
            gap_pin_code_negative(address);
            break;
        case HCI_EVENT_USER_CONFIRMATION_REQUEST:
            // SSP "just works": accept only while the pairing window is open.
            // Already-paired devices reconnect with their stored key and never get here.
            hci_event_user_confirmation_request_get_bd_addr(packet, address);
            if (pairing_window_open){
                printf("Accepting pairing from %s\n", bd_addr_to_str(address));
                gap_ssp_confirmation_response(address);
            } else {
                printf("Rejecting pairing from %s (pairing window closed)\n", bd_addr_to_str(address));
                gap_ssp_confirmation_negative(address);
            }
            break;
        case HCI_EVENT_LINK_KEY_NOTIFICATION:
            // BTstack has already stored the new key by the time this reaches us.
            reverse_bd_addr(&packet[2], address);
            printf("Paired with %s\n", bd_addr_to_str(address));
            close_pairing_window("device paired");
            break;
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// A2DP

static void a2dp_sink_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META) return;

    uint8_t status;
    switch (packet[2]){
        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_OTHER_CONFIGURATION:
            printf("A2DP: non-SBC codec requested - not supported\n");
            break;

        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION: {
            media_codec_configuration_sbc_t * cfg = &a2dp_connection.sbc_configuration;
            cfg->reconfigure        = a2dp_subevent_signaling_media_codec_sbc_configuration_get_reconfigure(packet);
            cfg->num_channels       = a2dp_subevent_signaling_media_codec_sbc_configuration_get_num_channels(packet);
            cfg->sampling_frequency = a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet);
            cfg->block_length       = a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(packet);
            cfg->subbands           = a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(packet);
            cfg->min_bitpool_value  = a2dp_subevent_signaling_media_codec_sbc_configuration_get_min_bitpool_value(packet);
            cfg->max_bitpool_value  = a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet);
            // Bluetooth spec values are 1-based; the SBC codec's enum is 0-based.
            cfg->allocation_method  = (btstack_sbc_allocation_method_t)
                    (a2dp_subevent_signaling_media_codec_sbc_configuration_get_allocation_method(packet) - 1);
            switch (a2dp_subevent_signaling_media_codec_sbc_configuration_get_channel_mode(packet)){
                case AVDTP_CHANNEL_MODE_JOINT_STEREO: cfg->channel_mode = SBC_CHANNEL_MODE_JOINT_STEREO; break;
                case AVDTP_CHANNEL_MODE_STEREO:       cfg->channel_mode = SBC_CHANNEL_MODE_STEREO;       break;
                case AVDTP_CHANNEL_MODE_DUAL_CHANNEL: cfg->channel_mode = SBC_CHANNEL_MODE_DUAL_CHANNEL; break;
                case AVDTP_CHANNEL_MODE_MONO:         cfg->channel_mode = SBC_CHANNEL_MODE_MONO;         break;
                default: btstack_assert(false); break;
            }
            printf("A2DP: SBC config %u Hz, %u ch, mode %d, blocks %u, subbands %u, bitpool %u-%u\n",
                   cfg->sampling_frequency, cfg->num_channels, cfg->channel_mode,
                   cfg->block_length, cfg->subbands, cfg->min_bitpool_value, cfg->max_bitpool_value);
            break;
        }

        case A2DP_SUBEVENT_STREAM_ESTABLISHED:
            status = a2dp_subevent_stream_established_get_status(packet);
            if (status != ERROR_CODE_SUCCESS){
                printf("A2DP: stream connection failed, status 0x%02x\n", status);
                break;
            }
            a2dp_subevent_stream_established_get_bd_addr(packet, a2dp_connection.addr);
            a2dp_connection.a2dp_cid        = a2dp_subevent_stream_established_get_a2dp_cid(packet);
            a2dp_connection.a2dp_local_seid = a2dp_subevent_stream_established_get_local_seid(packet);
            a2dp_connection.stream_state    = STREAM_STATE_OPEN;
            printf("A2DP: connected to %s\n", bd_addr_to_str(a2dp_connection.addr));
            set_led(true);
            close_pairing_window("device connected");
            update_buttons_enabled();
            break;

        case A2DP_SUBEVENT_STREAM_STARTED:
            printf("A2DP: stream started\n");
            a2dp_connection.stream_state = STREAM_STATE_PLAYING;
            if (a2dp_connection.sbc_configuration.reconfigure){
                media_processing_close();
            }
            // Playback itself starts once the SBC buffer reaches its minimum level.
            media_processing_init(&a2dp_connection.sbc_configuration);
            break;

        case A2DP_SUBEVENT_STREAM_SUSPENDED:
            printf("A2DP: stream paused\n");
            a2dp_connection.stream_state = STREAM_STATE_PAUSED;
            media_processing_pause();
            break;

        case A2DP_SUBEVENT_STREAM_RELEASED:
            printf("A2DP: stream released\n");
            a2dp_connection.stream_state = STREAM_STATE_CLOSED;
            media_processing_close();
            break;

        case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
            printf("A2DP: disconnected\n");
            a2dp_connection.a2dp_cid = 0;
            media_processing_close();
            set_led(false);
            update_buttons_enabled();
            break;

        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// AVRCP

static void avrcp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;

    switch (packet[2]){
        case AVRCP_SUBEVENT_CONNECTION_ESTABLISHED: {
            uint8_t status = avrcp_subevent_connection_established_get_status(packet);
            if (status != ERROR_CODE_SUCCESS){
                printf("AVRCP: connection failed, status 0x%02x\n", status);
                avrcp_connection.avrcp_cid = 0;
                return;
            }
            avrcp_connection.avrcp_cid = avrcp_subevent_connection_established_get_avrcp_cid(packet);
            printf("AVRCP: connected, cid 0x%02x\n", avrcp_connection.avrcp_cid);

            avrcp_target_support_event(avrcp_connection.avrcp_cid, AVRCP_NOTIFICATION_EVENT_VOLUME_CHANGED);
            avrcp_target_support_event(avrcp_connection.avrcp_cid, AVRCP_NOTIFICATION_EVENT_BATT_STATUS_CHANGED);
            avrcp_target_battery_status_changed(avrcp_connection.avrcp_cid, battery_status);
            avrcp_target_volume_changed(avrcp_connection.avrcp_cid, (uint8_t) (volume_percentage * 127 / 100));

            // Track the player's play/pause state so the Play button can toggle it.
            avrcp_connection.play_status_known = false;
            avrcp_controller_enable_notification(avrcp_connection.avrcp_cid, AVRCP_NOTIFICATION_EVENT_PLAYBACK_STATUS_CHANGED);
            update_buttons_enabled();
            return;
        }
        case AVRCP_SUBEVENT_CONNECTION_RELEASED:
            printf("AVRCP: disconnected\n");
            avrcp_connection.avrcp_cid = 0;
            avrcp_connection.play_status_known = false;
            update_buttons_enabled();
            return;
        default:
            break;
    }
}

// We act as AVRCP controller to send the media button commands. The only event
// we need is the player's playback status (for the Play/Pause toggle).
static void avrcp_controller_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;
    if (packet[2] != AVRCP_SUBEVENT_NOTIFICATION_PLAYBACK_STATUS_CHANGED) return;

    uint8_t play_status = avrcp_subevent_notification_playback_status_changed_get_play_status(packet);
    avrcp_connection.play_status_known = true;
    avrcp_connection.playing = (play_status == AVRCP_PLAYBACK_STATUS_PLAYING);
}

static void set_volume_percentage(int new_volume_percentage){
    if (new_volume_percentage < 0)   new_volume_percentage = 0;
    if (new_volume_percentage > 100) new_volume_percentage = 100;
    volume_percentage = new_volume_percentage;
    uint8_t volume = (uint8_t) (volume_percentage * 127 / 100);
    btstack_audio_sink_get_instance()->set_volume(volume);
    if (avrcp_connection.avrcp_cid){
        avrcp_target_volume_changed(avrcp_connection.avrcp_cid, volume);
    }
    printf("Volume %d%%\n", volume_percentage);
}

static void avrcp_target_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;

    switch (packet[2]){
        case AVRCP_SUBEVENT_NOTIFICATION_VOLUME_CHANGED: {
            // Absolute volume set by the source (e.g. phone volume buttons).
            uint8_t volume = avrcp_subevent_notification_volume_changed_get_absolute_volume(packet);
            volume_percentage = volume * 100 / 127;
            btstack_audio_sink_get_instance()->set_volume(volume);
            printf("Volume %d%% (%u)\n", volume_percentage, volume);
            break;
        }
        case AVRCP_SUBEVENT_OPERATION:
            if (avrcp_subevent_operation_get_button_pressed(packet) == 0) break;
            switch ((avrcp_operation_id_t) avrcp_subevent_operation_get_operation_id(packet)){
                case AVRCP_OPERATION_ID_VOLUME_UP:   set_volume_percentage(volume_percentage + 10); break;
                case AVRCP_OPERATION_ID_VOLUME_DOWN: set_volume_percentage(volume_percentage - 10); break;
                default: break;
            }
            break;
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Media buttons

// Buttons are only polled while a source is connected (A2DP or AVRCP).
static void update_buttons_enabled(void){
    buttons_set_enabled(a2dp_connection.a2dp_cid != 0 || avrcp_connection.avrcp_cid != 0);
}

static void log_avrcp_command(const char * name, uint8_t status){
    if (status == ERROR_CODE_SUCCESS){
        printf("AVRCP: sent %s\n", name);
    } else {
        printf("AVRCP: %s failed, status 0x%02x\n", name, status);
    }
}

static void toggle_play_pause(uint16_t cid){
    // Prefer the player's own status; before it has reported one, go by
    // whether audio is currently streaming.
    bool playing = avrcp_connection.play_status_known
                 ? avrcp_connection.playing
                 : (a2dp_connection.stream_state == STREAM_STATE_PLAYING);
    if (playing){
        log_avrcp_command("pause", avrcp_controller_pause(cid));
    } else {
        log_avrcp_command("play", avrcp_controller_play(cid));
    }
}

// Play:          press = play/pause
// Fast forward:  tap = next track,     hold = fast forward until released
// Rewind:        tap = previous track, hold = rewind until released
// Volume up/down: press = one step, hold = repeat
static void handle_button(button_id_t button, button_event_t event, bool held){
    uint16_t cid = avrcp_connection.avrcp_cid;
    if (!cid && button != BUTTON_VOLUME_UP && button != BUTTON_VOLUME_DOWN && event == BUTTON_EVENT_PRESSED){
        printf("Button ignored: no AVRCP connection to send commands on\n");
    }

    switch (button){
        case BUTTON_VOLUME_UP:
        case BUTTON_VOLUME_DOWN:
            // Volume is applied locally (and reported to the source), so this
            // works even if the source has not opened AVRCP.
            if (event == BUTTON_EVENT_PRESSED || event == BUTTON_EVENT_REPEAT){
                int step = (button == BUTTON_VOLUME_UP) ? BUTTON_VOLUME_STEP_PERCENT : -BUTTON_VOLUME_STEP_PERCENT;
                set_volume_percentage(volume_percentage + step);
            }
            break;

        case BUTTON_PLAY:
            if (cid && event == BUTTON_EVENT_PRESSED){
                toggle_play_pause(cid);
            }
            break;

        case BUTTON_FAST_FORWARD:
        case BUTTON_REWIND: {
            if (!cid) break;
            bool forward = (button == BUTTON_FAST_FORWARD);
            if (event == BUTTON_EVENT_HELD){
                if (forward){
                    log_avrcp_command("fast forward (hold)", avrcp_controller_press_and_hold_fast_forward(cid));
                } else {
                    log_avrcp_command("rewind (hold)", avrcp_controller_press_and_hold_rewind(cid));
                }
            } else if (event == BUTTON_EVENT_RELEASED){
                if (held){
                    log_avrcp_command("release hold", avrcp_controller_release_press_and_hold_cmd(cid));
                } else if (forward){
                    log_avrcp_command("next track", avrcp_controller_forward(cid));
                } else {
                    log_avrcp_command("previous track", avrcp_controller_backward(cid));
                }
            }
            break;
        }

        default:
            break;
    }
}

// ---------------------------------------------------------------------------

int main(void){
    // Feed the DAC silence from the very start, so it never sees a gap in I2S.
    btstack_audio_i2s_start_silence(44100);

    stdio_init_all();

    // Initialises the CYW43 (WiFi/BT chip) and the async context that BTstack runs on.
    if (cyw43_arch_init()){
        printf("cyw43_arch_init failed\n");
        return -1;
    }

    // Force the RT6154 3.3 V regulator into PWM mode (PS pin on WL_GPIO1).
    // In the default PFM mode, the bursty current of the Bluetooth radio causes
    // large supply ripple, which reaches the DAC/headphone amp as static.
    // Costs a few mA at light load (Pico 2 W datasheet, section 3.4).
    cyw43_arch_gpio_put(CYW43_WL_GPIO_SMPS_PIN, 1);

    setup_bluetooth();

    printf("Starting Bluetooth...\n");
    hci_power_control(HCI_POWER_ON);

    // Everything runs from interrupts/async context; the main loop just idles.
    while (true){
        sleep_ms(1000);
    }
}
