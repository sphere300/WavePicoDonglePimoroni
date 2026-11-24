/*
 * Copyright (C) 2023 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */

/*
 * Bluetooth audio receiver: A2DP sink + AVRCP controller/target.
 *
 * Derived from BTstack's example/a2dp_sink_demo.c. Modifications by Sphere300
 * (2026) are distributed under the same BTstack licence as above. Use on
 * Raspberry Pi Pico W / Pico 2 W hardware is additionally covered by
 * Raspberry Pi's BTstack licence (pico-sdk src/rp2_common/pico_btstack/LICENSE.RP).
 *
 * Audio pipeline:
 *   L2CAP media packet -> SBC frame ring buffer -> SBC decode (on demand,
 *   when the I2S sink asks for samples) -> resampler (small drift
 *   correction based on buffer level) -> I2S sink.
 *
 * AVRCP target: the source's absolute volume is applied in software.
 * AVRCP controller: used to send media commands and to track play status.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "btstack_resample.h"
#include "btstack_ring_buffer.h"

#include "audio_receiver.h"
#include "btstack_audio_i2s.h"

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

// Volume change for AVRCP volume up/down pass-through commands from the source.
#define AVRCP_VOLUME_STEP_PERCENT 10

// ---------------------------------------------------------------------------
// State

static audio_receiver_connection_handler_t connection_handler;

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

static void a2dp_sink_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size);
static void handle_l2cap_media_data_packet(uint8_t seid, uint8_t * packet, uint16_t size);
static void avrcp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size);
static void avrcp_controller_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size);
static void avrcp_target_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size);

static void notify_connection_changed(void){
    if (connection_handler) connection_handler();
}

// ---------------------------------------------------------------------------
// Setup

void audio_receiver_init(audio_receiver_connection_handler_t on_connection_changed){
    connection_handler = on_connection_changed;

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

    // Audio output
    const btstack_audio_sink_t * sink = btstack_audio_i2s_sink_get_instance();
    btstack_audio_sink_set_instance(sink);
    // Bring up I2S now (plays silence) so the DAC is ready before the first stream.
    sink->init(NUM_CHANNELS, 44100, NULL);
    sink->set_volume((uint8_t) (volume_percentage * 127 / 100));
}

// ---------------------------------------------------------------------------
// Queries / control

bool audio_receiver_a2dp_connected(void){
    return a2dp_connection.a2dp_cid != 0;
}

uint16_t audio_receiver_avrcp_cid(void){
    return avrcp_connection.avrcp_cid;
}

bool audio_receiver_is_playing(void){
    if (avrcp_connection.play_status_known) return avrcp_connection.playing;
    return a2dp_connection.stream_state == STREAM_STATE_PLAYING;
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

void audio_receiver_change_volume(int delta_percent){
    set_volume_percentage(volume_percentage + delta_percent);
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
            notify_connection_changed();
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
            notify_connection_changed();
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

            // Track the player's play/pause state (used for the Play/Pause toggle).
            avrcp_connection.play_status_known = false;
            avrcp_controller_enable_notification(avrcp_connection.avrcp_cid, AVRCP_NOTIFICATION_EVENT_PLAYBACK_STATUS_CHANGED);
            notify_connection_changed();
            return;
        }
        case AVRCP_SUBEVENT_CONNECTION_RELEASED:
            printf("AVRCP: disconnected\n");
            avrcp_connection.avrcp_cid = 0;
            avrcp_connection.play_status_known = false;
            notify_connection_changed();
            return;
        default:
            break;
    }
}

// The only controller event we need is the player's playback status.
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
                case AVRCP_OPERATION_ID_VOLUME_UP:   set_volume_percentage(volume_percentage + AVRCP_VOLUME_STEP_PERCENT); break;
                case AVRCP_OPERATION_ID_VOLUME_DOWN: set_volume_percentage(volume_percentage - AVRCP_VOLUME_STEP_PERCENT); break;
                default: break;
            }
            break;
        default:
            break;
    }
}
