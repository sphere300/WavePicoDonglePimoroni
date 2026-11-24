// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sphere300

/*
 * WavePicoDongle - Bluetooth A2DP audio receiver for Raspberry Pi Pico 2 W
 * with a Pimoroni Pico Audio Pack (PCM5100A I2S DAC, line out + headphone amp).
 *
 * This file: GAP setup, pairing window, media buttons, status LED and main().
 * The A2DP/AVRCP audio receiver lives in audio_receiver.c (BTstack licence).
 *
 * - Discoverable/connectable as a Bluetooth Classic headphone/speaker.
 * - Pairing window: for PAIRING_WINDOW_MS after power-up the dongle is
 *   discoverable and accepts one new device (Secure Simple Pairing with Secure
 *   Connections). The window closes early once a device pairs or connects.
 *   Afterwards it is hidden and only already-paired devices can reconnect.
 * - Media buttons (buttons.c): play/pause, next/fast forward, previous/rewind,
 *   volume up/down; polled and debounced only while a source is connected.
 *
 * Status LED (CYW43 LED): off = idle, on = source connected.
 */

#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"

#include "btstack.h"

#include "audio_receiver.h"
#include "btstack_audio_i2s.h"
#include "buttons.h"

#define DEVICE_NAME "WavePicoDongle"

// How long after power-up new devices may pair.
#define PAIRING_WINDOW_MS 30000

// Volume change per press (or per auto-repeat while held) of a volume button.
#define BUTTON_VOLUME_STEP_PERCENT 5

static btstack_packet_callback_registration_t hci_event_callback_registration;

static bool pairing_window_open;
static btstack_timer_source_t pairing_window_timer;

static void set_led(bool on){
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
}

// ---------------------------------------------------------------------------
// Pairing window

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

static void gap_event_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size){
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

static void setup_gap(void){
    // Always connectable (so paired devices can reconnect); discoverable and
    // pairable only during the pairing window (opened on HCI_STATE_WORKING).
    gap_set_local_name(DEVICE_NAME);
    gap_discoverable_control(0);
    gap_set_bondable_mode(0);
    // Class of Device: Service Class Audio + Rendering, Major Audio/Video, Minor Loudspeaker
    gap_set_class_of_device(0x240414);
    // Secure Simple Pairing with Secure Connections (P-256 ECDH). No display or
    // buttons, so pairing is "just works"; it is accepted manually in
    // gap_event_handler, and only while the pairing window is open.
    gap_ssp_set_enable(1);
    gap_secure_connections_enable(true);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    gap_ssp_set_auto_accept(0);
    // Require an encrypted link from a completed pairing for A2DP/AVRCP.
    // Only SDP is reachable without pairing.
    gap_set_security_level(LEVEL_2);
    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE);
    gap_set_allow_role_switch(true);

    hci_event_callback_registration.callback = &gap_event_handler;
    hci_add_event_handler(&hci_event_callback_registration);
}

// ---------------------------------------------------------------------------
// Connection state

static void handle_connection_changed(void){
    bool a2dp_connected = audio_receiver_a2dp_connected();
    set_led(a2dp_connected);
    if (a2dp_connected){
        close_pairing_window("device connected");
    }
    // Buttons are only polled while a source is connected (A2DP or AVRCP).
    buttons_set_enabled(a2dp_connected || audio_receiver_avrcp_cid() != 0);
}

// ---------------------------------------------------------------------------
// Media buttons

static void log_avrcp_command(const char * name, uint8_t status){
    if (status == ERROR_CODE_SUCCESS){
        printf("AVRCP: sent %s\n", name);
    } else {
        printf("AVRCP: %s failed, status 0x%02x\n", name, status);
    }
}

// Play:          press = play/pause
// Fast forward:  tap = next track,     hold = fast forward until released
// Rewind:        tap = previous track, hold = rewind until released
// Volume up/down: press = one step, hold = repeat
static void handle_button(button_id_t button, button_event_t event, bool held){
    uint16_t cid = audio_receiver_avrcp_cid();
    if (!cid && button != BUTTON_VOLUME_UP && button != BUTTON_VOLUME_DOWN && event == BUTTON_EVENT_PRESSED){
        printf("Button ignored: no AVRCP connection to send commands on\n");
    }

    switch (button){
        case BUTTON_VOLUME_UP:
        case BUTTON_VOLUME_DOWN:
            // Volume is applied locally (and reported to the source), so this
            // works even if the source has not opened AVRCP.
            if (event == BUTTON_EVENT_PRESSED || event == BUTTON_EVENT_REPEAT){
                audio_receiver_change_volume((button == BUTTON_VOLUME_UP) ? BUTTON_VOLUME_STEP_PERCENT
                                                                          : -BUTTON_VOLUME_STEP_PERCENT);
            }
            break;

        case BUTTON_PLAY:
            if (cid && event == BUTTON_EVENT_PRESSED){
                if (audio_receiver_is_playing()){
                    log_avrcp_command("pause", avrcp_controller_pause(cid));
                } else {
                    log_avrcp_command("play", avrcp_controller_play(cid));
                }
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

    audio_receiver_init(&handle_connection_changed);
    setup_gap();
    buttons_init(&handle_button);

    printf("Starting Bluetooth...\n");
    hci_power_control(HCI_POWER_ON);

    // Everything runs from interrupts/async context; the main loop just idles.
    while (true){
        sleep_ms(1000);
    }
}
