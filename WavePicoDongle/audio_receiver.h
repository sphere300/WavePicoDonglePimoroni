// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sphere300
//
// Interface to the Bluetooth audio receiver (A2DP sink + AVRCP), implemented
// in audio_receiver.c. Note that audio_receiver.c itself is derived from
// BTstack's a2dp_sink_demo.c and is under the BlueKitchen BTstack licence,
// not MIT; see the header of that file and LICENSE.

#ifndef AUDIO_RECEIVER_H
#define AUDIO_RECEIVER_H

#include <stdbool.h>
#include <stdint.h>

// Called on the BTstack thread whenever the A2DP or AVRCP connection state changes.
typedef void (*audio_receiver_connection_handler_t)(void);

// Set up L2CAP, SDP, A2DP sink, AVRCP (controller + target) and the I2S audio
// output. Call before hci_power_control(HCI_POWER_ON). GAP settings (name,
// discoverability, pairing) are left to the caller.
void audio_receiver_init(audio_receiver_connection_handler_t on_connection_changed);

// True while an A2DP stream connection to a source exists.
bool audio_receiver_a2dp_connected(void);

// AVRCP connection id, or 0 if no AVRCP connection is open.
uint16_t audio_receiver_avrcp_cid(void);

// Whether the source is currently playing: the player's own AVRCP status if
// it has reported one, otherwise whether audio is streaming.
bool audio_receiver_is_playing(void);

// Change the volume by delta_percent (clamped to 0..100) and report it to the source.
void audio_receiver_change_volume(int delta_percent);

#endif
