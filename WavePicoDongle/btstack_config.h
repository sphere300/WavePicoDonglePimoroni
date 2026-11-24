// SPDX-License-Identifier: MIT AND BSD-3-Clause
// Copyright (c) 2026 Sphere300
// Portions Copyright (c) 2020 Raspberry Pi (Trading) Ltd.: several settings
// follow pico-examples' pico_w/bt/config/btstack_config_common.h (BSD-3-Clause).

// BTstack configuration for WavePicoDongle (Pico 2 W, Bluetooth Classic A2DP sink).
#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// ENABLE_CLASSIC is provided by linking pico_btstack_classic.

// ---- Logging -----------------------------------------------------------
#define ENABLE_LOG_ERROR
// #define ENABLE_LOG_INFO          // very verbose; enable for debugging
#define ENABLE_PRINTF_HEXDUMP

// ---- HAL ---------------------------------------------------------------
#define HAVE_EMBEDDED_TIME_MS
#define HAVE_ASSERT                 // map btstack_assert onto Pico SDK assert()
// Use malloc for BTstack's object pools. Without it every MAX_NR_* pool has
// to be sized by hand, and pools left undefined silently default to 0.
#define HAVE_MALLOC

// ---- HCI ---------------------------------------------------------------
#define HCI_RESET_RESEND_TIMEOUT_MS   1000
#define HCI_OUTGOING_PRE_BUFFER_SIZE  4
#define HCI_ACL_PAYLOAD_SIZE          (1691 + 4)
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT  4

// Limit the number of ACL/SCO buffers used by the stack to avoid overrunning
// the cyw43 shared bus.
#define MAX_NR_CONTROLLER_ACL_BUFFERS 3
#define MAX_NR_CONTROLLER_SCO_PACKETS 3

// Controller-to-host flow control, also to protect the cyw43 shared bus.
#define ENABLE_HCI_CONTROLLER_TO_HOST_FLOW_CONTROL
#define HCI_HOST_ACL_PACKET_LEN       1024
#define HCI_HOST_ACL_PACKET_NUM       3
#define HCI_HOST_SCO_PACKET_LEN       120
#define HCI_HOST_SCO_PACKET_NUM       3

// ---- Persistent storage (link keys in flash via TLV) -------------------
#define NVM_NUM_LINK_KEYS             16
#define NVM_NUM_DEVICE_DB_ENTRIES     16

#endif // BTSTACK_CONFIG_H
