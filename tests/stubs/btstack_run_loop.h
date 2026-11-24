// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sphere300
//
// Host-test stand-in for BTstack's btstack_run_loop.h: just the timer calls
// buttons.c uses, backed by a fake clock in the test. Signatures match BTstack.

#ifndef TEST_STUB_BTSTACK_RUN_LOOP_H
#define TEST_STUB_BTSTACK_RUN_LOOP_H

#include <stdint.h>

typedef struct btstack_timer_source {
    void (*process)(struct btstack_timer_source * ts);
    uint32_t timeout;
} btstack_timer_source_t;

void     btstack_run_loop_set_timer_handler(btstack_timer_source_t * ts, void (*process)(btstack_timer_source_t * _ts));
void     btstack_run_loop_set_timer(btstack_timer_source_t * ts, uint32_t timeout_in_ms);
void     btstack_run_loop_add_timer(btstack_timer_source_t * ts);
int      btstack_run_loop_remove_timer(btstack_timer_source_t * ts);
uint32_t btstack_run_loop_get_time_ms(void);

#endif
