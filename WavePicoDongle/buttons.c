// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sphere300

/*
 * Debounced polling of the media buttons.
 *
 * Every BUTTONS_POLL_MS a BTstack timer samples all buttons. A button changes
 * state only after it has read the same new level for BUTTONS_DEBOUNCE_SAMPLES
 * consecutive polls (i.e. it has been stable for ~20 ms), which filters out
 * contact bounce on both press and release.
 */

#include <stdio.h>

#include "pico/stdlib.h"

#include "btstack_run_loop.h"
#include "buttons.h"

#define BUTTONS_POLL_MS          5
#define BUTTONS_DEBOUNCE_SAMPLES 4      // 4 x 5 ms = 20 ms stable
#define BUTTONS_HOLD_MS          600
#define BUTTONS_REPEAT_MS        200

static const uint button_pins[BUTTON_COUNT] = {
    [BUTTON_FAST_FORWARD] = 2,
    [BUTTON_PLAY]         = 21,
    [BUTTON_REWIND]       = 6,
    [BUTTON_VOLUME_DOWN]  = 5,
    [BUTTON_VOLUME_UP]    = 26,
};

typedef struct {
    bool     pressed;           // debounced state
    uint8_t  change_count;      // consecutive samples that differ from 'pressed'
    bool     ignore;            // pressed when polling started: wait for release
    bool     held;              // BUTTON_EVENT_HELD sent for this press
    uint32_t pressed_ms;        // time of the debounced press
    uint32_t next_repeat_ms;
} button_state_t;

static button_state_t   buttons[BUTTON_COUNT];
static button_handler_t button_handler;
static btstack_timer_source_t poll_timer;
static bool enabled;

static inline bool read_pressed(int i){
    return !gpio_get(button_pins[i]);    // shorted to ground = pressed
}

static void poll_buttons(btstack_timer_source_t * ts){
    uint32_t now = btstack_run_loop_get_time_ms();

    for (int i = 0; i < BUTTON_COUNT; i++){
        button_state_t * b = &buttons[i];
        bool raw = read_pressed(i);

        // Debounce: accept a new level only once it has been stable.
        if (raw != b->pressed){
            if (++b->change_count >= BUTTONS_DEBOUNCE_SAMPLES){
                b->change_count = 0;
                b->pressed = raw;
                printf("Button GP%u %s%s\n", button_pins[i], raw ? "pressed" : "released",
                       b->ignore ? " (ignored: held at connect)" : "");
                if (b->ignore){
                    if (!raw) b->ignore = false;   // released: arm for the next press
                    continue;
                }
                if (raw){
                    b->held = false;
                    b->pressed_ms = now;
                    button_handler((button_id_t) i, BUTTON_EVENT_PRESSED, false);
                } else {
                    button_handler((button_id_t) i, BUTTON_EVENT_RELEASED, b->held);
                }
            }
            continue;
        }
        b->change_count = 0;

        // Hold / auto-repeat while the button stays pressed.
        if (b->pressed && !b->ignore){
            if (!b->held){
                if ((int32_t) (now - b->pressed_ms) >= BUTTONS_HOLD_MS){
                    b->held = true;
                    b->next_repeat_ms = now + BUTTONS_REPEAT_MS;
                    button_handler((button_id_t) i, BUTTON_EVENT_HELD, true);
                }
            } else if ((int32_t) (now - b->next_repeat_ms) >= 0){
                b->next_repeat_ms += BUTTONS_REPEAT_MS;
                button_handler((button_id_t) i, BUTTON_EVENT_REPEAT, true);
            }
        }
    }

    btstack_run_loop_set_timer(ts, BUTTONS_POLL_MS);
    btstack_run_loop_add_timer(ts);
}

void buttons_init(button_handler_t handler){
    button_handler = handler;
    for (int i = 0; i < BUTTON_COUNT; i++){
        gpio_init(button_pins[i]);
        gpio_set_dir(button_pins[i], GPIO_IN);
        gpio_pull_up(button_pins[i]);
    }
    btstack_run_loop_set_timer_handler(&poll_timer, &poll_buttons);
}

void buttons_set_enabled(bool enable){
    if (enable == enabled) return;
    enabled = enable;

    if (!enable){
        btstack_run_loop_remove_timer(&poll_timer);
        printf("Buttons disabled\n");
        return;
    }

    // Start from the current levels; anything already pressed is ignored
    // until released.
    printf("Buttons enabled, levels:");
    for (int i = 0; i < BUTTON_COUNT; i++){
        bool raw = read_pressed(i);
        buttons[i] = (button_state_t) { .pressed = raw, .ignore = raw };
        printf(" GP%u=%s", button_pins[i], raw ? "PRESSED" : "up");
    }
    printf("\n");
    btstack_run_loop_set_timer(&poll_timer, BUTTONS_POLL_MS);
    btstack_run_loop_add_timer(&poll_timer);
}
