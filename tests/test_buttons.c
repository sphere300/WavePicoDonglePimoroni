// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sphere300
//
// Host unit tests for WavePicoDongle/buttons.c (debounce, hold, auto-repeat,
// ignore-at-connect, enable/disable, pin mapping).
//
// buttons.c is compiled unchanged against stub headers (tests/stubs); the
// fakes below model the GPIO pins (pull-ups, active low) and the BTstack
// run-loop timer with a simulated millisecond clock.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "btstack_run_loop.h"
#include "buttons.h"
#include "pico/stdlib.h"

// ---------------------------------------------------------------------------
// Fakes

#define NUM_GPIOS 30

static bool gpio_initialised[NUM_GPIOS];
static bool gpio_is_output[NUM_GPIOS];
static bool gpio_has_pull_up[NUM_GPIOS];
static bool gpio_shorted_to_ground[NUM_GPIOS];   // button held down

void gpio_init(uint gpio)                { gpio_initialised[gpio] = true; }
void gpio_set_dir(uint gpio, bool out)   { gpio_is_output[gpio] = out; }
void gpio_pull_up(uint gpio)             { gpio_has_pull_up[gpio] = true; }
bool gpio_get(uint gpio)                 { return !gpio_shorted_to_ground[gpio]; }   // pulled high unless pressed

static uint32_t now_ms;
static btstack_timer_source_t * active_timer;

void btstack_run_loop_set_timer_handler(btstack_timer_source_t * ts, void (*process)(btstack_timer_source_t * _ts)){
    ts->process = process;
}
void btstack_run_loop_set_timer(btstack_timer_source_t * ts, uint32_t timeout_in_ms){
    ts->timeout = now_ms + timeout_in_ms;
}
void btstack_run_loop_add_timer(btstack_timer_source_t * ts){
    active_timer = ts;
}
int btstack_run_loop_remove_timer(btstack_timer_source_t * ts){
    if (active_timer != ts) return 0;
    active_timer = NULL;
    return 1;
}
uint32_t btstack_run_loop_get_time_ms(void){
    return now_ms;
}

// Advance the simulated clock 1 ms at a time, firing the timer when due.
static void advance_ms(uint32_t ms){
    for (uint32_t i = 0; i < ms; i++){
        now_ms++;
        if (active_timer && (int32_t) (now_ms - active_timer->timeout) >= 0){
            btstack_timer_source_t * ts = active_timer;
            active_timer = NULL;          // one-shot, like BTstack; handler re-adds
            ts->process(ts);
        }
    }
}

// ---------------------------------------------------------------------------
// Event recording

typedef struct {
    button_id_t    button;
    button_event_t event;
    bool           held;
    uint32_t       time_ms;
} recorded_event_t;

#define MAX_EVENTS 64
static recorded_event_t events[MAX_EVENTS];
static int num_events;

static void record_event(button_id_t button, button_event_t event, bool held){
    if (num_events < MAX_EVENTS){
        events[num_events++] = (recorded_event_t) { button, event, held, now_ms };
    }
}

// ---------------------------------------------------------------------------
// Minimal test framework

static int failures;
static const char * current_test;

#define CHECK(cond) do { \
        if (!(cond)){ \
            printf("  FAIL %s:%d [%s]: %s\n", __FILE__, __LINE__, current_test, #cond); \
            failures++; \
        } \
    } while (0)

#define CHECK_EQ_INT(actual, expected) do { \
        long a_ = (long) (actual), e_ = (long) (expected); \
        if (a_ != e_){ \
            printf("  FAIL %s:%d [%s]: %s == %ld, expected %ld\n", __FILE__, __LINE__, current_test, #actual, a_, e_); \
            failures++; \
        } \
    } while (0)

static const uint PIN_FAST_FORWARD = 2;
static const uint PIN_PLAY         = 21;
static const uint PIN_REWIND       = 6;
static const uint PIN_VOLUME_DOWN  = 5;
static const uint PIN_VOLUME_UP    = 26;

static void reset(void){
    buttons_set_enabled(false);
    memset(gpio_initialised, 0, sizeof(gpio_initialised));
    memset(gpio_is_output, 0, sizeof(gpio_is_output));
    memset(gpio_has_pull_up, 0, sizeof(gpio_has_pull_up));
    memset(gpio_shorted_to_ground, 0, sizeof(gpio_shorted_to_ground));
    active_timer = NULL;
    now_ms = 1000;
    num_events = 0;
    buttons_init(&record_event);
}

static void press(uint pin)   { gpio_shorted_to_ground[pin] = true; }
static void release(uint pin) { gpio_shorted_to_ground[pin] = false; }

// ---------------------------------------------------------------------------
// Tests

static void test_init_configures_pins_as_pulled_up_inputs(void){
    const uint pins[] = { PIN_FAST_FORWARD, PIN_PLAY, PIN_REWIND, PIN_VOLUME_DOWN, PIN_VOLUME_UP };
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++){
        CHECK(gpio_initialised[pins[i]]);
        CHECK(!gpio_is_output[pins[i]]);
        CHECK(gpio_has_pull_up[pins[i]]);
    }
}

static void test_not_polled_until_enabled(void){
    press(PIN_PLAY);
    advance_ms(1000);
    CHECK_EQ_INT(num_events, 0);
    CHECK(active_timer == NULL);
}

static void test_pin_mapping(void){
    struct { uint pin; button_id_t id; } map[] = {
        { PIN_FAST_FORWARD, BUTTON_FAST_FORWARD },
        { PIN_PLAY,         BUTTON_PLAY },
        { PIN_REWIND,       BUTTON_REWIND },
        { PIN_VOLUME_DOWN,  BUTTON_VOLUME_DOWN },
        { PIN_VOLUME_UP,    BUTTON_VOLUME_UP },
    };
    buttons_set_enabled(true);
    for (size_t i = 0; i < sizeof(map) / sizeof(map[0]); i++){
        num_events = 0;
        press(map[i].pin);
        advance_ms(50);
        release(map[i].pin);
        advance_ms(50);
        CHECK_EQ_INT(num_events, 2);
        CHECK_EQ_INT(events[0].button, map[i].id);
        CHECK_EQ_INT(events[0].event, BUTTON_EVENT_PRESSED);
        CHECK_EQ_INT(events[1].button, map[i].id);
        CHECK_EQ_INT(events[1].event, BUTTON_EVENT_RELEASED);
    }
}

static void test_tap_reports_press_then_release(void){
    buttons_set_enabled(true);
    advance_ms(20);
    uint32_t pressed_at = now_ms;
    press(PIN_PLAY);
    advance_ms(100);
    CHECK_EQ_INT(num_events, 1);
    CHECK_EQ_INT(events[0].event, BUTTON_EVENT_PRESSED);
    CHECK(!events[0].held);
    // Debounce: 4 stable samples at 5 ms => reported 16-25 ms after the press.
    uint32_t latency = events[0].time_ms - pressed_at;
    CHECK(latency >= 15 && latency <= 25);

    release(PIN_PLAY);
    advance_ms(100);
    CHECK_EQ_INT(num_events, 2);
    CHECK_EQ_INT(events[1].event, BUTTON_EVENT_RELEASED);
    CHECK(!events[1].held);
}

static void test_short_glitch_is_filtered(void){
    buttons_set_enabled(true);
    advance_ms(20);
    press(PIN_VOLUME_UP);
    advance_ms(12);          // only 2 polls see it
    release(PIN_VOLUME_UP);
    advance_ms(200);
    CHECK_EQ_INT(num_events, 0);
}

static void test_contact_bounce_is_filtered(void){
    buttons_set_enabled(true);
    advance_ms(20);
    // Contacts chatter every poll for 100 ms, then settle released.
    for (int i = 0; i < 20; i++){
        if (i & 1) release(PIN_REWIND); else press(PIN_REWIND);
        advance_ms(5);
    }
    release(PIN_REWIND);
    advance_ms(200);
    CHECK_EQ_INT(num_events, 0);
}

static void test_bounce_on_press_reports_single_press(void){
    buttons_set_enabled(true);
    advance_ms(20);
    // Chatter for 30 ms, then held firmly.
    for (int i = 0; i < 6; i++){
        if (i & 1) release(PIN_PLAY); else press(PIN_PLAY);
        advance_ms(5);
    }
    press(PIN_PLAY);
    advance_ms(100);
    CHECK_EQ_INT(num_events, 1);
    CHECK_EQ_INT(events[0].event, BUTTON_EVENT_PRESSED);
}

static void test_hold_and_auto_repeat(void){
    buttons_set_enabled(true);
    advance_ms(20);
    press(PIN_FAST_FORWARD);
    advance_ms(1100);

    // PRESSED (~20 ms), HELD (+600 ms), REPEAT every 200 ms after that.
    CHECK_EQ_INT(num_events, 4);
    CHECK_EQ_INT(events[0].event, BUTTON_EVENT_PRESSED);
    CHECK_EQ_INT(events[1].event, BUTTON_EVENT_HELD);
    CHECK(events[1].held);
    CHECK_EQ_INT(events[2].event, BUTTON_EVENT_REPEAT);
    CHECK_EQ_INT(events[3].event, BUTTON_EVENT_REPEAT);

    uint32_t hold_after = events[1].time_ms - events[0].time_ms;
    CHECK(hold_after >= 600 && hold_after <= 610);
    uint32_t repeat_gap = events[3].time_ms - events[2].time_ms;
    CHECK(repeat_gap >= 195 && repeat_gap <= 205);

    release(PIN_FAST_FORWARD);
    advance_ms(100);
    CHECK_EQ_INT(num_events, 5);
    CHECK_EQ_INT(events[4].event, BUTTON_EVENT_RELEASED);
    CHECK(events[4].held);
}

static void test_release_before_hold_has_no_held_flag(void){
    buttons_set_enabled(true);
    advance_ms(20);
    press(PIN_REWIND);
    advance_ms(400);         // under the 600 ms hold threshold
    release(PIN_REWIND);
    advance_ms(100);
    CHECK_EQ_INT(num_events, 2);
    CHECK_EQ_INT(events[1].event, BUTTON_EVENT_RELEASED);
    CHECK(!events[1].held);
}

static void test_button_held_at_enable_is_ignored_until_released(void){
    press(PIN_PLAY);
    buttons_set_enabled(true);
    advance_ms(1500);        // long enough that a real press would have HELD/REPEAT
    CHECK_EQ_INT(num_events, 0);

    release(PIN_PLAY);
    advance_ms(100);
    CHECK_EQ_INT(num_events, 0);

    press(PIN_PLAY);         // next press works normally
    advance_ms(100);
    CHECK_EQ_INT(num_events, 1);
    CHECK_EQ_INT(events[0].event, BUTTON_EVENT_PRESSED);
}

static void test_disable_stops_polling(void){
    buttons_set_enabled(true);
    advance_ms(20);
    buttons_set_enabled(false);
    CHECK(active_timer == NULL);
    press(PIN_VOLUME_DOWN);
    advance_ms(1000);
    CHECK_EQ_INT(num_events, 0);
}

static void test_reenable_after_disable(void){
    buttons_set_enabled(true);
    buttons_set_enabled(false);
    buttons_set_enabled(true);
    advance_ms(20);
    press(PIN_VOLUME_UP);
    advance_ms(100);
    CHECK_EQ_INT(num_events, 1);
    CHECK_EQ_INT(events[0].button, BUTTON_VOLUME_UP);
}

static void test_simultaneous_buttons_are_independent(void){
    buttons_set_enabled(true);
    advance_ms(20);
    press(PIN_VOLUME_UP);
    press(PIN_PLAY);
    advance_ms(100);
    CHECK_EQ_INT(num_events, 2);
    bool saw_play = false, saw_volume_up = false;
    for (int i = 0; i < num_events; i++){
        CHECK_EQ_INT(events[i].event, BUTTON_EVENT_PRESSED);
        if (events[i].button == BUTTON_PLAY)      saw_play = true;
        if (events[i].button == BUTTON_VOLUME_UP) saw_volume_up = true;
    }
    CHECK(saw_play);
    CHECK(saw_volume_up);
}

static void test_clock_wraparound(void){
    buttons_set_enabled(false);
    now_ms = UINT32_MAX - 300;     // hold spans the 32-bit wrap
    buttons_set_enabled(true);
    press(PIN_FAST_FORWARD);
    advance_ms(1100);
    CHECK_EQ_INT(num_events, 4);
    CHECK_EQ_INT(events[1].event, BUTTON_EVENT_HELD);
    CHECK_EQ_INT(events[3].event, BUTTON_EVENT_REPEAT);
}

// ---------------------------------------------------------------------------

typedef struct {
    const char * name;
    void (*fn)(void);
} test_case_t;

int main(void){
    const test_case_t tests[] = {
        { "init_configures_pins_as_pulled_up_inputs",     test_init_configures_pins_as_pulled_up_inputs },
        { "not_polled_until_enabled",                     test_not_polled_until_enabled },
        { "pin_mapping",                                  test_pin_mapping },
        { "tap_reports_press_then_release",               test_tap_reports_press_then_release },
        { "short_glitch_is_filtered",                     test_short_glitch_is_filtered },
        { "contact_bounce_is_filtered",                   test_contact_bounce_is_filtered },
        { "bounce_on_press_reports_single_press",         test_bounce_on_press_reports_single_press },
        { "hold_and_auto_repeat",                         test_hold_and_auto_repeat },
        { "release_before_hold_has_no_held_flag",         test_release_before_hold_has_no_held_flag },
        { "button_held_at_enable_is_ignored_until_released", test_button_held_at_enable_is_ignored_until_released },
        { "disable_stops_polling",                        test_disable_stops_polling },
        { "reenable_after_disable",                       test_reenable_after_disable },
        { "simultaneous_buttons_are_independent",         test_simultaneous_buttons_are_independent },
        { "clock_wraparound",                             test_clock_wraparound },
    };

    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++){
        current_test = tests[i].name;
        int before = failures;
        reset();
        tests[i].fn();
        printf("%s %s\n", failures == before ? "PASS" : "FAIL", tests[i].name);
    }

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
