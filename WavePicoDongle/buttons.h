#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdbool.h>

// Momentary buttons wired between a GPIO and ground (internal pull-ups).
// Polled and debounced on the BTstack run loop, so callbacks may call BTstack.

typedef enum {
    BUTTON_FAST_FORWARD,    // GP2
    BUTTON_PLAY,            // GP21
    BUTTON_REWIND,          // GP6
    BUTTON_VOLUME_DOWN,     // GP5
    BUTTON_VOLUME_UP,       // GP26
    BUTTON_COUNT
} button_id_t;

typedef enum {
    BUTTON_EVENT_PRESSED,       // debounced press
    BUTTON_EVENT_HELD,          // still pressed after BUTTONS_HOLD_MS (once per press)
    BUTTON_EVENT_REPEAT,        // every BUTTONS_REPEAT_MS while held, after BUTTON_EVENT_HELD
    BUTTON_EVENT_RELEASED,      // debounced release; held tells whether HELD was sent
} button_event_t;

typedef void (*button_handler_t)(button_id_t button, button_event_t event, bool held);

// Configure the GPIOs. Polling does not start until buttons_set_enabled(true).
void buttons_init(button_handler_t handler);

// Start/stop polling. A button already pressed when polling starts is ignored
// until it is released, so nothing fires for a press made while disconnected.
void buttons_set_enabled(bool enabled);

#endif
