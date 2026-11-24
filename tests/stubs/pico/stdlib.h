// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sphere300
//
// Host-test stand-in for the Pico SDK's pico/stdlib.h: just the GPIO calls
// buttons.c uses, backed by a fake pin model in the test.

#ifndef TEST_STUB_PICO_STDLIB_H
#define TEST_STUB_PICO_STDLIB_H

#include <stdbool.h>
#include <stdint.h>

typedef unsigned int uint;

#define GPIO_IN  false
#define GPIO_OUT true

void gpio_init(uint gpio);
void gpio_set_dir(uint gpio, bool out);
void gpio_pull_up(uint gpio);
bool gpio_get(uint gpio);

#endif
