# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sphere300
#
# Host-side unit tests (run on a PC, no Pico needed).
#   cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build

cmake_minimum_required(VERSION 3.13)
project(WavePicoDongleTests C)

set(CMAKE_C_STANDARD 11)
enable_testing()

set(FIRMWARE_DIR ${CMAKE_CURRENT_LIST_DIR}/../WavePicoDongle)

# buttons.c is compiled unchanged; tests/stubs replaces the Pico SDK and BTstack headers.
add_executable(test_buttons
        test_buttons.c
        ${FIRMWARE_DIR}/buttons.c
        )
target_include_directories(test_buttons PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/stubs
        ${FIRMWARE_DIR}
        )
if (CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_buttons PRIVATE -Wall -Wextra -Werror)
endif()

add_test(NAME buttons COMMAND test_buttons)
