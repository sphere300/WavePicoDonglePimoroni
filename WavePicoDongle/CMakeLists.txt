# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(WavePicoDongle C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(WavePicoDongle
        WavePicoDongle.c
        btstack_audio_i2s.c
        buttons.c
        )

pico_generate_pio_header(WavePicoDongle ${CMAKE_CURRENT_LIST_DIR}/audio_i2s.pio)

pico_set_program_name(WavePicoDongle "WavePicoDongle")
pico_set_program_version(WavePicoDongle "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(WavePicoDongle 0)
pico_enable_stdio_usb(WavePicoDongle 1)

# Add the standard library to the build
target_link_libraries(WavePicoDongle
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(WavePicoDongle PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Bluetooth (BTstack Classic + A2DP SBC decoder on the CYW43) and I2S audio output
target_link_libraries(WavePicoDongle
        pico_cyw43_arch_none
        pico_btstack_cyw43
        pico_btstack_classic
        pico_btstack_sbc_decoder
        hardware_pio
        hardware_dma
        hardware_clocks
        )

pico_add_extra_outputs(WavePicoDongle)

