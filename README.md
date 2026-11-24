# WavePicoDongle

[![CI](https://github.com/sphere300/WavePicoDonglePimoroni/actions/workflows/ci.yml/badge.svg)](https://github.com/sphere300/WavePicoDonglePimoroni/actions/workflows/ci.yml)

A Bluetooth audio receiver built from a Raspberry Pi Pico 2 W and a Pimoroni Pico Audio Pack.
Pair a phone or computer with it and it plays the audio through the Audio Pack's line out and headphone jacks.
Five optional push buttons control playback and volume.

## Features

- **Bluetooth Classic A2DP sink**: shows up as a speaker/headphone named `WavePicoDongle`. SBC audio at 44.1 or 48 kHz.
- **Secure pairing window**: for 30 seconds after power-up the dongle is discoverable and accepts one new device, using Secure Simple Pairing with Secure Connections. After that it is hidden and only already-paired devices can connect.
- **Absolute volume (AVRCP)**: the phone's volume slider controls the dongle's volume. One volume control, not two stacked.
- **Media buttons**: play/pause, next/fast forward, previous/rewind, volume up/down.
- **Clean idle output**: the DAC gets a continuous I2S stream from power-up, so there are no pops and no static while nothing is playing (see [Noise fixes](#noise-fixes)).
- **Status LED**: the Pico's on-board LED is on while a device is connected.

## Hardware

| Part | Notes |
|---|---|
| Raspberry Pi Pico 2 W | RP2350 with CYW43439 radio. The original Pico W isn't tested. |
| Pimoroni Pico Audio Pack (PIM544) | PCM5100A I2S DAC with line out and a headphone amp |
| 5 momentary push buttons (optional) | Each between a GPIO and GND; internal pull-ups are used |

### Pin usage

| GPIO | Physical pin | Function |
|---|---|---|
| GP9 | 12 | I2S data (DAC DIN) |
| GP10 | 14 | I2S bit clock (DAC BCK) |
| GP11 | 15 | I2S word clock (DAC LRCK) |
| GP22 | 29 | DAC soft mute (XSMT), held high once audio starts |
| GP2 | 4 | Button: fast forward |
| GP21 | 27 | Button: play/pause |
| GP6 | 9 | Button: rewind |
| GP5 | 7 | Button: volume down |
| GP26 | 31 | Button: volume up |

BCK and LRCK must be on consecutive pins: the PIO program drives LRCK on the pin after BCK.
The I2S pins can be changed at build time with `-DAUDIO_I2S_DATA_PIN=`, `-DAUDIO_I2S_CLOCK_PIN_BASE=` and `-DAUDIO_MUTE_PIN=`.
Button pins are set in `button_pins[]` in [buttons.c](WavePicoDongle/buttons.c).

## Using it

### Pairing a new device

1. Power the dongle on (or power-cycle it).
2. Within 30 seconds, select **WavePicoDongle** in the phone's Bluetooth settings. No PIN is needed.
3. The window closes as soon as a device pairs or connects, or after 30 seconds.

Already-paired devices can reconnect at any time. If a paired phone reconnects automatically during the window, it closes the window. To add another device, turn the paired phone's Bluetooth off before power-cycling.

### Buttons

Buttons are only checked while a device is connected. A button already held down when a device connects is ignored until it is released.

| Button | Tap | Hold (over 600 ms) |
|---|---|---|
| Play | Play/pause | — |
| Fast forward | Next track | Fast forward until released |
| Rewind | Previous track | Rewind until released |
| Volume up | +5% | Repeats every 200 ms |
| Volume down | −5% | Repeats every 200 ms |

Support for held fast forward/rewind depends on the music app; next and previous track work almost everywhere.

> **Status:** the buttons are still being tested on hardware. The firmware currently logs button states and AVRCP command results over USB serial to help track down why presses have no effect.

### Serial log

The firmware logs to USB serial (any baud rate): pairing, connections, stream state, volume changes and button events.

## Building

Requirements:

- Raspberry Pi Pico SDK **2.3.1** (includes BTstack and the CYW43 driver)
- Arm GNU toolchain 15.2 (`arm-none-eabi-gcc`)
- CMake and Ninja

The easiest setup is the **Raspberry Pi Pico** VS Code extension, which installs these under `~/.pico-sdk` and uses the settings in [CMakeLists.txt](WavePicoDongle/CMakeLists.txt). Open the `WavePicoDongle` folder and press **Compile**.

From the command line:

```sh
cd WavePicoDongle
cmake -S . -B build -G Ninja -DPICO_BOARD=pico2_w
cmake --build build
```

Add `-DWAVE_STRICT_WARNINGS=ON` to treat warnings in the project's own sources as errors, as CI does.

### Tests

Host-side unit tests for the button logic (debounce, hold, auto-repeat, enable/disable) run on a PC, with no Pico needed. `buttons.c` is compiled unchanged against stub GPIO and timer headers in [tests/stubs](tests/stubs).

```sh
cmake -S tests -B tests/build
cmake --build tests/build
ctest --test-dir tests/build --output-on-failure
```

This needs a host C compiler (gcc or clang).

### Continuous integration

[.github/workflows/ci.yml](.github/workflows/ci.yml) runs on every push to `main` and on every pull request:

- **Unit tests:** built with AddressSanitizer and UndefinedBehaviorSanitizer.
- **Firmware build:** compiled in Release and Debug against Pico SDK 2.3.1, with strict warnings on. The Release `.uf2` is uploaded as a build artifact, kept for 14 days.

It uses only standard GitHub-hosted Linux runners, which are free for public repositories.

### Flashing

Hold **BOOTSEL** while plugging in the Pico, then copy `WavePicoDongle/build/WavePicoDongle.uf2` to the `RP2350` drive that appears.
Alternatively, run `picotool load -x build/WavePicoDongle.uf2 -f`.

## How it works

```
Phone ──A2DP (SBC)──▶ L2CAP media packets ──▶ SBC frame ring buffer
                                                    │  decoded on demand
                                                    ▼
             I2S DAC ◀── PIO ◀── DMA ◀── buffers ◀── resampler (drift correction)
```

- **Audio path** ([audio_receiver.c](WavePicoDongle/audio_receiver.c)): incoming SBC frames are queued in a ring buffer. When the I2S driver needs samples, frames are decoded and passed through a resampler that nudges the playback rate up or down slightly. This keeps the buffer level steady despite clock drift between the phone and the Pico. Playback begins once about 60 frames are buffered.
- **I2S driver** ([btstack_audio_i2s.c](WavePicoDongle/btstack_audio_i2s.c)): a PIO state machine generates I2S, fed by two DMA channels chained to each other, so buffers switch with no gap. Buffers are refilled on the BTstack thread, never inside the interrupt. If no audio buffer is ready, the DMA plays a silence buffer instead, so the stream never stops.
- **Volume**: AVRCP absolute volume (0–127) is turned into a gain with a square-law curve and applied to the samples in software. The PCM5100A has no hardware gain control.
- **Pairing** (`gap_event_handler` in [WavePicoDongle.c](WavePicoDongle/WavePicoDongle.c)): pairing requests are accepted only while the pairing window is open. Legacy PIN pairing is always refused.
- **Buttons** ([buttons.c](WavePicoDongle/buttons.c)): a BTstack timer polls the pins every 5 ms. A change is accepted after 4 matching samples (20 ms debounce). The module reports press, hold, repeat and release events to `handle_button()`, which sends AVRCP commands or changes the volume.

## Noise fixes

Three things caused hiss or static on the output, and the firmware works around all of them:

1. **DAC soft mute**: holding XSMT (GP22) low while idle produced static. The pin is now driven high as soon as I2S is running and stays high.
2. **DAC zero-data auto-mute**: after 1024 consecutive all-zero samples, the PCM510x enters a full analog mute (datasheet section 9.3.2.3; it can't be disabled in hardware mode). That mute is also noisy. Every frame therefore carries a ±1 LSB tone at fs/2, which the DAC's interpolation filter removes, so it is inaudible. The DAC never sees exact digital zero.
3. **Regulator ripple from the radio**: the Bluetooth radio draws current in bursts, and in the default PFM mode the Pico 2 W's RT6154 regulator turns those bursts into supply ripple. The firmware drives WL_GPIO1 high to force PWM mode (Pico 2 W datasheet, section 3.4). This costs a few mA.

If some noise remains, the causes are physical:
- Keep the Pico's antenna end away from the Audio Pack and the jack.
- Use a shielded cable.
- Try a different USB supply to rule out ground loops.

## Repository layout

```
WavePicoDongle/        Firmware project
  WavePicoDongle.c       GAP setup, pairing window, button actions, LED, main()
  audio_receiver.c/h     A2DP sink + AVRCP: audio pipeline, volume, play status (.c is BTstack-licensed)
  btstack_audio_i2s.c/h  PIO + DMA I2S audio sink for the PCM5100A
  audio_i2s.pio          PIO program generating I2S
  buttons.c/h            Debounced media-button polling
  btstack_config.h       BTstack configuration
  CMakeLists.txt
tests/                 Host unit tests (test_buttons.c) and stub headers
.github/workflows/     GitHub Actions CI
LICENSE                MIT licence, with the list of files under other licences
```

## License

This project is under the [MIT licence](LICENSE), **except** for these files, which keep the licence in their own headers:

| File | Licence |
|---|---|
| [audio_receiver.c](WavePicoDongle/audio_receiver.c) | BTstack licence, Copyright (C) 2023 BlueKitchen GmbH. Derived from BTstack's `example/a2dp_sink_demo.c`. BSD-style, but **non-commercial only**. |
| [audio_i2s.pio](WavePicoDongle/audio_i2s.pio), [pico_sdk_import.cmake](WavePicoDongle/pico_sdk_import.cmake) | BSD-3-Clause, Copyright (c) 2020 Raspberry Pi (Trading) Ltd. |
| [btstack_config.h](WavePicoDongle/btstack_config.h) | MIT, with portions under BSD-3-Clause from Raspberry Pi's pico-examples |

The firmware is built on [BTstack](https://github.com/bluekitchen/btstack) (Copyright (C) 2009 BlueKitchen GmbH) and the Raspberry Pi Pico SDK. Any built `.uf2` includes BTstack and is subject to its licence:

- **Personal, non-commercial use** is allowed under BTstack's own licence.
- **Commercial use** is allowed only on Raspberry Pi Pico W / Pico 2 W hardware, under Raspberry Pi's BTstack licence (`pico-sdk/src/rp2_common/pico_btstack/LICENSE.RP`), or with a commercial licence from BlueKitchen GmbH.

## Acknowledgements

The audio pipeline in `audio_receiver.c` is based on BTstack's `example/a2dp_sink_demo.c` by BlueKitchen GmbH.
