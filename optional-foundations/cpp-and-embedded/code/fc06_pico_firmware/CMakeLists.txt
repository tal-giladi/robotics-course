# FC.06 - a Pico SDK project. Compare it with fc04_wheelodom/CMakeLists.txt: same language,
# but the SDK brings in a toolchain file, a linker script and a post-build step that produces
# the .uf2 you drag onto the board.
#
#   export PICO_SDK_PATH=$HOME/pico/pico-sdk
#   cmake -S . -B build -DPICO_BOARD=pico2 -DCMAKE_BUILD_TYPE=RelWithDebInfo
#   cmake --build build -j
#   arm-none-eabi-size build/fc06_firmware.elf
#   # then: hold BOOTSEL, plug in, copy build/fc06_firmware.uf2 onto the RP2350 drive

cmake_minimum_required(VERSION 3.13)

# This must come BEFORE project(): it installs the arm-none-eabi cross toolchain so that
# project() probes the cross compiler instead of your host g++.
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(fc06_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()              # downloads nothing; configures the SDK targets for PICO_BOARD

add_executable(fc06_firmware main.cpp)

target_link_libraries(fc06_firmware
  pico_stdlib          # gpio, sleep, stdio, clocks, timers
  hardware_pwm
  hardware_watchdog
)

# stdout over USB CDC (the Pico appears as /dev/ttyACM0), not over the UART pins.
pico_enable_stdio_usb(fc06_firmware 1)
pico_enable_stdio_uart(fc06_firmware 0)

# Produce fc06_firmware.uf2 (plus .bin, .hex and the map file) next to the .elf.
pico_add_extra_outputs(fc06_firmware)
