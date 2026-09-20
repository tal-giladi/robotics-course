# FC.08 - build karmel_wheels.c against micro-ROS's precompiled Pico library.
#
# This file REPLACES the CMakeLists.txt of
# https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk (branch jazzy). Copy it and
# karmel_wheels.c into a clone of that repository; `libmicroros/` and the transport
# (pico_uart_transport.c) come from there.
#
# Differences from micro-ROS's own file: our source and target name, hardware_pwm for the
# LED "motor", and -DPICO_BOARD is expected on the command line (pico2 for karmel's Pico 2).

cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)   # BEFORE project(), see FC.06

project(karmel_wheels C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(libmicroros)

add_executable(karmel_wheels
    karmel_wheels.c
    pico_uart_transport.c          # micro-ROS's custom transport, from their repository
)

target_link_libraries(karmel_wheels
    pico_stdlib
    hardware_pwm
    microros                       # libmicroros/libmicroros.a
)

target_include_directories(karmel_wheels PUBLIC
    libmicroros/include
)

# -ffunction-sections/-fdata-sections + --gc-sections: only the ROS code you actually call
# is linked in. Without them the firmware is tens of kilobytes larger.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# micro-ROS speaks a binary protocol over this port: no CR/LF translation, ever.
pico_enable_stdio_usb(karmel_wheels 1)
pico_enable_stdio_uart(karmel_wheels 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

pico_add_extra_outputs(karmel_wheels)
