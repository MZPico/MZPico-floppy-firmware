cmake_minimum_required(VERSION 3.13)

# Prefer the in-tree SDK submodule (external/pico-sdk) unless the caller has
# already pointed PICO_SDK_PATH elsewhere (env or -D). Mirrors the reference
# firmware's layout so the repo is self-contained and buildable after
# `git submodule update --init`.
if(NOT DEFINED ENV{PICO_SDK_PATH} AND NOT PICO_SDK_PATH
   AND EXISTS ${CMAKE_CURRENT_LIST_DIR}/external/pico-sdk/pico_sdk_init.cmake)
    set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/external/pico-sdk)
endif()

# Pull in the Raspberry Pi Pico SDK (uses PICO_SDK_PATH from above/env/-D).
include(pico_sdk_import.cmake)

project(mzpico_floppy C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

add_executable(mzpico_floppy
    src/main.cpp
    src/gw_protocol.cpp
    src/floppy.cpp
    src/mfm_decode.cpp
    src/mfm_encode.cpp
    src/bus/mz_bus.cpp
    src/flux/arch_rp2.cpp
    src/usb_descriptors.c
)

# Generate the MZ-800 bus PIO header (bus.pio.h) and add it to the include path.
pico_generate_pio_header(mzpico_floppy ${CMAKE_CURRENT_LIST_DIR}/src/bus/bus.pio)

# src/ is on the include path so TinyUSB finds tusb_config.h.
target_include_directories(mzpico_floppy PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/src/flux
)

# -O3 for reliable PIO flux timing (per Adafruit_Floppy guidance).
target_compile_options(mzpico_floppy PRIVATE -O3)

target_link_libraries(mzpico_floppy
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_clocks
    hardware_vreg
    tinyusb_device
    tinyusb_board
)

# We speak the raw greaseweazle CDC protocol ourselves — no stdio on USB/UART.
pico_enable_stdio_usb(mzpico_floppy 0)
pico_enable_stdio_uart(mzpico_floppy 0)

# Produce mzpico_floppy.uf2 (+ .bin/.hex/.elf) in the build dir.
pico_add_extra_outputs(mzpico_floppy)
