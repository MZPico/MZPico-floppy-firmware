// MZ-800 expansion-bus pin map and low-level access helpers for the
// MZPico-800-Floppy-Board. Adapted from the reference firmware's bus.hpp
// (DELUXE shared-bus path) to this board's GPIOs. See docs/phase3-plan.md.
//
// Shared C0..C7 (GPIO0-7) carry address then data via two SN74LVC245s:
//   ~EN0 (26) = address-buffer OE   (U1, DIR fixed: bus AD -> Pico C)
//   ~EN1 (27) = data-buffer OE      (U2)
//   ~RD1 (28) = data-buffer DIR     (U2: 1 = bus D -> Pico C [read],
//                                        0 = Pico C -> bus D [drive])
// All three gates are a 3-bit PIO side-set (base EN0_PIN); see bus.pio.
#pragma once
#include "hardware/structs/sio.h"
#include <cstdint>

#define ALWAYS_INLINE inline __attribute__((always_inline))
#define RAM_FUNC __attribute__((section(".data.ram_func"))) __attribute__((noinline))

// Shared 8-bit bus C0..C7
#define DATA_BUS_BASE 0
#define DATA_BUS_COUNT 8
#define DATA_BUS_MASK (0xFFu << DATA_BUS_BASE)

// Control lines (active low)
#define IORQ_PIN 8
#define RD_PIN 9
#define WR_PIN 10
#define EXWAIT_PIN 11
#define INT_PIN 12
#define RESET_PIN 29

// Buffer gate pins (side-set by the bus PIO; base = EN0_PIN, 3 consecutive)
#define EN0_PIN 26 // address buffer OE
#define EN1_PIN 27 // data buffer OE
#define RD1_PIN 28 // data buffer direction

// FDC register window this firmware answers.
// The window is 8 ports wide: 0xD8..0xDB are the WD registers, 0xDC..0xDE the
// card's control latches (motor/drive, page/side, density) and 0xDF is EINT,
// the interrupt-enable latch. 0xDF is a real register, not padding -- CP/M 4.1
// writes it before and after every data command.
#define FDC_PORT_BASE 0xD8
#define FDC_PORT_LAST 0xDF

// Buffer-settle delay between switching a '245 gate and sampling/driving C0..C7.
// An LVC245 propagates in ~5 ns; the RP2040 GPIO input synchroniser costs 2
// sysclk. Six nops @180 MHz is ~33 ns -- ample, and ~5x cheaper than the
// `for (volatile int k...)` loop it replaces (which burned ~150 ns of the ~560 ns
// Z80 I/O window). See docs/phase3-plan.md "Board-specific bus handshake".
ALWAYS_INLINE void bus_settle(void) {
  __asm volatile("nop; nop; nop; nop; nop; nop" ::: "memory");
}

ALWAYS_INLINE uint8_t read_data_bus(void) {
  return (sio_hw->gpio_in >> DATA_BUS_BASE) & 0xFF;
}
ALWAYS_INLINE void write_data_bus(uint8_t value) {
  sio_hw->gpio_clr = DATA_BUS_MASK;
  sio_hw->gpio_set = ((uint32_t)value << DATA_BUS_BASE);
}
// ~INTRQ (GPIO12), active low, driven open-drain: pull low to assert, go Hi-Z to
// release, so we never fight another device sharing the MZ-800's INT line. The
// output latch is parked at 0 during init, so toggling the output-enable is all
// that is needed. gpio_oe_set/clr are write-1-to-modify, hence safe to call from
// either core without a read-modify-write race.
ALWAYS_INLINE void assert_int_line(void) {
  sio_hw->gpio_oe_set = (1u << INT_PIN);
}
ALWAYS_INLINE void release_int_line(void) {
  sio_hw->gpio_oe_clr = (1u << INT_PIN);
}

ALWAYS_INLINE void acquire_data_bus_for_writing(void) {
  sio_hw->gpio_oe_set = DATA_BUS_MASK;
}
ALWAYS_INLINE void release_data_bus(void) {
  sio_hw->gpio_oe_clr = DATA_BUS_MASK;
}

// Buffer gates (U1 address, U2 data + direction) are ALL owned by the bus PIO
// side-set (see bus.pio): the write SM captures the data byte itself, and the
// read SM turns U2 toward the bus only after core1 queues a go-word for a port
// we own. core1 never touches EN0/EN1/RD1 in normal operation; only the bench
// self-tests take them back as SIO (bus_read_pins / bus_drive_test).

// EXWAIT / INT are driven low (open-drain style: assert = drive low, release =
// float back to the bus pull-up).
ALWAYS_INLINE void set_exwait(void) {
  sio_hw->gpio_clr = 1u << EXWAIT_PIN;
  sio_hw->gpio_oe_set = 1u << EXWAIT_PIN;
}
ALWAYS_INLINE void release_exwait(void) {
  sio_hw->gpio_oe_clr = 1u << EXWAIT_PIN;
}
ALWAYS_INLINE void set_interrupt(void) {
  sio_hw->gpio_clr = 1u << INT_PIN;
  sio_hw->gpio_oe_set = 1u << INT_PIN;
}
ALWAYS_INLINE void release_interrupt(void) {
  sio_hw->gpio_oe_clr = 1u << INT_PIN;
}
ALWAYS_INLINE bool iorq_is_high(void) {
  return sio_hw->gpio_in & (1u << IORQ_PIN);
}
