// MZ-800 expansion-bus pin map and low-level access helpers for the
// MZPico-800-Floppy-Board. Adapted from the reference firmware's bus.hpp
// (DELUXE shared-bus path) to this board's GPIOs. See docs/phase3-plan.md.
//
// Shared C0..C7 (GPIO0-7) carry address then data via two SN74LVC245s:
//   ~EN0 (26) = address-buffer OE   (U1, DIR fixed: bus AD -> Pico C)
//   ~EN1 (27) = data-buffer OE      (U2)
//   ~RD1 (28) = data-buffer DIR     (U2: 1 = bus D -> Pico C [read],
//                                        0 = Pico C -> bus D [drive])
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
#define FDC_PORT_BASE 0xD8
#define FDC_PORT_LAST 0xDE

ALWAYS_INLINE uint8_t read_data_bus(void) {
  return (sio_hw->gpio_in >> DATA_BUS_BASE) & 0xFF;
}
ALWAYS_INLINE void write_data_bus(uint8_t value) {
  sio_hw->gpio_clr = DATA_BUS_MASK;
  sio_hw->gpio_set = ((uint32_t)value << DATA_BUS_BASE);
}
ALWAYS_INLINE void acquire_data_bus_for_writing(void) {
  sio_hw->gpio_oe_set = DATA_BUS_MASK;
}
ALWAYS_INLINE void release_data_bus(void) {
  sio_hw->gpio_oe_clr = DATA_BUS_MASK;
}

// Data buffer (U2) gate — driven directly by core1, NOT the PIO, so we only ever
// enable it (and thus touch the MZ-800 data bus) for ports we own. This avoids
// contention with the MZ-800's own I/O devices on non-FDC reads. EN1 low = data
// buffer on; RD1: 1 = bus->Pico (read), 0 = Pico->bus (drive).
ALWAYS_INLINE void data_dir_read(void) { sio_hw->gpio_set = 1u << RD1_PIN; }
ALWAYS_INLINE void data_dir_drive(void) { sio_hw->gpio_clr = 1u << RD1_PIN; }
ALWAYS_INLINE void data_buffer_on(void) { sio_hw->gpio_clr = 1u << EN1_PIN; }
ALWAYS_INLINE void data_buffer_off(void) { sio_hw->gpio_set = 1u << EN1_PIN; }

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
