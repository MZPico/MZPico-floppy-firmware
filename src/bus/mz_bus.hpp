// Milestone 3.0 "hello MZ-800" bus interface: answer I/O reads/writes on the FDC
// ports (0xD8-0xDE) so we can prove the shared-bus handshake before building the
// real WD FDC. Runs a tight listen loop on core1; core0 keeps the USB link for
// observing the shadow registers. See docs/phase3-plan.md.
#pragma once
#include <cstdint>

// Shadow of the 8 FDC-window registers (0xD8..0xDF), plus access counters, for
// the hello test. Values are stored as the *true* (un-inverted) byte; the bus
// layer handles the ROM's data-bus inversion. Shared core1 -> core0 (USB dump).
struct BusHelloState {
  volatile uint8_t reg[8];      // stored inverted value (what a read drives back)
  volatile uint8_t last_raw[8]; // last RAW (un-inverted) byte the MZ wrote, per port
  volatile uint32_t wr[8];      // per-port write count (32-bit: a uint16 wraps
  volatile uint32_t rd[8];      // in 0.5 s under a tight OUT loop)
  // Bit-activity accumulators over ALL I/O cycles the machine runs (not just our
  // ports). A bit set in *_low means the PIO sampled that C-line LOW at least
  // once; set in *_high means it sampled HIGH at least once. A healthy line
  // toggles -> appears in BOTH. A stuck line appears in only one. This tells us
  // whether GPIO0/C0 is ever seen low, independent of our port logic.
  volatile uint8_t addr_low;  // address-phase bits ever sampled 0
  volatile uint8_t addr_high; // address-phase bits ever sampled 1
  volatile uint8_t dat_low;   // write-data bits ever sampled 0
  volatile uint8_t dat_high;  // write-data bits ever sampled 1
};

extern BusHelloState g_bus_hello;

// Cycle-timing telemetry for the core1 listen loop. The Z80 I/O window is only
// ~560 ns; these counters say whether core1 is making it or not, so a data
// mismatch can be attributed to "arrived too late" rather than guessed at.
struct BusDiag {
  volatile uint32_t wr_late_at_pop;    // WRITE: IORQ already high when core1 got
                                       // the address -> capture skipped entirely
  volatile uint32_t wr_late_at_sample; // WRITE: IORQ rose while we were sampling
                                       // -> the byte we read may be corrupt
  volatile uint32_t rd_late_at_pop;    // READ: cycle over before we could drive
  volatile uint32_t wr_backlog;        // WRITE: >1 entry drained in one pass
  volatile uint32_t rd_backlog;        // READ: >1 entry drained in one pass
};

extern BusDiag g_bus_diag;

// Zero the per-port counters, bit-activity accumulators and cycle telemetry.
// (busdump counters are cumulative from boot otherwise, which makes a clean
// single-experiment measurement impossible.)
void bus_diag_reset(void);

// Init bus GPIO + PIO and launch the core1 listen loop. Call once from main().
void bus_init(void);

// Bench pin-READ test: return the live C0..C7 input byte (bit0 = GPIO0 = C0).
// If `isolate` is true, halt the bus SMs and set C0..C7 as plain SIO inputs
// (no pull) first, so nothing the PIO/core1 does can drive the lines -- then an
// external jumper on C0 (to GND / 3.3V) tells us directly whether the Pico
// GPIO0 pad + C0->pin-1 joint can read a real 0 and 1. Call bus_drive_test(_,0)
// to restore normal operation afterwards.
uint8_t bus_read_pins(bool isolate);

// Bench pin self-test: with both '245s forced OFF and the bus SMs halted, drive
// each C0..C7 pad with its OWN internal pull-up, read; then internal pull-down,
// read. No external jumper / bus needed. A healthy pad + un-driven net reads 1
// under pull-up and 0 under pull-down. *pu = byte read under pull-up (want
// 0xFF), *pd = byte read under pull-down (want 0x00). A bit stuck in both -> that
// pad is dead or its net is shorted to a rail (bit0 = GPIO0 = C0).
void bus_pin_selftest(uint8_t *pu, uint8_t *pd);

// Bench pin-drive test (hardware isolation for the C0..C7 lines).
//   mode 0: release C0..C7 back to inputs, data buffer off
//   mode 1: drive C0..C7 = value on the *Pico pins only* (data '245 OFF) -- safe
//           even in the MZ-800; measure each Pico C-pin with a meter
//   mode 2: also enable the data '245 toward the bus -- BENCH ONLY (board out of
//           the MZ-800), measures the connector D-pins
// NB: only meaningful with the board out of the MZ-800 (no bus cycles firing).
void bus_drive_test(uint8_t value, uint8_t mode);
