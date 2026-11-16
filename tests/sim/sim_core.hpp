// Deterministic host simulation of the two RP2040 cores and the board's pins.
//
// The firmware's two halves run as they do on hardware, but interleaved on one
// host thread against a VIRTUAL clock:
//
//   core0  -- the disk side (fdc_disk_task, FloppyDrive, flux capture/write).
//             Runs as a coroutine. It never sees real time: every HAL call it
//             makes (time_us_32, gpio_get, a flux drain...) costs a few virtual
//             ticks, and once a microsecond of cost has accrued it yields so the
//             other side can run. sleep_ms()/busy_wait jump straight ahead.
//   host   -- whatever drives the FDC registers: a scripted C++ test (its own
//             coroutine) or the emulated Z80 of an MZ-800. Register accesses
//             call fdc_reg_read/fdc_reg_write exactly as core1's listen loop
//             does, so the host side IS core1 for the purposes of the test.
//
// Because the interleave is driven only by virtual time, every run is exactly
// reproducible: a failure seen once is seen every time, and a test can poke
// the disk at an exact rotational phase.
//
// Granularity: core0 yields at least every microsecond, so a cross-core
// interaction is modelled to ~1 us, not to the instruction. Races narrower
// than that are out of reach of this harness (the review issues that depend on
// them are still worth a targeted test that forces the ordering).
#pragma once
#include <cstdint>
#include <functional>

namespace sim {

constexpr uint32_t TICKS_PER_US = 24; // the flux sample rate: 24 MHz
constexpr uint64_t us(uint64_t u) { return u * TICKS_PER_US; }
constexpr uint64_t ms(uint64_t m) { return m * 1000 * TICKS_PER_US; }

// Current virtual time, in 24 MHz ticks, as seen by the calling context.
uint64_t now();
bool in_core0();

// ---- core0 side (called from HAL shims) ----
void core0_cost(uint32_t ticks);  // charge CPU time; may yield
void core0_sleep_until(uint64_t t);

// ---- host side ----
// A host is resumed when its wake time comes round and returns its next wake
// time (UINT64_MAX = finished).
struct Host {
  virtual ~Host() {}
  virtual uint64_t run(uint64_t now) = 0;
};

// Script host: runs `fn` as a coroutine; inside it, use wait_ticks()/wait_us()
// to let virtual time pass.
void wait_ticks(uint64_t t);
inline void wait_us(uint64_t u) { wait_ticks(us(u)); }
inline void wait_ms(uint64_t m) { wait_ticks(ms(m)); }

// Run until `fn` returns (script host), with a hard virtual-time limit. Core0
// runs `core0_main` (fdc boot + disk-task loop, see board.hpp) underneath.
// Returns false if the limit was hit or core0 wedged.
bool run_script(const std::function<void()> &fn, uint64_t limit_ticks);
// Run with an arbitrary host (e.g. the Z80 machine) until done() or limit.
bool run_host(Host *host, const std::function<bool()> &done,
              uint64_t limit_ticks);

void set_core0_main(void (*fn)());
// True if core0 has spent `ticks` of virtual time without returning from a
// HAL wait it entered (used to report a core0 hang, e.g. issue #7).
const char *core0_hang_reason();
void core0_report_hang(const char *why);

// ---- pins (the RP2040's GPIO bank as the firmware sees it) ----
struct PinBoard {
  virtual ~PinBoard() {}
  virtual bool input(int pin, uint64_t now) = 0;          // level on an input
  virtual void output(int pin, bool level, uint64_t now) = 0; // a driven change
};
void set_pin_board(PinBoard *b);
bool pin_level(int pin);      // last driven output level
bool pin_is_output(int pin);
bool pin_oe(int pin);         // SIO output enable (INT/EXWAIT are open-drain)
void pin_set_input(int pin, bool level); // force an input (RESET line tests)

} // namespace sim
