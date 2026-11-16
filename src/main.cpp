// MZPico Floppy firmware -- WD179x FDC emulation for the Sharp MZ-800, backed
// in real time by a physical floppy drive on the MZPico-800-Floppy-Board. core1
// services the MZ-800 bus (bus/mz_bus, bus/fdc register side); core0 runs the
// disk side (fdc_disk_task) plus the USB link: a greaseweazle-compatible flux
// interface and the mzp_probe.py bench commands (gw_protocol). Bare Pico-SDK +
// TinyUSB.
#include "board_config.h"
#include "gw_protocol.hpp"
#include "bus/mz_bus.hpp"
#include "bus/fdc.hpp"

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "tusb.h"

int main() {
  // Before anything else: the '245 gates default to "both buffers on" until
  // driven (issue #10).
  bus_park_gates();

  // Overclock for reliable flux timing (Adafruit recommends ~200MHz on RP2040).
  vreg_set_voltage(VREG_VOLTAGE_1_15);
  sleep_ms(2);
  set_sys_clock_khz(200000, true);

  tusb_init();
  gw_init();
  bus_init(); // launch the MZ-800 bus listener on core1

  while (true) {
    // Prioritize disk-side FDC work first: the host is polling STATUS against
    // its own timeouts (the boot ROM's READ patience was measured at >= 285 ms,
    // CP/M's READ ADDRESS ~52 ms), so core0 scheduling latency here matters
    // more than USB/probe responsiveness.
    fdc_disk_task();
    tud_task();       // service USB (core0)
    gw_task();        // process one greaseweazle / MZP command if pending
    fdc_disk_task();  // run a second pass after USB/protocol servicing
  }
}
