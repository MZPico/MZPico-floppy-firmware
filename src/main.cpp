// MZPico Floppy firmware — Phase 1: greaseweazle-compatible flux reader/writer
// on bare Pico-SDK + TinyUSB. Drives the physical floppy on the
// MZPico-800-Floppy-Board; the MZ-800 bus/FDC emulation comes in a later phase.
#include "board_config.h"
#include "gw_protocol.hpp"

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "tusb.h"

int main() {
  // Overclock for reliable flux timing (Adafruit recommends ~200MHz on RP2040).
  vreg_set_voltage(VREG_VOLTAGE_1_15);
  sleep_ms(2);
  set_sys_clock_khz(200000, true);

  tusb_init();
  gw_init();

  while (true) {
    tud_task();  // service USB
    gw_task();   // process one greaseweazle command if pending
  }
}
