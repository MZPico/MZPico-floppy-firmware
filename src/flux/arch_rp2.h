// RP2040 PIO flux capture/write — bare Pico-SDK port of Adafruit_Floppy's
// arch_rp2. Only the two free functions are kept (no Arduino, no MFM, no
// Adafruit_FloppyBase). See src/flux/README-port.md for what changed.
//
// Original: https://github.com/adafruit/Adafruit_Floppy (MIT)
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Capture flux into `pulses`, greaseweazle-packed if store_greaseweazle.
// Returns number of bytes written. Waits up to index_wait_ms for a falling
// index edge before capturing (0 = capture immediately).
uint32_t rp2040_flux_capture(int index_pin, int rdpin, volatile uint8_t *pulses,
                             volatile uint8_t *pulse_end,
                             int32_t *falling_index_offset,
                             bool store_greaseweazle, uint32_t capture_counts,
                             uint32_t index_wait_ms);

// Write flux from `pulses` to the drive, gating WGATE around the write.
bool rp2040_flux_write(int index_pin, int wrgate_pin, int wrdata_pin,
                       uint8_t *pulses, uint8_t *pulse_end,
                       bool store_greaseweazle, bool is_apple2, bool use_index);

#ifdef __cplusplus
}
#endif
