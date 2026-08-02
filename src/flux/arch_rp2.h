// RP2040 PIO flux capture/write — bare Pico-SDK port of Adafruit_Floppy's
// arch_rp2. Only the two free functions are kept (no Arduino, no MFM, no
// Adafruit_FloppyBase). See src/flux/README-port.md for what changed.
//
// Original: https://github.com/adafruit/Adafruit_Floppy (MIT)
#pragma once
#include <stdbool.h>
#include <stddef.h>
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

// ---- Incremental (streaming) capture ----
// Unlike rp2040_flux_capture (a single blocking batch call that captures for a
// fixed count/duration then tears everything down), this lets a caller poll
// "whatever flux has arrived so far" repeatedly while the capture keeps
// running continuously underneath -- no dead-time gap between polls, so nothing
// is lost at the boundaries between calls. Meant for time-critical live reads
// where the caller wants to start decoding as soon as data is available rather
// than waiting for a whole revolution to be captured first.
//
// Usage: start() once, then drain() repeatedly (interleaved with attempts to
// decode the buffer so far) until done, then stop().
bool rp2040_flux_capture_start(int index_pin, int rdpin);

// Append whatever flux deltas are currently sitting in the capture FIFO onto
// `dest`, starting at `dest[already_written]`, up to `dest_cap` bytes total.
// Returns the new total byte count written (>= already_written). Non-blocking:
// returns immediately once the FIFO is drained, however little that was.
size_t rp2040_flux_capture_drain(volatile uint8_t *dest, size_t dest_cap,
                                 size_t already_written,
                                 bool store_greaseweazle);

// Stop sampling and release the PIO state machine/program claimed by start().
void rp2040_flux_capture_stop(void);

#ifdef __cplusplus
}
#endif
