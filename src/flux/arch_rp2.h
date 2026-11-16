// Derived from Adafruit_Floppy (https://github.com/adafruit/Adafruit_Floppy),
// MIT License, Copyright (c) 2021 Limor Fried for Adafruit Industries.
// See LICENSES/MIT-Adafruit_Floppy.txt. Changes: Apache-2.0, see LICENSE.
// RP2040 PIO flux capture/write — bare Pico-SDK port of Adafruit_Floppy's
// arch_rp2. Only the two free functions are kept (no Arduino, no MFM, no
// Adafruit_FloppyBase); since extended with a DMA-fed capture ring and the
// streaming/splice entry points declared below.
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

// ---- Live (real-time) capture, for the WRITE SECTOR splice ----
// The drain() API above is for "capture now, decode later". Locating a sector
// in order to overwrite it cannot work that way: by the time a buffer has been
// decoded the sector has long since passed the head. These three let a caller
// consume flux transition-by-transition AS THEY ARRIVE, decode the ID field on
// the fly, and hit the Gap 2 window that follows it.

// Next flux delta (in 24 MHz sample ticks, NOT clamped to 8 bits like the
// buffered path), or -1 if none arrived within timeout_us. Requires a capture
// started with rp2040_flux_capture_start(). Cheap enough to call at the ~350
// kHz DD transition rate.
int rp2040_flux_capture_next(uint32_t timeout_us);

// True if, since the capture started, the DMA lapped the consumer -- a whole
// ring behind, i.e. flux was lost and the consumer was resynced half a ring
// back from the write position (a hole in the stream). With the 16 KB ring
// that is tens of milliseconds of neglect, not the FIFO's 12-24 us. The FDC's
// read capture checks it after every drain and starts over (issue #5).
bool rp2040_flux_capture_overrun(void);


// Drain and sum whatever deltas are already queued in the ring, in 24 MHz
// ticks. That backlog is disk time that has ALREADY passed under the head, so
// a splice that measures its gap with a wall clock has to subtract it.
uint32_t rp2040_flux_capture_backlog(void);

// Stop sampling but keep the state machine/program claimed (unlike _stop()).
// Used to silence the reader for the duration of a write without paying for a
// PIO program reload afterwards.
void rp2040_flux_capture_disable(void);

// ---- Pre-armed write, for the WRITE SECTOR splice ----
// rp2040_flux_write() claims a PIO state machine, writes, and releases it. A
// splice cannot afford the claim/load inside its ~700 us window, so prepare()
// does it up front and write_now() starts the flux immediately -- no index
// wait, no allocation.
bool rp2040_flux_write_prepare(int wrdata_pin);
bool rp2040_flux_write_now(int wrgate_pin, const uint8_t *pulses, size_t n);
void rp2040_flux_write_release(void);

#ifdef __cplusplus
}
#endif
