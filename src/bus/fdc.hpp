// Milestone 3.1 — WD179x/FD1791 Type I + STATUS bridge to a real drive.
//
// Split across the two cores (see docs/wd-fidelity.md):
//   core1 (bus)  : fdc_reg_read/fdc_reg_write — decode commands, serve the WD
//                  register file (STATUS/TRACK/SECTOR/DATA) with the MZ-800 ROM's
//                  per-port data-bus inversion. Fast, no blocking, no flash.
//   core0 (disk) : fdc_disk_task — apply motor/select/side and run the physical
//                  seek (STEP/DIR/TRK00) in the background while the CPU polls
//                  BUSY. This is the async model: long physical latency never
//                  stalls the Z80 (~EXWAIT covers only the µs register access).
//
// Type I only here (RESTORE/SEEK/STEP/STEP-IN/STEP-OUT) + Force Interrupt. The
// ROM paces Type I by polling STATUS (waits `status & 0x81 == 0` = ready & not
// busy), so no INTRQ is needed yet. Type II/III (sector data) arrive in 3.2.
#pragma once
#include <cstdint>

class FloppyDrive;

// core0: hand the FDC the drive to actuate. Call once before the MZ talks to us.
void fdc_init(FloppyDrive *drive);

// core1 (bus, hot path): register access. `addr` in 0xD8..0xDE. On write, `raw`
// is the byte seen on the bus. fdc_reg_read returns the exact byte to drive back
// onto the bus (inversion already applied for 0xD8..0xDB).
void fdc_reg_write(uint8_t addr, uint8_t raw);
uint8_t fdc_reg_read(uint8_t addr);

// core0: run any pending physical operation (motor/select/side/seek). Poll from
// the main loop. No-op until the MZ-800 first accesses an FDC port, so it never
// fights the greaseweazle/USB drive control when no machine is attached.
void fdc_disk_task(void);

// Debug snapshot of the FDC state (for the USB probe). Call on core0.
struct FdcDebug {
  uint8_t status;  // composed Type I STATUS (true, un-inverted)
  uint8_t track;   // Track register
  uint8_t sector;  // Sector register
  uint8_t data;    // Data register
  uint8_t command; // last command opcode (true)
  uint8_t flags;   // b7 motor b2 side b1 wp b0 busy
  uint8_t drive;   // selected drive
  uint8_t at_track0;
  uint8_t phys_track; // physical head position (0xFF = unknown)
  uint8_t op_seq;     // low byte of seek request counter
  uint8_t op_done;    // low byte of seek completion counter
};
void fdc_debug_snapshot(FdcDebug *out);

// Bench-only SYNCHRONOUS seek (no MZ-800, no async handshake): select + spin +
// goto_track(target) directly on core0, then fill *out with the result. target 0
// = RESTORE. Returns after the physical move completes, so the USB reply carries
// the real outcome (phys_track / TRK00) — nothing to poll during the blocking
// motor/step, which would otherwise starve USB.
// mode 0: motor_raw + `param` ms fixed delay (no index wait).
// mode 1: spin_motor with motor_delay_ms=`param` (waits for the INDEX pulse).
void fdc_bench_seek(uint8_t target, uint16_t param, uint8_t mode, FdcDebug *out);

// Fill `out` (>=16 bytes) with the recent command opcodes (oldest->newest),
// return the count written; *count = total commands ever seen.
uint8_t fdc_debug_cmdlog(uint8_t *out, uint16_t *count);
