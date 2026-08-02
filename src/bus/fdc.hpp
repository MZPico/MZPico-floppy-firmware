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
  // Type II (READ SECTOR) state
  uint8_t rd_flags;  // b0 type2 b1 reading b2 drq b3 multi b4 rnf b5 crc b6 deleted
  uint8_t rd_sector; // sector currently being fetched
  uint16_t xfer_pos; // bytes handed to the host in this sector (up to 1024)
  uint16_t xfer_len; // the current sector's data length (up to 1024)
  uint8_t rd_seq;    // low byte of read request counter
  uint8_t rd_done;   // low byte of read completion counter
  uint32_t rd_elapsed_us; // time from READ command to Force Interrupt abort;
                          // 0 if no read has been aborted yet this session
  uint32_t cmd_gap_us; // time from the last Type I seek's BUSY-clear to the
                       // most recent command being issued (0 if no seek yet)
  uint8_t verify_start_ok; // did the last Verify's capture start successfully?
  uint32_t verify_n;       // flux bytes drained by the last Verify attempt
  uint16_t cmd_writes_total;
  uint8_t cmd_last_raw;     // last raw byte written to D8
  uint8_t cmd_last_decoded; // last decoded command (~raw)
  // READ-path diagnostics: why the last Type II read succeeded or RNF'd.
  uint8_t rd_capture_ok;    // the capture window started
  uint32_t rd_flux_n;       // flux bytes captured
  uint32_t rd_capture_us;   // time spent capturing
  uint32_t rd_search_us;    // time spent decoding
  uint8_t rd_ids_seen;      // ID fields decoded (only filled in on failure)
  uint8_t rd_ids[16];       // sector numbers actually present on the track
  uint8_t rd_want;          // sector the host asked for
  uint8_t rd_result;        // b0 found b1 has_data b2 crc_ok b3 deleted
  // The host's measured BUSY/DRQ patience after a READ command.
  uint32_t status_polls;    // STATUS reads since the READ was issued
  uint32_t status_span_us;  // READ command -> most recent STATUS read
  // Bus test on port 0xDF: write incrementing bytes, read transformed echo.
  uint8_t bt_active;
  uint8_t bt_last_rx;
  uint8_t bt_last_echo;
  uint16_t bt_rx_count;
  uint16_t bt_rd_count;
  uint16_t bt_err_count;
};
void fdc_debug_snapshot(FdcDebug *out);

// Detailed port-0xDF loopback diagnostics. `err_gap` vs `err_garbage` is the
// key split: a gap means whole I/O cycles were lost (the bytes we did capture
// were correct), garbage means the bytes themselves came back wrong. Combined
// with BusDiag's late-cycle counters that pins the failure to a specific stage.
struct FdcBusTest {
  uint8_t active;
  uint8_t last_rx;
  uint8_t last_echo;
  uint32_t rx_count;
  uint32_t rd_count;
  uint32_t err_count;
  uint32_t err_gap;     // got == expect + 1..8 -> cycle(s) dropped
  uint32_t err_garbage; // anything else -> captured byte corrupt
  uint8_t bits_want1;   // OR of bits that should have been 1 but read 0
  uint8_t bits_want0;   // OR of bits that should have been 0 but read 1
  uint8_t mm_exp[8];    // last 8 mismatches, oldest -> newest
  uint8_t mm_got[8];
  uint8_t mm_n; // valid entries in mm_exp/mm_got (0..8)
  // Sequence-independent line health: for an incrementing test sequence every
  // bit must be high exactly 50.00 % of the time. Deviation = that line's fault
  // duty, measured without the last+1 check coupling adjacent writes.
  uint32_t bit_high[8];
  // Mismatches directly following another mismatch. High -> single-cycle
  // glitches (error + recovery pairs). Near zero -> the line sticks for long
  // runs and only trips the check at the 0xFF->0x00 roll.
  uint32_t err_adjacent;
  // READ direction, verified via the test program echoing each byte it read
  // from 0xDF back to 0xDE. 0 reports = the read-check program was not run.
  uint32_t rd_reports;
  uint32_t rd_err;
  uint8_t rd_bits_want1;
  uint8_t rd_bits_want0;
  uint8_t rd_last_exp;
  uint8_t rd_last_got;
  uint32_t rd_bit_high[8]; // per-line duty of the echoed byte (50.00 % = clean)
};
void fdc_bustest_snapshot(FdcBusTest *out);

// Invert side select: map the ROM's side 0 to physical head 1 and vice versa.
// DEFAULT ON -- the MZ-800 floppy card drives side-select inverted relative to
// the PC/Shugart convention, so this is required for the machine to boot at
// all. Exposed only so both mappings can be A/B'd from the probe without a
// reflash. Takes effect on the ROM's next write to port 0xDD.
void fdc_set_side_invert(bool on);
bool fdc_get_side_invert(void);

// Reset bus-test counters/state (port 0xDF loopback test). Also clears the
// bus-level per-port counters and cycle telemetry (bus_diag_reset).
void fdc_bustest_reset(void);

// Bench-only SYNCHRONOUS seek (no MZ-800, no async handshake): select + spin +
// goto_track(target) directly on core0, then fill *out with the result. target 0
// = RESTORE. Returns after the physical move completes, so the USB reply carries
// the real outcome (phys_track / TRK00) — nothing to poll during the blocking
// motor/step, which would otherwise starve USB.
// mode 0: motor_raw + `param` ms fixed delay (no index wait).
// mode 1: spin_motor with motor_delay_ms=`param` (waits for the INDEX pulse).
void fdc_bench_seek(uint8_t target, uint16_t param, uint8_t mode, FdcDebug *out);

// Bench-only SYNCHRONOUS read of one sector (no MZ-800): select+spin, seek to
// `track`, select `side`, capture the track's flux and decode sector `sector`.
// Copies up to `out_size` decoded data bytes into `out`. Returns the number of
// data bytes decoded (0 on RNF / no data). *flags gets b0 found b1 has_data
// b2 data_crc_ok b3 deleted. Validates the whole READ path against a real disk
// over USB before it's wired to the MZ-800 bus.
uint16_t fdc_bench_read_sector(uint8_t track, uint8_t sector, uint8_t side,
                               uint8_t density, uint8_t *out, uint16_t out_size,
                               uint8_t *flags);

// Fill `out_decoded`/`out_raw` (>=16 bytes each) with recent command bytes
// (oldest->newest). Decoded is the WD opcode (~raw), raw is the byte sampled on
// the data bus at D8. Returns count written; *count = total commands seen.
uint8_t fdc_debug_cmdlog(uint8_t *out_decoded, uint8_t *out_raw,
                         uint16_t *count);
