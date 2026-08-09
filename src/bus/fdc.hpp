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
// Type I (RESTORE/SEEK/STEP/STEP-IN/STEP-OUT), Type II (READ SECTOR), Type III
// (READ ADDRESS) and Type IV (Force Interrupt).
//
// The boot ROM and CP/M 2.3 pace everything by polling STATUS (waiting for
// `status & 0x81 == 0` = ready & not busy), which is why this worked for a long
// time with ~INT left undriven. It is not enough in general: CP/M 4.1 waits on
// the interrupt and freezes without it, and even the boot ROM's Force Interrupt
// is 0xD8 -- bit3 (I3) set, i.e. "interrupt immediately". INTRQ is now driven;
// see fdc_set_int_mode() for the wiring question it leaves open.
#pragma once
#include <cstddef>
#include <cstdint>

class FloppyDrive;

// core0: hand the FDC the drives it may actuate, in unit order (element 0 =
// the drive on ~DS0). Call once before the MZ talks to us. Which one is live at
// any moment follows port 0xDC's drive field; units past `n` report NOT READY.
void fdc_init(FloppyDrive *const *drives, int n);

// Pin every access to drive 0 regardless of what port 0xDC selects. Escape
// hatch for the case where a host writes a nonzero drive field we did not
// expect; see g_drive_lock in fdc.cpp. Default OFF (multi-drive active).
void fdc_set_drive_lock(bool on);
bool fdc_get_drive_lock(void);
// core1/USB: force the selected unit, for bench use (scan/wrverify/fmtverify on
// drive 1 without an MZ-800 attached). Same field the host writes.
void fdc_set_drive(uint8_t unit);
uint8_t fdc_get_drive(void);

// USB bench commands that drive the floppy themselves (scan/streamscan) rather
// than going through the FDC command path: resolve the selected unit and return
// it, or null if the host selected a unit this board has no select line for.
// Without this they act on drive 0 whatever `drive 1` said -- which does not
// merely mislead, it makes fmtverify format one disk and verify another.
// Release it with fdc_bench_release() so the FDC re-selects from scratch.
FloppyDrive *fdc_bench_drive(void);
void fdc_bench_release(void);

// Per-drive state for ALL units at once, selected or not. fdcdump only ever
// shows the drive that happens to be selected, which makes the property this
// whole subsystem rests on -- that two drives keep their head positions apart
// across selects -- invisible from outside. Returns false for a unit index this
// board has no select line for. `track` is the drive's own position belief
// (-1 = unknown), `target` the head target core1 folds Type I steps into.
// flags: b0 ready (index seen on the last spin-up), b1 ever spun up,
//        b2 currently selected.
bool fdc_drive_state(int unit, int *track, int *target, uint8_t *flags);

// core1 (bus, hot path): register access. `addr` in 0xD8..0xDE. On write, `raw`
// is the byte seen on the bus. fdc_reg_read returns the exact byte to drive back
// onto the bus (inversion already applied for 0xD8..0xDB).
void fdc_reg_write(uint8_t addr, uint8_t raw);
uint8_t fdc_reg_read(uint8_t addr);

// core1 (bus): arm the MZ-800 bus RESET interrupt (GPIO29, active HIGH, idles
// low). Call ONCE from core1's entry point, before the listen loop -- arming it
// there is what routes the interrupt to core1 rather than core0.
//
// An interrupt rather than a poll, because polling this from the listen loop
// measurably broke the machine: that loop has ~560 ns to answer a Z80 I/O
// cycle, and even a couple of instructions per pass made core1 late enough that
// the MZ-800 reported "no floppy driver detected". core1 rather than core0,
// because core0 blocks for ~250 ms at a time inside a flux capture with
// interrupts disabled. See fdc_bus_reset_isr() for the full reasoning.
void fdc_bus_reset_irq_init(void);

// Runtime enable for the above. Kept even though the interrupt costs nothing
// until the line moves: this line's handling has already broken the machine
// once, so it must be possible to neutralise it without a reflash.
void fdc_set_reset_poll(bool on);
bool fdc_get_reset_poll(void);

// Host-initiated WRITE TRACK (formatting). DEFAULT OFF -- a raw 0x00 on the
// bus-inverted command port decodes to 0xFF = WRITE TRACK, so one glitched byte
// could otherwise reformat a track. The bench format path is unaffected.
void fdc_set_format_enable(bool on);
bool fdc_get_format_enable(void);

// Head-position self-correction (goto_track's TRK00 cross-check and the
// adopt-the-disk's-cylinder resync). On by default; off restores the seek
// behaviour from before those were added.
void fdc_set_posfix(bool on);
bool fdc_get_posfix(void);

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
  uint32_t verify_attempts; // Verify passes run this session (0 = none, so
                            // verify_start_ok/verify_n are meaningless)
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
  // ~INT state. Without these an interrupt-driven host that hangs is a dead
  // end: you cannot tell "EINT never armed" from "armed but we never asserted"
  // from "asserted and the host never came back for the byte".
  uint8_t int_flags;        // b0 eint b1 line asserted b2 force-int latched
  uint8_t int_mode;         // the active fdc_set_int_mode() setting
  uint32_t intrq_asserts;   // ~INT Hi-Z -> low transitions this session
  uint32_t lost_data_events; // DRQ'd sectors the host never drained
  // ---- WRITE SECTOR (3.3) ----
  uint32_t write_attempts;   // WRITE SECTOR commands (host or bench probe)
  uint32_t wr_refused;       // ...refused before the disk (WP / writes off)
  uint32_t wr_ok;            // data fields actually spliced onto the media
  uint32_t wr_rnf;           // the sector's ID never passed the head
  uint32_t wr_abandoned;     // write ended before its buffer filled
  uint16_t wr_ab_want;       // bytes expected by the last abandoned write
  uint16_t wr_ab_got;        // ...and how many the host actually sent
  uint8_t wr_ab_track;
  uint8_t wr_ab_side;
  uint8_t wr_ab_sector;
  uint32_t wr_cyl_mismatch;  // found it, but on the wrong cylinder
  uint32_t wr_size_mismatch; // found it, but its size != the host's data
  uint32_t wr_late;          // ID decoded too late to hit its Gap 2 window
  uint32_t wr_splice_us;     // last splice: search time
  uint16_t wr_ids_seen;      // last splice: ID fields decoded
  uint8_t wr_last_result;    // b0 wrote b1 id_found b2 cyl b3 size b4 late
                             // b5 no_flux
  uint8_t wr_flags;          // b0 writing b1 armed b2 writes enabled b3 ~WPT
  // Cumulative read quality (see the sec_* counters in fdc.cpp). A snapshot of
  // the *last* read cannot show a corrupt sector the host already consumed.
  uint32_t sec_ok;           // sectors delivered with a good data CRC
  uint32_t sec_crc_err;      // sectors delivered with a BAD data CRC
  uint32_t sec_rnf;          // sector reads that found no matching ID
  uint8_t last_id_cyl;       // ID field of the last delivered sector (0xFF=none)
  uint8_t last_id_head;
  uint32_t sec_cyl_mismatch; // sectors whose ID cylinder != the Track register
  uint8_t last_trk_reg;      // Track register when last_id_cyl was sampled
  uint8_t last_head_target;  // our model of the head position at that read
  uint8_t last_phys_track;   // FloppyDrive's _track belief (0xFE = unknown)
  uint8_t last_want_side;    // physical head selected at that moment
  // Bus test on port 0xDF: write incrementing bytes, read transformed echo.
  uint8_t bt_active;
  uint8_t bt_last_rx;
  uint8_t bt_last_echo;
  uint16_t bt_rx_count;
  uint16_t bt_rd_count;
  uint16_t bt_err_count;
};
void fdc_debug_snapshot(FdcDebug *out);

// First-failure latch. Every other diagnostic describes the LAST operation,
// which is the wrong one as soon as a host retries: the successful retry
// overwrites the evidence of the failure that caused the visible symptom. This
// holds the FIRST failure since power-up, in full and unaltered, plus a running
// count of all of them. `valid` is 0 until something has actually failed.
struct FdcFirstFail {
  uint8_t valid;
  uint8_t kind;        // 1 RNF, 2 data CRC, 3 wrong cylinder, 4 Verify/seek
  uint32_t count;      // failures since power-up (not just the latched one)
  uint32_t at_ms;      // ms since boot -- says whether it was a cold-start
  uint8_t want_sec;
  uint8_t trk_reg;     // the position chain, all sampled at the failure
  uint8_t head_target;
  uint8_t phys_track;
  uint8_t side;
  uint8_t seen_cyl;    // what the disk said instead (0xFF = it said nothing)
  uint8_t seen_head;
  uint8_t capture_ok;
  uint32_t flux_n;
  uint32_t capture_us;
  uint8_t ids_seen;
  uint8_t ids[16];     // the sector numbers really on the track
  // Head-position resyncs: an ID field proved the head was elsewhere and we
  // adopted the disk's answer (fdc_resync_head).
  uint32_t pos_resyncs;
  uint8_t last_resync_from;
  uint8_t last_resync_to;
  // Bus RESET (GPIO29). `line` is the level RIGHT NOW, which is the number to
  // look at first: the whole design rests on the line idling LOW, so a machine
  // sitting at a BASIC prompt must read 0 here. If it reads 1, the polarity
  // assumption is wrong for this board and the poller is holding EINT cleared,
  // silently killing interrupts for an interrupt-driven host.
  uint32_t reset_events;
  uint32_t reset_at_ms;
  uint8_t reset_line;
  uint8_t reset_active;
};
void fdc_firstfail_snapshot(FdcFirstFail *out);

// Raw bus-RESET line telemetry, recorded with no polarity assumption -- what
// the pin actually did, as opposed to what we decided about it. This is the
// tool for "I pressed reset and nothing was counted": it separates a line that
// never moves from one that pulses the other way from a pulse too short for
// the debounce, which the event counter alone cannot do.
struct FdcResetDiag {
  uint8_t line;          // level right now
  uint32_t samples;      // core1 polls (i.e. is the poller even running?)
  uint32_t rise, fall;   // edges seen since the counters were cleared
  uint32_t high_max_us;  // longest completed HIGH excursion
  uint32_t low_min_us;   // shortest completed LOW excursion
  uint32_t last_high_us; // most recent completed HIGH length
  uint32_t events;       // confirmed resets acted on
  uint32_t at_ms;
};
void fdc_resetdiag_snapshot(FdcResetDiag *out);
void fdc_resetdiag_clear(void);
void fdc_firstfail_reset(void);

// Ordered I/O trace (see the io_* ring in fdc.cpp). Entries run oldest ->
// newest; `rep` counts coalesced consecutive identical accesses, so a STATUS
// poll loop shows up as one entry with a large repeat rather than flooding the
// ring. Values are raw bus bytes -- still complemented for 0xD8..0xDB.
uint8_t fdc_iotrace_count(void);
uint32_t fdc_iotrace_total(void);
uint8_t fdc_iotrace_get(uint8_t idx, uint8_t *port, uint8_t *val,
                        uint8_t *is_rd, uint16_t *rep);

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

// How ~INT (GPIO12) is driven: 0 = never, 1 = INTRQ at command completion (WD
// default), 2 = INTRQ | DRQ for a card that ORs both onto the Z80 interrupt.
// Switchable because the MZ-800 floppy card's wiring of the WD's two separate
// INTRQ/DRQ pins is not known from the ROM alone.
void fdc_set_int_mode(uint8_t mode);
uint8_t fdc_get_int_mode(void);

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

// Bench-only SYNCHRONOUS write of one sector (no MZ-800): select+spin, seek to
// `track`, select `side`, then locate sector `sector` on the spinning disk and
// splice `n` bytes of `data` into its data field (see mfm_splice.hpp). This is
// the whole WRITE path minus the bus, so a sector can be written and read back
// over USB before any machine is allowed near it.
//
// `n` must equal the sector's recorded size or the write is refused. Returns
// true only if flux actually went onto the media; *flags gets the splice result
// (b0 wrote b1 id_found b2 cyl_mismatch b3 size_mismatch b4 late b5 no_flux).
bool fdc_bench_write_sector(uint8_t track, uint8_t sector, uint8_t side,
                            const uint8_t *data, uint16_t n, uint8_t *flags);

// Bench-only SYNCHRONOUS format of one track (no MZ-800): build a standard IBM
// System-34 MFM track image with `n_sectors` sectors of 128<<size_code bytes
// filled with `fill`, numbered 1..n and laid out with the given `interleave`
// step, then write it index-to-index. Goes through the same encoder the host's
// WRITE TRACK uses, so it exercises the real path.
//
// THIS DESTROYS THE TRACK. Returns the image length in bytes (0 on failure);
// *out_result: 0 ok, 1 write protected, 2 image too long, 3 write failed.
uint16_t fdc_bench_format(uint8_t track, uint8_t side, uint8_t n_sectors,
                          uint8_t size_code, uint8_t interleave, uint8_t fill,
                          uint8_t *out_result);

// Bench-only SYNCHRONOUS READ TRACK (no MZ-800): seek, capture a revolution and
// decode every byte on the track into the controller's track buffer. Uses
// `flux_buf` (>= a revolution's worth) as capture scratch. Returns the number
// of decoded bytes; read them with fdc_track_buffer().
uint16_t fdc_bench_read_track(uint8_t track, uint8_t side, uint8_t *flux_buf,
                              size_t flux_cap);
const uint8_t *fdc_track_buffer(void);

// Master enable for WRITE SECTOR from the MZ-800 (see g_write_enable). OFF puts
// the bridge back to read-only: write commands are refused with WRITE PROTECT
// and nothing can reach the media. Does not affect fdc_bench_write_sector().
void fdc_set_write_enable(bool on);
bool fdc_get_write_enable(void);

// Fill `out_decoded`/`out_raw` (>=16 bytes each) with recent command bytes
// (oldest->newest). Decoded is the WD opcode (~raw), raw is the byte sampled on
// the data bus at D8. Returns count written; *count = total commands seen.
uint8_t fdc_debug_cmdlog(uint8_t *out_decoded, uint8_t *out_raw,
                         uint16_t *count);
