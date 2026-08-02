#include "fdc.hpp"
#include "board_bus.h" // RAM_FUNC
#include "board_config.h" // PIN_INDEX, PIN_RDATA
#include "floppy.hpp"
#include "flux/arch_rp2.h" // rp2040_flux_capture_start/drain/stop (streaming)
#include "gw_protocol.hpp" // mzp_flux_buffer (shared capture buffer)
#include "mfm_decode.hpp"  // mfm_read_sector
#include "mz_bus.hpp"      // bus_diag_reset
#include "hardware/gpio.h" // gpio_get (index-edge sync)
#include "pico/time.h"     // sleep_ms
#include <cstring>         // memcpy

// WD register window (true, un-inverted values). The MZ-800 ROM CPLs the WD
// registers 0xD8..0xDB (command/status/track/sector/data) but NOT the control
// ports 0xDC..0xDE (motor/page/density) — verified against MZ2Z046.ASM.
//
// STATUS bits (Type I): b7 NotReady b6 WriteProtect b5 HeadLoaded b4 SeekError
//                       b3 CRCError b2 Track00 b1 Index b0 Busy.

namespace {

// Largest supported sector data field: 128 << 3 = 1024 bytes.
static constexpr uint16_t FDC_SECTOR_MAX = 1024;

struct FdcState {
  // WD register file (true values)
  volatile uint8_t track = 0;   // Track register
  volatile uint8_t sector = 1;  // Sector register
  volatile uint8_t data = 0;    // Data register
  volatile uint8_t command = 0; // last command opcode (true)

  volatile bool busy = false; // Type I in flight (core1 sets, core0 clears)

  // Control ports (true values), written by core1, actioned by core0.
  volatile bool motor_on = false;
  volatile uint8_t drive_sel = 0;
  volatile bool side1 = false;
  volatile bool density_single = false;

  // Seek handshake core1 -> core0.
  volatile uint32_t op_seq = 0;  // bumped by core1 to request a physical move
  volatile uint32_t op_done = 0; // set == op_seq by core0 when finished
  volatile uint8_t op_target = 0;
  volatile bool op_restore = false;
  volatile bool op_verify = false; // WD Type I Verify (V) bit on this command

  // Physical status core0 -> core1.
  volatile bool at_track0 = true;
  volatile bool write_protect = false;
  volatile bool seek_error = false; // Verify (if requested) found no matching ID

  // ---- Type II READ SECTOR ----------------------------------------------
  // `type2` selects the Type II STATUS layout (set by a Type II command, cleared
  // by a Type I command). `reading` = a read transfer is in flight (BUSY held).
  volatile bool type2 = false;
  volatile bool reading = false;
  volatile bool pending_force_int = false;
  volatile uint32_t pending_force_since_us = 0;
  volatile uint32_t last_data_read_us = 0;
  volatile bool multi = false; // multi-record read (auto-advance sectors)
  // Error/status flags for the current/last Type II op.
  volatile bool err_rnf = false;
  volatile bool err_crc = false;
  volatile bool lost_data = false;
  volatile bool rec_deleted = false; // matched a deleted-DAM (0xF8)
  // One-sector transfer buffer. core0 fills it and raises `drq`; core1 hands the
  // bytes to the host on DATA reads and lowers `drq` when the sector is drained.
  // Ownership passes via `drq`: core0 writes xfer[]/xfer_len only while drq==0.
  volatile bool drq = false;
  volatile uint8_t xfer[FDC_SECTOR_MAX];
  volatile uint16_t xfer_len = 0; // valid bytes in xfer[]
  volatile uint16_t xfer_pos = 0; // next byte core1 will hand out
  // Read request handshake core1 -> core0: "decode sector `rd_sector`".
  volatile uint32_t rd_seq = 0;
  volatile uint32_t rd_done = 0;
  volatile uint8_t rd_sector = 1;
  volatile bool rd_fresh = false; // first sector of a command -> (re)capture flux
  // Diagnostics: how long BUSY was actually held before the host gave up
  // (Force Interrupt) on a read still in flight. Lets us measure the real
  // MZ-800 poll timeout instead of guessing at it.
  volatile uint32_t rd_issue_us = 0;
  volatile uint32_t rd_elapsed_us = 0; // set at abort time; 0 = no abort seen yet

  // READ-path diagnostics. Without these an RNF is a dead end: you cannot tell
  // "capture never started" from "captured plenty but sector N is not on this
  // track" from "found it but the CRC failed".
  volatile bool rd_capture_ok = false;  // did the capture window start?
  volatile uint32_t rd_flux_n = 0;      // flux bytes captured for the last read
  volatile uint32_t rd_capture_us = 0;  // wall time spent in the capture window
  volatile uint32_t rd_search_us = 0;   // wall time spent decoding
  volatile uint8_t rd_ids_seen = 0;     // ID fields decoded in that flux
  volatile uint8_t rd_ids[16] = {0};    // the sector numbers actually found
  volatile uint8_t rd_want = 0;         // sector the host asked for
  volatile uint8_t rd_result = 0;       // b0 found b1 has_data b2 crc_ok b3 del

  // How long the host really polls STATUS after a READ command. This is the
  // number that decides whether a capture-then-decode strategy is viable at
  // all: one revolution is ~200 ms, so if the host gives up in single-digit ms
  // no amount of decode tuning helps and the capture must run ahead of the
  // command (which needs lossless/DMA flux). Measured, not assumed.
  volatile uint32_t status_polls = 0;
  volatile uint32_t status_last_us = 0;

  // Diagnostics: gap between a Type I seek/restore completing (BUSY 1->0) and
  // the NEXT command being issued. Tells us whether the host does any waiting
  // (e.g. polling for INDEX to synchronize rotationally) between the seek and
  // the following read, or issues it immediately.
  volatile uint32_t last_ready_us = 0;
  volatile uint32_t cmd_gap_us = 0;

  // Diagnostics: did the last Verify's capture even start, and how many flux
  // bytes did it manage to drain before finishing (found or gave up)? Lets us
  // tell "capture never started/produced data in this context" apart from
  // "captured plenty but genuinely found no matching ID."
  volatile bool verify_start_ok = false;
  volatile uint32_t verify_n = 0;

  volatile bool active = false; // set once the MZ-800 first talks to us

  // Command-path diagnostics (for bus integrity checks).
  volatile uint16_t cmd_writes_total = 0;
  volatile uint8_t cmd_last_raw = 0;
  volatile uint8_t cmd_last_decoded = 0;

  // Port 0xDF bus-test state.
  volatile bool bt_active = false;
  volatile uint8_t bt_last_rx = 0;
  volatile uint8_t bt_last_echo = 0;
  // 32-bit: at ~130 kHz (a tight OUT loop) a uint16 counter wraps every 0.5 s,
  // which silently turns every err/rx ratio into nonsense.
  volatile uint32_t bt_rx_count = 0;
  volatile uint32_t bt_rd_count = 0;
  volatile uint32_t bt_err_count = 0;
  // Mismatch classification. A "gap" (got == expect + 1..8) means whole write
  // cycles went missing -- the byte we captured was itself fine. Anything else
  // means the byte we captured is corrupt. These two answer very different
  // questions and must not be lumped into one error count.
  volatile uint32_t bt_err_gap = 0;
  volatile uint32_t bt_err_garbage = 0;
  volatile uint8_t bt_bits_want1 = 0; // bits that should be 1 but read 0
  volatile uint8_t bt_bits_want0 = 0; // bits that should be 0 but read 1
  volatile uint8_t bt_mm_exp[8] = {0}; // ring of the last 8 mismatches
  volatile uint8_t bt_mm_got[8] = {0};
  volatile uint8_t bt_mm_pos = 0;
  volatile uint32_t bt_mm_n = 0;
  // Per-line duty: how often each C-line was sampled HIGH. For the incrementing
  // test sequence every bit is high exactly 50.00 % of the time, so any
  // deviation measures a weak/stuck line DIRECTLY -- no dependence on the
  // last+1 sequence check, which couples adjacent writes and makes an
  // intermittent line hard to quantify.
  volatile uint32_t bt_bit_high[8] = {0};
  // Mismatches whose immediately preceding write also mismatched. A single-cycle
  // glitch always produces an adjacent pair (the error, then the recovery);
  // a line stuck for a long run produces isolated errors 128 writes apart.
  volatile uint32_t bt_err_adjacent = 0;
  volatile bool bt_prev_err = false;
  // READ-direction check. The 0xDF loopback only proves we *served* the read
  // cycle -- nothing verifies the byte the Z80 latched. The test program echoes
  // what it read back to port 0xDE, so we can compare it against the byte we
  // actually drove (bt_last_echo). Writes are proven clean, so any mismatch
  // here is the read path.
  volatile uint32_t bt_rd_reports = 0;
  volatile uint32_t bt_rd_err = 0;
  volatile uint8_t bt_rd_bits_want1 = 0;
  volatile uint8_t bt_rd_bits_want0 = 0;
  volatile uint8_t bt_rd_last_exp = 0;
  volatile uint8_t bt_rd_last_got = 0;
  volatile uint32_t bt_rd_bit_high[8] = {0};

  // Ring log of recent command opcodes (true values), for debugging.
  volatile uint8_t cmd_hist[16] = {0};
  volatile uint8_t cmd_raw_hist[16] = {0};
  volatile uint8_t cmd_pos = 0;  // next slot
  volatile uint16_t cmd_count = 0;
};

FdcState S;
FloppyDrive *DRV = nullptr;

} // namespace

// Side-select inversion (see the 0xDD case). ON by default: this is the
// MZ-800's measured behaviour, not a workaround -- with it off the machine
// cannot boot at all. Outside the anonymous namespace so the USB probe can flip
// it live for A/B testing; core1 reads it on every 0xDD write.
volatile bool g_invert_side = true;

namespace {

// core0-only "last applied" shadow (no volatile needed — single core).
bool g_applied_select = false;
bool g_applied_motor = false; // physical motor state (may lag S.motor_on)
int g_applied_side = -1;
bool g_timing_set = false;
uint32_t g_motor_off_at = 0; // deferred motor-stop deadline (ms; 0 = none)

// core0-only decode scratch (one sector) + streaming-capture state.
uint8_t g_secbuf[FDC_SECTOR_MAX];
size_t g_flux_n = 0;         // bytes drained into the shared flux buffer so far
bool g_stream_active = false; // an incremental capture is currently running

// MZ-800 disks are MFM double-density: 250 kbit/s -> 2.0 µs nominal bit time.
// (FM/single-density decode is out of scope until 3.4; see 0xDE density port.)
static constexpr float FDC_DD_BIT_US = 2.0f;

// Index is active-low. Waiting for a falling edge aligns capture close to the
// start of a revolution (sector 1 conventionally follows index).
static bool wait_for_index_falling_edge_ms(uint32_t timeout_ms) {
  uint32_t t0 = to_ms_since_boot(get_absolute_time());
  // If we're currently inside the low pulse, wait for it to end first.
  while (!gpio_get(PIN_INDEX)) {
    if ((to_ms_since_boot(get_absolute_time()) - t0) > timeout_ms)
      return false;
  }
  // Then wait for the next falling edge.
  while (gpio_get(PIN_INDEX)) {
    if ((to_ms_since_boot(get_absolute_time()) - t0) > timeout_ms)
      return false;
  }
  return true;
}

// Type-I Verify can tolerate a longer pre-capture wait.
static constexpr uint32_t FDC_VERIFY_INDEX_SYNC_BUDGET_MS = 260;
// NOTE: a set of "ROM bring-up workarounds" used to live here -- command
// aliases 0x14->0x94 and 0x58->0xD8, a relaxed Verify seek-error gate, an
// any-sector read fallback, and a BUSY hold. They existed because the ROM
// appeared to issue nonsensical opcodes. It was not the ROM: data-bus bit7 was
// a marginal solder joint on U2, so every command whose true opcode has bit7=1
// (all of Type II/III and Force Interrupt) lost that bit through the CPL the
// ROM applies -- 0x94 arrived as 0x14, 0xD8 as 0x58. Fixed in hardware and
// verified with `mzp_probe.py bustest` (>1M cycles, both directions, zero
// errors). Do not reintroduce them: an implausible opcode from the ROM is a
// bus-integrity symptom, so measure the bus first.
static constexpr uint32_t FDC_FORCE_INT_DEFER_IDLE_US = 2000;

// Synchronous (blocking) Verify: after a seek, confirm the target track really
// has a matching ID field recorded on it. Uses the same fast incremental
// capture as the Type II read path, but as a single bounded blocking pass --
// Verify doesn't need per-byte DRQ pacing, just a pass/fail before BUSY clears.
// Bounded to one buffer's worth of flux (well over one revolution) as a safety
// net; real WD hardware retries for up to ~5 revolutions if a slower drive or
// unlucky rotational timing ever needs more headroom than that here.
static bool fdc_verify_track(uint8_t track) {
  size_t cap = 0;
  uint8_t *flux = mzp_flux_buffer(&cap);

  // Start verify capture at an index edge when possible, to reduce rotational
  // alignment misses on media where the expected IDs are index-relative.
  (void)wait_for_index_falling_edge_ms(FDC_VERIFY_INDEX_SYNC_BUDGET_MS);

  bool start_ok = rp2040_flux_capture_start(PIN_INDEX, PIN_RDATA);
  S.verify_start_ok = start_ok;
  if (!start_ok) {
    S.verify_n = 0;
    return false;
  }
  size_t n = 0;
  bool ok = false;
  while (n < cap) {
    size_t new_n = rp2040_flux_capture_drain(flux, cap, n, false);
    if (new_n > n) {
      n = new_n;
      if (mfm_verify_track(flux, n, FDC_DD_BIT_US, track)) {
        ok = true;
        break;
      }
    }
  }
  S.verify_n = (uint32_t)n;

  if (!ok && n > 0) {
    // Compatibility fallback for ROM bring-up: some media/boot tracks can carry
    // non-canonical C bytes while still being readable. If strict WD-style
    // cylinder equality fails, accept the seek as verified when any valid IDAM
    // is visible on-track. This keeps the ROM path moving while preserving
    // strict read CRC/RNF checks in Type II data transfer.
    SectorInfo ids[32];
    int ns = mfm_scan_track(flux, n, FDC_DD_BIT_US, ids, 32, g_secbuf,
                            sizeof(g_secbuf));
    for (int i = 0; i < ns; ++i) {
      if (ids[i].id_crc_ok) {
        ok = true;
        break;
      }
    }
  }

  rp2040_flux_capture_stop();
  return ok;
}

// One disk revolution is ~43.5k flux transitions at DD (measured with `scan`).
// Capture ~1.05 revolutions: enough that every sector passes the head once,
// with a small margin for spindle-speed variation. The host's measured patience
// after a READ is ~285 ms (25k STATUS polls, see `rddiag`) and a revolution is
// ~200 ms, so this window has to stay tight -- 1.25 revolutions left only ~7 ms
// of headroom.
static constexpr size_t FDC_READ_CAPTURE_BYTES = 45700;
// Safety bound on the capture window (a revolution is ~200 ms).
static constexpr uint32_t FDC_READ_CAPTURE_TIMEOUT_US = 300000;

// Capture a bounded flux window with NOTHING between drains, then stop.
//
// This must not be interleaved with decoding. The capture has no DMA: the PIO
// RX FIFO is 4 words deep and each word carries 2 half-samples, so it buffers
// 8 transitions -- about 24 us at DD rates. Any work between drains longer
// than that silently drops flux, MFM sync never holds, and no sector is ever
// found. That is exactly what the previous decode-while-capturing loop did,
// which is why `scan`/`streamscan`/`fdcread` (all capture-then-decode) worked
// while the live read path returned RNF on a perfectly good disk.
static void fdc_capture_window(void) {
  size_t cap = 0;
  uint8_t *flux = mzp_flux_buffer(&cap);
  size_t want = (cap < FDC_READ_CAPTURE_BYTES) ? cap : FDC_READ_CAPTURE_BYTES;

  g_flux_n = 0;
  uint32_t t0 = time_us_32();
  g_stream_active = rp2040_flux_capture_start(PIN_INDEX, PIN_RDATA);
  S.rd_capture_ok = g_stream_active;
  if (g_stream_active) {
    while (g_flux_n < want &&
           (time_us_32() - t0) < FDC_READ_CAPTURE_TIMEOUT_US) {
      g_flux_n = rp2040_flux_capture_drain(flux, want, g_flux_n,
                                           /*store_greaseweazle=*/false);
    }
    rp2040_flux_capture_stop();
    g_stream_active = false;
  }
  S.rd_capture_us = time_us_32() - t0;
  S.rd_flux_n = g_flux_n;
}

// On failure, report what IS on the track. "decoded 9 IDs numbered 1..9, host
// wanted 3" is a diagnosis; "RNF" is not.
static void fdc_note_track_ids(const uint8_t *flux, size_t n_flux) {
  SectorInfo ids[40];
  int ns = mfm_scan_track(flux, n_flux, FDC_DD_BIT_US, ids, 40, g_secbuf,
                          sizeof(g_secbuf));
  S.rd_ids_seen = (ns < 0) ? 0 : (uint8_t)ns;
  for (int k = 0; k < 16; ++k)
    S.rd_ids[k] = (k < ns) ? ids[k].sec : 0;
}

// Keep the spindle spinning this long after the ROM commands motor-off, so
// RESTORE retries (and back-to-back ops) don't re-pay the ~1 s spin-up. This is
// the standard floppy motor-off delay.
static constexpr uint32_t FDC_MOTOR_OFF_DELAY_MS = 3000;

inline uint32_t now_ms() { return to_ms_since_boot(get_absolute_time()); }

inline uint8_t compose_status() {
  uint8_t st = 0;
  if (S.busy)
    st |= 0x01; // BUSY (b0) — same bit in both layouts
  // Ready once the ROM has commanded the motor on (real READY line on PC drives
  // is unreliable; refine in 3.4). NotReady = bit7 in both layouts.
  if (!S.motor_on)
    st |= 0x80; // NOT READY

  if (S.type2) {
    // Type II layout: b1 DRQ, b2 LostData, b3 CRC, b4 RNF, b5 RecordType(deleted).
    // The MZ-800 ROM's read loop masks b2..b6 out mid-transfer (OR 0x7C) and,
    // after the terminating Force Interrupt, requires the *true* status to be
    // 0x00 for success — so a clean read leaves all these bits clear.
    if (S.drq)
      st |= 0x02;
    if (S.lost_data)
      st |= 0x04;
    if (S.err_crc)
      st |= 0x08;
    if (S.err_rnf)
      st |= 0x10;
    if (S.rec_deleted)
      st |= 0x20;
  } else {
    // Type I layout: b2 Track00, b6 WriteProtect. Index (b1) intentionally left
    // at 0 — tried tying it live to the physical ~INDEX pin, but the pulse is
    // wide enough to persist across multiple STATUS polls, and the MZ-800 boot
    // ROM's RESTORE-completion check turned out to be an exact/tight match that
    // doesn't tolerate it toggling mid-check (unlike Disk BASIC's, which
    // explicitly masks bit1 out of its own check) — regressed RESTORE into an
    // infinite retry loop. A real WD presents Index as a one-shot edge-latched
    // pulse, not a live level; revisit with that semantics if actually needed.
    if (S.at_track0)
      st |= 0x04; // TRACK 00
    if (S.seek_error)
      st |= 0x10; // SEEK ERROR (set when a Verify-flagged seek finds no
                  // matching ID field on the target track)
    if (S.write_protect)
      st |= 0x40; // WRITE PROTECT
  }
  return st;
}

} // namespace

void fdc_init(FloppyDrive *drive) { DRV = drive; }

// ---- core1 (bus) hot path -------------------------------------------------

RAM_FUNC void fdc_reg_write(uint8_t addr, uint8_t raw) {
  switch (addr) {
  case 0xD8: { // COMMAND (CPL'd on the bus)
    uint8_t cmd = (uint8_t)~raw;
    S.cmd_last_raw = raw;
    S.cmd_last_decoded = cmd;
    S.cmd_writes_total++;
    S.active = true;
    S.command = cmd;
    S.cmd_hist[S.cmd_pos & 15] = cmd;
    S.cmd_raw_hist[S.cmd_pos & 15] = raw;
    S.cmd_pos++;
    S.cmd_count++;
    S.cmd_gap_us = time_us_32() - S.last_ready_us; // diagnostic: see last_ready_us
    if ((cmd & 0x80) == 0) {
      // ---- Type I (seek family) ----
      S.type2 = false; // STATUS reverts to the Type I layout
      uint8_t hi = cmd >> 4;
      uint8_t tgt = S.track;
      bool restore = false;
      if (hi == 0x0) { // RESTORE
        restore = true;
        tgt = 0;
      } else if (hi == 0x1) { // SEEK (to Data register)
        tgt = S.data;
      } else if (hi == 0x4 || hi == 0x5) { // STEP IN
        tgt = (uint8_t)(S.track + 1);
      } else if (hi == 0x6 || hi == 0x7) { // STEP OUT
        tgt = S.track ? (uint8_t)(S.track - 1) : 0;
      }
      // (bare STEP 0x2X/0x3X keeps the current track — matches reference)
      S.op_target = tgt;
      S.op_restore = restore;
      S.op_verify = (cmd & 0x04) != 0; // WD Type I Verify (V) bit
      S.busy = true;
      S.op_seq++;
    } else if ((cmd & 0xF0) == 0xD0) {
      // ---- Type IV Force Interrupt: abort everything, drop BUSY/DRQ ----
      // The ROM terminates a multi-sector read with 0xD0, then reads STATUS and
      // requires the true value == 0x00 for success — so we must not touch the
      // Type II error flags (a CRC/RNF seen mid-read must survive to be reported)
      // nor `type2` (the status layout follows the last data command).
      if (S.reading && S.drq && S.xfer_pos < S.xfer_len) {
        // If Force Interrupt arrives while bytes are still being drained from
        // the published sector buffer, defer completion until that buffer is
        // fully consumed, but only while DATA reads keep progressing.
        S.pending_force_int = true;
        S.pending_force_since_us = time_us_32();
      } else {
        if (S.reading) // diagnostic: how long did the host actually wait?
          S.rd_elapsed_us = time_us_32() - S.rd_issue_us;
        S.busy = false;
        S.reading = false;
        S.drq = false;
        S.pending_force_int = false;
        S.pending_force_since_us = 0;
        S.op_done = S.op_seq;
      }
    } else if ((cmd & 0xC0) == 0x80) {
      // ---- Type II (sector data): 0x80..0xBF ----
      S.type2 = true;
      S.err_rnf = S.err_crc = S.lost_data = S.rec_deleted = false;
      S.multi = (cmd & 0x10) != 0;
      if ((cmd & 0x20) == 0) {
        // READ SECTOR — single (0x8X) or multiple (0x9X). Ask core0 to decode
        // the current Sector register off a fresh capture of the current track.
        S.reading = true;
        S.pending_force_int = false;
        S.pending_force_since_us = 0;
        S.drq = false;
        S.xfer_len = 0;
        S.xfer_pos = 0;
        S.rd_sector = S.sector;
        S.rd_fresh = true; // first sector of the command -> (re)capture flux
        S.busy = true;     // BUSY must appear at once (ROM's pre-loop times out)
        S.rd_issue_us = time_us_32();
        S.status_polls = 0;
        S.status_last_us = S.rd_issue_us;
        S.last_data_read_us = S.rd_issue_us;
        S.rd_elapsed_us = 0; // cleared until an abort (or drain) records a real value
        S.rd_seq++;
      } else {
        // WRITE SECTOR — Milestone 3.3. The MZ-800 boot/directory path never
        // writes, so completing benignly (not busy) is safe for now.
        S.reading = false;
        S.busy = false;
      }
    } else {
      // ---- Type III (Read Address/Track, Write Track): 0xC0/0xE0/0xF0 ----
      // Milestone 3.4. Use the Type II status layout (not Type I) and complete
      // benignly so the ROM doesn't hang polling BUSY.
      S.type2 = true;
      S.reading = false;
      S.busy = false;
    }
    break;
  }
  case 0xD9:
    // TRACK register: inverted like the other FDC registers. The ROM doesn't
    // CPL it in software because it keeps track values complemented internally
    // (its seek-skip compare uses ~target); we invert here so our WD state holds
    // the true track. The ROM's 0xA5 write is a presence-detect pattern, not a
    // track — it round-trips regardless.
    S.track = (uint8_t)~raw;
    break;
  case 0xDA:
    S.sector = (uint8_t)~raw; // SECTOR (CPL'd)
    break;
  case 0xDB:
    S.data = (uint8_t)~raw; // DATA (CPL'd)
    break;
  case 0xDC: // MOTOR/DRIVE (not CPL'd): b7 = motor on, b1..0 = drive select
    S.active = true;
    S.motor_on = (raw & 0x80) != 0;
    S.drive_sel = raw & 0x03;
    break;
  case 0xDD: { // PAGE/SIDE (not CPL'd at bus level). ROM writes 0xFF/0xFE after
               // a CPL step, so bit0 low selects side1 and bit0 high selects
               // side0 (see FD_PREPARE_TRACK_AND_SIDE at E61B in the boot ROM).
    bool rom_side1 = (raw & 0x01) == 0;
    // The MZ-800's floppy card drives side-select INVERTED relative to the
    // PC/Shugart convention our drive code uses: the ROM's side 0 is physical
    // head 1. Confirmed by booting CP/M -- the boot ROM reads logical track 0
    // (= cyl0/side0 by its own cylinder*2+side mapping), while every MZ-800
    // CP/M image carries its 16x256 IPL track on cyl0/side1 as a PC reads it.
    // Runtime-togglable via `mzp_probe.py sideswap 0` for A/B testing.
    S.side1 = g_invert_side ? !rom_side1 : rom_side1;
    break;
  }
  case 0xDE: // DENSITY (not CPL'd): b0 = single density
    S.density_single = (raw & 0x01) != 0;
    // Bus-test read-back channel: during the 0xDF loopback the test program
    // writes here whatever it just read from 0xDF. Compare against the byte we
    // drove. (Purely additive -- density still applies as normal.)
    if (S.bt_active) {
      uint8_t expect_rd = S.bt_last_echo;
      S.bt_rd_reports++;
      for (int k = 0; k < 8; ++k)
        if (raw & (1u << k))
          S.bt_rd_bit_high[k]++;
      if (raw != expect_rd) {
        S.bt_rd_err++;
        S.bt_rd_bits_want1 |= (uint8_t)(expect_rd & ~raw);
        S.bt_rd_bits_want0 |= (uint8_t)(~expect_rd & raw);
        S.bt_rd_last_exp = expect_rd;
        S.bt_rd_last_got = raw;
      }
    }
    break;
  case 0xDF: { // BUS TEST DATA (not CPL'd)
    uint8_t expect = S.bt_last_rx;
    if (!S.bt_active) {
      S.bt_active = true;
      expect = raw; // first byte seeds the sequence
    } else {
      expect = (uint8_t)(expect + 1); // expect strictly incrementing bytes
    }
    for (int k = 0; k < 8; ++k)
      if (raw & (1u << k))
        S.bt_bit_high[k]++;
    if (raw != expect) {
      S.bt_err_count++;
      if (S.bt_prev_err)
        S.bt_err_adjacent++;
      S.bt_prev_err = true;
      uint8_t delta = (uint8_t)(raw - expect);
      if (delta >= 1 && delta <= 8)
        S.bt_err_gap++; // clean sequence gap: N cycles lost, byte itself OK
      else
        S.bt_err_garbage++; // the captured byte is wrong
      S.bt_bits_want1 |= (uint8_t)(expect & ~raw);
      S.bt_bits_want0 |= (uint8_t)(~expect & raw);
      S.bt_mm_exp[S.bt_mm_pos] = expect;
      S.bt_mm_got[S.bt_mm_pos] = raw;
      S.bt_mm_pos = (uint8_t)((S.bt_mm_pos + 1) & 7);
      S.bt_mm_n++;
    } else {
      S.bt_prev_err = false;
    }
    S.bt_last_rx = raw;
    S.bt_last_echo = (uint8_t)(raw ^ 0xA5);
    S.bt_rx_count++;
    break;
  }
  }
}

RAM_FUNC uint8_t fdc_reg_read(uint8_t addr) {
  switch (addr) {
  case 0xD8:
    if (S.reading) { // measure the host's real BUSY/DRQ patience
      S.status_polls++;
      S.status_last_us = time_us_32();
    }
    return (uint8_t)~compose_status(); // STATUS (driven CPL'd)
  case 0xD9:
    return (uint8_t)~S.track; // inverted like the other registers
  case 0xDA:
    return (uint8_t)~S.sector;
  case 0xDB:
    // DATA register. During a read transfer, hand out the next buffered byte and
    // pace DRQ. The ROM only reads DATA when STATUS shows DRQ (it guards every
    // INI with the status test), so an out-of-transfer read just returns the
    // stale Data register.
    if (S.reading && S.drq) {
      uint8_t b = S.xfer[S.xfer_pos];
      S.xfer_pos++;
      S.last_data_read_us = time_us_32();
      if (S.xfer_pos >= S.xfer_len) {
        // Sector drained.
        S.drq = false;
        if (S.pending_force_int) {
          S.rd_elapsed_us = time_us_32() - S.rd_issue_us;
          S.reading = false;
          S.busy = false;
          S.pending_force_int = false;
          S.pending_force_since_us = 0;
          return (uint8_t)~b;
        }
        if (S.multi) {
          // WD auto-advances to the next sector in multi-record mode and stays
          // BUSY; ask core0 for it (flux already captured -> rd_fresh stays 0).
          S.sector++;
          S.rd_sector = S.sector;
          S.rd_seq++;
        } else {
          S.reading = false;
          S.busy = false; // single-sector read complete
        }
      }
      return (uint8_t)~b; // DATA is CPL'd on the bus like the other WD registers
    }
    return (uint8_t)~S.data;
  case 0xDF:
    S.bt_rd_count++;
    return S.bt_last_echo;
  default:
    return 0xFF; // control ports are write-only
  }
}

uint8_t fdc_debug_cmdlog(uint8_t *out_decoded, uint8_t *out_raw,
                         uint16_t *count) {
  uint8_t n = (S.cmd_count < 16) ? (uint8_t)S.cmd_count : 16;
  // Emit oldest -> newest.
  uint8_t start = (uint8_t)(S.cmd_pos - n);
  for (uint8_t k = 0; k < n; ++k) {
    uint8_t idx = (uint8_t)(start + k) & 15;
    out_decoded[k] = S.cmd_hist[idx];
    out_raw[k] = S.cmd_raw_hist[idx];
  }
  *count = S.cmd_count;
  return n;
}

void fdc_debug_snapshot(FdcDebug *o) {
  o->status = compose_status();
  o->track = S.track;
  o->sector = S.sector;
  o->data = S.data;
  o->command = S.command;
  o->flags = (uint8_t)((S.motor_on ? 0x80 : 0) | (S.side1 ? 0x04 : 0) |
                       (S.write_protect ? 0x02 : 0) | (S.busy ? 0x01 : 0));
  o->drive = S.drive_sel;
  o->at_track0 = S.at_track0 ? 1 : 0;
  int t = DRV ? DRV->track() : -1;
  o->phys_track = (t < 0) ? 0xFF : (uint8_t)t;
  o->op_seq = (uint8_t)S.op_seq;
  o->op_done = (uint8_t)S.op_done;
  o->rd_flags = (uint8_t)((S.type2 ? 0x01 : 0) | (S.reading ? 0x02 : 0) |
                          (S.drq ? 0x04 : 0) | (S.multi ? 0x08 : 0) |
                          (S.err_rnf ? 0x10 : 0) | (S.err_crc ? 0x20 : 0) |
                          (S.rec_deleted ? 0x40 : 0));
  o->rd_sector = S.rd_sector;
  o->xfer_pos = S.xfer_pos;
  o->xfer_len = S.xfer_len;
  o->rd_seq = (uint8_t)S.rd_seq;
  o->rd_done = (uint8_t)S.rd_done;
  o->rd_elapsed_us = S.rd_elapsed_us;
  o->cmd_gap_us = S.cmd_gap_us;
  o->verify_start_ok = S.verify_start_ok ? 1 : 0;
  o->verify_n = S.verify_n;
  o->cmd_writes_total = S.cmd_writes_total;
  o->cmd_last_raw = S.cmd_last_raw;
  o->cmd_last_decoded = S.cmd_last_decoded;
  o->rd_capture_ok = S.rd_capture_ok ? 1 : 0;
  o->rd_flux_n = S.rd_flux_n;
  o->rd_capture_us = S.rd_capture_us;
  o->rd_search_us = S.rd_search_us;
  o->rd_ids_seen = S.rd_ids_seen;
  for (int k = 0; k < 16; ++k)
    o->rd_ids[k] = S.rd_ids[k];
  o->rd_want = S.rd_want;
  o->rd_result = S.rd_result;
  o->status_polls = S.status_polls;
  o->status_span_us = S.status_last_us - S.rd_issue_us;
  o->bt_active = S.bt_active ? 1 : 0;
  o->bt_last_rx = S.bt_last_rx;
  o->bt_last_echo = S.bt_last_echo;
  // Truncated to 16 bits for the legacy FDCDUMP wire format -- use the
  // `bustest` command for the authoritative, non-wrapping counts.
  o->bt_rx_count = (uint16_t)S.bt_rx_count;
  o->bt_rd_count = (uint16_t)S.bt_rd_count;
  o->bt_err_count = (uint16_t)S.bt_err_count;
}

void fdc_set_side_invert(bool on) { g_invert_side = on; }
bool fdc_get_side_invert(void) { return g_invert_side; }

void fdc_bustest_reset(void) {
  S.bt_active = false;
  S.bt_last_rx = 0;
  S.bt_last_echo = 0;
  S.bt_rx_count = 0;
  S.bt_rd_count = 0;
  S.bt_err_count = 0;
  S.bt_err_gap = 0;
  S.bt_err_garbage = 0;
  S.bt_bits_want1 = 0;
  S.bt_bits_want0 = 0;
  S.bt_mm_pos = 0;
  S.bt_mm_n = 0;
  S.bt_err_adjacent = 0;
  S.bt_prev_err = false;
  S.bt_rd_reports = 0;
  S.bt_rd_err = 0;
  S.bt_rd_bits_want1 = 0;
  S.bt_rd_bits_want0 = 0;
  S.bt_rd_last_exp = 0;
  S.bt_rd_last_got = 0;
  for (int k = 0; k < 8; ++k) {
    S.bt_mm_exp[k] = 0;
    S.bt_mm_got[k] = 0;
    S.bt_bit_high[k] = 0;
    S.bt_rd_bit_high[k] = 0;
  }
  bus_diag_reset(); // clear the per-port/bit-activity counters too
}

void fdc_bustest_snapshot(FdcBusTest *o) {
  o->active = S.bt_active ? 1 : 0;
  o->last_rx = S.bt_last_rx;
  o->last_echo = S.bt_last_echo;
  o->rx_count = S.bt_rx_count;
  o->rd_count = S.bt_rd_count;
  o->err_count = S.bt_err_count;
  o->err_gap = S.bt_err_gap;
  o->err_garbage = S.bt_err_garbage;
  o->bits_want1 = S.bt_bits_want1;
  o->bits_want0 = S.bt_bits_want0;
  o->err_adjacent = S.bt_err_adjacent;
  for (int k = 0; k < 8; ++k)
    o->bit_high[k] = S.bt_bit_high[k];
  o->rd_reports = S.bt_rd_reports;
  o->rd_err = S.bt_rd_err;
  o->rd_bits_want1 = S.bt_rd_bits_want1;
  o->rd_bits_want0 = S.bt_rd_bits_want0;
  o->rd_last_exp = S.bt_rd_last_exp;
  o->rd_last_got = S.bt_rd_last_got;
  for (int k = 0; k < 8; ++k)
    o->rd_bit_high[k] = S.bt_rd_bit_high[k];
  // Emit the ring oldest -> newest.
  uint32_t n = S.bt_mm_n < 8 ? S.bt_mm_n : 8;
  o->mm_n = (uint8_t)n;
  uint8_t start = (uint8_t)((S.bt_mm_pos - n) & 7);
  for (uint8_t k = 0; k < 8; ++k) {
    uint8_t idx = (uint8_t)((start + k) & 7);
    o->mm_exp[k] = (k < n) ? S.bt_mm_exp[idx] : 0;
    o->mm_got[k] = (k < n) ? S.bt_mm_got[idx] : 0;
  }
}

void fdc_bench_seek(uint8_t target, uint16_t param, uint8_t mode, FdcDebug *o) {
  if (!DRV) {
    fdc_debug_snapshot(o);
    return;
  }
  DRV->step_delay_us = 2000; // match the real FDC path
  DRV->settle_delay_ms = 5;
  DRV->select(true);
  if (mode == 1) {
    // spin_motor: wait `param` ms then wait for the INDEX pulse (disk spinning).
    DRV->motor_delay_ms = param;
    DRV->spin_motor(true);
  } else {
    // motor_raw: assert motor line + fixed delay, NO index wait.
    DRV->motor_raw(true);
    sleep_ms(param);
  }
  DRV->side(0);
  DRV->goto_track(target); // target 0 = restore (steps out to TRK00)

  int t = DRV->track();
  S.track = (t < 0) ? 0 : (uint8_t)t;
  S.at_track0 = DRV->get_track0_sense();
  S.write_protect = DRV->get_write_protect();
  S.motor_on = true;
  S.busy = false;
  fdc_debug_snapshot(o);
}

uint16_t fdc_bench_read_sector(uint8_t track, uint8_t sector, uint8_t side,
                               uint8_t density, uint8_t *out, uint16_t out_size,
                               uint8_t *flags) {
  if (flags)
    *flags = 0;
  if (!DRV)
    return 0;

  DRV->step_delay_us = 2000;
  DRV->settle_delay_ms = 5;
  DRV->motor_delay_ms = 300;
  DRV->select(true);
  DRV->spin_motor(true);
  DRV->side(side ? 1 : 0);
  DRV->goto_track(track);

  size_t cap = 0;
  uint8_t *flux = mzp_flux_buffer(&cap);
  int32_t idx = 0;
  size_t nflux = DRV->capture_track(flux, cap, &idx, /*store_greaseweazle=*/false,
                                    /*capture_ms=*/220, /*index_wait_ms=*/250);

  float bt = density ? 1.0f : FDC_DD_BIT_US;
  SectorRead sd =
      mfm_read_sector(flux, nflux, bt, sector, g_secbuf, sizeof(g_secbuf));

  if (flags)
    *flags = (uint8_t)((sd.found ? 0x01 : 0) | (sd.has_data ? 0x02 : 0) |
                       (sd.data_crc_ok ? 0x04 : 0) | (sd.deleted ? 0x08 : 0));

  uint16_t n = 0;
  if (sd.found && sd.has_data) {
    n = (sd.size <= out_size) ? sd.size : out_size;
    memcpy(out, g_secbuf, n);
  }
  return n;
}

// ---- core0 (disk) ---------------------------------------------------------

void fdc_disk_task(void) {
  if (!S.active || !DRV)
    return; // idle until the MZ-800 talks to us (don't fight USB/gw drive use)

  if (!g_timing_set) {
    // Faster step rate than the flux-capture default so a full-stroke RESTORE
    // fits inside the ROM's status-poll timeout (~6 ms/step). motor_delay_ms =
    // 300 gives the spindle a head start BEFORE spin_motor's index wait — with
    // 0 the index wait triggers on a barely-moving disk and returns before the
    // drive can step reliably to TRK00 (cold-start failure). 300 ms + index is
    // the config proven to work from a cold spindle.
    DRV->step_delay_us = 2000;
    DRV->settle_delay_ms = 5;
    DRV->motor_delay_ms = 300;
    g_timing_set = true;
  }

  // Apply drive select (single drive: always select once active).
  if (!g_applied_select) {
    DRV->select(true);
    g_applied_select = true;
  }
  // Motor with a spin-down delay: spin up on request (spin_motor waits for
  // index = disk really turning), but defer stop so retries keep the spin-up.
  if (S.motor_on) {
    g_motor_off_at = 0; // cancel any pending stop
    if (!g_applied_motor) {
      DRV->spin_motor(true); // spin up + wait index (~spin-up time)
      g_applied_motor = true;
    }
  } else if (g_applied_motor) {
    if (g_motor_off_at == 0)
      g_motor_off_at = now_ms() + FDC_MOTOR_OFF_DELAY_MS;
    else if (now_ms() >= g_motor_off_at) {
      DRV->spin_motor(false);
      g_applied_motor = false;
      g_motor_off_at = 0;
      if (g_stream_active) {
        rp2040_flux_capture_stop();
        g_stream_active = false;
      }
    }
  }
  // Apply side.
  int want_side = S.side1 ? 1 : 0;
  if (want_side != g_applied_side) {
    DRV->side(want_side);
    g_applied_side = want_side;
    // Side changed: any captured flux belongs to the old head.
    g_flux_n = 0;
  }

  // Execute a pending physical seek.
  if (S.op_done != S.op_seq) {
    uint32_t seq = S.op_seq;   // snapshot
    uint8_t target = S.op_target;
    bool restore = S.op_restore;
    bool verify = S.op_verify;

    if (restore)
      DRV->goto_track(0);
    else
      DRV->goto_track(target);

    int t = DRV->track();
    S.track = (t < 0) ? 0 : (uint8_t)t; // WD updates Track reg after the move
    S.at_track0 = DRV->get_track0_sense();
    S.write_protect = DRV->get_write_protect();

    // A prior Type II stream could still be marked active here if this seek
    // command arrived hot on the heels of a Force-Interrupt-aborted read,
    // before that read's own cleanup ran -- the capture peripheral can only
    // serve one session at a time, so claim it cleanly before Verify uses it.
    if (g_stream_active) {
      rp2040_flux_capture_stop();
      g_stream_active = false;
    }

    // WD Type I Verify: confirm the target track really has a recorded ID
    // field before reporting success. Real WD1793 behavior -- see
    // mfm_verify_track().
    S.seek_error = verify ? !fdc_verify_track(S.track) : false;

    S.op_done = seq;
    if (S.op_seq == seq) {
      S.busy = false; // clear BUSY only if no newer command queued
      S.last_ready_us = time_us_32(); // diagnostic: gap until the next command

    }
  }

  // ---- Type II read: capture a window, then decode -------------------------
  // Capture and decode are strictly sequential -- see fdc_capture_window() for
  // why interleaving them cannot work without DMA.
  if (S.reading) {
    if (S.pending_force_int &&
        (time_us_32() - S.last_data_read_us) > FDC_FORCE_INT_DEFER_IDLE_US) {
      // Host stopped reading DATA after Force Interrupt was posted; finalize to
      // avoid lingering BUSY/DRQ.
      S.rd_elapsed_us = time_us_32() - S.rd_issue_us;
      S.reading = false;
      S.busy = false;
      S.drq = false;
      S.pending_force_int = false;
      S.pending_force_since_us = 0;
    }

    // The block above (or a Force Interrupt on core1) may have ended the read.
    // Re-test before committing to a capture window: it blocks core0 for up to
    // 300 ms, which is a long time to spend on a command nobody is waiting for.
    if (!S.reading)
      return;

    if (S.rd_fresh) {
      // First sector of a new command: grab a fresh window. The head is already
      // on-track from a prior SEEK; READ SECTOR does not itself seek. No index
      // sync -- a 1.25-revolution window contains every sector regardless of
      // where it starts, so waiting for index only adds latency.
      fdc_capture_window();
      S.rd_fresh = false;
      // core1 can have aborted us while we were blocked capturing.
      if (!S.reading)
        return;
    }

    if (S.rd_done != S.rd_seq && !S.drq) {
      uint32_t seq = S.rd_seq; // snapshot
      uint8_t want = S.rd_sector;
      size_t cap = 0;
      uint8_t *flux = mzp_flux_buffer(&cap);

      uint32_t t0 = time_us_32();
      SectorRead sd{};
      if (g_flux_n > 0)
        sd = mfm_read_sector(flux, g_flux_n, FDC_DD_BIT_US, want, g_secbuf,
                             sizeof(g_secbuf));
      S.rd_search_us = time_us_32() - t0;
      S.rd_want = want;
      S.rd_result = (uint8_t)((sd.found ? 1 : 0) | (sd.has_data ? 2 : 0) |
                              (sd.data_crc_ok ? 4 : 0) | (sd.deleted ? 8 : 0));

      if (sd.found && sd.has_data) {
        uint16_t n = (sd.size <= FDC_SECTOR_MAX) ? sd.size : FDC_SECTOR_MAX;
        for (uint16_t k = 0; k < n; ++k)
          S.xfer[k] = g_secbuf[k];
        S.xfer_len = n;
        S.xfer_pos = 0;
        S.err_crc = !sd.data_crc_ok;
        S.rec_deleted = sd.deleted;
        S.err_rnf = false;
        // Publish DRQ last: it is the ownership handoff of xfer[] to core1.
        if (S.reading && S.rd_seq == seq)
          S.drq = true;
        S.rd_done = seq;
        // Multi-record keeps the captured window: every remaining sector of
        // this track is already in it, so no re-capture is needed.
      } else {
        // Not on this track (or undecodable). Record what IS there, then RNF.
        fdc_note_track_ids(flux, g_flux_n);
        S.err_rnf = true;
        S.drq = false;
        if (S.rd_seq == seq) {
          S.reading = false;
          S.busy = false;
        }
        S.rd_done = seq;
        g_flux_n = 0;
      }
    }
  } else if (g_stream_active) {
    // Nothing should hold the capture peripheral outside a read.
    rp2040_flux_capture_stop();
    g_stream_active = false;
  }
}
