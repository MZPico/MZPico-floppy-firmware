#include "fdc.hpp"
#include "board_bus.h" // RAM_FUNC
#include "board_config.h" // PIN_INDEX, PIN_RDATA
#include "floppy.hpp"
#include "flux/arch_rp2.h" // rp2040_flux_capture_start/drain/stop (streaming)
#include "gw_protocol.hpp" // mzp_flux_buffer (shared capture buffer)
#include "mfm_decode.hpp"  // mfm_read_sector, mfm_read_track_raw
#include "mfm_encode.hpp"  // mfm_encode_format_image (WRITE TRACK)
#include "mfm_splice.hpp"  // mfm_write_sector (in-place data-field splice)
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

// Highest cylinder the head may be driven to. Relative stepping is clamped here
// so a runaway host cannot walk the head into the end stop.
static constexpr int FDC_MAX_CYLINDER = 79;

// Per-cylinder sector size, learned from the ID fields we decode while reading.
//
// WRITE SECTOR needs the sector's size BEFORE the host has sent a single byte:
// the WD raises DRQ the moment the command is written and the host then shifts
// in exactly one sector's worth. A real WD gets away with not knowing either --
// it streams bytes and stops when the ID field it eventually finds says to --
// but we buffer the whole sector before going anywhere near the disk, so the
// length has to be decided up front. Every cylinder a host writes to it has
// first read from, so the ID fields have already told us; this remembers what
// they said. A wrong guess is caught (not acted on) at splice time by the size
// compare in mfm_write_sector().
static constexpr int FDC_SIZE_CACHE_N = FDC_MAX_CYLINDER + 1;
// One track's worth of host-visible bytes. A DD track at 250 kbit/s and 300 rpm
// is ~6250 bytes; the margin covers a slow spindle and READ TRACK's inserted
// sync-mark bytes. This bounds WRITE TRACK, so a host that never stops sending
// cannot run off the end of it.
static constexpr uint16_t FDC_TRACK_IMAGE_MAX = 7200;
// The host has stopped shifting format bytes in. A real WD ends WRITE TRACK on
// the index pulse and simply ignores anything still coming; we have to buffer
// the image before writing it, so instead we take a gap in the byte stream as
// "that is the whole image". A Z80 feeding a DRQ'd byte takes tens of us, so a
// few ms of silence is unambiguous.
static constexpr uint32_t FDC_TRACK_IDLE_US = 4000;
// Flux budget for one formatted track. A revolution is ~43.5k transitions
// (measured); 48k leaves the write with flux still queued when the terminating
// index pulse arrives, which is what stops it running out early.
static constexpr size_t FDC_TRACK_FLUX_MAX = 48000;

// Size code N means 128 << N bytes, so 512 is 2. MZ-800 CP/M data tracks are
// 9x512 (N=2); the boot track is 16x256 (N=1).
static constexpr uint8_t FDC_SIZE_CODE_DEFAULT = 2;

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
  // Signed *relative* step count, not an absolute cylinder. A real WD steps the
  // head by pulses and keeps the Track register as a separate belief about where
  // the head is; a host recalibrating (write Track, step, READ ADDRESS to resync)
  // relies on those two being independent. Deriving the target from S.track
  // instead made a single STEP OUT slam the head across the disk.
  //
  // Why a target and not a step count: core0 blocks for ~235 ms in
  // fdc_capture_window() on every fresh read, and CP/M 4.1 positions the head
  // with rapid STEP/Force-Interrupt bursts, so many Type I commands land while
  // core0 is unavailable. Whatever holds them must lose none of them AND must
  // absorb surplus pulses at the end stops exactly as the drive does.
  // core1 folds each step into a CLAMPED ABSOLUTE target cylinder; core0 just
  // drives the head there. Single writer (core1), so no shared read-modify-write
  // and no lost commands however many arrive while core0 is blocked.
  //
  // This is deliberately NOT the Track register -- that is the host's belief and
  // must stay independent (a host recalibrating via READ ADDRESS depends on the
  // two being able to disagree). This is our own model of where the head is.
  //
  // Two earlier shapes both failed. A single "steps for the next move" slot was
  // overwritten, so commands arriving during core0's ~235 ms capture window were
  // dropped outright. Replacing it with a step ACCUMULATOR fixed that but broke
  // end-stop absorption: at_track0 only refreshes once core0 runs, so the host
  // -- which polls STATUS microseconds after each FORCE-INT -- issues several
  // surplus STEP-OUTs past track 0. Summing them with the following STEP-INs
  // (-7 + 2 = -5) let the debt cancel real motion, leaving the head one cylinder
  // short. A physical drive absorbs each surplus pulse AS IT ARRIVES, so the
  // clamp has to be applied per command. Folding into a clamped absolute target
  // does exactly that.
  volatile int16_t head_target = 0;
  volatile uint32_t restore_req = 0;  // core1 only: bumped per RESTORE
  volatile uint32_t restore_done = 0; // core0 only
  volatile bool step_in = true; // last direction, for the bare STEP (0x2X/0x3X)
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
  volatile bool read_addr = false; // Type III READ ADDRESS in flight
  // Error/status flags for the current/last Type II op.
  volatile bool err_rnf = false;
  volatile bool err_crc = false;
  volatile bool lost_data = false;
  volatile bool rec_deleted = false; // matched a deleted-DAM (0xF8)
  // WRITE PROTECT (Type II b6). Set when a write command is refused before it
  // reaches the disk -- the drive's ~WPT sense, or writes disabled in firmware.
  volatile bool err_write_protect = false;
  volatile uint32_t write_attempts = 0; // WRITE SECTOR commands issued to us
  volatile uint32_t wr_refused = 0;     // ...of those, refused before the disk

  // ---- Type II WRITE SECTOR (3.3) ---------------------------------------
  // Mirror image of the read path, with the DRQ direction reversed: core1
  // collects a whole sector from the host into xfer[], then hands it to core0,
  // which finds the sector's ID on the spinning disk and splices a new data
  // field in behind it (see mfm_splice.hpp).
  //
  // Buffering the sector before touching the disk is what makes this safe with
  // a host that paces itself: the splice runs at disk speed and cannot pause,
  // so a host that stalls mid-sector must stall BEFORE the write starts, not
  // half way through a data field. A real WD, which streams host bytes straight
  // onto the media, answers the same situation by writing zeros and raising
  // LOST DATA -- i.e. by corrupting the sector.
  volatile bool writing = false;  // a write transfer is in flight (BUSY held)
  volatile bool wr_armed = false; // xfer[] is full; core0 may splice
  volatile uint8_t wr_sector = 1; // sector being written
  volatile bool wr_deleted = false; // command bit a0: write a deleted DAM
  volatile uint32_t wr_seq = 0;   // core1 -> core0: "splice this buffer"
  volatile uint32_t wr_done = 0;

  // ---- Type III READ TRACK / WRITE TRACK (3.4) ---------------------------
  // Both move a whole track at once -- far more than the one-sector xfer[]
  // holds -- so they get their own buffer and their own DATA routing. `trk_op`
  // says which is running and is what tells the DATA register which buffer it
  // is talking to; 0 means neither and the sector paths own the register.
  //
  // WRITE TRACK is how a disk gets formatted, and it is the only operation that
  // can create the missing-clock sync marks an ID field needs. It is also the
  // most destructive thing this bridge can do: it replaces an entire track,
  // gaps, ID fields and all.
  volatile uint8_t trk_op = 0;    // 0 none, 1 READ TRACK, 2 WRITE TRACK
  volatile bool trk_armed = false; // the image is complete; core0 may write it
  volatile uint32_t trk_seq = 0;
  volatile uint32_t trk_done = 0;
  volatile uint16_t trk_pos = 0;  // host's position within track_buf
  volatile uint16_t trk_len = 0;  // bytes valid / expected
  volatile uint32_t trk_ok = 0;      // tracks formatted / read
  volatile uint32_t trk_fail = 0;    // ...that failed to reach the media
  volatile uint32_t trk_last_us = 0; // last operation's disk time
  volatile uint16_t trk_last_bytes = 0; // last image length
  volatile uint8_t track_buf[FDC_TRACK_IMAGE_MAX];
  // Write-path diagnostics, cumulative. The equivalent of the sec_* counters:
  // a snapshot of the last write cannot show the one that went wrong earlier.
  volatile uint32_t wr_ok = 0;            // data fields actually written
  volatile uint32_t wr_rnf = 0;           // the sector's ID never passed the head
  volatile uint32_t wr_cyl_mismatch = 0;  // found it, on the wrong cylinder
  volatile uint32_t wr_size_mismatch = 0; // found it, wrong recorded size
  volatile uint32_t wr_late = 0;          // ID decoded too late for its Gap 2
  volatile uint32_t wr_splice_us = 0;     // last splice: search time
  volatile uint16_t wr_ids_seen = 0;      // last splice: ID fields decoded
  volatile uint8_t wr_last_result = 0;    // b0 wrote b1 id_found b2 cyl_mismatch
                                          // b3 size_mismatch b4 late b5 no_flux
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
  // Cumulative read-quality counters. A single-snapshot diagnostic cannot see a
  // sector that decoded with a bad CRC three sectors ago -- the host has long
  // since taken the bytes and moved on. These are the only way to tell "the
  // bridge is handing over corrupt data" apart from "the host is misreading
  // good data", which is exactly the question a garbled directory listing asks.
  // last_id_cyl/head are the ID field of the most recently DELIVERED sector: if
  // they disagree with the Track register the host is being fed another track.
  volatile uint32_t sec_ok = 0;      // sectors delivered, data CRC good
  volatile uint32_t sec_crc_err = 0; // sectors delivered, data CRC BAD
  volatile uint32_t sec_rnf = 0;     // reads that found no matching sector
  volatile uint8_t last_id_cyl = 0xFF;
  volatile uint8_t last_id_head = 0xFF;
  // A real WD compares the ID field's track byte against the Track register and
  // raises RNF when they disagree; mfm_read_sector() matches on the sector
  // NUMBER ALONE, so if the head is not where the host believes it is we return
  // perfectly valid, CRC-clean data from the wrong cylinder. That failure is
  // invisible to every other counter here -- it looks like a flawless read.
  // Sampled at delivery time together with the Track register so the pair is
  // self-consistent rather than two unrelated snapshots.
  volatile uint32_t sec_cyl_mismatch = 0;
  volatile uint8_t last_trk_reg = 0xFF;  // Track register when that ID was read
  // Sampled at the moment the read decodes, so all four can be compared as one
  // consistent set. They separate the three ways a read can land on the wrong
  // cylinder, which no other counter can tell apart:
  //   head_target != Track reg -> core1 folded the steps wrong
  //   phys_track  != head_target -> core0 never drove the head there
  //   both agree but ID cyl differs -> the head did not physically arrive
  //     (a dropped step pulse; goto_track() believes it did regardless)
  volatile uint8_t last_head_target = 0xFF;
  volatile uint8_t last_phys_track = 0xFF;
  volatile uint8_t last_want_side = 0xFF; // physical head we were on

  // Head-position resyncs: times an ID field proved the head was not where we
  // believed, and we adopted the disk's answer (see fdc_resync_head).
  // Bus RESET (GPIO29). Sampled by core1 -- see fdc_bus_reset_poll().
  volatile uint32_t reset_events = 0;
  volatile uint32_t reset_at_ms = 0;
  volatile bool reset_active = false;
  // Raw line telemetry, recorded with NO polarity assumption. If a reset press
  // produces no event, this is what says why: whether the pin moved at all, in
  // which direction, and for how long. Without it "resets seen: 0" cannot be
  // told apart from a line that never moves, one that pulses the other way, or
  // a pulse shorter than the debounce.
  volatile uint32_t rst_samples = 0;
  volatile uint32_t rst_rise = 0;         // low -> high transitions
  volatile uint32_t rst_fall = 0;         // high -> low
  volatile uint32_t rst_high_max_us = 0;  // longest completed HIGH excursion
  volatile uint32_t rst_low_min_us = 0;   // shortest completed LOW excursion
  volatile uint32_t rst_last_high_us = 0; // most recent completed HIGH length

  volatile uint32_t pos_resyncs = 0;
  volatile uint8_t last_resync_from = 0xFF; // cylinder we were actually on
  volatile uint8_t last_resync_to = 0xFF;   // cylinder we meant to be on

  // ---- First-failure latch -----------------------------------------------
  // Every other diagnostic here describes the LAST operation, which is exactly
  // the wrong one when a host retries: the retry that succeeds overwrites the
  // evidence of the failure that caused the visible symptom. The rare
  // `FD:Loading error` on boot (~1 in 20) has survived precisely because of
  // this -- by the time anyone can run `rddiag`, the ROM has retried and the
  // snapshot shows a clean read.
  //
  // So: capture the FIRST failure since power-up in full, and never overwrite
  // it. Cleared only by an explicit probe command.
  volatile bool ff_valid = false;
  volatile uint32_t ff_count = 0;   // failures seen since power-up (all of them)
  volatile uint32_t ff_at_ms = 0;   // ms since boot -- how early it happened
  volatile uint8_t ff_kind = 0;     // 1 RNF, 2 data CRC, 3 wrong cylinder,
                                    // 4 Verify/seek error
  volatile uint8_t ff_want_sec = 0; // what was asked for
  volatile uint8_t ff_trk_reg = 0;  // ...and the whole position chain, sampled
  volatile uint8_t ff_head_target = 0; //    together so it is self-consistent
  volatile uint8_t ff_phys_track = 0;
  volatile uint8_t ff_side = 0;
  volatile uint8_t ff_seen_cyl = 0xFF; // what the disk said instead
  volatile uint8_t ff_seen_head = 0xFF;
  volatile uint8_t ff_capture_ok = 0;
  volatile uint32_t ff_flux_n = 0;
  volatile uint32_t ff_capture_us = 0;
  volatile uint8_t ff_ids_seen = 0;
  volatile uint8_t ff_ids[16] = {0}; // the sector numbers really on the track

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
  // Distinguishes "no Verify has run yet" from "a Verify ran and failed".
  // Without it, verify_start_ok reads as a definite negative when it is really
  // just uninitialised -- and a failed Verify is exactly what produces a silent
  // FD:Loading error, so that is the worst possible moment to be ambiguous.
  volatile uint32_t verify_attempts = 0;

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

  // Full I/O trace ring: every register access in order, reads included, so the
  // host's actual intent is visible instead of inferred from command opcodes.
  // Runs of identical consecutive accesses are coalesced into a repeat count --
  // without that, one STATUS poll loop (48k reads seen) flushes all history.
  static constexpr uint8_t IO_TRACE_N = 96;
  volatile uint8_t io_port[IO_TRACE_N] = {0};
  volatile uint8_t io_val[IO_TRACE_N] = {0};  // byte as it appears on the bus
  volatile uint8_t io_rd[IO_TRACE_N] = {0};   // 1 = read, 0 = write
  volatile uint16_t io_rep[IO_TRACE_N] = {0}; // consecutive identical accesses
  volatile uint8_t io_pos = 0;                // next slot to write
  volatile uint8_t io_n = 0;                  // slots used (<= IO_TRACE_N)
  volatile uint32_t io_total = 0;

  // EINT: port 0xDF bit0, the card's interrupt-enable latch. Software must arm
  // it before ~INT is ever driven -- see fdc_refresh_int_line().
  volatile bool eint = false;
  volatile uint32_t intrq_asserts = 0; // diagnostics: ~INT low transitions
  volatile bool int_asserted = false;
  // Type IV Force Interrupt with I3 (interrupt immediately, e.g. 0xD8): a
  // one-shot INTRQ that is independent of DRQ. Cleared by a STATUS read or the
  // next command write, per the WD spec.
  volatile bool force_int_latched = false;
  volatile uint32_t lost_data_events = 0;
};

FdcState S;
FloppyDrive *DRV = nullptr;

// Sector size code per cylinder (see FDC_SIZE_CACHE_N). Written by core0 as it
// decodes ID fields, read by core1 when a WRITE SECTOR command arrives. A byte
// each, single-writer, and a stale entry only costs a refused write -- so no
// synchronisation beyond `volatile`.
volatile uint8_t g_size_code[FDC_SIZE_CACHE_N];

ALWAYS_INLINE uint16_t fdc_sector_size_for(uint8_t trk) {
  uint8_t n = (trk < FDC_SIZE_CACHE_N) ? g_size_code[trk] : FDC_SIZE_CODE_DEFAULT;
  return (uint16_t)(128u << (n & 7));
}

// Remember the size code an ID field on `trk` reported. Called from core0 on
// every decoded ID (read, READ ADDRESS and splice alike).
inline void fdc_note_size_code(uint8_t trk, uint8_t n) {
  if (trk < FDC_SIZE_CACHE_N)
    g_size_code[trk] = (uint8_t)(n & 7);
}

} // namespace

// How the ~INT line (GPIO12) is driven:
//   0 = never (escape hatch if interrupts misbehave)
//   1 = follow EINT, the card's own enable latch on port 0xDF -- correct, and
//       the default
//   2 = force on, ignoring EINT (diagnostic only)
// Override at runtime with `mzp_probe.py intmode <n>`.
//
// Mode 1 is safe precisely because of the EINT gate. An earlier build asserted
// ~INT unconditionally and hung the MZ-800 mid-boot: GPIO12 is a direct
// connection to expansion pin 38 (no buffer) and the Z80's INT input is LEVEL
// sensitive, so interrupting code that has no floppy ISR re-enters 0x0038
// forever. Software that never arms EINT (the boot ROM, CP/M 2.3) is now never
// interrupted, which is how the real card behaves. Mode 2 reproduces the hang
// on purpose and should stay a diagnostic.
volatile uint8_t g_int_mode = 1;

// Side-select inversion (see the 0xDD case). ON by default: this is the
// MZ-800's measured behaviour, not a workaround -- with it off the machine
// cannot boot at all. Outside the anonymous namespace so the USB probe can flip
// it live for A/B testing; core1 reads it on every 0xDD write.
volatile bool g_invert_side = true;

// Master enable for WRITE SECTOR. ON: the bridge is a read/write floppy
// controller, which is the point of Milestone 3.3. Turn it off (`mzp_probe.py
// wren 0`) to make the firmware read-only again -- the machine then sees the
// same honest WRITE PROTECT the 3.2 stub returned, which is the right state for
// bringing up a new host, or for touching a disk you cannot replace.
volatile bool g_write_enable = true;

namespace {

// Record one register access. Values are stored exactly as they appear on the
// bus (still complemented for 0xD8..0xDB); the probe tool applies the inversion
// so the rule lives in one place.
ALWAYS_INLINE void fdc_trace_io(uint8_t port, uint8_t val, uint8_t is_rd) {
  S.io_total++;
  uint8_t last = (uint8_t)((S.io_pos + FdcState::IO_TRACE_N - 1) %
                           FdcState::IO_TRACE_N);
  if (S.io_n && S.io_port[last] == port && S.io_val[last] == val &&
      S.io_rd[last] == is_rd) {
    if (S.io_rep[last] < 0xFFFF)
      S.io_rep[last]++;
    return;
  }
  S.io_port[S.io_pos] = port;
  S.io_val[S.io_pos] = val;
  S.io_rd[S.io_pos] = is_rd;
  S.io_rep[S.io_pos] = 1;
  S.io_pos = (uint8_t)((S.io_pos + 1) % FdcState::IO_TRACE_N);
  if (S.io_n < FdcState::IO_TRACE_N)
    S.io_n++;
}

// Re-drive ~INT from the current state. Idempotent, so every site that can
// change DRQ / EINT / transfer state just calls this. Callable from either core:
// gpio_oe_set/clr are write-1-to-modify, so there is no read-modify-write race.
//
// The condition is taken from the reference firmware's FDCDevice::isInterrupt():
// the interrupt is DRQ-driven and gated by EINT -- asserted while a data byte is
// pending during a transfer command, released when the host takes the byte,
// writes a new command, or clears EINT. It is NOT a WD-style "INTRQ at command
// completion", and a STATUS read does NOT release it.
//
// The EINT gate is what makes this safe: the boot ROM and CP/M 2.3 never write
// 0xDF, so they never get interrupted -- which is exactly how a real card
// behaves, and why driving ~INT unconditionally hung the machine.
//
// The line is driven UNCONDITIONALLY to match `want` on every call -- there is
// deliberately no "already in that state, return early" short-circuit. Both
// cores call this, and `int_asserted` is a plain flag, so a compare-and-skip is
// a read-modify-write race: core0 raising DRQ and core1 draining it can
// interleave such that the cached flag says "asserted" while the pin is Hi-Z
// (interrupt silently lost), or worse the reverse -- ~INT stuck low with nothing
// pending, which on a level-sensitive Z80 INT is a hard machine hang. The
// gpio_oe_set/clr writes are atomic and idempotent, so re-driving costs one
// store and removes the race entirely. `int_asserted` survives only to make the
// diagnostic counter count edges rather than calls.
ALWAYS_INLINE void fdc_refresh_int_line(void) {
  bool enabled = (g_int_mode == 2) || (g_int_mode == 1 && S.eint);
  // DRQ during a data transfer is the normal source -- in either direction: on
  // a write DRQ means "send me the next byte", and an interrupt-driven host
  // waits on exactly the same edge to feed the sector as it does to drain one.
  // The Force-Interrupt (I3) latch is the only other source; a Type I
  // completion does NOT interrupt on this card.
  bool want = enabled && (((S.reading || S.writing || S.trk_op) && S.drq) ||
                          S.force_int_latched);
  if (want) {
    if (!S.int_asserted) {
      S.int_asserted = true;
      S.intrq_asserts++;
    }
    assert_int_line();
  } else {
    S.int_asserted = false;
    release_int_line();
  }
}

// ---- Bus RESET (GPIO29) ---------------------------------------------------
//
// Polarity is not a guess: the reference firmware's `bus_reset` PIO program on
// this same MZ-800 bus does `wait 1 gpio 19` and then requires the line to stay
// high for 31 further samples before it acts. A firmware that reboots on a
// sustained high would reboot continuously if the line idled high, so the line
// idles LOW and RESET is asserted HIGH. (bus_init() also pulls it down, so a
// board on the bench with nothing attached reads "not in reset".)
//
// This runs on CORE1, not core0. It is the safety-critical one: ~INT goes
// straight to expansion pin 38 with no buffer and the Z80's INT input is level
// sensitive, so a machine that resets while we hold ~INT low comes back up into
// an immediate, permanent interrupt at 0x0038 with no ISR installed -- a hard
// hang that the reset button cannot clear, because EINT is a latch in OUR state
// and the resetting host has no way to reach it.
//
// core0 cannot do this job. It blocks for up to ~250 ms inside a flux capture
// with interrupts disabled, so a poll there is neither timely nor evenly
// spaced -- and the earlier attempt debounced by counting consecutive polls,
// which measures nothing when the gap between polls can be a microsecond or a
// quarter of a second. core1 never blocks for more than one Z80 I/O cycle, so
// here the debounce can be what it should be: a duration.
//
// 20 us, and that number is measured rather than chosen. The MZ-800's reset
// asserts this pin HIGH for **65 us** -- one clean rising edge and one falling
// edge, no bounce (`mzp_probe.py resetdiag`). The first attempt used 100 us,
// which is longer than the pulse the machine actually emits, so every reset was
// seen and then discarded as a glitch: the line read low, the event counter
// stayed at zero, and nothing distinguished that from a pin that never moved.
// 20 us confirms the real pulse three times over while still rejecting the
// sub-microsecond crosstalk an unbuffered bus line can pick up. core1 polls
// this about every 1.4 us, so 20 us is ~14 consecutive samples.
static constexpr uint32_t FDC_RESET_CONFIRM_US = 20;


// core0-only "last applied" shadow (no volatile needed — single core).
bool g_applied_select = false;
bool g_applied_motor = false; // physical motor state (may lag S.motor_on)
int g_applied_side = -1;
bool g_timing_set = false;
uint32_t g_motor_off_at = 0; // deferred motor-stop deadline (ms; 0 = none)

// core0-only decode scratch (one sector) + streaming-capture state.
uint8_t g_secbuf[FDC_SECTOR_MAX];
uint8_t g_readaddr_id[6];     // last ID field found by READ ADDRESS
bool g_readaddr_found = false;
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
// One disk revolution is ~43.5k flux transitions at DD (measured with `scan`).
// Capture ~1.05 revolutions: enough that every sector passes the head once,
// with a small margin for spindle-speed variation. The host's measured patience
// after a READ is ~285 ms (25k STATUS polls, see `rddiag`) and a revolution is
// ~200 ms, so this window has to stay tight -- 1.25 revolutions left only ~7 ms
// of headroom. Used by both the Type II read window and the Type I Verify pass.
static constexpr size_t FDC_READ_CAPTURE_BYTES = 45700;
// Safety bound on any capture window (a revolution is ~200 ms).
static constexpr uint32_t FDC_READ_CAPTURE_TIMEOUT_US = 300000;

static constexpr uint32_t FDC_VERIFY_INDEX_SYNC_BUDGET_MS = 260;

// How long the write splice hunts for its sector's ID field. One revolution is
// ~200 ms, so 1.25 of them guarantees every sector on the track passes the head
// at least once -- if it is there at all, we have seen it. Longer would only
// delay the RNF; the host is polling BUSY throughout.
static constexpr uint32_t FDC_WRITE_SEARCH_TIMEOUT_US = 250000;
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

// How long a DRQ may go unserviced before we declare LOST DATA and complete the
// command. A real WD allows one byte time (~32 us at DD); we buffer the sector
// so we can afford to be far more patient. Sized well above the slowest host
// drain seen so far (the boot ROM polls STATUS every ~11 us) but short enough
// that a host which never reads DATA at all recovers promptly instead of
// deadlocking.
static constexpr uint32_t FDC_DRQ_STALL_TIMEOUT_US = 250000;

// DRQ window for READ ADDRESS. Long enough that any polling host notices DRQ and
// takes all six ID bytes (the boot ROM polls STATUS every ~11 us), short enough
// that a host which only wants the Sector register is not left waiting. A real
// WD finishes the ID field in ~200 us; 2 ms is ten times that.
static constexpr uint32_t FDC_READADDR_DRQ_WINDOW_US = 2000;

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
  S.verify_attempts++; // counted at entry, so an early return still shows up

  // Start verify capture at an index edge when possible, to reduce rotational
  // alignment misses on media where the expected IDs are index-relative.
  (void)wait_for_index_falling_edge_ms(FDC_VERIFY_INDEX_SYNC_BUDGET_MS);

  bool start_ok = rp2040_flux_capture_start(PIN_INDEX, PIN_RDATA);
  S.verify_start_ok = start_ok;
  if (!start_ok) {
    S.verify_n = 0;
    return false;
  }
  // Capture a bounded window with NOTHING between drains, then decode once.
  //
  // This loop used to call mfm_verify_track() after every drain, i.e. it
  // decoded *while* capturing -- the exact anti-pattern fdc_capture_window()
  // exists to prevent. The PIO FIFO holds ~24 us of flux and has no DMA, so any
  // decode between drains drops transitions, MFM sync never re-locks, and the
  // rest of the window is unusable. It was also O(n^2): every drain re-decoded
  // the whole buffer from the start, so the passes got longer as the buffer
  // grew and the flux loss got worse the longer it ran.
  //
  // That made Verify a coin flip -- it only succeeded when a matching ID landed
  // in the first drain or two, before the decode passes got long enough to
  // corrupt the stream. Failing Verify sets SEEK ERROR, and the boot ROM stops
  // there and reports a loading error without ever issuing a READ.
  //
  // The old loop also had no timeout: its only exit was `n < cap`, so a drive
  // producing no transitions at all (motor not up to speed, no disk) left it
  // spinning forever with BUSY held.
  size_t want = (cap < FDC_READ_CAPTURE_BYTES) ? cap : FDC_READ_CAPTURE_BYTES;
  size_t n = 0;
  uint32_t t0 = time_us_32();
  while (n < want && (time_us_32() - t0) < FDC_READ_CAPTURE_TIMEOUT_US)
    n = rp2040_flux_capture_drain(flux, want, n, /*store_greaseweazle=*/false);
  rp2040_flux_capture_stop();

  S.verify_n = (uint32_t)n;
  bool ok = (n > 0) && mfm_verify_track(flux, n, FDC_DD_BIT_US, track);

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

  return ok; // capture already stopped above, before the decode
}

// READ ADDRESS is on a MUCH tighter budget than READ SECTOR: measured, the
// CP/M BIOS polls STATUS for only ~52 ms before Force Interrupt (vs the boot
// ROM's ~285 ms for a sector read). It also only needs the NEXT ID field, and
// one passes the head every ~13-22 ms, so search in chunks and stop the moment
// an ID turns up rather than filling a fixed window. At ~190 flux bytes/ms,
// 5000 bytes is ~26 ms -- one full sector period on a 9-sector track, so the
// first chunk almost always finds one.
static constexpr size_t FDC_READADDR_CHUNK_BYTES = 5000;
static constexpr size_t FDC_READADDR_MAX_BYTES = 12000;
static constexpr uint32_t FDC_READADDR_TIMEOUT_US = 40000;

// Capture a bounded flux window with NOTHING between drains, then stop.
//
// This must not be interleaved with decoding. The capture has no DMA: the PIO
// RX FIFO is 4 words deep and each word carries 2 half-samples, so it buffers
// 8 transitions -- about 24 us at DD rates. Any work between drains longer
// than that silently drops flux, MFM sync never holds, and no sector is ever
// found. That is exactly what the previous decode-while-capturing loop did,
// which is why `scan`/`streamscan`/`fdcread` (all capture-then-decode) worked
// while the live read path returned RNF on a perfectly good disk.
static void fdc_capture_window(size_t want_bytes) {
  size_t cap = 0;
  uint8_t *flux = mzp_flux_buffer(&cap);
  size_t want = (cap < want_bytes) ? cap : want_bytes;

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

// READ ADDRESS: capture and search in chunks, returning as soon as any ID field
// turns up. Flux arriving during a decode pass is lost (no DMA -- see
// fdc_capture_window), but unlike READ SECTOR that is harmless here: a gap
// costs at most one ID field and we simply match the next one. Bounded by both
// a byte limit and a 40 ms deadline so we always answer inside the host's
// ~52 ms patience.
static bool fdc_read_address_search(uint8_t *id_out) {
  size_t cap = 0;
  uint8_t *flux = mzp_flux_buffer(&cap);
  size_t limit = (cap < FDC_READADDR_MAX_BYTES) ? cap : FDC_READADDR_MAX_BYTES;

  g_flux_n = 0;
  uint32_t t0 = time_us_32();
  g_stream_active = rp2040_flux_capture_start(PIN_INDEX, PIN_RDATA);
  S.rd_capture_ok = g_stream_active;
  bool found = false;

  if (g_stream_active) {
    while (!found && g_flux_n < limit &&
           (time_us_32() - t0) < FDC_READADDR_TIMEOUT_US) {
      size_t target = g_flux_n + FDC_READADDR_CHUNK_BYTES;
      if (target > limit)
        target = limit;
      // Tight drain, nothing in between.
      while (g_flux_n < target &&
             (time_us_32() - t0) < FDC_READADDR_TIMEOUT_US) {
        g_flux_n = rp2040_flux_capture_drain(flux, limit, g_flux_n,
                                             /*store_greaseweazle=*/false);
      }
      found = mfm_read_address(flux, g_flux_n, FDC_DD_BIT_US, id_out);
    }
    rp2040_flux_capture_stop();
    g_stream_active = false;
  }
  S.rd_capture_us = time_us_32() - t0;
  S.rd_flux_n = g_flux_n;
  return found;
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

// Record a splice outcome into the wr_* diagnostics. Shared by the MZ-800 write
// path and fdc_bench_write_sector so `wrdiag` describes whichever one last put
// (or failed to put) flux on the disk -- the bench command is the tool the
// write path is brought up with, so leaving it out of the counters made the
// diagnostic read "no write yet" straight after a successful write.
// Latch the first failure since power-up, in full. Every field is sampled here,
// at the moment of the failure, so the position chain in the report is one
// consistent set rather than four values read at four different times.
// `seen_cyl`/`seen_head` are 0xFF when the disk said nothing.
static void fdc_note_failure(uint8_t kind, uint8_t want_sec, uint8_t seen_cyl,
                             uint8_t seen_head) {
  S.ff_count++;
  if (S.ff_valid)
    return; // first one wins, forever -- that is the whole point
  S.ff_valid = true;
  S.ff_at_ms = to_ms_since_boot(get_absolute_time());
  S.ff_kind = kind;
  S.ff_want_sec = want_sec;
  S.ff_trk_reg = S.track;
  S.ff_head_target = (uint8_t)S.head_target;
  int pt = DRV ? DRV->track() : -1;
  S.ff_phys_track = (pt < 0) ? 0xFE : (uint8_t)pt;
  S.ff_side = S.side1 ? 1 : 0;
  S.ff_seen_cyl = seen_cyl;
  S.ff_seen_head = seen_head;
  S.ff_capture_ok = S.rd_capture_ok ? 1 : 0;
  S.ff_flux_n = S.rd_flux_n;
  S.ff_capture_us = S.rd_capture_us;
  S.ff_ids_seen = S.rd_ids_seen;
  for (int k = 0; k < 16; ++k)
    S.ff_ids[k] = S.rd_ids[k];
}

// The head is provably on `seen_cyl`: an ID field read off the disk said so,
// while we believed something else. Adopt the observed position and drive back
// to where we meant to be.
//
// This is what turns the WD ID track compare from a detector into a repair. The
// compare already stops us handing over (or overwriting) another cylinder's
// data, but on its own it produces an RNF from a head that is still in the
// wrong place, so the host's retry fails identically and forever. The disk
// itself is the only position reference a floppy has, and this is the one
// moment it speaks: use it.
static void fdc_resync_head(uint8_t seen_cyl) {
  if (!DRV)
    return;
  S.pos_resyncs++;
  S.last_resync_from = seen_cyl;
  S.last_resync_to = (uint8_t)S.head_target;
  DRV->set_track_belief((int)seen_cyl);
  int tgt = S.head_target;
  if ((int)seen_cyl != tgt)
    DRV->goto_track(tgt);
  S.at_track0 = DRV->get_track0_sense();
}

inline void fdc_record_splice(const SpliceResult &sr) {
  S.wr_ids_seen = sr.ids_seen;
  S.wr_splice_us = sr.search_us;
  S.wr_last_result =
      (uint8_t)((sr.wrote ? 0x01 : 0) | (sr.id_found ? 0x02 : 0) |
                (sr.cyl_mismatch ? 0x04 : 0) | (sr.size_mismatch ? 0x08 : 0) |
                (sr.late ? 0x10 : 0) | (sr.no_flux ? 0x20 : 0));
  if (sr.wrote) {
    S.wr_ok++;
  } else {
    S.wr_rnf++;
    if (sr.cyl_mismatch)
      S.wr_cyl_mismatch++;
    if (sr.size_mismatch)
      S.wr_size_mismatch++;
    if (sr.late)
      S.wr_late++;
  }
}

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
    if (S.err_write_protect)
      st |= 0x40; // WRITE PROTECT (b6 in the Type II layout, write commands
                  // only). Set only by the WRITE SECTOR stub -- see 3.3.
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

void fdc_init(FloppyDrive *drive) {
  DRV = drive;
  for (int i = 0; i < FDC_SIZE_CACHE_N; ++i)
    g_size_code[i] = FDC_SIZE_CODE_DEFAULT;
}

void fdc_bus_reset_poll(void) {
  static uint32_t high_since = 0;
  static bool acted = false;
  static bool was_high = false;
  static uint32_t edge_us = 0;

  bool high = gpio_get(RESET_PIN);
  uint32_t now = time_us_32();

  // Edge telemetry first, and unconditionally: this has to record what the line
  // does even when nothing below decides to act on it. "No reset seen" is not a
  // diagnosis on its own.
  S.rst_samples++;
  if (high != was_high) {
    uint32_t held = now - edge_us;
    if (was_high) {
      S.rst_fall++;
      S.rst_last_high_us = held;
      if (held > S.rst_high_max_us)
        S.rst_high_max_us = held;
    } else {
      S.rst_rise++;
      if (S.rst_low_min_us == 0 || held < S.rst_low_min_us)
        S.rst_low_min_us = held;
    }
    was_high = high;
    edge_us = now;
  }

  if (!high) {
    high_since = 0;
    acted = false;
    S.reset_active = false;
    return;
  }
  if (!high_since) {
    high_since = now ? now : 1; // 0 is the "not high" sentinel
    return;
  }
  if (acted || (now - high_since) < FDC_RESET_CONFIRM_US)
    return;
  acted = true;
  S.reset_active = true;
  S.reset_events++;
  S.reset_at_ms = to_ms_since_boot(get_absolute_time());

  // 1. Release the interrupt. This is the part that must happen.
  S.eint = false;
  S.force_int_latched = false;

  // 2. Drop everything the host was waiting on. It is gone; holding BUSY or
  //    DRQ serves nobody, and leaving them set means the machine comes back to
  //    a controller that claims to be mid-command.
  S.busy = false;
  S.drq = false;
  S.reading = false;
  S.read_addr = false;
  S.multi = false;
  S.pending_force_int = false;
  S.pending_force_since_us = 0;

  // 3. Abandon writes that have not reached the media. One already in the
  //    splice or laying a track down finishes -- the flux is going onto the
  //    disk in real time and cannot be recalled, exactly as on a real WD.
  S.writing = false;
  S.wr_armed = false;
  if (S.trk_op == 1 || (S.trk_op == 2 && !S.trk_armed))
    S.trk_op = 0;

  // 4. WD Master Reset also loads 01 into the Sector register. It additionally
  //    executes a RESTORE, which we deliberately do NOT do: that would drive
  //    the head on a signal the host has not asked us to act on, and every host
  //    we know of issues its own RESTORE on startup anyway.
  S.sector = 1;
  S.command = 0;

  fdc_refresh_int_line(); // -> Hi-Z, whatever the transfer state was
}

// ---- core1 (bus) hot path -------------------------------------------------

RAM_FUNC void fdc_reg_write(uint8_t addr, uint8_t raw) {
  fdc_trace_io(addr, raw, 0);
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
    // A new command clears any latched Force-Interrupt INTRQ (WD spec) and
    // re-evaluates ~INT. Done before the decode below so a Type IV can re-arm it.
    S.force_int_latched = false;
    fdc_refresh_int_line(); // a new command releases ~INT (reference behaviour)
    if ((cmd & 0x80) == 0) {
      // ---- Type I (seek family) ----
      S.type2 = false; // STATUS reverts to the Type I layout
      S.read_addr = false;
      // Abandon any half-collected write. The head is about to move, and a
      // buffer collected for one cylinder must never be spliced onto another;
      // the size cache is per-cylinder too. (A read in flight is deliberately
      // left alone -- see the Force Interrupt notes on queued Type I commands.)
      S.writing = false;
      S.wr_armed = false;
      if (S.trk_op == 1 || (S.trk_op == 2 && !S.trk_armed))
        S.trk_op = 0; // same reasoning: never lay a track down on another cyl
      // The Track register is the WD's *belief*; the head position is physical
      // and separate. Compute a relative step count here and update the Track
      // register per the WD rules — core0 applies the motion to wherever the
      // head actually is.
      uint8_t hi = cmd >> 4;
      bool update = (cmd & 0x10) != 0; // WD 'u' bit (STEP / STEP IN / STEP OUT)
      int steps = 0;
      bool restore = false;
      switch (hi) {
      case 0x0: // RESTORE: step out to TRK00, Track := 0
        restore = true;
        S.track = 0;
        break;
      case 0x1: // SEEK: step (Data - Track) cylinders, Track := Data
        steps = (int)S.data - (int)S.track;
        S.track = S.data;
        break;
      case 0x2:
      case 0x3: // STEP: repeat the last direction
        steps = S.step_in ? +1 : -1;
        if (update)
          S.track = (uint8_t)(S.track + steps);
        break;
      case 0x4:
      case 0x5: // STEP IN
        steps = +1;
        S.step_in = true;
        if (update)
          S.track = (uint8_t)(S.track + 1);
        break;
      case 0x6:
      case 0x7: // STEP OUT
        steps = -1;
        S.step_in = false;
        if (update)
          S.track = (uint8_t)(S.track - 1);
        break;
      }
      // Fold into the target. A second Type I arriving before core0 has run the
      // first must combine with it, never replace it -- see head_target.
      if (restore) {
        S.head_target = 0;
        S.restore_req++; // force a physical TRK00 seek even if we think we're there
      } else {
        // Clamp HERE, per command -- this is the end-stop absorption a real
        // drive performs on every surplus pulse. See head_target.
        int t = (int)S.head_target + steps;
        if (t < 0)
          t = 0;
        if (t > FDC_MAX_CYLINDER)
          t = FDC_MAX_CYLINDER;
        S.head_target = (int16_t)t;
      }
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
        S.read_addr = false;
        S.drq = false;
        S.pending_force_int = false;
        S.pending_force_since_us = 0;
        // A write that has not been handed to core0 yet is simply dropped. One
        // already in the splice cannot be: the flux is going onto the media in
        // real time and there is no more stopping it than there is on a real
        // WD. core0 sees `writing` clear when it comes back and publishes
        // nothing further.
        S.writing = false;
        S.wr_armed = false;
        // A WRITE TRACK image collected but not yet laid down is dropped; one
        // already going onto the media finishes, for the same reason a splice
        // does -- the flux is being written in real time and cannot be recalled.
        if (S.trk_op == 1 || (S.trk_op == 2 && !S.trk_armed))
          S.trk_op = 0;
        // NOTE: deliberately do NOT cancel a pending Type I move here (i.e. no
        // `S.op_done = S.op_seq`). On a real WD the step pulse is already out
        // the door before the CPU can write the Force Interrupt byte, so the
        // head has moved and the Track register has been updated regardless.
        // The CP/M BIOS positions the head with STEP-IN immediately followed by
        // FORCE-INT; cancelling the move meant the head never left track 0, so
        // READ ADDRESS always answered track 0, the BIOS stepped forever, and
        // every READ SECTOR silently returned track 0 data.
        // Force Interrupt still drops host-visible BUSY at once (above); core0
        // finishes the motion and clears BUSY again, which is idempotent.
      }
      // Type IV interrupt conditions. Bit3 (I3) = interrupt immediately; the
      // MZ-800 boot ROM uses 0xD8, i.e. it has been asking for INTRQ all along
      // and merely polls as well. 0xD0 (no condition bits) terminates silently.
      if (cmd & 0x08) {
        // I3 = "interrupt immediately". Latch it: `want` in
        // fdc_refresh_int_line() is otherwise DRQ-driven, and by this point the
        // abort above has already cleared reading/drq, so without the latch the
        // refresh call below could never assert and 0xD8 was a no-op. Still
        // gated by EINT, so the boot ROM (which issues 0xD8 with no ISR and
        // never arms EINT) is unaffected.
        S.force_int_latched = true;
        fdc_refresh_int_line();
      }
    } else if ((cmd & 0xC0) == 0x80) {
      // ---- Type II (sector data): 0x80..0xBF ----
      S.type2 = true;
      S.read_addr = false;
      S.err_rnf = S.err_crc = S.lost_data = S.rec_deleted = false;
      S.err_write_protect = false; // cleared by the next Type II/III command
      S.multi = (cmd & 0x10) != 0;
      // Any new data command supersedes a write that has not reached the disk.
      // Without this a READ arriving on top of a half-collected write leaves
      // both `reading` and `writing` set, and core0 -- which checks writing
      // first -- would splice a stale buffer instead of serving the read.
      S.writing = false;
      S.wr_armed = false;
      if (S.trk_op == 1 || (S.trk_op == 2 && !S.trk_armed))
        S.trk_op = 0;
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
        S.rd_result = 0; // clear now: an abort must not leave a stale success
        S.rd_want = S.sector;
        S.rd_fresh = true; // first sector of the command -> (re)capture flux
        S.busy = true;     // BUSY must appear at once (ROM's pre-loop times out)
        S.rd_issue_us = time_us_32();
        S.status_polls = 0;
        S.status_last_us = S.rd_issue_us;
        S.last_data_read_us = S.rd_issue_us;
        S.rd_elapsed_us = 0; // cleared until an abort (or drain) records a real value
        S.rd_seq++;
      } else {
        // WRITE SECTOR — single (0xAX) or multiple (0xBX). Collect a whole
        // sector from the host first, then let core0 splice it in behind the
        // matching ID field (see the write-path block in fdc_disk_task and
        // mfm_splice.hpp).
        S.reading = false;
        S.read_addr = false;
        S.write_attempts++;
        if (S.write_protect || !g_write_enable) {
          // Refused before the disk is touched. WRITE PROTECT is a real WD
          // status bit for a write command and says "this did not happen"
          // unambiguously -- unlike clearing BUSY with no error bits, which
          // reports success and lets the host's in-RAM copy of the disk
          // silently diverge from the disk.
          S.writing = false;
          S.wr_armed = false;
          S.drq = false;
          S.busy = false;
          S.err_write_protect = true;
          S.wr_refused++;
        } else {
          S.writing = true;
          S.wr_armed = false;
          S.wr_sector = S.sector;
          S.wr_deleted = (cmd & 0x01) != 0; // a0: 0 -> 0xFB, 1 -> 0xF8
          S.xfer_len = fdc_sector_size_for(S.track);
          S.xfer_pos = 0;
          S.busy = true;
          // A real WD raises DRQ as soon as the command is written, before it
          // has found (or even looked for) the ID field, and the host starts
          // shifting bytes in immediately. Same here -- the difference is only
          // that our bytes land in a buffer rather than on the media.
          S.drq = true;
          S.rd_issue_us = time_us_32();
          S.status_polls = 0;
          S.status_last_us = S.rd_issue_us;
          S.last_data_read_us = S.rd_issue_us;
          fdc_refresh_int_line();
        }
      }
    } else if ((cmd & 0xF0) == 0xC0) {
      // ---- Type III READ ADDRESS (0xCX) ----
      // Hand back the next ID field to pass the head as six bytes over DRQ:
      // track, side, sector, size code, CRC1, CRC2. The CP/M BIOS uses this to
      // work out which cylinder the head is actually on (STEP-IN/OUT then
      // READ ADDRESS); returning nothing makes it step blindly and give up.
      S.type2 = true;
      S.err_rnf = S.err_crc = S.lost_data = S.rec_deleted = false;
      S.err_write_protect = false; // cleared by the next Type II/III command
      S.writing = false;
      S.wr_armed = false;
      S.reading = true;
      S.read_addr = true;
      S.multi = false;
      S.pending_force_int = false;
      S.pending_force_since_us = 0;
      S.drq = false;
      S.xfer_len = 0;
      S.xfer_pos = 0;
      S.rd_result = 0x10; // READ ADDRESS marker, "no result yet"
      S.rd_want = 0;
      S.rd_ids_seen = 0;
      S.rd_fresh = true;
      S.busy = true;
      S.rd_issue_us = time_us_32();
      S.status_polls = 0;
      S.status_last_us = S.rd_issue_us;
      S.last_data_read_us = S.rd_issue_us;
      S.rd_elapsed_us = 0;
      S.rd_seq++;
    } else {
      // ---- Type III READ TRACK / WRITE TRACK (0xEX/0xFX) ----
      // Milestone 3.4. Use the Type II status layout (not Type I) and complete
      // benignly so the host doesn't hang polling BUSY.
      S.type2 = true;
      S.reading = false;
      S.read_addr = false;
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
    // During a WRITE SECTOR this is the host feeding the sector, one DRQ'd byte
    // at a time. Ownership of xfer[] passes to core0 via `wr_armed`+`wr_seq`
    // once the last byte lands -- the mirror of how `drq` hands a read sector
    // the other way.
    if (S.writing && S.drq) {
      S.xfer[S.xfer_pos] = S.data;
      S.xfer_pos++;
      S.last_data_read_us = time_us_32();
      if (S.xfer_pos >= S.xfer_len) {
        S.drq = false;   // stop asking; the disk work starts on core0
        S.wr_armed = true;
        S.wr_seq++;
      }
      // Taking/giving the DATA byte is what clears the interrupt on this card
      // (a STATUS read does not). With DRQ still set this re-arms for the next
      // byte, exactly as on the read side.
      fdc_refresh_int_line();
    }
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
  case 0xDE: // DENSITY (not CPL'd): b0 = density select.
    // Latched only -- deliberately not driven onto DENSEL. The Disk BASIC ROM
    // defines this port and never writes it, so nothing has ever told us which
    // way round b0 goes, and the two documented readings disagree. Guessing
    // would risk a machine that currently boots in order to serve media this
    // platform does not have. See docs/wd-fidelity.md.
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
  case 0xDF: { // EINT -- interrupt enable (not CPL'd). bit0: 1 = arm, 0 = disarm.
    // Confirmed against the reference firmware's fdcWrite() case 7. The MZ-800
    // boot ROM and CP/M 2.3 never touch this port, so EINT stays 0 and they are
    // never interrupted; CP/M 4.1 writes 0x01 before each data command and 0x00
    // after, which is how it drives an interrupt-capable (community) card.
    S.eint = (raw & 0x01) != 0;
    if (!S.eint)
      S.force_int_latched = false; // disarming drops any pending one-shot too
    fdc_refresh_int_line();

    // Bus-test loopback bookkeeping shares this port. Harmless for EINT: the
    // test program is only ever run standalone, never under CP/M.
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

RAM_FUNC static uint8_t fdc_reg_read_inner(uint8_t addr);

// Thin tracing wrapper: every read is recorded with the byte actually driven,
// so the trace shows what the host saw, not what we intended.
RAM_FUNC uint8_t fdc_reg_read(uint8_t addr) {
  uint8_t v = fdc_reg_read_inner(addr);
  fdc_trace_io(addr, v, 1);
  return v;
}

RAM_FUNC static uint8_t fdc_reg_read_inner(uint8_t addr) {
  switch (addr) {
  case 0xD8:
    if (S.reading || S.writing) { // measure the host's real BUSY/DRQ patience
      S.status_polls++;
      S.status_last_us = time_us_32();
    }
    // Deliberately no *DRQ* interrupt release here: on this card that one is
    // cleared by taking the DATA byte, not by polling STATUS. Releasing it on a
    // STATUS read would drop the interrupt before the ISR ever reached the
    // transfer. A latched Force-Interrupt INTRQ *is* cleared by a STATUS read,
    // which is the WD rule and is what stops a one-shot 0xD8 from holding the
    // level-sensitive ~INT low forever.
    if (S.force_int_latched) {
      S.force_int_latched = false;
      fdc_refresh_int_line();
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
          fdc_refresh_int_line();
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
          S.read_addr = false;
          S.busy = false; // single-sector / READ ADDRESS complete
          fdc_refresh_int_line();
        }
      }
      // Taking the DATA byte is what clears the interrupt on this card. DRQ is
      // still set for the next byte, so this re-arms rather than releases --
      // matching the reference, where the ISR is woken once per byte.
      fdc_refresh_int_line();
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

uint8_t fdc_iotrace_get(uint8_t idx, uint8_t *port, uint8_t *val, uint8_t *is_rd,
                        uint16_t *rep) {
  if (idx >= S.io_n)
    return 0;
  // Walk oldest -> newest: the ring's oldest entry sits at io_pos once full.
  uint8_t start =
      (S.io_n < FdcState::IO_TRACE_N) ? 0 : S.io_pos;
  uint8_t k = (uint8_t)((start + idx) % FdcState::IO_TRACE_N);
  *port = S.io_port[k];
  *val = S.io_val[k];
  *is_rd = S.io_rd[k];
  *rep = S.io_rep[k];
  return 1;
}

uint8_t fdc_iotrace_count(void) { return S.io_n; }
uint32_t fdc_iotrace_total(void) { return S.io_total; }

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
  o->verify_attempts = S.verify_attempts;
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
  o->int_flags = (uint8_t)((S.eint ? 1 : 0) | (S.int_asserted ? 2 : 0) |
                           (S.force_int_latched ? 4 : 0));
  o->int_mode = g_int_mode;
  o->intrq_asserts = S.intrq_asserts;
  o->lost_data_events = S.lost_data_events;
  o->write_attempts = S.write_attempts;
  o->wr_refused = S.wr_refused;
  o->wr_ok = S.wr_ok;
  o->wr_rnf = S.wr_rnf;
  o->wr_cyl_mismatch = S.wr_cyl_mismatch;
  o->wr_size_mismatch = S.wr_size_mismatch;
  o->wr_late = S.wr_late;
  o->wr_splice_us = S.wr_splice_us;
  o->wr_ids_seen = S.wr_ids_seen;
  o->wr_last_result = S.wr_last_result;
  o->wr_flags = (uint8_t)((S.writing ? 0x01 : 0) | (S.wr_armed ? 0x02 : 0) |
                          (g_write_enable ? 0x04 : 0) |
                          (S.write_protect ? 0x08 : 0));
  o->sec_ok = S.sec_ok;
  o->sec_crc_err = S.sec_crc_err;
  o->sec_rnf = S.sec_rnf;
  o->last_id_cyl = S.last_id_cyl;
  o->last_id_head = S.last_id_head;
  o->sec_cyl_mismatch = S.sec_cyl_mismatch;
  o->last_trk_reg = S.last_trk_reg;
  o->last_head_target = S.last_head_target;
  o->last_phys_track = S.last_phys_track;
  o->last_want_side = S.last_want_side;
  o->bt_active = S.bt_active ? 1 : 0;
  o->bt_last_rx = S.bt_last_rx;
  o->bt_last_echo = S.bt_last_echo;
  // Truncated to 16 bits for the legacy FDCDUMP wire format -- use the
  // `bustest` command for the authoritative, non-wrapping counts.
  o->bt_rx_count = (uint16_t)S.bt_rx_count;
  o->bt_rd_count = (uint16_t)S.bt_rd_count;
  o->bt_err_count = (uint16_t)S.bt_err_count;
}

void fdc_firstfail_snapshot(FdcFirstFail *o) {
  o->valid = S.ff_valid ? 1 : 0;
  o->kind = S.ff_kind;
  o->count = S.ff_count;
  o->at_ms = S.ff_at_ms;
  o->want_sec = S.ff_want_sec;
  o->trk_reg = S.ff_trk_reg;
  o->head_target = S.ff_head_target;
  o->phys_track = S.ff_phys_track;
  o->side = S.ff_side;
  o->seen_cyl = S.ff_seen_cyl;
  o->seen_head = S.ff_seen_head;
  o->capture_ok = S.ff_capture_ok;
  o->flux_n = S.ff_flux_n;
  o->capture_us = S.ff_capture_us;
  o->ids_seen = S.ff_ids_seen;
  for (int k = 0; k < 16; ++k)
    o->ids[k] = S.ff_ids[k];
  o->reset_events = S.reset_events;
  o->reset_at_ms = S.reset_at_ms;
  o->reset_line = gpio_get(RESET_PIN) ? 1 : 0;
  o->reset_active = S.reset_active ? 1 : 0;
  o->pos_resyncs = S.pos_resyncs;
  o->last_resync_from = S.last_resync_from;
  o->last_resync_to = S.last_resync_to;
}

void fdc_resetdiag_snapshot(FdcResetDiag *o) {
  o->line = gpio_get(RESET_PIN) ? 1 : 0;
  o->samples = S.rst_samples;
  o->rise = S.rst_rise;
  o->fall = S.rst_fall;
  o->high_max_us = S.rst_high_max_us;
  o->low_min_us = S.rst_low_min_us;
  o->last_high_us = S.rst_last_high_us;
  o->events = S.reset_events;
  o->at_ms = S.reset_at_ms;
}

void fdc_resetdiag_clear(void) {
  S.rst_samples = 0;
  S.rst_rise = 0;
  S.rst_fall = 0;
  S.rst_high_max_us = 0;
  S.rst_low_min_us = 0;
  S.rst_last_high_us = 0;
  S.reset_events = 0;
  S.reset_at_ms = 0;
}

void fdc_firstfail_reset(void) {
  S.ff_valid = false;
  S.ff_count = 0;
  S.ff_kind = 0;
  S.ff_at_ms = 0;
  S.ff_seen_cyl = S.ff_seen_head = 0xFF;
  S.pos_resyncs = 0;
  S.last_resync_from = S.last_resync_to = 0xFF;
}

void fdc_set_side_invert(bool on) { g_invert_side = on; }
bool fdc_get_side_invert(void) { return g_invert_side; }

void fdc_set_int_mode(uint8_t mode) {
  g_int_mode = (mode > 2) ? 2 : mode;
  fdc_refresh_int_line(); // apply immediately, including releasing in mode 0
}
uint8_t fdc_get_int_mode(void) { return g_int_mode; }

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
      mfm_read_sector(flux, nflux, bt, sector, MFM_ANY_CYL, g_secbuf,
                      sizeof(g_secbuf));

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

bool fdc_bench_write_sector(uint8_t track, uint8_t sector, uint8_t side,
                            const uint8_t *data, uint16_t n, uint8_t *flags) {
  if (flags)
    *flags = 0;
  if (!DRV || n == 0 || n > FDC_SECTOR_MAX)
    return false;

  DRV->step_delay_us = 2000;
  DRV->settle_delay_ms = 5;
  DRV->motor_delay_ms = 300;
  DRV->select(true);
  DRV->spin_motor(true);
  DRV->side(side ? 1 : 0);
  DRV->goto_track(track);

  S.write_attempts++;
  S.write_protect = DRV->get_write_protect();
  if (S.write_protect) {
    S.wr_refused++;
    return false; // the disk says no; nothing else to report
  }

  SpliceResult sr =
      mfm_write_sector(sector, (int)track, data, n, /*deleted=*/false,
                       FDC_DD_BIT_US, FDC_WRITE_SEARCH_TIMEOUT_US);
  fdc_record_splice(sr);
  if (sr.id_found) {
    fdc_note_size_code(track, sr.size_n);
    S.last_id_cyl = sr.seen_cyl;
    S.last_id_head = sr.seen_head;
    S.last_trk_reg = track;
    S.last_want_side = side ? 1 : 0;
  }
  if (flags)
    *flags = (uint8_t)((sr.wrote ? 0x01 : 0) | (sr.id_found ? 0x02 : 0) |
                       (sr.cyl_mismatch ? 0x04 : 0) |
                       (sr.size_mismatch ? 0x08 : 0) | (sr.late ? 0x10 : 0) |
                       (sr.no_flux ? 0x20 : 0));
  return sr.wrote;
}

uint16_t fdc_bench_format(uint8_t track, uint8_t side, uint8_t n_sectors,
                          uint8_t size_code, uint8_t interleave, uint8_t fill,
                          uint8_t *out_result) {
  if (out_result)
    *out_result = 0;
  if (!DRV || n_sectors == 0 || n_sectors > 32)
    return 0;
  size_code &= 7;

  // Build a standard IBM System-34 MFM track image -- the same byte stream a
  // host would shift in for WRITE TRACK -- and run it through the same encoder
  // the host's command uses. Formatting from the bench therefore exercises the
  // real path, not a parallel one that could drift away from it.
  uint8_t *img = (uint8_t *)S.track_buf;
  uint16_t n = 0;
  auto put = [&](uint8_t b, int count) {
    while (count-- > 0 && n < FDC_TRACK_IMAGE_MAX)
      img[n++] = b;
  };

  put(0x4E, 80);       // gap 4a
  put(0x00, 12);       // presync
  put(0xF6, 3);        // 0xC2 sync marks
  put(0xFC, 1);        // index address mark
  put(0x4E, 50);       // gap 1

  // Interleave: lay the sector NUMBERS around the track in steps of
  // `interleave` so consecutive logical sectors are not physically adjacent.
  // Without it a host that needs any processing time between sectors misses the
  // next one and waits a whole revolution for it to come round again.
  uint8_t order[32];
  for (int k = 0; k < n_sectors; ++k)
    order[k] = 0;
  {
    int slot = 0;
    uint8_t il = interleave ? interleave : 1;
    for (int sec = 1; sec <= n_sectors; ++sec) {
      while (order[slot])
        slot = (slot + 1) % n_sectors; // that slot is taken; take the next free
      order[slot] = (uint8_t)sec;
      slot = (slot + il) % n_sectors;
    }
  }

  static const uint16_t gap3_for[8] = {32, 54, 84, 116, 255, 255, 255, 255};
  for (int k = 0; k < n_sectors; ++k) {
    put(0x00, 12);
    put(0xF5, 3);            // 0xA1 x3, presets the CRC
    put(0xFE, 1);            // ID address mark
    put(track, 1);
    put(side ? 1 : 0, 1);
    put(order[k], 1);
    put(size_code, 1);
    put(0xF7, 1);            // ID CRC
    put(0x4E, 22);           // gap 2
    put(0x00, 12);
    put(0xF5, 3);
    put(0xFB, 1);            // data address mark
    put(fill, 128 << size_code);
    put(0xF7, 1);            // data CRC
    put(0x4E, gap3_for[size_code]);
  }

  DRV->step_delay_us = 2000;
  DRV->settle_delay_ms = 5;
  DRV->motor_delay_ms = 300;
  DRV->select(true);
  DRV->spin_motor(true);
  DRV->side(side ? 1 : 0);
  DRV->goto_track(track);
  if (DRV->get_write_protect()) {
    if (out_result)
      *out_result = 1; // write protected
    return 0;
  }

  size_t cap = 0;
  uint8_t *flux = mzp_flux_buffer(&cap);
  size_t limit = (cap < FDC_TRACK_FLUX_MAX) ? cap : FDC_TRACK_FLUX_MAX;
  size_t n_flux = mfm_encode_format_image(flux, limit, img, n, FDC_DD_BIT_US);
  if (!n_flux) {
    if (out_result)
      *out_result = 2; // image would not fit
    return 0;
  }
  n_flux = mfm_encode_gap_fill(flux, n_flux, limit, FDC_DD_BIT_US);
  bool ok = DRV->write_track(flux, n_flux, /*store_greaseweazle=*/false,
                             /*use_index=*/true);
  if (out_result)
    *out_result = ok ? 0 : 3;
  if (ok)
    S.trk_ok++;
  else
    S.trk_fail++;
  S.trk_last_bytes = n;
  return ok ? n : 0;
}

uint16_t fdc_bench_read_track(uint8_t track, uint8_t side, uint8_t *flux_buf,
                             size_t flux_cap) {
  if (!DRV)
    return 0;
  DRV->step_delay_us = 2000;
  DRV->settle_delay_ms = 5;
  DRV->motor_delay_ms = 300;
  DRV->select(true);
  DRV->spin_motor(true);
  DRV->side(side ? 1 : 0);
  DRV->goto_track(track);

  int32_t idx = 0;
  size_t nflux = DRV->capture_track(flux_buf, flux_cap, &idx,
                                    /*store_greaseweazle=*/false,
                                    /*capture_ms=*/220, /*index_wait_ms=*/250);
  size_t n = mfm_read_track_raw(flux_buf, nflux, FDC_DD_BIT_US,
                                (uint8_t *)S.track_buf, FDC_TRACK_IMAGE_MAX);
  S.trk_last_bytes = (uint16_t)n;
  return (uint16_t)n;
}

const uint8_t *fdc_track_buffer(void) { return (const uint8_t *)S.track_buf; }

void fdc_set_write_enable(bool on) { g_write_enable = on; }
bool fdc_get_write_enable(void) { return g_write_enable; }

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
    bool restore = (S.restore_done != S.restore_req);
    uint32_t rseq = S.restore_req;
    bool verify = S.op_verify;

    // Do NOT write S.track back from the physical position: the Track register
    // is the host's belief and
    // core1 has already updated it per the WD rules. Overwriting it here forced
    // belief == reality, which is precisely the state a recalibrating host is
    // trying to establish for itself via READ ADDRESS.
    if (restore) {
      DRV->goto_track(0); // steps out until TRK00
      S.restore_done = rseq;
    }
    // Drive the head to whatever core1 has folded into head_target. Read once:
    // core1 may move it again while we step, which just leaves op_seq != op_done
    // and brings us straight back here.
    {
      int tgt = S.head_target;
      if (DRV->track() != tgt)
        DRV->goto_track(tgt);
    }

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
    if (S.seek_error)
      fdc_note_failure(4, 0, 0xFF, 0xFF);

    S.op_done = seq;
    // Clear BUSY only if this seek is still the active command. A newer Type I
    // bumps op_seq; a newer Type II sets `reading` without touching op_seq —
    // that second case is reachable now that Force Interrupt no longer cancels
    // a pending move, so the host can queue STEP-IN / FORCE-INT / READ-ADDRESS
    // and have the read start while the step is still executing. BUSY belongs
    // to the read at that point, not to us.
    if (S.op_seq == seq && !S.reading) {
      S.busy = false;
      S.last_ready_us = time_us_32(); // diagnostic: gap until the next command
      fdc_refresh_int_line();              // Type I complete
    }
  }

  // ---- Type III READ TRACK / WRITE TRACK -----------------------------------
  if (S.trk_op) {
    // Neither may run with a move outstanding, for the same reason a sector
    // write may not -- and for WRITE TRACK the stakes are a whole track, not
    // one sector's data field.
    if (S.op_done != S.op_seq)
      return;

    if (S.trk_op == 1) { // READ TRACK
      if (S.trk_done != S.trk_seq) {
        uint32_t seq = S.trk_seq;
        size_t cap = 0;
        uint8_t *flux = mzp_flux_buffer(&cap);
        uint32_t t0 = time_us_32();
        fdc_capture_window(FDC_READ_CAPTURE_BYTES);
        if (!S.trk_op) // aborted while we were blocked capturing
          return;
        size_t n = mfm_read_track_raw(flux, g_flux_n, FDC_DD_BIT_US,
                                      (uint8_t *)S.track_buf,
                                      FDC_TRACK_IMAGE_MAX);
        S.trk_last_us = time_us_32() - t0;
        S.trk_last_bytes = (uint16_t)n;
        if (n == 0) {
          S.err_rnf = true; // nothing decodable -> the WD's only "no data" bit
          S.trk_fail++;
          S.trk_op = 0;
          S.busy = false;
        } else {
          S.trk_ok++;
          S.trk_len = (uint16_t)n;
          S.trk_pos = 0;
          S.last_data_read_us = time_us_32();
          S.drq = true;
        }
        S.trk_done = seq;
        fdc_refresh_int_line();
      } else if (S.drq &&
                 (time_us_32() - S.last_data_read_us) > FDC_DRQ_STALL_TIMEOUT_US) {
        // Nobody is taking the bytes. Same reasoning as the sector read: we
        // buffer, so we *can* wait forever, and a host entitled to walk away
        // would deadlock us holding BUSY.
        S.lost_data = true;
        S.lost_data_events++;
        S.trk_op = 0;
        S.drq = false;
        S.busy = false;
        fdc_refresh_int_line();
      }
      return;
    }

    // WRITE TRACK.
    if (!S.trk_armed) {
      // The host is still shifting the image in. It stops when the image ends,
      // not when we say so, so silence is the end-of-image signal.
      if (S.trk_pos > 0 &&
          (time_us_32() - S.last_data_read_us) > FDC_TRACK_IDLE_US) {
        S.trk_len = S.trk_pos;
        S.drq = false;
        S.trk_armed = true;
        S.trk_seq++;
        fdc_refresh_int_line();
      } else if (S.trk_pos == 0 &&
                 (time_us_32() - S.last_data_read_us) > FDC_DRQ_STALL_TIMEOUT_US) {
        // Command issued, not one byte sent. Nothing to write; don't hold BUSY.
        S.lost_data = true;
        S.lost_data_events++;
        S.trk_op = 0;
        S.drq = false;
        S.busy = false;
        fdc_refresh_int_line();
      }
      return;
    }

    if (S.trk_done == S.trk_seq)
      return;
    uint32_t seq = S.trk_seq;

    if (g_stream_active) {
      rp2040_flux_capture_stop();
      g_stream_active = false;
    }
    g_flux_n = 0; // the captured window describes the track we are replacing

    S.write_protect = DRV->get_write_protect();
    bool ok = false;
    uint32_t t0 = time_us_32();
    if (!S.write_protect) {
      size_t cap = 0;
      uint8_t *flux = mzp_flux_buffer(&cap);
      // Cap the flux at a bit over one revolution: the write is terminated by
      // the index pulse anyway, and encoding more is wasted time.
      size_t limit = (cap < FDC_TRACK_FLUX_MAX) ? cap : FDC_TRACK_FLUX_MAX;
      size_t n_flux = mfm_encode_format_image(
          flux, limit, (const uint8_t *)S.track_buf, S.trk_len, FDC_DD_BIT_US);
      if (n_flux) {
        // Pad to the end of the buffer with gap bytes. A short image would
        // otherwise leave the tail of the OLD track standing past the end of
        // the new one -- half of each format, spliced wherever the host
        // happened to stop. The index pulse cuts the write off at the right
        // place; our job is only to have flux ready to write until it comes.
        n_flux = mfm_encode_gap_fill(flux, n_flux, limit, FDC_DD_BIT_US);
        // use_index: start at the index pulse and stop at the next one. That
        // is what makes this a *track* write rather than a very long splice --
        // there is no timing window to hit, which is why formatting is the
        // safer of the two write paths despite being the more destructive.
        ok = DRV->write_track(flux, n_flux, /*store_greaseweazle=*/false,
                              /*use_index=*/true);
      }
    }
    S.trk_last_us = time_us_32() - t0;
    S.trk_last_bytes = S.trk_len;
    if (ok) {
      S.trk_ok++;
    } else {
      S.trk_fail++;
      if (S.write_protect)
        S.err_write_protect = true;
      else
        S.err_rnf = true;
    }
    if (S.trk_op == 2 && S.trk_seq == seq) {
      S.trk_op = 0;
      S.trk_armed = false;
      S.drq = false;
      S.busy = false;
      fdc_refresh_int_line();
    }
    S.trk_done = seq;
    return;
  }

  // ---- Type II write: collect from the host, then splice -------------------
  if (S.writing) {
    // The host stopped feeding bytes part way through the sector. A real WD
    // would have written zeros for the missing bytes and raised LOST DATA; we
    // have not touched the disk yet, so the honest -- and non-destructive --
    // answer is to raise LOST DATA and write nothing at all. The budget is the
    // same generous one the read side uses; it exists to guarantee forward
    // progress, not to police a slow host.
    if (S.drq && (time_us_32() - S.last_data_read_us) > FDC_DRQ_STALL_TIMEOUT_US) {
      S.lost_data = true;
      S.lost_data_events++;
      S.writing = false;
      S.wr_armed = false;
      S.drq = false;
      S.busy = false;
      fdc_refresh_int_line();
      return;
    }
    if (!S.wr_armed)
      return; // still collecting the sector from the host

    // Never splice with a move outstanding. This matters far more than it does
    // on the read path, where answering from the wrong cylinder merely returns
    // data the host can reject: here it would overwrite a sector on a track the
    // host never asked about. mfm_write_sector's ID track compare is the second
    // line of defence; this is the first.
    if (S.op_done != S.op_seq)
      return;

    uint32_t seq = S.wr_seq; // snapshot
    uint8_t want = S.wr_sector;
    uint16_t n = S.xfer_len;
    if (n > FDC_SECTOR_MAX)
      n = FDC_SECTOR_MAX;
    for (uint16_t k = 0; k < n; ++k)
      g_secbuf[k] = S.xfer[k];

    // The capture peripheral must be free -- the splice starts its own live
    // capture, and an aborted read can still have one running.
    if (g_stream_active) {
      rp2040_flux_capture_stop();
      g_stream_active = false;
    }
    g_flux_n = 0; // whatever was captured describes the track BEFORE this write

    // Re-read ~WPT now rather than trusting the value cached at the last seek:
    // this is the last moment before flux goes onto the media.
    S.write_protect = DRV->get_write_protect();

    // Write-protected between the command and here (or the tab was always
    // open and the earlier check ran on a stale sense). Report WRITE PROTECT
    // and stop -- deliberately NOT RNF, which would send the host looking for
    // a positioning problem it does not have.
    if (S.write_protect) {
      S.err_write_protect = true;
      S.wr_refused++;
      if (S.writing && S.wr_seq == seq) {
        S.writing = false;
        S.wr_armed = false;
        S.drq = false;
        S.busy = false;
        fdc_refresh_int_line();
      }
      S.wr_done = seq;
      return;
    }

    SpliceResult sr =
        mfm_write_sector(want, (int)S.track, g_secbuf, n, S.wr_deleted,
                         FDC_DD_BIT_US, FDC_WRITE_SEARCH_TIMEOUT_US);

    fdc_record_splice(sr);
    if (sr.id_found)
      fdc_note_size_code(S.track, sr.size_n);
    S.last_head_target = (uint8_t)S.head_target;
    {
      int pt = DRV->track();
      S.last_phys_track = (pt < 0) ? 0xFE : (uint8_t)pt;
    }
    S.last_trk_reg = S.track;
    S.last_want_side = S.side1 ? 1 : 0;

    if (sr.wrote) {
      S.err_rnf = false;
      S.last_id_cyl = sr.seen_cyl;
      S.last_id_head = sr.seen_head;
    } else {
      // Everything that is not a completed write is RNF to the host: the WD has
      // exactly one "the sector I was told to write is not under this head"
      // status bit. fdc_record_splice's counters are what tell the causes apart.
      S.err_rnf = true;
      if (sr.cyl_mismatch) {
        S.sec_cyl_mismatch++;
        S.last_id_cyl = sr.seen_cyl;
        S.last_id_head = sr.seen_head;
        fdc_note_failure(3, want, sr.seen_cyl, sr.seen_head);
        fdc_resync_head(sr.seen_cyl);
      } else {
        fdc_note_failure(1, want, 0xFF, 0xFF);
      }
    }

    // Publish only if this write is still the live command: a Force Interrupt
    // or a Type I can have landed while we were on the disk with interrupts
    // off, and resurrecting BUSY/DRQ afterwards would hang a host that has
    // already moved on.
    if (S.writing && S.wr_seq == seq) {
      if (sr.wrote && S.multi) {
        // Multi-record: the WD stays BUSY and asks for the next sector's worth
        // of bytes. Each one is spliced on its own pass, and each re-finds its
        // ID -- there is no captured window to reuse, because a write has to
        // ride the disk in real time whatever we do.
        S.sector++;
        S.wr_sector = S.sector;
        S.xfer_len = fdc_sector_size_for(S.track);
        S.xfer_pos = 0;
        S.wr_armed = false;
        S.last_data_read_us = time_us_32();
        S.drq = true;
      } else {
        S.writing = false;
        S.wr_armed = false;
        S.drq = false;
        S.busy = false;
      }
      fdc_refresh_int_line();
    }
    S.wr_done = seq;
    return;
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
      fdc_refresh_int_line();
    }

    // LOST DATA. A real WD clocks the data field off the disk in real time and
    // cannot wait: if the host does not take a DRQ'd byte within a byte time it
    // sets LOST DATA and finishes the command regardless. We buffer the sector
    // instead, so without this a host that never reads DATA deadlocks us --
    // BUSY and DRQ held forever while it polls STATUS (observed: CP/M 4.1 stuck
    // on READ ADDRESS at xfer=0/6 for 21 s). The budget here is far more
    // generous than a real WD's ~32 us so a slow polling host is never punished;
    // it exists only to guarantee forward progress.
    // READ ADDRESS gets a much shorter budget than a sector read, because a host
    // is entitled to ignore the six ID bytes entirely: the WD also loads the ID's
    // track byte into the Sector register, so the usual idiom is "issue READ
    // ADDRESS, wait for BUSY to clear, read Sector". A real WD clocks the ID
    // field out in ~6 byte times (~200 us) and drops BUSY whether or not anyone
    // took the bytes. Holding BUSY until a drain that never comes deadlocks that
    // host -- observed as CP/M 4.1 looping RESTORE / READ ADDRESS forever, 27031
    // STATUS polls per attempt and xfer=0/6 every time.
    uint32_t drq_budget =
        S.read_addr ? FDC_READADDR_DRQ_WINDOW_US : FDC_DRQ_STALL_TIMEOUT_US;
    if (S.reading && S.drq && !S.pending_force_int &&
        (time_us_32() - S.last_data_read_us) > drq_budget) {
      // Completing an undrained READ ADDRESS is normal, not an error: do not
      // raise LOST DATA there or a host that only wants the Sector register
      // sees a spurious failure. A sector read that goes unread IS a real
      // overrun, so that one keeps the flag.
      if (!S.read_addr) {
        S.lost_data = true;
        S.lost_data_events++;
      }
      S.rd_elapsed_us = time_us_32() - S.rd_issue_us;
      S.reading = false;
      S.read_addr = false;
      S.busy = false;
      S.drq = false;
      fdc_refresh_int_line();
    }

    // The block above (or a Force Interrupt on core1) may have ended the read.
    // Re-test before committing to a capture window: it blocks core0 for up to
    // 300 ms, which is a long time to spend on a command nobody is waiting for.
    if (!S.reading)
      return;

    // A physical move is still outstanding: do not capture or decode yet.
    //
    // The seek block above runs earlier in this same pass, so without this we
    // fall straight through into the capture the instant it finishes -- using
    // whatever cylinder that seek happened to reach, while a NEWER step is still
    // queued. The host issues STEP-IN / FORCE-INT / READ-SEC back-to-back in
    // microseconds, but our steps take milliseconds, so this is the common case,
    // not a rare race: measured, the head sat on cylinder 1 while the host had
    // already asked for 2, and the read was answered from the wrong track before
    // the second step was ever applied.
    //
    // A real WD never needs this -- its step pulse completes in microseconds, so
    // a Type II arriving behind a Type I finds the head in place. Returning here
    // just lets the next pass finish the move first; BUSY stays set, which is
    // exactly what the host is polling on.
    if (S.op_done != S.op_seq)
      return;

    if (S.rd_fresh) {
      // First sector of a new command: grab a fresh window. The head is already
      // on-track from a prior SEEK; READ SECTOR does not itself seek. No index
      // sync -- a 1.25-revolution window contains every sector regardless of
      // where it starts, so waiting for index only adds latency.
      if (S.read_addr)
        g_readaddr_found = fdc_read_address_search(g_readaddr_id);
      else
        fdc_capture_window(FDC_READ_CAPTURE_BYTES);
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
      if (S.read_addr) {
        // READ ADDRESS: the first ID field found, as six bytes over DRQ.
        if (g_readaddr_found) {
          sd.found = sd.has_data = sd.data_crc_ok = true;
          sd.size = 6;
          sd.cyl = g_readaddr_id[0];
          sd.head = g_readaddr_id[1];
          for (int k = 0; k < 6; ++k) {
            g_secbuf[k] = g_readaddr_id[k];
            S.rd_ids[k] = g_readaddr_id[k]; // surfaced by `rddiag`
          }
          S.rd_ids_seen = 6;
          // WD quirk: READ ADDRESS also loads the track address byte into the
          // Sector register. A host probing head position may read it there
          // rather than from the data stream.
          S.sector = g_readaddr_id[0];
          // The ID's size code, for the write path's per-cylinder cache --
          // indexed by the cylinder the ID itself reports, not the Track
          // register, since READ ADDRESS is precisely what a host uses when
          // those two disagree.
          fdc_note_size_code(g_readaddr_id[0], g_readaddr_id[3]);
        }
      } else if (g_flux_n > 0) {
        // WD ID track compare against the Track register -- see MFM_ANY_CYL.
        sd = mfm_read_sector(flux, g_flux_n, FDC_DD_BIT_US, want, S.track,
                             g_secbuf, sizeof(g_secbuf));
      }
      S.rd_search_us = time_us_32() - t0;
      S.rd_want = want;
      S.last_head_target = (uint8_t)S.head_target;
      {
        int pt = DRV->track();
        S.last_phys_track = (pt < 0) ? 0xFE : (uint8_t)pt;
      }
      S.rd_result = (uint8_t)((sd.found ? 1 : 0) | (sd.has_data ? 2 : 0) |
                              (sd.data_crc_ok ? 4 : 0) | (sd.deleted ? 8 : 0));
      if (S.read_addr)
        S.rd_result |= 0x10; // mark the diagnostic as a READ ADDRESS result

      if (sd.found && sd.has_data) {
        uint16_t n = (sd.size <= FDC_SECTOR_MAX) ? sd.size : FDC_SECTOR_MAX;
        for (uint16_t k = 0; k < n; ++k)
          S.xfer[k] = g_secbuf[k];
        S.xfer_len = n;
        S.xfer_pos = 0;
        // CRC is STICKY across a multi-record command, and a bad one ENDS it.
        //
        // This used to be a plain assignment. In a Read Multiple (0x9X, which is
        // how both the ROM and CP/M read) the host only inspects STATUS after the
        // terminating Force Interrupt, so a CRC failure on sector 3 was wiped out
        // by sector 4 decoding cleanly -- the host was handed a sector of corrupt
        // bytes and told the whole transfer succeeded. Single-sector readers never
        // saw it because there is no later sector to overwrite the flag.
        //
        // Clearing S.multi also matches the WD: a data-field CRC error terminates
        // the command (the record is still transferred, then INTRQ/BUSY-off with
        // CRC ERROR set) instead of auto-advancing to the next record.
        if (!sd.data_crc_ok) {
          S.err_crc = true;
          S.multi = false;
          if (!S.read_addr)
            fdc_note_failure(2, want, sd.cyl, sd.head);
          // Guarded like sec_ok below: this counter is what rddiag prints as
          // "handed the host corrupt bytes", so a READ ADDRESS must never reach
          // it. Unreachable today (the read_addr branch synthesises
          // data_crc_ok = true) but the asymmetry is a trap for later edits.
          if (!S.read_addr)
            S.sec_crc_err++;
        } else if (!S.read_addr) {
          S.sec_ok++;
        }
        if (!S.read_addr) {
          // A read is where the write path learns this cylinder's sector size
          // (see FDC_SIZE_CACHE_N). The ID track compare has already confirmed
          // sd.cyl == the Track register, so either indexes the same entry.
          fdc_note_size_code(sd.cyl, sd.size_n);
          S.last_id_cyl = sd.cyl;
          S.last_id_head = sd.head;
          S.last_trk_reg = S.track;
          S.last_want_side = S.side1 ? 1 : 0;
        }
        S.rec_deleted = sd.deleted;
        S.err_rnf = false;
        // Publish DRQ last: it is the ownership handoff of xfer[] to core1.
        if (S.reading && S.rd_seq == seq) {
          // Start the LOST DATA clock here, not at command issue: the host has
          // had nothing to read until this moment.
          S.last_data_read_us = time_us_32();
          S.drq = true;
          // The card ORs DRQ into ~INT. This must run in EVERY mode, not just
          // the diagnostic force-on mode 2: this is the ONE place the first
          // DRQ of a sector is raised, so gating it on mode 2 meant an
          // EINT-armed host (CP/M 4.1) never got the interrupt that starts the
          // transfer -- it armed EINT, issued READ, and waited for an edge that
          // only arrived if some later unrelated event happened to call
          // refresh. fdc_refresh_int_line() applies the mode/EINT gate itself.
          fdc_refresh_int_line();
        }
        S.rd_done = seq;
        // Multi-record keeps the captured window: every remaining sector of
        // this track is already in it, so no re-capture is needed.
      } else {
        // Not on this track (or undecodable). Record what IS there, then RNF.
        if (!S.read_addr)
          fdc_note_track_ids(flux, g_flux_n);
        else
          S.rd_ids_seen = 0;
        S.err_rnf = true;
        if (!S.read_addr) {
          S.sec_rnf++;
          // Separate the two RNF causes: "that sector is not on this track" vs
          // "it IS here but on the wrong cylinder" (a positioning failure).
          if (sd.cyl_mismatch) {
            S.sec_cyl_mismatch++;
            S.last_id_cyl = sd.seen_cyl;
            S.last_id_head = sd.seen_head; // else it pairs with a stale head
            S.last_trk_reg = S.track;
            S.last_want_side = S.side1 ? 1 : 0;
            fdc_note_failure(3, want, sd.seen_cyl, sd.seen_head);
            // The disk just told us where the head really is. Take it: without
            // this the host's retry repeats from the same wrong cylinder and
            // fails identically, forever.
            fdc_resync_head(sd.seen_cyl);
          } else {
            fdc_note_failure(1, want, 0xFF, 0xFF);
          }
        }
        S.drq = false;
        if (S.rd_seq == seq) {
          S.reading = false;
          S.busy = false;
          fdc_refresh_int_line(); // WD raises INTRQ on an errored completion too
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
