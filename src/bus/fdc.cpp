#include "fdc.hpp"
#include "board_bus.h" // RAM_FUNC
#include "floppy.hpp"
#include "pico/time.h" // sleep_ms

// WD register window (true, un-inverted values). The MZ-800 ROM CPLs the WD
// registers 0xD8..0xDB (command/status/track/sector/data) but NOT the control
// ports 0xDC..0xDE (motor/page/density) — verified against MZ2Z046.ASM.
//
// STATUS bits (Type I): b7 NotReady b6 WriteProtect b5 HeadLoaded b4 SeekError
//                       b3 CRCError b2 Track00 b1 Index b0 Busy.

namespace {

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

  // Physical status core0 -> core1.
  volatile bool at_track0 = true;
  volatile bool write_protect = false;

  volatile bool active = false; // set once the MZ-800 first talks to us

  // Ring log of recent command opcodes (true values), for debugging.
  volatile uint8_t cmd_hist[16] = {0};
  volatile uint8_t cmd_pos = 0;  // next slot
  volatile uint16_t cmd_count = 0;
};

FdcState S;
FloppyDrive *DRV = nullptr;

// core0-only "last applied" shadow (no volatile needed — single core).
bool g_applied_select = false;
bool g_applied_motor = false; // physical motor state (may lag S.motor_on)
int g_applied_side = -1;
bool g_timing_set = false;
uint32_t g_motor_off_at = 0; // deferred motor-stop deadline (ms; 0 = none)

// Keep the spindle spinning this long after the ROM commands motor-off, so
// RESTORE retries (and back-to-back ops) don't re-pay the ~1 s spin-up. This is
// the standard floppy motor-off delay.
static constexpr uint32_t FDC_MOTOR_OFF_DELAY_MS = 3000;

inline uint32_t now_ms() { return to_ms_since_boot(get_absolute_time()); }

inline uint8_t compose_status() {
  uint8_t st = 0;
  if (S.busy)
    st |= 0x01; // BUSY
  if (S.at_track0)
    st |= 0x04; // TRACK 00
  if (S.write_protect)
    st |= 0x40; // WRITE PROTECT
  // Ready once the ROM has commanded the motor on (real READY line on PC drives
  // is unreliable; refine in 3.4). NotReady = bit7.
  if (!S.motor_on)
    st |= 0x80; // NOT READY
  return st;
}

} // namespace

void fdc_init(FloppyDrive *drive) { DRV = drive; }

// ---- core1 (bus) hot path -------------------------------------------------

RAM_FUNC void fdc_reg_write(uint8_t addr, uint8_t raw) {
  switch (addr) {
  case 0xD8: { // COMMAND (CPL'd on the bus)
    uint8_t cmd = (uint8_t)~raw;
    S.active = true;
    S.command = cmd;
    S.cmd_hist[S.cmd_pos & 15] = cmd;
    S.cmd_pos++;
    S.cmd_count++;
    if ((cmd & 0x80) == 0) {
      // ---- Type I (seek family) ----
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
      S.busy = true;
      S.op_seq++;
    } else if ((cmd & 0xF0) == 0xD0) {
      // ---- Type IV Force Interrupt: abort, drop BUSY ----
      S.busy = false;
      S.op_done = S.op_seq;
    } else {
      // Type II/III (sector data) not implemented until 3.2. Complete benignly
      // so the ROM sees "not busy" rather than hanging.
      S.busy = false;
    }
    break;
  }
  case 0xD9:
    S.track = (uint8_t)~raw; // TRACK (CPL'd)
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
  case 0xDD: // PAGE/SIDE (not CPL'd): b0 = side/head select
    S.side1 = (raw & 0x01) != 0;
    break;
  case 0xDE: // DENSITY (not CPL'd): b0 = single density
    S.density_single = (raw & 0x01) != 0;
    break;
  }
}

RAM_FUNC uint8_t fdc_reg_read(uint8_t addr) {
  switch (addr) {
  case 0xD8:
    return (uint8_t)~compose_status(); // STATUS (driven CPL'd)
  case 0xD9:
    return (uint8_t)~S.track;
  case 0xDA:
    return (uint8_t)~S.sector;
  case 0xDB:
    return (uint8_t)~S.data;
  default:
    return 0xFF; // control ports are write-only
  }
}

uint8_t fdc_debug_cmdlog(uint8_t *out, uint16_t *count) {
  uint8_t n = (S.cmd_count < 16) ? (uint8_t)S.cmd_count : 16;
  // Emit oldest -> newest.
  uint8_t start = (uint8_t)(S.cmd_pos - n);
  for (uint8_t k = 0; k < n; ++k)
    out[k] = S.cmd_hist[(uint8_t)(start + k) & 15];
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
    }
  }
  // Apply side.
  int want_side = S.side1 ? 1 : 0;
  if (want_side != g_applied_side) {
    DRV->side(want_side);
    g_applied_side = want_side;
  }

  // Execute a pending physical seek.
  if (S.op_done != S.op_seq) {
    uint32_t seq = S.op_seq;   // snapshot
    uint8_t target = S.op_target;
    bool restore = S.op_restore;

    if (restore)
      DRV->goto_track(0);
    else
      DRV->goto_track(target);

    int t = DRV->track();
    S.track = (t < 0) ? 0 : (uint8_t)t; // WD updates Track reg after the move
    S.at_track0 = DRV->get_track0_sense();
    S.write_protect = DRV->get_write_protect();

    S.op_done = seq;
    if (S.op_seq == seq)
      S.busy = false; // clear BUSY only if no newer command queued
  }
}
