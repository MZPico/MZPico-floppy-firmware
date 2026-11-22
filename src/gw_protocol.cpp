// Derived from Adafruit_Floppy (https://github.com/adafruit/Adafruit_Floppy),
// MIT License, Copyright (c) 2021 Limor Fried for Adafruit Industries.
// See LICENSES/MIT-Adafruit_Floppy.txt. Changes: Apache-2.0, see LICENSE.
// greaseweazle command loop — ported from Adafruit_Floppy's greaseweazle.ino.
// Serial (USB CDC) -> TinyUSB tud_cdc_*; Serial1 debug -> DBG() no-op;
// millis()/yield() -> Pico-SDK time / tud_task(); floppy-> -> FloppyDrive.
#include "hardware/gpio.h"
#include "gw_protocol.hpp"
#include "gitver.h"

#include "board_config.h"
#include "floppy.hpp"
#include "mfm_decode.hpp"
#include "mfm_encode.hpp"
#include "bus/mz_bus.hpp"
#include "mfm_splice.hpp"
#include "bus/fdc.hpp"
#include "flux/arch_rp2.h"

#include "tusb.h"
#include "pico/time.h"

#include <cstring>

// ---- greaseweazle command / ack codes ----
#define GW_CMD_GETINFO 0
#define GW_CMD_GETINFO_FIRMWARE 0
#define GW_CMD_GETINFO_BANDWIDTH 1
#define GW_CMD_SEEK 2
#define GW_CMD_HEAD 3
#define GW_CMD_SETPARAMS 4
#define GW_CMD_GETPARAMS 5
#define GW_CMD_GETPARAMS_DELAYS 0
#define GW_CMD_MOTOR 6
#define GW_CMD_READFLUX 7
#define GW_CMD_WRITEFLUX 8
#define GW_CMD_GETFLUXSTATUS 9
#define GW_CMD_SELECT 12
#define GW_CMD_DESELECT 13
#define GW_CMD_SETBUSTYPE 14
#define GW_CMD_SETBUSTYPE_IBM 1
#define GW_CMD_SETBUSTYPE_SHUGART 2
#define GW_CMD_SETPIN 15
#define GW_CMD_SETPIN_DENSITY 2
#define GW_CMD_RESET 16
#define GW_CMD_SOURCEBYTES 18
#define GW_CMD_SINKBYTES 19
#define GW_CMD_GETPIN 20
#define GW_CMD_GETPIN_TRACK0 26

// MZPico extension commands (outside the greaseweazle command range, so the
// gw host tools never send them; only tools/mzp_probe.py does).
#define MZP_CMD_SCAN 0x80 // scan one track's MFM sectors (Phase-2 validation)
#define MZP_CMD_ENCTEST 0x81 // encode->decode round-trip (encoder validation)
#define MZP_CMD_BUSDUMP 0x82 // read MZ-800 bus hello shadow regs + counters
#define MZP_CMD_DRIVE 0x83   // bench: drive C0..C7 to a value and hold
#define MZP_CMD_GPIOREAD 0x84 // bench: read live C0..C7 input byte (jumper test)
#define MZP_CMD_PINTEST 0x85   // bench: per-pad internal pull-up/down self-test
#define MZP_CMD_FDCDUMP 0x86   // read WD FDC register/state snapshot
#define MZP_CMD_FDCLOG 0x87    // read WD FDC recent-command ring log
#define MZP_CMD_FDCSEEK 0x88   // bench: inject motor-on + RESTORE/SEEK via the FDC
#define MZP_CMD_FDCREAD 0x89   // bench: READ SECTOR via the FDC (capture+decode)
#define MZP_CMD_STREAMSCAN 0x8A // bench: scan a track using the incremental
                                // (streaming) capture API, for A/B comparison
                                // against MZP_CMD_SCAN's batch capture
#define MZP_CMD_BUSTEST_RESET 0x8B // reset port-0xDF bus-test counters/state
#define MZP_CMD_BUSTEST 0x8C // port-0xDF loopback + cycle-timing diagnostics
#define MZP_CMD_RDDIAG 0x8D  // why the last Type II read succeeded / RNF'd
#define MZP_CMD_SIDESWAP 0x8E // invert ROM side-select -> physical head
#define MZP_CMD_INTMODE 0x8F  // ~INT: 0=off 1=DRQ, EINT-gated 2=DRQ, ignore EINT
#define MZP_CMD_IOTRACE 0x90  // ordered register-access trace (reads + writes)
#define MZP_CMD_FDCWRITE 0x91 // bench: WRITE SECTOR via the FDC (splice)
#define MZP_CMD_WRDIAG 0x92   // why the last WRITE SECTOR wrote / refused
#define MZP_CMD_WREN 0x93     // enable/disable writes from the MZ-800
#define MZP_CMD_FIRSTFAIL 0x94 // first read/seek failure since power-up (sticky)
#define MZP_CMD_FDCFORMAT 0x95 // bench: WRITE TRACK (format) one track
#define MZP_CMD_READTRACK 0x96 // bench: READ TRACK -- raw decoded byte stream
#define MZP_CMD_RESETDIAG 0x97 // raw bus-RESET line telemetry (no assumptions)
#define MZP_CMD_RESETPOLL 0x98 // enable/disable core1's bus-RESET IRQ handling
#define MZP_CMD_POSFIX 0x99    // enable/disable head-position self-correction
#define MZP_CMD_FMTEN 0x9A     // enable/disable host-initiated WRITE TRACK
#define MZP_CMD_DRVSEL 0x9B    // bench: which unit (~DS0/~DS1) the FDC drives
#define MZP_CMD_DRVLOCK 0x9C   // pin every access to drive 0 (escape hatch)
#define MZP_CMD_DRVSTAT 0x9D   // per-drive state for ALL units at once
#define MZP_CMD_DENSITY 0x9E   // density: force DD/HD or auto-detect (default)
#define MZP_CMD_IDXBIT 0x9F    // Type I STATUS Index bit on/off (default on)
#define MZP_CMD_FWVER 0xA0     // firmware git hash -- WHICH build is running
#define MZP_CMD_DENSEL 0xA1    // DENSEL pin override: 0/1 force, 2 auto, 3 query
#define MZP_CMD_RDWIN 0xA2     // bench: one read through the LIVE read window
#define MZP_CMD_RECALDIAG 0xA3 // recalibrates that never found TRK00 (#31)
#define MZP_MAX_SECTORS 40
#define MZP_MAX_ENC_SECTORS 18

#define GW_ACK_OK 0
#define GW_ACK_BADCMD 1
#define GW_ACK_NOINDEX 2
#define GW_ACK_NOTRACK0 3
#define GW_ACK_WRPROT 6
#define GW_ACK_NOUNIT 7
#define GW_ACK_BADPIN 10
// Bench drive command refused: the MZ-800 has a command in flight (#15).
#define MZP_ACK_HOST_BUSY 0x80

#define DBG(...)                                                               \
  do {                                                                         \
  } while (0)

// ---- state ----
// The last argument is `readypin`, and it is -1 by design, not an oversight.
// ~DEN_RDY (GPIO13) is a single shared net carrying density-out AND ready-in, so
// it cannot be sampled as an input while we drive density; PC drives do not
// supply READY anyway. get_ready_sense() returns true unconditionally for
// readypin < 0, which is the right answer for a PC drive. (This used to be
// PIN_DS1 -- GPIO25, a drive-select *output* -- so ready-sense was reading back
// our own select line.)
//
// Two instances, one per ~DSn line, sharing every other pin. Everything that is
// genuinely per-drive -- the head position, and the TRK00 cross-check built on
// it -- is per-instance state, which is exactly what a second drive needs and
// what a single instance could not express. The shared lines are handed over on
// each select; see FloppyDrive::adopt_shared_lines().
static FloppyDrive g_drive0(PIN_DENSITY, PIN_INDEX, PIN_DS0, PIN_MOTOR,
                            PIN_DIR, PIN_STEP, PIN_WDATA, PIN_WGATE, PIN_TRK00,
                            PIN_WPT, PIN_RDATA, PIN_HDSEL, -1);
static FloppyDrive g_drive1(PIN_DENSITY, PIN_INDEX, PIN_DS1, PIN_MOTOR,
                            PIN_DIR, PIN_STEP, PIN_WDATA, PIN_WGATE, PIN_TRK00,
                            PIN_WPT, PIN_RDATA, PIN_HDSEL, -1);
static FloppyDrive *g_drives[2] = {&g_drive0, &g_drive1};
// The greaseweazle/USB path is single-drive by protocol: gw has no concept of a
// second unit here, so it always talks to drive 0. Bench access to drive 1 goes
// through the FDC bridge's own drive select (`mzp_probe.py drive 1`).
static FloppyDrive &g_drive = g_drive0;
static FloppyDrive *floppy = nullptr;

static uint8_t cmd_buffer[32];
static uint8_t reply_buffer[512]; // WRDIAG alone is 252 bytes (#25)
static uint8_t flux_transitions[MAX_FLUX_PULSE_PER_TRACK];
static bool flux_status = false;

// MZP scan-track scratch
static SectorInfo g_scan_sectors[MZP_MAX_SECTORS];
static uint8_t g_scan_scratch[1024]; // largest sector (128 << 3)
static uint8_t g_enc_sectors[MZP_MAX_ENC_SECTORS * 512]; // encode-test source

uint8_t *mzp_flux_buffer(size_t *cap) {
  if (cap)
    *cap = sizeof(flux_transitions);
  return flux_transitions;
}

static uint32_t bandwidth_timer;
static uint32_t transfered_bytes;
static uint32_t timestamp = 0;

// ---- helpers ----
static inline uint32_t millis() {
  return to_ms_since_boot(get_absolute_time());
}
template <class T> static inline T tmin(T a, T b) { return a < b ? a : b; }

static uint32_t cdc_avail() { return tud_cdc_available(); }

// One byte, or -1 after `timeout_ms` with nothing (#9). The old version waited
// forever: two stray bytes on the port (a terminal, ModemManager's "AT") made
// a frame header announcing a payload that never came, and core0 -- which
// also runs the disk task -- was gone for good, BUSY held, floppy dead.
// `serve_fdc` keeps the disk task running while we wait; only safe while no
// USB command is executing (i.e. while a frame is still being received).
static int cdc_read1_to(uint32_t timeout_ms, bool serve_fdc) {
  uint8_t b;
  uint32_t t0 = millis();
  for (;;) {
    if (tud_cdc_available() && tud_cdc_read(&b, 1) == 1)
      return b;
    tud_task();
    if (serve_fdc)
      fdc_disk_task();
    if (millis() - t0 > timeout_ms)
      return -1;
  }
}
static constexpr uint32_t CDC_FRAME_BYTE_TIMEOUT_MS = 200;
static constexpr uint32_t CDC_STREAM_TIMEOUT_MS = 2000;

static void cdc_write_all(const void *data, uint32_t len) {
  const uint8_t *p = (const uint8_t *)data;
  while (len) {
    uint32_t n = tud_cdc_write(p, len);
    if (n) {
      p += n;
      len -= n;
    }
    tud_cdc_write_flush();
    tud_task();
  }
  tud_cdc_write_flush();
}
static void cdc_write_byte(uint8_t b) { cdc_write_all(&b, 1); }

static bool setbustype(int bustype) {
  if (floppy)
    floppy->end();
  floppy = nullptr;
  switch (bustype) {
  case -1:
  case GW_CMD_SETBUSTYPE_IBM:
  case GW_CMD_SETBUSTYPE_SHUGART:
    floppy = &g_drive;
    break;
  default:
    return false;
  }
  return floppy->begin();
}

// Read one framed command: [cmd, len, payload...] with len counting the whole
// frame. Returns the frame length, 0 if no complete command is waiting, or -1
// for a malformed frame -- which the caller answers with BADCMD.
//
// A malformed frame must still leave the stream FRAMED. The old version
// returned with an oversized frame's payload unread (each payload byte was
// then parsed as a fresh command -- one bad frame became a storm of BADCMDs
// and, worse, whatever commands the payload happened to spell), and a len of 0
// returned "no command" after consuming two bytes. So: an oversized frame's
// declared payload is drained and discarded here, and both malformed shapes
// are reported instead of half-handled.
//
// A frame whose payload does not arrive within CDC_FRAME_BYTE_TIMEOUT_MS per
// byte is dropped and reported (-1), and the disk task keeps running while
// the rest of a frame is awaited (#9). The buffer is zeroed first, so a short
// frame can never run a command with the previous command's argument bytes.
static int get_cmd(uint8_t *buff, uint8_t maxbuff) {
  if (cdc_avail() < 2)
    return 0;
  memset(buff, 0, maxbuff);
  buff[0] = (uint8_t)cdc_read1_to(CDC_FRAME_BYTE_TIMEOUT_MS, true);
  buff[1] = (uint8_t)cdc_read1_to(CDC_FRAME_BYTE_TIMEOUT_MS, true);
  uint8_t len = buff[1];
  if (len < 2)
    return -1; // header alone is broken; nothing more of it to consume
  for (int i = 2; i < len; i++) {
    int b = cdc_read1_to(CDC_FRAME_BYTE_TIMEOUT_MS, true);
    if (b < 0)
      return -1; // truncated: drop it, the next bytes start a new frame
    if (i < maxbuff)
      buff[i] = (uint8_t)b;
  }
  return len > maxbuff ? -1 : len; // oversized: drained above, reported here
}

void gw_init() {
  floppy = nullptr;
  timestamp = millis();
  // Bring up the drive GPIOs and hand the drive to the FDC bridge. The FDC disk
  // task stays idle until the MZ-800 first touches an FDC port, so it doesn't
  // conflict with greaseweazle/USB drive control when no machine is attached.
  // Both drive-select lines must be parked HIGH (= deselected; ~DSn is active
  // low) before anything else. RP2040 pads power up as inputs with a pull-DOWN,
  // so a select line left alone is asserted permanently and that drive answers
  // alongside -- or instead of -- the one we mean. Each FloppyDrive::begin()
  // parks its own ~DSn as part of soft_reset(), so bringing both up covers both
  // lines; ~DS1 no longer needs the explicit parking it used to.
  //
  // Order matters only in that drive 1 is brought up FIRST: soft_reset() also
  // re-parks the shared pins, so the last begin() to run leaves them in the
  // known state, and drive 0 is the one we start selected on.
  g_drive1.begin();
  g_drive0.begin();
  fdc_init(g_drives, 2);
}

static void gw_dispatch(uint8_t cmd, uint8_t len, int i);

void gw_task() {
  int cmd_len = get_cmd(cmd_buffer, sizeof(cmd_buffer));
  if (!cmd_len) {
    // idle watchdog: park the head and stop the motor after 10s of USB
    // silence -- but NEVER once the MZ-800 has spoken. The FDC bridge and this
    // gw path share the same FloppyDrive instances, and `floppy` stays pointed
    // at drive 0 after any USB session; without this guard, running `gw rpm`
    // and then booting the machine (no power cycle) had the watchdog see the
    // FDC's spinning motor every 10 s, park the head to track 0 and stop the
    // spindle under CP/M. The mirror of fdc_disk_task's "idle until the MZ-800
    // talks to us" rule.
    if ((millis() > timestamp) && ((millis() - timestamp) > 10000)) {
      if (!fdc_is_active()) {
        fdc_bench_idle();
        if (floppy && floppy->motor_is_spinning()) {
          floppy->select(true);
          floppy->goto_track(0);
          floppy->spin_motor(false);
        }
        if (floppy && floppy->drive_is_selected())
          floppy->select(false);
      }
      timestamp = millis();
    }
    return;
  }
  timestamp = millis();

  int i = 0;
  uint8_t cmd = cmd_buffer[0];
  uint8_t len = cmd_buffer[1];
  memset(reply_buffer, 0, sizeof(reply_buffer));
  reply_buffer[i++] = cmd; // echo the cmd byte

  if (cmd_len < 0) { // malformed frame (see get_cmd); payload already drained
    reply_buffer[i++] = GW_ACK_BADCMD;
    cdc_write_all(reply_buffer, 2);
    return;
  }

  // Commands that move, spin, read or write the drive share it -- and the
  // flux buffer and the head -- with the live controller. Refuse them while
  // the MZ-800 has a command in flight, and drop the controller's shadows of
  // the drive after each one (#15).
  bool drive_cmd = false;
  switch (cmd) {
  case GW_CMD_SEEK: case GW_CMD_HEAD: case GW_CMD_MOTOR: case GW_CMD_SELECT:
  case GW_CMD_DESELECT:
  case GW_CMD_READFLUX: case GW_CMD_WRITEFLUX: case GW_CMD_SETBUSTYPE:
  case MZP_CMD_SCAN: case MZP_CMD_STREAMSCAN: case MZP_CMD_FDCSEEK:
  case MZP_CMD_FDCREAD: case MZP_CMD_FDCWRITE: case MZP_CMD_FDCFORMAT:
  case MZP_CMD_READTRACK: case MZP_CMD_RDWIN:
    drive_cmd = true;
    break;
  default:
    break;
  }
  if (drive_cmd && fdc_host_busy()) {
    reply_buffer[i++] = MZP_ACK_HOST_BUSY;
    cdc_write_all(reply_buffer, 2);
    return;
  }
  gw_dispatch(cmd, len, i);
  if (drive_cmd)
    fdc_bench_done();
}

static void gw_dispatch(uint8_t cmd, uint8_t len, int i) {
  (void)len;
  if (cmd == GW_CMD_GETINFO) {
    uint8_t sub_cmd = cmd_buffer[2];
    if (sub_cmd == GW_CMD_GETINFO_FIRMWARE) {
      reply_buffer[i++] = GW_ACK_OK;
      reply_buffer[i++] = GW_FIRMVER_MAJOR;
      reply_buffer[i++] = GW_FIRMVER_MINOR;
      reply_buffer[i++] = 1; // is main firmware
      reply_buffer[i++] = GW_MAXCMD;
      uint32_t sf = GW_SAMPLE_FREQ;
      reply_buffer[i++] = sf & 0xFF;
      reply_buffer[i++] = (sf >> 8) & 0xFF;
      reply_buffer[i++] = (sf >> 16) & 0xFF;
      reply_buffer[i++] = (sf >> 24) & 0xFF;
      reply_buffer[i++] = GW_HW_MODEL;
      reply_buffer[i++] = GW_HW_SUBMODEL;
      reply_buffer[i++] = GW_USB_SPEED;
      cdc_write_all(reply_buffer, 34);
    } else if (sub_cmd == GW_CMD_GETINFO_BANDWIDTH) {
      reply_buffer[i++] = GW_ACK_OK;
      uint32_t bytes = transfered_bytes;
      uint32_t usec = bandwidth_timer * 1000;
      for (int k = 0; k < 2; k++) { // min then max (same values)
        reply_buffer[i++] = bytes & 0xFF;
        reply_buffer[i++] = bytes >> 8;
        reply_buffer[i++] = bytes >> 16;
        reply_buffer[i++] = bytes >> 24;
        reply_buffer[i++] = usec & 0xFF;
        reply_buffer[i++] = usec >> 8;
        reply_buffer[i++] = usec >> 16;
        reply_buffer[i++] = usec >> 24;
      }
      cdc_write_all(reply_buffer, 34);
    } else { // unknown sub-command: answer it, never leave the host hanging (#9)
      reply_buffer[i++] = GW_ACK_BADCMD;
      cdc_write_all(reply_buffer, 2);
    }
  }

  else if (cmd == GW_CMD_GETPARAMS) {
    // `floppy` is null until SETBUSTYPE; without this guard a host that asks
    // for the delays first reads them from address 0.
    if (!floppy) {
      reply_buffer[i++] = GW_ACK_BADCMD;
      cdc_write_all(reply_buffer, 2);
      return;
    }
    uint8_t sub_cmd = cmd_buffer[2];
    if (sub_cmd == GW_CMD_GETPARAMS_DELAYS) {
      reply_buffer[i++] = GW_ACK_OK;
      reply_buffer[i++] = floppy->select_delay_us & 0xFF;
      reply_buffer[i++] = floppy->select_delay_us >> 8;
      reply_buffer[i++] = floppy->step_delay_us & 0xFF;
      reply_buffer[i++] = floppy->step_delay_us >> 8;
      reply_buffer[i++] = floppy->settle_delay_ms & 0xFF;
      reply_buffer[i++] = floppy->settle_delay_ms >> 8;
      reply_buffer[i++] = floppy->motor_delay_ms & 0xFF;
      reply_buffer[i++] = floppy->motor_delay_ms >> 8;
      reply_buffer[i++] = floppy->watchdog_delay_ms & 0xFF;
      reply_buffer[i++] = floppy->watchdog_delay_ms >> 8;
      cdc_write_all(reply_buffer, 12);
    } else { // unknown sub-command (#9)
      reply_buffer[i++] = GW_ACK_BADCMD;
      cdc_write_all(reply_buffer, 2);
    }
  }

  else if (cmd == GW_CMD_SETPARAMS) {
    // accept and ack (parameters are not applied; delays stay at defaults)
    reply_buffer[i++] = GW_ACK_OK;
    cdc_write_all(reply_buffer, 2);
  }

  else if (cmd == GW_CMD_RESET) {
    if (floppy)
      floppy->soft_reset();
    reply_buffer[i++] = GW_ACK_OK;
    cdc_write_all(reply_buffer, 2);
  }

  else if (cmd == GW_CMD_SETBUSTYPE) {
    bool result = setbustype(cmd_buffer[2]);
    reply_buffer[i++] = result ? GW_ACK_OK : GW_ACK_BADCMD;
    cdc_write_all(reply_buffer, 2);
  }

  else if (cmd == GW_CMD_SEEK) {
    if (!floppy) {
      reply_buffer[i++] = GW_ACK_BADCMD;
      cdc_write_all(reply_buffer, 2);
      return;
    }
    int track = cmd_buffer[2];
    if (len > 3)
      track |= cmd_buffer[3] << 8;
    bool r = floppy->goto_track(track);
    reply_buffer[i++] = r ? GW_ACK_OK : GW_ACK_NOTRACK0;
    cdc_write_all(reply_buffer, 2);
  }

  else if (cmd == GW_CMD_HEAD) {
    if (!floppy) {
      reply_buffer[i++] = GW_ACK_BADCMD;
      cdc_write_all(reply_buffer, 2);
      return;
    }
    floppy->side(cmd_buffer[2]);
    reply_buffer[i++] = GW_ACK_OK;
    cdc_write_all(reply_buffer, 2);
  }

  else if (cmd == GW_CMD_MOTOR) {
    if (!floppy) {
      reply_buffer[i++] = GW_ACK_BADCMD;
      cdc_write_all(reply_buffer, 2);
      return;
    }
    floppy->spin_motor(cmd_buffer[3]);
    reply_buffer[i++] = GW_ACK_OK;
    cdc_write_all(reply_buffer, 2);
  }

  else if (cmd == GW_CMD_SELECT) {
    if (!floppy) {
      reply_buffer[i++] = GW_ACK_BADCMD;
      cdc_write_all(reply_buffer, 2);
      return;
    }
    if (cmd_buffer[2] == 0) {
      floppy->select(true);
      reply_buffer[i++] = GW_ACK_OK;
    } else {
      reply_buffer[i++] = GW_ACK_NOUNIT;
    }
    cdc_write_all(reply_buffer, 2);
  }

  else if (cmd == GW_CMD_DESELECT) {
    if (!floppy) {
      reply_buffer[i++] = GW_ACK_BADCMD;
      cdc_write_all(reply_buffer, 2);
      return;
    }
    floppy->select(false);
    reply_buffer[i++] = GW_ACK_OK;
    cdc_write_all(reply_buffer, 2);
  }

  else if (cmd == GW_CMD_READFLUX) {
    if (!floppy) {
      reply_buffer[i++] = GW_ACK_BADCMD;
      cdc_write_all(reply_buffer, 2);
      return;
    }
    uint32_t flux_ticks = ((uint32_t)cmd_buffer[5] << 24) |
                          ((uint32_t)cmd_buffer[4] << 16) |
                          ((uint32_t)cmd_buffer[3] << 8) | cmd_buffer[2];
    uint16_t revs = ((uint16_t)cmd_buffer[7] << 8) | cmd_buffer[6];
    bool use_index;
    if (revs) {
      revs -= 1;
      use_index = true;
    } else {
      use_index = false;
    }
    if (floppy->track() == -1)
      floppy->goto_track(0);

    uint32_t capture_ms = 0;
    if (flux_ticks) {
      capture_ms = 1000.0f * (float)flux_ticks / (float)GW_SAMPLE_FREQ;
      revs = 1;
    }

    reply_buffer[i++] = GW_ACK_OK;
    cdc_write_all(reply_buffer, 2);

    while (revs--) {
      int32_t index_offset;
      uint32_t captured_pulses = floppy->capture_track(
          flux_transitions, sizeof(flux_transitions), &index_offset, true,
          capture_ms, use_index ? 250 : 0);

      uint8_t idx[6] = {0xFF, 1, 1, 1, 1, 1}; // FLUXOP INDEX marker
      cdc_write_all(idx, 6);

      uint8_t *flux_ptr = flux_transitions;
      while (index_offset > 0 && captured_pulses > 0) {
        int32_t to_send =
            tmin((int32_t)captured_pulses, tmin(index_offset, (int32_t)256));
        cdc_write_all(flux_ptr, to_send);
        flux_ptr += to_send;
        captured_pulses -= to_send;
        index_offset -= to_send;
      }
      if (!revs || capture_ms)
        cdc_write_all(idx, 6);
      if (capture_ms) {
        while (captured_pulses) {
          uint32_t to_send = tmin(captured_pulses, (uint32_t)256);
          cdc_write_all(flux_ptr, to_send);
          flux_ptr += to_send;
          captured_pulses -= to_send;
        }
      }
    }
    while (cdc_avail()) {
      uint8_t b;
      tud_cdc_read(&b, 1);
    } // flush input (fluxengine quirk)
    cdc_write_byte(0);
    flux_status = true;
  }

  else if (cmd == GW_CMD_WRITEFLUX) {
    if (!floppy) {
      reply_buffer[i++] = GW_ACK_BADCMD;
      cdc_write_all(reply_buffer, 2);
      return;
    }
    if (floppy->get_write_protect()) {
      reply_buffer[i++] = GW_ACK_WRPROT;
      cdc_write_all(reply_buffer, 2);
    } else {
      uint8_t cue_at_index = cmd_buffer[2];
      reply_buffer[i++] = GW_ACK_OK;
      cdc_write_all(reply_buffer, 2);

      uint32_t fluxors = 0;
      uint8_t flux = 0xFF;
      bool stalled = false;
      while (flux && (fluxors < MAX_FLUX_PULSE_PER_TRACK)) {
        int b = cdc_read1_to(CDC_STREAM_TIMEOUT_MS, false);
        if (b < 0) {
          stalled = true; // the host stopped sending: never write a fragment
          break;
        }
        flux = (uint8_t)b;
        flux_transitions[fluxors++] = flux;
      }
      if (fluxors == MAX_FLUX_PULSE_PER_TRACK || stalled || fluxors < 8) {
        // overrun, a stalled stream, or one too short to hold the 7-byte
        // trailer (fluxors - 7 would wrap): bail with a status the host sees
        // on GETFLUXSTATUS
        flux_status = false;
      } else {
        flux_status = floppy->write_track(flux_transitions, fluxors - 7, true,
                                          cue_at_index);
      }
      cdc_write_byte(0);
    }
  }

  else if (cmd == GW_CMD_GETFLUXSTATUS) {
    reply_buffer[i++] = flux_status ? GW_ACK_OK : GW_ACK_BADPIN;
    cdc_write_all(reply_buffer, 2);
  }

  else if (cmd == GW_CMD_SINKBYTES) {
    uint32_t numbytes = ((uint32_t)cmd_buffer[5] << 24) |
                        ((uint32_t)cmd_buffer[4] << 16) |
                        ((uint32_t)cmd_buffer[3] << 8) | cmd_buffer[2];
    reply_buffer[i++] = GW_ACK_OK;
    cdc_write_all(reply_buffer, 2);
    bandwidth_timer = millis();
    transfered_bytes = numbytes;
    uint32_t last_rx = millis();
    while (numbytes != 0) {
      uint32_t avail = cdc_avail();
      if (avail == 0) {
        tud_task();
        if (millis() - last_rx > CDC_STREAM_TIMEOUT_MS)
          break; // the host stopped sending (#9)
        continue;
      }
      last_rx = millis();
      uint32_t to_read = tmin(numbytes, tmin((uint32_t)sizeof(reply_buffer), avail));
      numbytes -= tud_cdc_read(reply_buffer, to_read);
    }
    bandwidth_timer = millis() - bandwidth_timer;
    cdc_write_byte(GW_ACK_OK);
  }

  else if (cmd == GW_CMD_SOURCEBYTES) {
    uint32_t numbytes = ((uint32_t)cmd_buffer[5] << 24) |
                        ((uint32_t)cmd_buffer[4] << 16) |
                        ((uint32_t)cmd_buffer[3] << 8) | cmd_buffer[2];
    uint32_t seed = ((uint32_t)cmd_buffer[9] << 24) |
                    ((uint32_t)cmd_buffer[8] << 16) |
                    ((uint32_t)cmd_buffer[7] << 8) | cmd_buffer[6];
    reply_buffer[i++] = GW_ACK_OK;
    cdc_write_all(reply_buffer, 2);
    bandwidth_timer = millis();
    transfered_bytes = numbytes;
    uint32_t randnum = seed;
    while (numbytes != 0) {
      uint32_t to_write = tmin(numbytes, (uint32_t)sizeof(reply_buffer));
      for (uint32_t k = 0; k < to_write; k++) {
        reply_buffer[k] = randnum;
        if (randnum & 0x01)
          randnum = (randnum >> 1) ^ 0x80000062;
        else
          randnum >>= 1;
      }
      cdc_write_all(reply_buffer, to_write);
      numbytes -= to_write;
    }
    bandwidth_timer = millis() - bandwidth_timer;
  }

  else if (cmd == GW_CMD_GETPIN) {
    if (!floppy) { // same guard as SETPIN: null until SETBUSTYPE
      reply_buffer[i++] = GW_ACK_BADCMD;
      cdc_write_all(reply_buffer, 2);
      return;
    }
    uint32_t pin = cmd_buffer[2];
    if (pin == GW_CMD_GETPIN_TRACK0) {
      reply_buffer[i++] = GW_ACK_OK;
      reply_buffer[i++] = !floppy->get_track0_sense();
    } else {
      reply_buffer[i++] = GW_ACK_BADPIN;
      reply_buffer[i++] = 0;
    }
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == GW_CMD_SETPIN) {
    if (!floppy) {
      reply_buffer[i++] = GW_ACK_BADCMD;
      cdc_write_all(reply_buffer, 2);
      return;
    }
    uint32_t pin = cmd_buffer[2];
    bool value = cmd_buffer[3];
    if (pin == GW_CMD_SETPIN_DENSITY) {
      reply_buffer[i++] = floppy->set_density(value) ? GW_ACK_OK : GW_ACK_BADCMD;
    } else {
      reply_buffer[i++] = GW_ACK_BADPIN;
    }
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_SCAN) {
    // MZPico Phase-2 validation: seek a track, capture raw flux, decode its
    // sectors on-device, and report the layout. cmd_buffer[2]=track,
    // [3]=side, [4]=density (0=DD/2.0us, 1=HD/1.0us).
    // Follow the FDC's drive select rather than the greaseweazle drive: with
    // `drive 1` set, this must read drive 1. Hardwired to drive 0 it does not
    // merely mislead -- fmtverify would format one disk and verify another, and
    // a single-drive check of the select lines would always pass. Null = the
    // host selected a unit this board has no select line for.
    FloppyDrive *floppy = fdc_bench_drive();
    if (!floppy) {
      reply_buffer[i++] = GW_ACK_BADCMD;
      cdc_write_all(reply_buffer, 2);
      return;
    }
    uint8_t track = cmd_buffer[2];
    uint8_t sidev = cmd_buffer[3];
    uint8_t dens = cmd_buffer[4];
    float bt = dens ? 1.0f : 2.0f;

    floppy->select(true);
    floppy->spin_motor(true);
    floppy->side(sidev);
    floppy->goto_track(track);

    int32_t idx;
    uint32_t nflux = floppy->capture_track(flux_transitions,
                                           sizeof(flux_transitions), &idx,
                                           /*store_greaseweazle=*/false,
                                           /*capture_ms=*/fdc_bench_capture_ms(track),
                                           /*index_wait_ms=*/250);
    int ns = mfm_scan_track(flux_transitions, nflux, bt, g_scan_sectors,
                            MZP_MAX_SECTORS, g_scan_scratch,
                            sizeof(g_scan_scratch));

    floppy->spin_motor(false);
    fdc_bench_release();

    // Report: [cmd, ACK, ns, nflux(4 LE), then {cyl,head,sec,n,flags} * ns].
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = (uint8_t)ns;
    reply_buffer[i++] = nflux & 0xFF;
    reply_buffer[i++] = (nflux >> 8) & 0xFF;
    reply_buffer[i++] = (nflux >> 16) & 0xFF;
    reply_buffer[i++] = (nflux >> 24) & 0xFF;
    cdc_write_all(reply_buffer, i);
    for (int k = 0; k < ns; k++) {
      SectorInfo *s = &g_scan_sectors[k];
      uint8_t rec[5] = {
          s->cyl, s->head, s->sec, s->n,
          (uint8_t)((s->id_crc_ok ? 1 : 0) | (s->data_crc_ok ? 2 : 0) |
                    (s->has_data ? 4 : 0) | (s->deleted ? 8 : 0))};
      cdc_write_all(rec, 5);
    }
  }

  else if (cmd == MZP_CMD_STREAMSCAN) {
    // A/B test: same seek + decode as MZP_CMD_SCAN, but flux comes from the
    // incremental (streaming) capture API instead of the batch capture_track
    // call, to isolate whether that new capture path itself produces valid,
    // decodable flux -- independent of any live-FDC timing/settle questions.
    // cmd_buffer[2]=track, [3]=side, [4]=density, [5..6]=capture_ms (LE).
    // Follow the FDC's drive select rather than the greaseweazle drive: with
    // `drive 1` set, this must read drive 1. Hardwired to drive 0 it does not
    // merely mislead -- fmtverify would format one disk and verify another, and
    // a single-drive check of the select lines would always pass. Null = the
    // host selected a unit this board has no select line for.
    FloppyDrive *floppy = fdc_bench_drive();
    if (!floppy) {
      reply_buffer[i++] = GW_ACK_BADCMD;
      cdc_write_all(reply_buffer, 2);
      return;
    }
    uint8_t track = cmd_buffer[2];
    uint8_t sidev = cmd_buffer[3];
    uint8_t dens = cmd_buffer[4];
    uint16_t capture_ms = (uint16_t)cmd_buffer[5] | ((uint16_t)cmd_buffer[6] << 8);
    if (capture_ms == 0)
      capture_ms = (uint16_t)fdc_bench_capture_ms(track);
    float bt = dens ? 1.0f : 2.0f;

    floppy->select(true);
    floppy->spin_motor(true);
    floppy->side(sidev);
    floppy->goto_track(track);

    uint32_t nflux = 0;
    if (rp2040_flux_capture_start(PIN_INDEX, PIN_RDATA)) {
      uint32_t t0 = to_ms_since_boot(get_absolute_time());
      while (to_ms_since_boot(get_absolute_time()) - t0 < capture_ms &&
             nflux < sizeof(flux_transitions)) {
        nflux = rp2040_flux_capture_drain(flux_transitions,
                                          sizeof(flux_transitions), nflux,
                                          /*store_greaseweazle=*/false);
      }
      rp2040_flux_capture_stop();
    }
    int ns = mfm_scan_track(flux_transitions, nflux, bt, g_scan_sectors,
                            MZP_MAX_SECTORS, g_scan_scratch,
                            sizeof(g_scan_scratch));

    floppy->spin_motor(false);
    fdc_bench_release();

    // Same reply shape as MZP_CMD_SCAN: [cmd, ACK, ns, nflux(4 LE)] then
    // {cyl,head,sec,n,flags} * ns.
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = (uint8_t)ns;
    reply_buffer[i++] = nflux & 0xFF;
    reply_buffer[i++] = (nflux >> 8) & 0xFF;
    reply_buffer[i++] = (nflux >> 16) & 0xFF;
    reply_buffer[i++] = (nflux >> 24) & 0xFF;
    cdc_write_all(reply_buffer, i);
    for (int k = 0; k < ns; k++) {
      SectorInfo *s = &g_scan_sectors[k];
      uint8_t rec[5] = {
          s->cyl, s->head, s->sec, s->n,
          (uint8_t)((s->id_crc_ok ? 1 : 0) | (s->data_crc_ok ? 2 : 0) |
                    (s->has_data ? 4 : 0) | (s->deleted ? 8 : 0))};
      cdc_write_all(rec, 5);
    }
  }

  else if (cmd == MZP_CMD_ENCTEST) {
    // Encoder validation, no disk: encode a synthetic uniform track then decode
    // it with the scanner and count sectors that round-trip with valid CRCs.
    // cmd_buffer[2]=n_sectors, [3]=size_code (0..2), [4]=density.
    uint8_t nsec = cmd_buffer[2];
    uint8_t sizecode = cmd_buffer[3] & 3;
    uint8_t dens = cmd_buffer[4];
    float bt = dens ? 1.0f : 2.0f;
    if (sizecode > 2)
      sizecode = 2; // buffer holds up to 512-byte sectors
    if (nsec > MZP_MAX_ENC_SECTORS)
      nsec = MZP_MAX_ENC_SECTORS;
    size_t ssize = 128u << sizecode;

    for (size_t k = 0; k < (size_t)nsec * ssize; k++)
      g_enc_sectors[k] = (uint8_t)(k * 7 + 3); // deterministic pattern

    size_t nflux =
        mfm_encode_track(flux_transitions, sizeof(flux_transitions),
                         g_enc_sectors, nsec, sizecode, bt, 0, 0);
    int ns = mfm_scan_track(flux_transitions, nflux, bt, g_scan_sectors,
                            MZP_MAX_SECTORS, g_scan_scratch,
                            sizeof(g_scan_scratch));
    int valid = 0;
    for (int k = 0; k < ns; k++)
      if (g_scan_sectors[k].id_crc_ok && g_scan_sectors[k].data_crc_ok)
        valid++;

    // Report: [cmd, ACK, nsec, ns, valid, nflux(4 LE)] then {..}*ns.
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = nsec;
    reply_buffer[i++] = (uint8_t)ns;
    reply_buffer[i++] = (uint8_t)valid;
    reply_buffer[i++] = nflux & 0xFF;
    reply_buffer[i++] = (nflux >> 8) & 0xFF;
    reply_buffer[i++] = (nflux >> 16) & 0xFF;
    reply_buffer[i++] = (nflux >> 24) & 0xFF;
    cdc_write_all(reply_buffer, i);
    for (int k = 0; k < ns; k++) {
      SectorInfo *s = &g_scan_sectors[k];
      uint8_t rec[5] = {
          s->cyl, s->head, s->sec, s->n,
          (uint8_t)((s->id_crc_ok ? 1 : 0) | (s->data_crc_ok ? 2 : 0) |
                    (s->has_data ? 4 : 0) | (s->deleted ? 8 : 0))};
      cdc_write_all(rec, 5);
    }
  }

  else if (cmd == MZP_CMD_BUSDUMP) {
    // Per-port telemetry for ports 0xD8..0xDF:
    // [cmd, ACK, last_raw[8], wr[8]*4 LE, rd[8]*4 LE, addr_low, addr_high,
    //  dat_low, dat_high] -- 78 bytes.
    reply_buffer[i++] = GW_ACK_OK;
    for (int k = 0; k < 8; k++)
      reply_buffer[i++] = g_bus_hello.last_raw[k];
    for (int k = 0; k < 8; k++) {
      uint32_t w = g_bus_hello.wr[k];
      for (int b = 0; b < 4; b++)
        reply_buffer[i++] = (w >> (8 * b)) & 0xFF;
    }
    for (int k = 0; k < 8; k++) {
      uint32_t r = g_bus_hello.rd[k];
      for (int b = 0; b < 4; b++)
        reply_buffer[i++] = (r >> (8 * b)) & 0xFF;
    }
    reply_buffer[i++] = g_bus_hello.addr_low;
    reply_buffer[i++] = g_bus_hello.addr_high;
    reply_buffer[i++] = g_bus_hello.dat_low;
    reply_buffer[i++] = g_bus_hello.dat_high;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_DRIVE) {
    // Bench hardware isolation: drive C0..C7 to a value and hold.
    // cmd_buffer[2]=value, [3]=mode (0 release, 1 Pico pins only, 2 to bus).
    bus_drive_test(cmd_buffer[2], cmd_buffer[3]);
    reply_buffer[i++] = GW_ACK_OK;
    cdc_write_all(reply_buffer, 2);
  }

  else if (cmd == MZP_CMD_GPIOREAD) {
    // Bench pin-read: report the live C0..C7 input byte. cmd_buffer[2] != 0 ->
    // isolate (halt SMs, C-pins = SIO in, no pull) so a jumper on C0 owns it.
    uint8_t raw = bus_read_pins(cmd_buffer[2] != 0);
    if (cmd_buffer[2] != 0)
      bus_bench_restore(); // the read is done: give the bus back (#17)
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = raw;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_FDCDUMP) {
    // [cmd, ACK, status, track, sector, data, command, flags, drive,
    //  at_track0, phys_track, op_seq, op_done,
    //  rd_flags, rd_sector, xfer_pos(2 LE), xfer_len(2 LE), rd_seq, rd_done,
    //  rd_elapsed_us(4 LE), cmd_gap_us(4 LE), verify_start_ok, verify_n(4 LE),
    //  cmd_writes_total(2 LE), cmd_last_raw, cmd_last_decoded,
    //  bt_active, bt_last_rx, bt_last_echo,
    //  bt_rx_count(2 LE), bt_rd_count(2 LE), bt_err_count(2 LE)].
    FdcDebug d;
    fdc_debug_snapshot(&d);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = d.status;
    reply_buffer[i++] = d.track;
    reply_buffer[i++] = d.sector;
    reply_buffer[i++] = d.data;
    reply_buffer[i++] = d.command;
    reply_buffer[i++] = d.flags;
    reply_buffer[i++] = d.drive;
    reply_buffer[i++] = d.at_track0;
    reply_buffer[i++] = d.phys_track;
    reply_buffer[i++] = d.op_seq;
    reply_buffer[i++] = d.op_done;
    reply_buffer[i++] = d.rd_flags;
    reply_buffer[i++] = d.rd_sector;
    reply_buffer[i++] = d.xfer_pos & 0xFF;
    reply_buffer[i++] = (d.xfer_pos >> 8) & 0xFF;
    reply_buffer[i++] = d.xfer_len & 0xFF;
    reply_buffer[i++] = (d.xfer_len >> 8) & 0xFF;
    reply_buffer[i++] = d.rd_seq;
    reply_buffer[i++] = d.rd_done;
    reply_buffer[i++] = d.rd_elapsed_us & 0xFF;
    reply_buffer[i++] = (d.rd_elapsed_us >> 8) & 0xFF;
    reply_buffer[i++] = (d.rd_elapsed_us >> 16) & 0xFF;
    reply_buffer[i++] = (d.rd_elapsed_us >> 24) & 0xFF;
    reply_buffer[i++] = d.cmd_gap_us & 0xFF;
    reply_buffer[i++] = (d.cmd_gap_us >> 8) & 0xFF;
    reply_buffer[i++] = (d.cmd_gap_us >> 16) & 0xFF;
    reply_buffer[i++] = (d.cmd_gap_us >> 24) & 0xFF;
    reply_buffer[i++] = d.verify_start_ok;
    reply_buffer[i++] = d.verify_n & 0xFF;
    reply_buffer[i++] = (d.verify_n >> 8) & 0xFF;
    reply_buffer[i++] = (d.verify_n >> 16) & 0xFF;
    reply_buffer[i++] = (d.verify_n >> 24) & 0xFF;
    reply_buffer[i++] = d.cmd_writes_total & 0xFF;
    reply_buffer[i++] = (d.cmd_writes_total >> 8) & 0xFF;
    reply_buffer[i++] = d.cmd_last_raw;
    reply_buffer[i++] = d.cmd_last_decoded;
    reply_buffer[i++] = d.bt_active;
    reply_buffer[i++] = d.bt_last_rx;
    reply_buffer[i++] = d.bt_last_echo;
    reply_buffer[i++] = d.bt_rx_count & 0xFF;
    reply_buffer[i++] = (d.bt_rx_count >> 8) & 0xFF;
    reply_buffer[i++] = d.bt_rd_count & 0xFF;
    reply_buffer[i++] = (d.bt_rd_count >> 8) & 0xFF;
    reply_buffer[i++] = d.bt_err_count & 0xFF;
    reply_buffer[i++] = (d.bt_err_count >> 8) & 0xFF;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_FDCSEEK) {
    // Bench (no MZ-800): synchronously select + motor + seek and report the
    // result. cmd_buffer[2]=target, [3..4]=param ms (LE), [5]=mode. Same
    // layout as FDCDUMP, so the probe reuses fdc_line().
    uint8_t target = cmd_buffer[2];
    uint16_t param = (uint16_t)cmd_buffer[3] | ((uint16_t)cmd_buffer[4] << 8);
    uint8_t mode = cmd_buffer[5];
    FdcDebug d;
    fdc_bench_seek(target, param, mode, &d);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = d.status;
    reply_buffer[i++] = d.track;
    reply_buffer[i++] = d.sector;
    reply_buffer[i++] = d.data;
    reply_buffer[i++] = d.command;
    reply_buffer[i++] = d.flags;
    reply_buffer[i++] = d.drive;
    reply_buffer[i++] = d.at_track0;
    reply_buffer[i++] = d.phys_track;
    reply_buffer[i++] = d.op_seq;
    reply_buffer[i++] = d.op_done;
    reply_buffer[i++] = d.rd_flags;
    reply_buffer[i++] = d.rd_sector;
    reply_buffer[i++] = d.xfer_pos & 0xFF;
    reply_buffer[i++] = (d.xfer_pos >> 8) & 0xFF;
    reply_buffer[i++] = d.xfer_len & 0xFF;
    reply_buffer[i++] = (d.xfer_len >> 8) & 0xFF;
    reply_buffer[i++] = d.rd_seq;
    reply_buffer[i++] = d.rd_done;
    reply_buffer[i++] = d.rd_elapsed_us & 0xFF;
    reply_buffer[i++] = (d.rd_elapsed_us >> 8) & 0xFF;
    reply_buffer[i++] = (d.rd_elapsed_us >> 16) & 0xFF;
    reply_buffer[i++] = (d.rd_elapsed_us >> 24) & 0xFF;
    reply_buffer[i++] = d.cmd_gap_us & 0xFF;
    reply_buffer[i++] = (d.cmd_gap_us >> 8) & 0xFF;
    reply_buffer[i++] = (d.cmd_gap_us >> 16) & 0xFF;
    reply_buffer[i++] = (d.cmd_gap_us >> 24) & 0xFF;
    reply_buffer[i++] = d.verify_start_ok;
    reply_buffer[i++] = d.verify_n & 0xFF;
    reply_buffer[i++] = (d.verify_n >> 8) & 0xFF;
    reply_buffer[i++] = (d.verify_n >> 16) & 0xFF;
    reply_buffer[i++] = (d.verify_n >> 24) & 0xFF;
    reply_buffer[i++] = d.cmd_writes_total & 0xFF;
    reply_buffer[i++] = (d.cmd_writes_total >> 8) & 0xFF;
    reply_buffer[i++] = d.cmd_last_raw;
    reply_buffer[i++] = d.cmd_last_decoded;
    reply_buffer[i++] = d.bt_active;
    reply_buffer[i++] = d.bt_last_rx;
    reply_buffer[i++] = d.bt_last_echo;
    reply_buffer[i++] = d.bt_rx_count & 0xFF;
    reply_buffer[i++] = (d.bt_rx_count >> 8) & 0xFF;
    reply_buffer[i++] = d.bt_rd_count & 0xFF;
    reply_buffer[i++] = (d.bt_rd_count >> 8) & 0xFF;
    reply_buffer[i++] = d.bt_err_count & 0xFF;
    reply_buffer[i++] = (d.bt_err_count >> 8) & 0xFF;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_RDDIAG) {
    // [cmd, ACK, capture_ok, flux_n(4), capture_us(4), search_us(4),
    //  rd_elapsed_us(4), want, result, ids_seen, ids[FDC_IDS_N=20],
    //  status_polls(4), status_span_us(4),
    //  int_flags, int_mode, intrq_asserts(4), lost_data_events(4),
    //  write_attempts(4), verify_start_ok, verify_n(4), seek_error,
    //  verify_attempts(4), sec_ok(4), sec_crc_err(4), sec_rnf(4),
    //  last_id_cyl, last_id_head, sec_cyl_mismatch(4), last_trk_reg,
    //  last_want_side, last_head_target, last_phys_track,
    //  rev_us(4), window_us(4)] -- 104 bytes.
    FdcDebug d;
    fdc_debug_snapshot(&d);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = d.rd_capture_ok;
    const uint32_t v32[] = {d.rd_flux_n, d.rd_capture_us, d.rd_search_us,
                            d.rd_elapsed_us};
    for (uint32_t v : v32) {
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (v >> (8 * k)) & 0xFF;
    }
    reply_buffer[i++] = d.rd_want;
    reply_buffer[i++] = d.rd_result;
    reply_buffer[i++] = d.rd_ids_seen;
    for (int k = 0; k < FDC_IDS_N; k++)
      reply_buffer[i++] = d.rd_ids[k];
    const uint32_t p32[] = {d.status_polls, d.status_span_us};
    for (uint32_t v : p32) {
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (v >> (8 * k)) & 0xFF;
    }
    reply_buffer[i++] = d.int_flags;
    reply_buffer[i++] = d.int_mode;
    const uint32_t n32[] = {d.intrq_asserts, d.lost_data_events,
                            d.write_attempts};
    for (uint32_t v : n32) {
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (v >> (8 * k)) & 0xFF;
    }
    // Type I Verify: a failed Verify sets SEEK ERROR and the boot ROM stops
    // there without ever issuing a READ, so a read-only diagnostic cannot see
    // why. start_ok/n separate "capture never started" from "captured plenty
    // and genuinely found no matching ID".
    reply_buffer[i++] = d.verify_start_ok;
    for (int k = 0; k < 4; k++)
      reply_buffer[i++] = (d.verify_n >> (8 * k)) & 0xFF;
    reply_buffer[i++] = (uint8_t)((d.status & 0x10) ? 1 : 0); // SEEK ERROR
    for (int k = 0; k < 4; k++)
      reply_buffer[i++] = (d.verify_attempts >> (8 * k)) & 0xFF;
    // Cumulative read quality. The fields above all describe the LAST read;
    // a directory that lists garbage is a question about every read before it.
    const uint32_t q32[] = {d.sec_ok, d.sec_crc_err, d.sec_rnf};
    for (uint32_t v : q32) {
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (v >> (8 * k)) & 0xFF;
    }
    reply_buffer[i++] = d.last_id_cyl;
    reply_buffer[i++] = d.last_id_head;
    for (int k = 0; k < 4; k++)
      reply_buffer[i++] = (d.sec_cyl_mismatch >> (8 * k)) & 0xFF;
    reply_buffer[i++] = d.last_trk_reg;
    reply_buffer[i++] = d.last_want_side;
    reply_buffer[i++] = d.last_head_target;
    reply_buffer[i++] = d.last_phys_track;
    const uint32_t w32[] = {d.rev_us, d.window_us};
    for (uint32_t v : w32) {
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (v >> (8 * k)) & 0xFF;
    }
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_SIDESWAP) {
    // cmd_buffer[2]: 0 = normal, 1 = invert, 2 = query only.
    if (cmd_buffer[2] < 2)
      fdc_set_side_invert(cmd_buffer[2] != 0);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = fdc_get_side_invert() ? 1 : 0;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_IOTRACE) {
    // Header: ack, count, total(4). Then `count` x 5 bytes:
    // port, value, is_read, rep_lo, rep_hi -- oldest first.
    uint8_t n = fdc_iotrace_count();
    uint32_t total = fdc_iotrace_total();
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = n;
    for (int k = 0; k < 4; ++k)
      reply_buffer[i++] = (uint8_t)(total >> (8 * k));
    cdc_write_all(reply_buffer, i);
    for (uint8_t k = 0; k < n; ++k) {
      uint8_t rec[5];
      uint16_t rep = 0;
      if (!fdc_iotrace_get(k, &rec[0], &rec[1], &rec[2], &rep))
        break;
      rec[3] = (uint8_t)(rep & 0xFF);
      rec[4] = (uint8_t)(rep >> 8);
      cdc_write_all(rec, 5);
    }
  }

  else if (cmd == MZP_CMD_INTMODE) {
    // cmd_buffer[2]: 0/1/2 = set the ~INT drive mode, 0xFF = query only.
    if (cmd_buffer[2] <= 2)
      fdc_set_int_mode(cmd_buffer[2]);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = fdc_get_int_mode();
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_BUSTEST_RESET) {
    fdc_bustest_reset();
    reply_buffer[i++] = GW_ACK_OK;
    cdc_write_all(reply_buffer, 2);
  }

  else if (cmd == MZP_CMD_BUSTEST) {
    // [cmd, ACK, active, last_rx, last_echo, rx(4), rd(4), err(4), err_gap(4),
    //  err_garbage(4), bits_want1, bits_want0, mm_n, mm_exp[8], mm_got[8],
    //  wr_late_at_pop(4), reserved(4) (was wr_late_at_sample; 0), rd_late_at_pop(4),
    //  wr_backlog(4), rd_backlog(4), bit_high[8]*4, err_adjacent(4),
    //  rd_reports(4), rd_err(4), rd_bits_want1, rd_bits_want0, rd_last_exp,
    //  rd_last_got, rd_bit_high[8]*4, rd_late_at_drive(4)]  -- 148 bytes.
    FdcBusTest b;
    fdc_bustest_snapshot(&b);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = b.active;
    reply_buffer[i++] = b.last_rx;
    reply_buffer[i++] = b.last_echo;
    const uint32_t w32[] = {b.rx_count, b.rd_count, b.err_count,
                            b.err_gap,  b.err_garbage};
    for (uint32_t v : w32) {
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (v >> (8 * k)) & 0xFF;
    }
    reply_buffer[i++] = b.bits_want1;
    reply_buffer[i++] = b.bits_want0;
    reply_buffer[i++] = b.mm_n;
    for (int k = 0; k < 8; k++)
      reply_buffer[i++] = b.mm_exp[k];
    for (int k = 0; k < 8; k++)
      reply_buffer[i++] = b.mm_got[k];
    const uint32_t d32[] = {
        g_bus_diag.wr_late_at_pop, 0u /*reserved*/,
        g_bus_diag.rd_late_at_pop, g_bus_diag.wr_backlog,
        g_bus_diag.rd_backlog};
    for (uint32_t v : d32) {
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (v >> (8 * k)) & 0xFF;
    }
    for (int j = 0; j < 8; j++)
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (b.bit_high[j] >> (8 * k)) & 0xFF;
    for (int k = 0; k < 4; k++)
      reply_buffer[i++] = (b.err_adjacent >> (8 * k)) & 0xFF;
    const uint32_t rd32[] = {b.rd_reports, b.rd_err};
    for (uint32_t v : rd32) {
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (v >> (8 * k)) & 0xFF;
    }
    reply_buffer[i++] = b.rd_bits_want1;
    reply_buffer[i++] = b.rd_bits_want0;
    reply_buffer[i++] = b.rd_last_exp;
    reply_buffer[i++] = b.rd_last_got;
    for (int j = 0; j < 8; j++)
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (b.rd_bit_high[j] >> (8 * k)) & 0xFF;
    for (int k = 0; k < 4; k++) // appended: older probes read the first 144
      reply_buffer[i++] = (g_bus_diag.rd_late_at_drive >> (8 * k)) & 0xFF;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_RECALDIAG) {
    // [cmd, ACK, fails(4), retry_ok(4), retry_fail(4), first_valid,
    //  first_at_ms(4), first_drive, first_site, first_target, first_state,
    //  first_motor_on_ms(4), first_retry] -- 28 bytes. Read-only; cleared
    //  together with the firstfail latch.
    FdcRecalDiag d;
    fdc_recaldiag_snapshot(&d);
    reply_buffer[i++] = GW_ACK_OK;
    const uint32_t a32[] = {d.fails, d.retry_ok, d.retry_fail};
    for (uint32_t v : a32)
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (v >> (8 * k)) & 0xFF;
    reply_buffer[i++] = d.first_valid;
    for (int k = 0; k < 4; k++)
      reply_buffer[i++] = (d.first_at_ms >> (8 * k)) & 0xFF;
    reply_buffer[i++] = d.first_drive;
    reply_buffer[i++] = d.first_site;
    reply_buffer[i++] = d.first_target;
    reply_buffer[i++] = d.first_state;
    for (int k = 0; k < 4; k++)
      reply_buffer[i++] = (d.first_motor_on_ms >> (8 * k)) & 0xFF;
    reply_buffer[i++] = d.first_retry;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_RDWIN) {
    // [cmd, len, track, sector, side, legacy, delay_us(4)] ->
    // [cmd, ACK, flags, flux_n(4), capture_us(4), rev_us(4)]
    uint32_t delay = (uint32_t)cmd_buffer[6] | ((uint32_t)cmd_buffer[7] << 8) |
                     ((uint32_t)cmd_buffer[8] << 16) |
                     ((uint32_t)cmd_buffer[9] << 24);
    if (delay > 1000000)
      delay = 1000000;
    FdcRdWin w;
    bool ok = fdc_bench_read_window(cmd_buffer[2], cmd_buffer[3], cmd_buffer[4],
                                    cmd_buffer[5], delay, &w);
    reply_buffer[i++] = ok ? GW_ACK_OK : GW_ACK_NOUNIT;
    reply_buffer[i++] = w.flags;
    const uint32_t v32[] = {w.flux_n, w.capture_us, w.rev_us};
    for (uint32_t v : v32) {
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (v >> (8 * k)) & 0xFF;
    }
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_FDCREAD) {
    // Bench (no MZ-800): synchronously seek + capture + decode one sector and
    // stream its data bytes. cmd_buffer[2]=track,[3]=sector,[4]=side,[5]=density.
    // Reply header: [cmd, ACK, flags, size_lo, size_hi] then `size` data bytes.
    // flags: b0 found b1 has_data b2 data_crc_ok b3 deleted.
    uint8_t track = cmd_buffer[2];
    uint8_t sector = cmd_buffer[3];
    uint8_t side = cmd_buffer[4];
    uint8_t density = cmd_buffer[5];
    uint8_t flags = 0;
    // Decode into g_scan_scratch (1024 B); the capture buffer is separate.
    uint16_t n = fdc_bench_read_sector(track, sector, side, density,
                                       g_scan_scratch, sizeof(g_scan_scratch),
                                       &flags);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = flags;
    reply_buffer[i++] = n & 0xFF;
    reply_buffer[i++] = (n >> 8) & 0xFF;
    cdc_write_all(reply_buffer, i);
    // Stream the sector payload in CDC-sized chunks (like READFLUX).
    uint16_t sent = 0;
    while (sent < n) {
      uint16_t chunk = (uint16_t)tmin((uint32_t)(n - sent), (uint32_t)256);
      cdc_write_all(g_scan_scratch + sent, chunk);
      sent += chunk;
    }
  }

  else if (cmd == MZP_CMD_FDCWRITE) {
    // Bench (no MZ-800): seek, then splice a generated pattern into one
    // sector's data field. cmd_buffer[2]=track [3]=sector [4]=side
    // [5]=size code N [6]=pattern mode [7]=seed.
    // Reply: [cmd, ACK, wrote, flags, size_lo, size_hi].
    //
    // The payload is generated on this side rather than shipped over USB so the
    // test needs no bulk transfer: the probe reproduces the same three
    // generators and compares what MZP_CMD_FDCREAD gives back. Mode 2 (LFSR)
    // is the one that matters -- ramps and constants are easy MFM, while
    // pseudo-random data exercises the long and short flux intervals that a
    // marginal splice or a mistimed write-current turn-on shows up in.
    uint8_t track = cmd_buffer[2];
    uint8_t sector = cmd_buffer[3];
    uint8_t side = cmd_buffer[4];
    uint8_t size_n = cmd_buffer[5] & 7;
    uint8_t mode = cmd_buffer[6];
    uint8_t seed = cmd_buffer[7];
    uint16_t n = (uint16_t)(128u << size_n);
    if (n > sizeof(g_scan_scratch))
      n = sizeof(g_scan_scratch);

    uint16_t lfsr = (uint16_t)((seed << 8) | 0xA5);
    for (uint16_t k = 0; k < n; ++k) {
      if (mode == 0) {
        g_scan_scratch[k] = (uint8_t)(seed + k);
      } else if (mode == 1) {
        g_scan_scratch[k] = seed;
      } else {
        for (int b = 0; b < 8; ++b) {
          uint16_t bit = (uint16_t)((lfsr ^ (lfsr >> 2) ^ (lfsr >> 3) ^
                                     (lfsr >> 5)) & 1u);
          lfsr = (uint16_t)((lfsr >> 1) | (bit << 15));
        }
        g_scan_scratch[k] = (uint8_t)(lfsr & 0xFF);
      }
    }

    uint8_t flags = 0;
    bool wrote =
        fdc_bench_write_sector(track, sector, side, g_scan_scratch, n, &flags);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = wrote ? 1 : 0;
    reply_buffer[i++] = flags;
    reply_buffer[i++] = n & 0xFF;
    reply_buffer[i++] = (n >> 8) & 0xFF;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_FDCFORMAT) {
    // Bench (no MZ-800): format one track. cmd_buffer[2]=track [3]=side
    // [4]=n_sectors [5]=size code [6]=interleave [7]=fill byte.
    // Reply: [cmd, ACK, result, img_lo, img_hi]. result: 0 ok, 1 write
    // protected, 2 image too long, 3 write failed.
    uint8_t result = 0;
    uint16_t n = fdc_bench_format(cmd_buffer[2], cmd_buffer[3], cmd_buffer[4],
                                  cmd_buffer[5], cmd_buffer[6], cmd_buffer[7],
                                  &result);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = result;
    reply_buffer[i++] = n & 0xFF;
    reply_buffer[i++] = (n >> 8) & 0xFF;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_READTRACK) {
    // Bench (no MZ-800): capture a revolution and stream the raw decoded byte
    // stream -- gaps, address marks, CRCs, everything, uninterpreted.
    // cmd_buffer[2]=track [3]=side. Reply: [cmd, ACK, n_lo, n_hi] + n bytes.
    uint8_t track = cmd_buffer[2], side = cmd_buffer[3];
    uint16_t n = fdc_bench_read_track(track, side, flux_transitions,
                                      sizeof(flux_transitions));
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = n & 0xFF;
    reply_buffer[i++] = (n >> 8) & 0xFF;
    cdc_write_all(reply_buffer, i);
    uint16_t sent = 0;
    while (sent < n) {
      uint16_t chunk = (uint16_t)tmin((uint32_t)(n - sent), (uint32_t)256);
      cdc_write_all(fdc_track_buffer() + sent, chunk);
      sent += chunk;
    }
  }

  else if (cmd == MZP_CMD_RESETDIAG) {
    // cmd_buffer[2]: 1 = clear the counters after reporting.
    // [cmd, ACK, line, samples(4), rise(4), fall(4), high_max(4), reserved(4),
    //  last_high(4), events(4), at_ms(4)] -- 35 bytes. The reserved slot was
    //  the shortest LOW excursion, never measured with edge IRQs; kept at 0
    //  so older probes still parse the reply.
    FdcResetDiag d;
    fdc_resetdiag_snapshot(&d);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = d.line;
    const uint32_t v32[] = {d.samples,     d.rise,   d.fall,   d.high_max_us,
                            0u /*reserved*/, d.last_high_us, d.events, d.at_ms};
    for (uint32_t v : v32) {
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (v >> (8 * k)) & 0xFF;
    }
    cdc_write_all(reply_buffer, i);
    if (cmd_buffer[2] == 1)
      fdc_resetdiag_clear();
  }

  else if (cmd == MZP_CMD_RESETPOLL) {
    // cmd_buffer[2]: 0 = off, 1 = on, 2 = query. Bus RESET is an edge
    // interrupt on core1 (not a poll, despite the name); this exists to take
    // its handling out of the picture while diagnosing a bus-timing problem,
    // without a reflash changing two things.
    if (cmd_buffer[2] < 2)
      fdc_set_reset_poll(cmd_buffer[2] != 0);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = fdc_get_reset_poll() ? 1 : 0;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_POSFIX) {
    // cmd_buffer[2]: 0 = off, 1 = on, 2 = query. Off restores the seek path to
    // what it was before the head-position self-correction was added, so a
    // seek-timing regression can be bisected without a reflash.
    if (cmd_buffer[2] < 2)
      fdc_set_posfix(cmd_buffer[2] != 0);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = fdc_get_posfix() ? 1 : 0;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_FMTEN) {
    // cmd_buffer[2]: 0 = off, 1 = on (default), 2 = query.
    if (cmd_buffer[2] < 2)
      fdc_set_format_enable(cmd_buffer[2] != 0);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = fdc_get_format_enable() ? 1 : 0;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_DRVSEL) {
    // cmd_buffer[2]: 0..3 = select that unit, 4 (or more) = query only.
    // Bench access to drive 1: every fdcread/fdcwrite/scan/fdcformat below
    // follows the same drive field the MZ-800 writes to port 0xDC, so setting
    // it here points the whole bench path at the second drive.
    if (cmd_buffer[2] < 4)
      fdc_set_drive(cmd_buffer[2]);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = fdc_get_drive();
    // The live levels on both select lines, so "did the select actually move?"
    // is answered by measurement rather than by whether a read happened to
    // succeed. ~DSn is active LOW, so 0 = asserted. A drive that answers on
    // both units shows up here as either both lines low (we never released the
    // old one) or one line low while the other unit still reads the disk (the
    // drive is strapped to answer regardless, or the cable twist puts both on
    // its select input) -- and those want completely different fixes.
    reply_buffer[i++] = (uint8_t)(gpio_get(PIN_DS0) ? 1 : 0);
    reply_buffer[i++] = (uint8_t)(gpio_get(PIN_DS1) ? 1 : 0);
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_DRVSTAT) {
    // [cmd, ACK, cur_unit, ds0, ds1, then per unit: track, target, flags]
    // Unselected units included: the property multi-drive rests on is that two
    // heads stay in DIFFERENT places across selects, and fdcdump can only ever
    // see the selected one.
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = fdc_get_drive();
    reply_buffer[i++] = (uint8_t)(gpio_get(PIN_DS0) ? 1 : 0);
    reply_buffer[i++] = (uint8_t)(gpio_get(PIN_DS1) ? 1 : 0);
    for (int u = 0; u < 2; ++u) {
      int trk = -1, tgt = 0;
      uint8_t fl = 0;
      if (!fdc_drive_state(u, &trk, &tgt, &fl))
        trk = -1;
      reply_buffer[i++] = (trk < 0) ? 0xFF : (uint8_t)trk; // 0xFF = unknown
      reply_buffer[i++] = (uint8_t)tgt;
      reply_buffer[i++] = fl;
    }
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_DENSITY) {
    // cmd_buffer[2]: 0 = force DD, 1 = force HD, 2 = auto, 3 = detect,
    // 4 = query. Reply: [cmd, ACK, mode, unit0_hd, unit1_hd, flips(4 LE),
    //         host_valid, host_hd].
    if (cmd_buffer[2] < 4)
      fdc_set_density_mode(cmd_buffer[2]);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = fdc_get_density_mode();
    reply_buffer[i++] = fdc_drive_density_hd(0) ? 1 : 0;
    reply_buffer[i++] = fdc_drive_density_hd(1) ? 1 : 0;
    uint32_t flips = fdc_density_flips();
    reply_buffer[i++] = flips & 0xFF;
    reply_buffer[i++] = (flips >> 8) & 0xFF;
    reply_buffer[i++] = (flips >> 16) & 0xFF;
    reply_buffer[i++] = (flips >> 24) & 0xFF;
    bool hv = false, hh = false;
    fdc_density_host(&hv, &hh);
    reply_buffer[i++] = hv ? 1 : 0;
    reply_buffer[i++] = hh ? 1 : 0;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_FWVER) {
    // [cmd, ACK, len, hash bytes]. Exists because a whole hardware test
    // session was once diagnosed against the wrong firmware: several stamped
    // uf2s in circulation, no way to ask the board which one it was.
    static const char ver[] = FW_GIT_HASH;
    uint8_t n = (uint8_t)(sizeof(ver) - 1);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = n;
    for (uint8_t k = 0; k < n; k++)
      reply_buffer[i++] = (uint8_t)ver[k];
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_DENSEL) {
    // cmd_buffer[2]: 0/1 = pin that level, 2 = follow density, 3 = query.
    // Reply: [cmd, ACK, override(-1/0/1 as u8), pin level now (0xFF unknown)].
    if (cmd_buffer[2] < 2)
      fdc_set_densel((int8_t)cmd_buffer[2]);
    else if (cmd_buffer[2] == 2)
      fdc_set_densel(-1);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = (uint8_t)fdc_get_densel();
    int lvl = fdc_get_densel_level();
    reply_buffer[i++] = (lvl < 0) ? 0xFF : (uint8_t)lvl;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_IDXBIT) {
    // cmd_buffer[2]: 0 = never, 1 = HD-declared only (default), 2 = always,
    // 3 = query.
    if (cmd_buffer[2] < 3)
      fdc_set_index_bit(cmd_buffer[2]);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = fdc_get_index_bit();
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_DRVLOCK) {
    // cmd_buffer[2]: 0 = multi-drive (default), 1 = pin to drive 0, 2 = query.
    if (cmd_buffer[2] < 2)
      fdc_set_drive_lock(cmd_buffer[2] != 0);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = fdc_get_drive_lock() ? 1 : 0;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_FIRSTFAIL) {
    // cmd_buffer[2]: 1 = clear the latch after reporting (arm for a new run).
    // [cmd, ACK, valid, kind, count(4), at_ms(4), want_sec, trk_reg,
    //  head_target, phys_track, side, seen_cyl, seen_head, capture_ok,
    //  flux_n(4), capture_us(4), ids_seen, ids[FDC_IDS_N=20], pos_resyncs(4),
    //  resync_from, resync_to, reset_events(4), reset_at_ms(4),
    //  reset_line, reset_active] -- 67 bytes (drive+motor follow seen_head).
    FdcFirstFail f;
    fdc_firstfail_snapshot(&f);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = f.valid;
    reply_buffer[i++] = f.kind;
    const uint32_t c32[] = {f.count, f.at_ms};
    for (uint32_t v : c32) {
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (v >> (8 * k)) & 0xFF;
    }
    reply_buffer[i++] = f.want_sec;
    reply_buffer[i++] = f.trk_reg;
    reply_buffer[i++] = f.head_target;
    reply_buffer[i++] = f.phys_track;
    reply_buffer[i++] = f.side;
    reply_buffer[i++] = f.seen_cyl;
    reply_buffer[i++] = f.seen_head;
    reply_buffer[i++] = f.drive;
    reply_buffer[i++] = f.motor;
    reply_buffer[i++] = f.capture_ok;
    const uint32_t f32[] = {f.flux_n, f.capture_us};
    for (uint32_t v : f32) {
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (v >> (8 * k)) & 0xFF;
    }
    reply_buffer[i++] = f.ids_seen;
    for (int k = 0; k < FDC_IDS_N; k++)
      reply_buffer[i++] = f.ids[k];
    for (int k = 0; k < 4; k++)
      reply_buffer[i++] = (f.pos_resyncs >> (8 * k)) & 0xFF;
    reply_buffer[i++] = f.last_resync_from;
    reply_buffer[i++] = f.last_resync_to;
    const uint32_t r32[] = {f.reset_events, f.reset_at_ms};
    for (uint32_t v : r32) {
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (v >> (8 * k)) & 0xFF;
    }
    reply_buffer[i++] = f.reset_line;
    reply_buffer[i++] = f.reset_active;
    cdc_write_all(reply_buffer, i);
    if (cmd_buffer[2] == 1)
      fdc_firstfail_reset();
  }

  else if (cmd == MZP_CMD_WRDIAG) {
    // [cmd, ACK, wr_flags, wr_last_result, ids_seen(2), splice_us(4),
    //  attempts(4), refused(4), ok(4), rnf(4), cyl_mismatch(4),
    //  size_mismatch(4), late(4), lost_data_events(4)]
    FdcDebug d;
    fdc_debug_snapshot(&d);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = d.wr_flags;
    reply_buffer[i++] = d.wr_last_result;
    reply_buffer[i++] = (uint8_t)(d.wr_ids_seen & 0xFF);
    reply_buffer[i++] = (uint8_t)(d.wr_ids_seen >> 8);
    const uint32_t w32[] = {d.wr_splice_us,     d.write_attempts,
                            d.wr_refused,       d.wr_ok,
                            d.wr_rnf,           d.wr_cyl_mismatch,
                            d.wr_size_mismatch, d.wr_late,
                            d.lost_data_events,  d.wr_abandoned,
                            d.wr_short_armed,    d.trk_ok,
                            d.trk_fail,          d.trk_refused};
    for (uint32_t v : w32) {
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (v >> (8 * k)) & 0xFF;
    }
    // ...and where the last abandoned write stopped: want vs got is the
    // diagnosis in two numbers.
    reply_buffer[i++] = (uint8_t)(d.wr_ab_want & 0xFF);
    reply_buffer[i++] = (uint8_t)(d.wr_ab_want >> 8);
    reply_buffer[i++] = (uint8_t)(d.wr_ab_got & 0xFF);
    reply_buffer[i++] = (uint8_t)(d.wr_ab_got >> 8);
    reply_buffer[i++] = d.wr_ab_track;
    reply_buffer[i++] = d.wr_ab_side;
    reply_buffer[i++] = d.wr_ab_sector;
    // WRITE/READ TRACK state. The refusal REASON matters most: WP, the write
    // switch and the format gate are indistinguishable to the host.
    reply_buffer[i++] = d.trk_refuse_why;
    reply_buffer[i++] = d.trk_op;
    reply_buffer[i++] = d.trk_armed;
    reply_buffer[i++] = (uint8_t)(d.trk_pos & 0xFF);
    reply_buffer[i++] = (uint8_t)(d.trk_pos >> 8);
    reply_buffer[i++] = (uint8_t)(d.trk_last_bytes & 0xFF);
    reply_buffer[i++] = (uint8_t)(d.trk_last_bytes >> 8);
    // Failed-write ring: wrf_n(4), then 8 x [trk, sec, side, ids(1), flags,
    // motor, hunt_ms(2)], oldest -> newest. 76 bytes -> 144.
    for (int k = 0; k < 4; k++)
      reply_buffer[i++] = (d.wrf_n >> (8 * k)) & 0xFF;
    // 10 bytes per entry: [trk, sec, side, ids(1), flags, ctx, hunt_10ms,
    // phys, 0, 0]. 76 + 4 + 80 = 160 so far.
    for (int e = 0; e < 8; e++) {
      reply_buffer[i++] = d.wrf_trk[e];
      reply_buffer[i++] = d.wrf_sec[e];
      reply_buffer[i++] = d.wrf_side[e];
      reply_buffer[i++] =
          (uint8_t)((d.wrf_ids[e] > 255) ? 255 : d.wrf_ids[e]);
      reply_buffer[i++] = d.wrf_flags[e];
      reply_buffer[i++] = d.wrf_ctx[e];
      uint32_t t10 = d.wrf_us[e] / 10000;
      reply_buffer[i++] = (uint8_t)((t10 > 255) ? 255 : t10);
      reply_buffer[i++] = d.wrf_phys[e];
      reply_buffer[i++] = 0; // reserved (keeps the 10-byte entry stride)
      reply_buffer[i++] = 0;
    }
    // Track-write ring: twr_n(4), then 8 x [ds(2), trk, side, phys, nsec,
    // flags, census(2), len(2)] with timestamps in DECISECONDS (u16 covers
    // 109 min) so the whole reply stays inside the 256-byte buffer. 252 total.
    for (int k = 0; k < 4; k++)
      reply_buffer[i++] = (d.twr_n >> (8 * k)) & 0xFF;
    for (int e = 0; e < 8; e++) {
      uint32_t ds = d.twr_ms[e] / 100;
      if (ds > 0xFFFF)
        ds = 0xFFFF;
      reply_buffer[i++] = (uint8_t)(ds & 0xFF);
      reply_buffer[i++] = (uint8_t)(ds >> 8);
      reply_buffer[i++] = d.twr_trk[e];
      reply_buffer[i++] = d.twr_side[e];
      reply_buffer[i++] = d.twr_phys[e];
      reply_buffer[i++] = d.twr_nsec[e];
      reply_buffer[i++] = d.twr_flags[e];
      reply_buffer[i++] = (uint8_t)(d.twr_census[e] & 0xFF);
      reply_buffer[i++] = (uint8_t)(d.twr_census[e] >> 8);
      reply_buffer[i++] = (uint8_t)(d.twr_len[e] & 0xFF);
      reply_buffer[i++] = (uint8_t)(d.twr_len[e] >> 8);
    }
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_WREN) {
    // cmd_buffer[2]: 0 = read-only, 1 = writes enabled, 2 = query only.
    if (cmd_buffer[2] < 2)
      fdc_set_write_enable(cmd_buffer[2] != 0);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = fdc_get_write_enable() ? 1 : 0;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_FDCLOG) {
    // [cmd, ACK, count_lo, count_hi, n, decoded[n], raw[n], seen[32]]
    // both rings oldest->newest, same slot order; seen = 256-bit map of every
    // distinct raw command byte since power-up (bit `raw` set).
    uint8_t dec[16];
    uint8_t raw[16];
    uint16_t count = 0;
    uint8_t n = fdc_debug_cmdlog(dec, raw, &count);
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = count & 0xFF;
    reply_buffer[i++] = (count >> 8) & 0xFF;
    reply_buffer[i++] = n;
    for (uint8_t k = 0; k < n; k++)
      reply_buffer[i++] = dec[k];
    for (uint8_t k = 0; k < n; k++)
      reply_buffer[i++] = raw[k];
    uint32_t seen[8];
    fdc_cmd_seen(seen);
    for (int w = 0; w < 8; w++)
      for (int k = 0; k < 4; k++)
        reply_buffer[i++] = (seen[w] >> (8 * k)) & 0xFF;
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_PINTEST) {
    // Bench per-pad self-test: [cmd, ACK, pullup_byte, pulldown_byte].
    uint8_t pu = 0, pd = 0;
    bus_pin_selftest(&pu, &pd);
    bus_bench_restore(); // SMs restarted, pulls and gates restored (#17)
    reply_buffer[i++] = GW_ACK_OK;
    reply_buffer[i++] = pu;
    reply_buffer[i++] = pd;
    cdc_write_all(reply_buffer, i);
  }

  else {
    reply_buffer[i++] = GW_ACK_BADCMD;
    cdc_write_all(reply_buffer, 2);
  }
}
