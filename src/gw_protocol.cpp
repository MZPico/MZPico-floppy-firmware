// greaseweazle command loop — ported from Adafruit_Floppy's greaseweazle.ino.
// Serial (USB CDC) -> TinyUSB tud_cdc_*; Serial1 debug -> DBG() no-op;
// millis()/yield() -> Pico-SDK time / tud_task(); floppy-> -> FloppyDrive.
#include "gw_protocol.hpp"

#include "board_config.h"
#include "floppy.hpp"
#include "mfm_decode.hpp"

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
#define MZP_MAX_SECTORS 40

#define GW_ACK_OK 0
#define GW_ACK_BADCMD 1
#define GW_ACK_NOINDEX 2
#define GW_ACK_NOTRACK0 3
#define GW_ACK_WRPROT 6
#define GW_ACK_NOUNIT 7
#define GW_ACK_BADPIN 10

#define DBG(...)                                                               \
  do {                                                                         \
  } while (0)

// ---- state ----
static FloppyDrive g_drive(PIN_DENSITY, PIN_INDEX, PIN_SELECT, PIN_MOTOR,
                           PIN_DIR, PIN_STEP, PIN_WDATA, PIN_WGATE, PIN_TRK00,
                           PIN_WPT, PIN_RDATA, PIN_HDSEL, PIN_DS1);
static FloppyDrive *floppy = nullptr;

static uint8_t cmd_buffer[32];
static uint8_t reply_buffer[128];
static uint8_t flux_transitions[MAX_FLUX_PULSE_PER_TRACK];
static bool flux_status = false;

// MZP scan-track scratch
static SectorInfo g_scan_sectors[MZP_MAX_SECTORS];
static uint8_t g_scan_scratch[1024]; // largest sector (128 << 3)

static uint32_t bandwidth_timer;
static uint32_t transfered_bytes;
static uint32_t timestamp = 0;

// ---- helpers ----
static inline uint32_t millis() {
  return to_ms_since_boot(get_absolute_time());
}
template <class T> static inline T tmin(T a, T b) { return a < b ? a : b; }

static uint32_t cdc_avail() { return tud_cdc_available(); }

static uint8_t cdc_read1() {
  uint8_t b;
  for (;;) {
    if (tud_cdc_available() && tud_cdc_read(&b, 1) == 1)
      return b;
    tud_task();
  }
}

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

static uint8_t get_cmd(uint8_t *buff, uint8_t maxbuff) {
  if (cdc_avail() < 2)
    return 0;
  buff[0] = cdc_read1();
  buff[1] = cdc_read1();
  uint8_t len = buff[1];
  if (len < 2 || len > maxbuff)
    return len; // malformed; caller will BADCMD
  for (int i = 2; i < len; i++)
    buff[i] = cdc_read1();
  return len;
}

void gw_init() {
  floppy = nullptr;
  timestamp = millis();
}

void gw_task() {
  uint8_t cmd_len = get_cmd(cmd_buffer, sizeof(cmd_buffer));
  if (!cmd_len) {
    // idle watchdog: park the head and stop the motor after 10s
    if ((millis() > timestamp) && ((millis() - timestamp) > 10000)) {
      if (floppy && floppy->motor_is_spinning()) {
        floppy->select(true);
        floppy->goto_track(0);
        floppy->spin_motor(false);
      }
      if (floppy && floppy->drive_is_selected())
        floppy->select(false);
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
    }
  }

  else if (cmd == GW_CMD_GETPARAMS) {
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
    }
  }

  else if (cmd == GW_CMD_SETPARAMS) {
    // accept and ack (delays kept at defaults for Phase 1)
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
      while (flux && (fluxors < MAX_FLUX_PULSE_PER_TRACK)) {
        flux = cdc_read1();
        flux_transitions[fluxors++] = flux;
      }
      if (fluxors == MAX_FLUX_PULSE_PER_TRACK) {
        // overrun; bail with a status the host can see on GETFLUXSTATUS
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
    while (numbytes != 0) {
      uint32_t avail = cdc_avail();
      if (avail == 0) {
        tud_task();
        continue;
      }
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
    if (!floppy)
      setbustype(GW_CMD_SETBUSTYPE_IBM);
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
                                           /*capture_ms=*/220,
                                           /*index_wait_ms=*/250);
    int ns = mfm_scan_track(flux_transitions, nflux, bt, g_scan_sectors,
                            MZP_MAX_SECTORS, g_scan_scratch,
                            sizeof(g_scan_scratch));

    floppy->spin_motor(false);
    floppy->select(false);

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

  else {
    reply_buffer[i++] = GW_ACK_BADCMD;
    cdc_write_all(reply_buffer, 2);
  }
}
