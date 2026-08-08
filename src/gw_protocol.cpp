// greaseweazle command loop — ported from Adafruit_Floppy's greaseweazle.ino.
// Serial (USB CDC) -> TinyUSB tud_cdc_*; Serial1 debug -> DBG() no-op;
// millis()/yield() -> Pico-SDK time / tud_task(); floppy-> -> FloppyDrive.
#include "hardware/gpio.h"
#include "gw_protocol.hpp"

#include "board_config.h"
#include "floppy.hpp"
#include "mfm_decode.hpp"
#include "mfm_encode.hpp"
#include "bus/mz_bus.hpp"
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
#define MZP_CMD_INTMODE 0x8F  // how ~INT is driven: 0=off 1=INTRQ 2=INTRQ|DRQ
#define MZP_CMD_IOTRACE 0x90  // ordered register-access trace (reads + writes)
#define MZP_MAX_SECTORS 40
#define MZP_MAX_ENC_SECTORS 18

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
// The last argument is `readypin`, and it is -1 by design, not an oversight.
// ~DEN_RDY (GPIO13) is a single shared net carrying density-out AND ready-in, so
// it cannot be sampled as an input while we drive density; PC drives do not
// supply READY anyway. get_ready_sense() returns true unconditionally for
// readypin < 0, which is the right answer for a PC drive. (This used to be
// PIN_DS1 -- GPIO25, a drive-select *output* -- so ready-sense was reading back
// our own select line.)
static FloppyDrive g_drive(PIN_DENSITY, PIN_INDEX, PIN_SELECT, PIN_MOTOR,
                           PIN_DIR, PIN_STEP, PIN_WDATA, PIN_WGATE, PIN_TRK00,
                           PIN_WPT, PIN_RDATA, PIN_HDSEL, -1);
static FloppyDrive *floppy = nullptr;

static uint8_t cmd_buffer[32];
static uint8_t reply_buffer[256];
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
  // Bring up the drive GPIOs and hand the drive to the FDC bridge. The FDC disk
  // task stays idle until the MZ-800 first touches an FDC port, so it doesn't
  // conflict with greaseweazle/USB drive control when no machine is attached.
  // Park the unused second drive-select HIGH (= deselected; ~DS1 is active low)
  // BEFORE bringing the drive up. Nothing else touches GPIO25: FloppyDrive only
  // knows about PIN_SELECT (= ~DS0). RP2040 pads power up as inputs with a
  // pull-DOWN, so leaving this pin alone asserts ~DS1 permanently and a second
  // drive answers alongside (or instead of) drive 0. This used to happen by
  // accident -- PIN_DS1 was passed as FloppyDrive's `readypin`, whose
  // init_in_pullup() parked it high as a side effect. That slot is now -1 (see
  // the constructor above), so the parking has to be explicit.
  gpio_init(PIN_DS1);
  gpio_put(PIN_DS1, 1);
  gpio_set_dir(PIN_DS1, GPIO_OUT);

  g_drive.begin();
  fdc_init(&g_drive);
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

  else if (cmd == MZP_CMD_STREAMSCAN) {
    // A/B test: same seek + decode as MZP_CMD_SCAN, but flux comes from the
    // incremental (streaming) capture API instead of the batch capture_track
    // call, to isolate whether that new capture path itself produces valid,
    // decodable flux -- independent of any live-FDC timing/settle questions.
    // cmd_buffer[2]=track, [3]=side, [4]=density, [5..6]=capture_ms (LE).
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
    uint16_t capture_ms = (uint16_t)cmd_buffer[5] | ((uint16_t)cmd_buffer[6] << 8);
    if (capture_ms == 0)
      capture_ms = 220;
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
    floppy->select(false);

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
    //  rd_elapsed_us(4), want, result, ids_seen, ids[16],
    //  status_polls(4), status_span_us(4),
    //  int_flags, int_mode, intrq_asserts(4), lost_data_events(4),
    //  write_attempts(4), verify_start_ok, verify_n(4), seek_error] -- 66 bytes.
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
    for (int k = 0; k < 16; k++)
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
    //  wr_late_at_pop(4), wr_late_at_sample(4), rd_late_at_pop(4),
    //  wr_backlog(4), rd_backlog(4), bit_high[8]*4, err_adjacent(4),
    //  rd_reports(4), rd_err(4), rd_bits_want1, rd_bits_want0, rd_last_exp,
    //  rd_last_got, rd_bit_high[8]*4]  -- 144 bytes total.
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
        g_bus_diag.wr_late_at_pop, g_bus_diag.wr_late_at_sample,
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

  else if (cmd == MZP_CMD_FDCLOG) {
    // [cmd, ACK, count_lo, count_hi, n, decoded[n], raw[n]]
    // both oldest->newest, same slot order.
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
    cdc_write_all(reply_buffer, i);
  }

  else if (cmd == MZP_CMD_PINTEST) {
    // Bench per-pad self-test: [cmd, ACK, pullup_byte, pulldown_byte].
    uint8_t pu = 0, pd = 0;
    bus_pin_selftest(&pu, &pd);
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
