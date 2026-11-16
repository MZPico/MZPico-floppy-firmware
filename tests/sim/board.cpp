#include "board.hpp"

#include <cstdio>
#include <cstring>

#include "board_config.h"
#include "floppy.hpp"
#include "gw_protocol.hpp"
#include "mz_bus.hpp"

// ---- link-time stand-ins for the parts of the firmware not under test ----
static uint8_t g_flux[MAX_FLUX_PULSE_PER_TRACK];
uint8_t *mzp_flux_buffer(size_t *cap) {
  if (cap)
    *cap = sizeof(g_flux);
  return g_flux;
}
void bus_diag_reset(void) {}

static FloppyDrive g_drive0(PIN_DENSITY, PIN_INDEX, PIN_DS0, PIN_MOTOR, PIN_DIR,
                            PIN_STEP, PIN_WDATA, PIN_WGATE, PIN_TRK00, PIN_WPT,
                            PIN_RDATA, PIN_HDSEL, -1);
static FloppyDrive g_drive1(PIN_DENSITY, PIN_INDEX, PIN_DS1, PIN_MOTOR, PIN_DIR,
                            PIN_STEP, PIN_WDATA, PIN_WGATE, PIN_TRK00, PIN_WPT,
                            PIN_RDATA, PIN_HDSEL, -1);
static FloppyDrive *g_drives[2] = {&g_drive0, &g_drive1};

namespace sim {

static void core0_main() {
  // main.cpp's loop, minus USB: the disk task, forever.
  while (true) {
    fdc_disk_task();
    core0_cost(TICKS_PER_US);
  }
}

void board_boot() {
  set_pin_board(&cable());
  // gw_init(): drive 1 first so drive 0's begin() leaves the shared pins.
  g_drive1.begin();
  g_drive0.begin();
  fdc_init(g_drives, 2);
  fdc_bus_reset_irq_init();
  set_core0_main(core0_main);
}

void insert(int unit, Disk *disk) { cable().drive[unit].disk = disk; }

// Z80 OUT (n),A is 11 T-states at 3.546895 MHz = 3.1 us.
static constexpr uint64_t IO_TICKS = 74;

static bool inverted(uint8_t port) { return port >= 0xD8 && port <= 0xDB; }

void out(uint8_t port, uint8_t v) {
  fdc_reg_write(port, inverted(port) ? (uint8_t)~v : v);
  wait_ticks(IO_TICKS);
}

uint8_t in(uint8_t port) {
  uint8_t raw = fdc_reg_read(port);
  wait_ticks(IO_TICKS);
  return inverted(port) ? (uint8_t)~raw : raw;
}

namespace wd {

void select(int unit, bool motor) {
  out(0xDC, (uint8_t)((motor ? 0x80 : 0x00) | 0x04 | (unit & 3)));
}
void side(int rom_side) { out(0xDD, rom_side ? 0xFE : 0xFF); }
void density(bool hd) { out(0xDE, hd ? 0x81 : 0x00); }
uint8_t status() { return in(0xD8); }
void command(uint8_t cmd) {
  out(0xD8, cmd);
  wait_us(30); // a WD needs ~ tens of us before STATUS reflects the command
}

uint8_t wait_ready(uint32_t timeout_ms) {
  uint64_t end = now() + ms(timeout_ms);
  while (now() < end) {
    uint8_t s = status();
    if (!(s & BUSY))
      return s;
  }
  return 0xFF;
}

uint8_t restore(bool verify) {
  command(verify ? 0x04 : 0x00);
  return wait_ready();
}
uint8_t seek(uint8_t cyl, bool verify) {
  out(0xDB, cyl);
  command(verify ? 0x14 : 0x10);
  return wait_ready();
}
uint8_t step(int dir, bool update) {
  uint8_t c = dir > 0 ? 0x40 : dir < 0 ? 0x60 : 0x20;
  command((uint8_t)(c | (update ? 0x10 : 0)));
  return wait_ready();
}

Xfer read_sector(uint8_t track, uint8_t sector, size_t max_bytes, bool multi,
                 uint32_t timeout_ms) {
  Xfer x;
  out(0xD9, track);
  out(0xDA, sector);
  command(multi ? 0x90 : 0x80);
  uint64_t end = now() + ms(timeout_ms);
  while (now() < end) {
    uint8_t s = status();
    if (s & DRQ) {
      uint8_t b = in(0xDB);
      if (x.data.size() < max_bytes)
        x.data.push_back(b);
      if (multi && x.data.size() >= max_bytes) {
        // As the ROM does: Force Interrupt straight after the last byte it
        // wants, then read STATUS.
        force_interrupt();
        x.status = status();
        return x;
      }
      continue;
    }
    if (!(s & BUSY)) {
      x.status = s;
      return x;
    }
  }
  x.timed_out = true;
  force_interrupt();
  x.status = status();
  return x;
}

Xfer read_address(uint32_t timeout_ms) {
  Xfer x;
  command(0xC0);
  uint64_t end = now() + ms(timeout_ms);
  while (now() < end) {
    uint8_t s = status();
    if (s & DRQ) {
      x.data.push_back(in(0xDB));
      continue;
    }
    if (!(s & BUSY)) {
      x.status = s;
      return x;
    }
  }
  x.timed_out = true;
  force_interrupt();
  x.status = status();
  return x;
}

// Feed `data` over DRQ. A sector write stops at the end of its data, as a
// host does (the firmware ends the sector on the gap); a track write keeps
// feeding gap bytes until the index terminates the command, as FORMAT does.
static Xfer feed(uint8_t cmd, const std::vector<uint8_t> &data,
                 uint32_t timeout_ms, bool pad) {
  Xfer x;
  command(cmd);
  size_t i = 0;
  uint64_t end = now() + ms(timeout_ms);
  while (now() < end) {
    uint8_t s = status();
    if ((s & DRQ) && (pad || i < data.size())) {
      out(0xDB, i < data.size() ? data[i] : 0x4E);
      ++i;
      continue;
    }
    if (!(s & BUSY)) {
      x.status = s;
      x.data.resize(i);
      return x;
    }
  }
  x.timed_out = true;
  force_interrupt();
  x.status = status();
  return x;
}

Xfer write_sector(uint8_t track, uint8_t sector, const std::vector<uint8_t> &data,
                  bool deleted, uint32_t timeout_ms) {
  out(0xD9, track);
  out(0xDA, sector);
  return feed(deleted ? 0xA1 : 0xA0, data, timeout_ms, false);
}

Xfer write_track(const std::vector<uint8_t> &image, uint32_t timeout_ms) {
  return feed(0xF0, image, timeout_ms, true);
}

Xfer read_track(size_t max_bytes, uint32_t timeout_ms) {
  Xfer x;
  command(0xE0);
  uint64_t end = now() + ms(timeout_ms);
  while (now() < end) {
    uint8_t s = status();
    if (s & DRQ) {
      uint8_t b = in(0xDB);
      if (x.data.size() < max_bytes)
        x.data.push_back(b);
      continue;
    }
    if (!(s & BUSY)) {
      x.status = s;
      return x;
    }
  }
  x.timed_out = true;
  force_interrupt();
  x.status = status();
  return x;
}

void force_interrupt(uint8_t cond) {
  out(0xD8, (uint8_t)(0xD0 | (cond & 0x0F)));
  wait_us(20);
}

std::vector<uint8_t> format_image(uint8_t cyl, uint8_t head, int nsec,
                                  int size_code, bool hd, uint8_t fill,
                                  const std::vector<int> &order) {
  std::vector<uint8_t> v;
  auto rep = [&](int n, uint8_t b) { v.insert(v.end(), (size_t)n, b); };
  int gap3 = (size_code <= 1 && nsec > 10) ? 32 : 84;
  rep(80, 0x4E);
  rep(12, 0x00);
  rep(3, 0xF6);
  v.push_back(0xFC);
  rep(50, 0x4E);
  for (int i = 0; i < nsec; ++i) {
    int r = order.empty() ? i + 1 : order[(size_t)i];
    rep(12, 0x00);
    rep(3, 0xF5);
    v.push_back(0xFE);
    v.push_back(cyl);
    v.push_back(head);
    v.push_back((uint8_t)r);
    v.push_back((uint8_t)size_code);
    v.push_back(0xF7);
    rep(22, 0x4E);
    rep(12, 0x00);
    rep(3, 0xF5);
    v.push_back(0xFB);
    rep(128 << size_code, fill);
    v.push_back(0xF7);
    rep(gap3, 0x4E);
  }
  rep(hd ? 600 : 300, 0x4E); // gap 4b; the index cuts the write off
  return v;
}

} // namespace wd
} // namespace sim
