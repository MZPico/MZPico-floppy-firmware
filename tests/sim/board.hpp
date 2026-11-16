// The simulated MZPico-800-Floppy-Board: the firmware's FDC bridge (fdc.cpp,
// floppy.cpp, MFM code -- unmodified) wired to the simulated cable, plus
// helpers that drive the FDC registers the way an MZ-800 does.
#pragma once
#include <cstdint>
#include <vector>

#include "disk_image.hpp"
#include "fdc.hpp"
#include "sim_core.hpp"
#include "sim_drive.hpp"

namespace sim {

// Power-up: GPIO init, both FloppyDrive instances, fdc_init. Call once per
// process before running anything (each test runs in its own process).
void board_boot();
void insert(int unit, Disk *disk);

// ---- MZ-800 side, used from inside run_script() ----
// Port I/O as the Z80 performs it: 0xD8..0xDB carry the CPL'd value on the
// bus (the ROM complements every FDC byte); 0xDC..0xDF are written plainly.
// Each access costs one Z80 IN/OUT instruction (~3 us), so a polling loop
// polls at a realistic rate.
void out(uint8_t port, uint8_t value);
uint8_t in(uint8_t port);

namespace wd {
constexpr uint8_t BUSY = 0x01, DRQ = 0x02, INDEX = 0x02, LOST = 0x04,
                  TRK0 = 0x04, CRC = 0x08, RNF = 0x10, SEEKERR = 0x10,
                  DELETED = 0x20, HEADLOADED = 0x20, WPROT = 0x40,
                  NOTREADY = 0x80;

void select(int unit, bool motor = true); // port 0xDC
void side(int rom_side);                  // port 0xDD, ROM side numbering
void density(bool hd);                    // port 0xDE
uint8_t status();
void command(uint8_t cmd);
// Poll STATUS until BUSY clears; returns the final status (0xFF on timeout).
uint8_t wait_ready(uint32_t timeout_ms = 3000);
uint8_t restore(bool verify = false);
uint8_t seek(uint8_t cyl, bool verify = false);
uint8_t step(int dir, bool update = true); // +1 in, -1 out, 0 = repeat

struct Xfer {
  uint8_t status = 0xFF;
  std::vector<uint8_t> data;
  bool timed_out = false;
};
// READ SECTOR (0x80) / READ MULTIPLE (0x90): poll DRQ, take bytes.
Xfer read_sector(uint8_t track, uint8_t sector, size_t max_bytes = 1024,
                 bool multi = false, uint32_t timeout_ms = 3000);
Xfer read_address(uint32_t timeout_ms = 1000);
// WRITE SECTOR (0xA0): feed `data` over DRQ.
Xfer write_sector(uint8_t track, uint8_t sector, const std::vector<uint8_t> &data,
                  bool deleted = false, uint32_t timeout_ms = 3000);
// WRITE TRACK (0xF0): feed a WD format image over DRQ.
Xfer write_track(const std::vector<uint8_t> &image, uint32_t timeout_ms = 3000);
Xfer read_track(size_t max_bytes = 16000, uint32_t timeout_ms = 3000);
void force_interrupt(uint8_t cond = 0);

// A standard WD format image for one track (what FORMAT4 feeds WRITE TRACK).
std::vector<uint8_t> format_image(uint8_t cyl, uint8_t head, int nsec,
                                  int size_code, bool hd, uint8_t fill = 0xE5,
                                  const std::vector<int> &order = {});
} // namespace wd

} // namespace sim
