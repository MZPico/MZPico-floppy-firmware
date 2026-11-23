// WD179x bridge (src/bus/fdc.cpp), driven through its registers the way the
// MZ-800 drives it, against a simulated drive and disk. Each TEST is a fresh
// power-up in its own process.
//
// Conventions: wd::side() takes the ROM's side number; the MZ-800 card drives
// ~HDSEL inverted, so ROM side 1 = physical head 0 and ROM side 0 = head 1.
#include <algorithm>
#include <cstdio>
#include <cstdlib>

#include "board.hpp"
#include "board_config.h"
#include "floppy.hpp"
#include "testfw.hpp"

using namespace sim;

namespace {

Disk g_disk[2];

void boot_with(Geometry g, Geometry g1 = Geometry::MZ_DD_DATA, bool two = false) {
  g_disk[0] = make_disk(g, 80, "A");
  insert(0, &g_disk[0]);
  if (two) {
    g_disk[1] = make_filled_disk(g1, 0x5A);
    insert(1, &g_disk[1]);
  }
  board_boot();
}

int phys_head(int rom_side) { return rom_side ? 0 : 1; }

bool matches_pattern(const std::vector<uint8_t> &d, int c, int h, int r) {
  for (size_t i = 0; i < d.size(); ++i)
    if (d[i] != pattern_byte(c, h, r, (int)i))
      return false;
  return !d.empty();
}

// Run a script with a generous virtual-time budget; a test that wedges the
// firmware fails here rather than hanging CI.
void run(const std::function<void()> &fn, uint64_t limit = ms(60000)) {
  bool ok = run_script(fn, limit);
  if (!ok)
    testfw::note_failure(__FILE__, __LINE__, "script did not complete (hang or time limit)");
}

FdcDebug dbg() {
  FdcDebug d{};
  fdc_debug_snapshot(&d);
  return d;
}

uint8_t error_bits(uint8_t st) { return st & (wd::RNF | wd::CRC | wd::LOST | wd::WPROT | wd::NOTREADY); }

} // namespace

// ---------------- Type I ----------------

TEST(restore_reaches_track0) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    uint8_t st = wd::restore();
    CHECK(st & wd::TRK0);
    CHECK_EQ(in(0xD9), 0);
    CHECK_EQ(cable().drive[0].cyl, 0);
  });
}

TEST(seek_moves_head_and_track_register) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    uint8_t st = wd::seek(40);
    CHECK(!(st & wd::SEEKERR));
    CHECK_EQ(in(0xD9), 40);
    // The head moves in the background (core0) after BUSY drops on some
    // paths; give it time, then check the physical position.
    wait_ms(300);
    CHECK_EQ(cable().drive[0].cyl, 40);
  });
}

TEST(seek_with_verify_passes_on_good_media) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    uint8_t st = wd::seek(12, true);
    CHECK(!(st & wd::SEEKERR));
    CHECK(!(st & wd::CRC));
  });
}

TEST(seek_verify_detects_wrong_cylinder) {
  // Cylinder 12 of this disk carries IDs that say cylinder 13: a WD179x
  // reports SEEK ERROR for a Verify there.
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
  for (int h = 0; h < 2; ++h) {
    TrackSpec t;
    for (int r = 1; r <= 9; ++r) {
      SectorSpec s;
      s.c = 13; s.h = (uint8_t)h; s.r = (uint8_t)r; s.n = 2;
      t.sectors.push_back(s);
    }
    g_disk[0].t[12][h] = encode_track(t);
  }
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    uint8_t st = wd::seek(12, true);
    CHECK(st & wd::SEEKERR);
  });
}

TEST(step_in_and_out_follow_the_head) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    for (int i = 0; i < 5; ++i)
      wd::step(+1);
    wd::step(-1);
    wd::step(0); // repeat last direction (out)
    wait_ms(200);
    CHECK_EQ(in(0xD9), 3);
    CHECK_EQ(cable().drive[0].cyl, 3);
  });
}

// CP/M 4.1 positions the head with STEP + FORCE INTERRUPT bursts microseconds
// apart and never waits for BUSY, while each real step takes milliseconds.
// Every queued step must still land, and surplus STEP-OUTs past TRK00 must be
// absorbed per command, as a real drive absorbs them.
TEST(cpm_style_step_bursts_land_exactly) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    for (int i = 0; i < 6; ++i) { // 6 x STEP OUT at track 0: absorbed
      out(0xD8, 0x60);
      wd::force_interrupt();
    }
    for (int i = 0; i < 9; ++i) { // then 9 x STEP IN, no BUSY waits
      out(0xD8, 0x50);
      wd::force_interrupt();
    }
    wait_ms(500);
    CHECK_EQ(cable().drive[0].cyl, 9);
    // A read issued straight after must wait for the move, then hit cyl 9.
    out(0xD9, 9);
    wd::side(1);
    wd::Xfer x = wd::read_sector(9, 4, 512);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 9, 0, 4));
  });
}

TEST(read_right_after_seek_waits_for_the_head) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    out(0xDB, 30);
    out(0xD8, 0x10); // SEEK 30, do not wait for BUSY
    wd::side(1);
    wd::Xfer x = wd::read_sector(30, 2, 512);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 30, 0, 2));
  });
}

// ---------------- Type II: READ ----------------

TEST(read_every_sector_of_sampled_tracks_both_sides) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    const int cyls[] = {0, 1, 39, 79};
    for (int c : cyls) {
      wd::seek((uint8_t)c);
      for (int rs = 0; rs < 2; ++rs) {
        wd::side(rs);
        for (int r = 1; r <= 9; ++r) {
          wd::Xfer x = wd::read_sector((uint8_t)c, (uint8_t)r, 512);
          if (error_bits(x.status) || !matches_pattern(x.data, c, phys_head(rs), r)) {
            std::fprintf(stderr, "  c%d rs%d r%d st=%02x n=%zu\n", c, rs, r,
                         x.status, x.data.size());
            CHECK(false);
          }
        }
      }
    }
    FdcDebug d = dbg();
    CHECK_EQ(d.status & 0, 0);
  }, ms(120000));
}

TEST(read_multiple_whole_track_then_force_interrupt) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    wd::Xfer x = wd::read_sector(5, 1, 9 * 512, /*multi=*/true, 5000);
    REQUIRE(x.data.size() == 9 * 512);
    for (int r = 1; r <= 9; ++r) {
      std::vector<uint8_t> s(x.data.begin() + (r - 1) * 512,
                             x.data.begin() + r * 512);
      CHECK(matches_pattern(s, 5, 0, r));
    }
    // The ROM requires the true STATUS to be 0 after its Force Interrupt.
    CHECK_EQ(error_bits(x.status), 0);
  });
}

TEST(read_boot_track_16x256_on_rom_side0) {
  boot_with(Geometry::MZ_DD_MASTER);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(0); // ROM side 0 = physical head 1 = the IPL track
    for (int r = 1; r <= 16; ++r) {
      wd::Xfer x = wd::read_sector(0, (uint8_t)r, 256);
      CHECK_EQ(error_bits(x.status), 0);
      CHECK_EQ(x.data.size(), 256u);
      CHECK(matches_pattern(x.data, 0, 1, r));
    }
  });
}

TEST(read_reports_crc_error) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
  TrackSpec t;
  for (int r = 1; r <= 9; ++r) {
    SectorSpec s;
    s.c = 3; s.h = 0; s.r = (uint8_t)r; s.n = 2;
    s.data.assign(512, 0x11);
    s.bad_data_crc = (r == 6);
    t.sectors.push_back(s);
  }
  g_disk[0].t[3][0] = encode_track(t);
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(3);
    wd::side(1);
    CHECK_EQ(error_bits(wd::read_sector(3, 5, 512).status), 0);
    uint8_t st = wd::read_sector(3, 6, 512).status;
    CHECK(st & wd::CRC);
    CHECK(!(st & wd::RNF));
  });
}

TEST(read_reports_deleted_data_mark) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
  TrackSpec t;
  for (int r = 1; r <= 9; ++r) {
    SectorSpec s;
    s.c = 4; s.h = 0; s.r = (uint8_t)r; s.n = 2;
    s.data.assign(512, (uint8_t)r);
    s.deleted = (r == 2);
    t.sectors.push_back(s);
  }
  g_disk[0].t[4][0] = encode_track(t);
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(4);
    wd::side(1);
    wd::Xfer x = wd::read_sector(4, 2, 512);
    CHECK(x.status & wd::DELETED);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK_EQ(x.data.size(), 512u);
    CHECK(!(wd::read_sector(4, 3, 512).status & wd::DELETED));
  });
}

TEST(read_missing_sector_is_rnf) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    uint8_t st = wd::read_sector(0, 12, 512).status;
    CHECK(st & wd::RNF);
  });
}

// The WD ID track compare: a sector-number match on the wrong cylinder is RNF,
// never data. Without it a positioning bug is invisible.
TEST(read_with_wrong_track_register_is_rnf) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(10);
    wd::side(1);
    wd::Xfer x = wd::read_sector(11, 1, 512); // head is on 10, host says 11
    CHECK(x.status & wd::RNF);
    CHECK(x.data.empty());
  });
}

// Regression for 16e5310: a fixed transition-count window was shorter than one
// revolution on dense data (an empty directory, 0x00 stored as 0xFF), so reads
// failed at some rotational phases. Read at many phases.
TEST(read_window_covers_dense_tracks_at_every_phase) {
  g_disk[0] = make_filled_disk(Geometry::MZ_DD_DATA, 0xFF, 4);
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    srand(7);
    int bad = 0;
    for (int i = 0; i < 60; ++i) {
      wait_us((uint64_t)(rand() % 200000));
      wd::Xfer x = wd::read_sector(0, (uint8_t)(1 + i % 9), 512);
      if (error_bits(x.status) || x.data.size() != 512)
        ++bad;
    }
    CHECK_EQ(bad, 0);
  }, ms(120000));
}

TEST(read_window_on_slow_spindle_218ms) {
  g_disk[0] = make_filled_disk(Geometry::MZ_DD_DATA, 0xFF, 4);
  insert(0, &g_disk[0]);
  cable().drive[0].rev_ticks = us(218700);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    srand(9);
    int bad = 0;
    for (int i = 0; i < 40; ++i) {
      wait_us((uint64_t)(rand() % 220000));
      wd::Xfer x = wd::read_sector(0, (uint8_t)(1 + i % 9), 512);
      if (error_bits(x.status)) {
        ++bad;
        if (bad < 4) {
          FdcDebug d = dbg();
          std::fprintf(stderr, "  st=%02x flux=%u cap=%uus rev=%u win=%u ids=%u\n",
                       x.status, d.rd_flux_n, d.rd_capture_us, d.rev_us,
                       d.window_us, d.rd_ids_seen);
        }
      }
    }
    CHECK_EQ(bad, 0);
    FdcDebug d = dbg();
    // The measured period should be the drive's, not the 200 ms default.
    CHECK(d.rev_us > 215000 && d.rev_us < 222000);
  }, ms(120000));
}

TEST(read_address_returns_the_next_id) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(21);
    wd::side(0); // physical head 1
    wd::Xfer x = wd::read_address();
    CHECK_EQ(error_bits(x.status), 0);
    REQUIRE(x.data.size() == 6);
    CHECK_EQ(x.data[0], 21);
    CHECK_EQ(x.data[1], 1);
    CHECK(x.data[2] >= 1 && x.data[2] <= 9);
    CHECK_EQ(x.data[3], 2);
    // WD: READ ADDRESS copies the track address into the Sector register.
    CHECK_EQ(in(0xDA), 21);
  });
}

TEST(empty_drive_reports_not_ready) {
  board_boot(); // no disk inserted
  run([] {
    wd::select(0);
    wait_ms(1500);
    wd::restore();
    wait_ms(1500);
    CHECK(wd::status() & wd::NOTREADY);
  });
}

// ---------------- Type II: WRITE ----------------

TEST(write_sector_roundtrip_leaves_neighbours_intact) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(17);
    wd::side(1);
    std::vector<uint8_t> v(512);
    for (int i = 0; i < 512; ++i)
      v[i] = (uint8_t)(i * 5 + 3);
    wd::Xfer w = wd::write_sector(17, 5, v);
    CHECK_EQ(error_bits(w.status), 0);
    wd::Xfer r = wd::read_sector(17, 5, 512);
    CHECK(r.data == v);
    CHECK(matches_pattern(wd::read_sector(17, 4, 512).data, 17, 0, 4));
    CHECK(matches_pattern(wd::read_sector(17, 6, 512).data, 17, 0, 6));
  });
  // Off the media itself, not through the firmware: every sector still valid.
  CHECK_EQ(count_valid_sectors(g_disk[0].at(17, 0), false), 9);
  DecodedSector s = decode_sector(g_disk[0].at(17, 0), 5, false);
  CHECK(s.crc_ok);
  CHECK_EQ(s.data.size(), 512u);
  if (s.data.size() == 512)
    CHECK_EQ(s.data[100], (uint8_t)(100 * 5 + 3));
  CHECK_EQ(count_valid_sectors(g_disk[0].at(17, 1), false), 9); // other side
}

TEST(write_many_sectors_soak) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    srand(3);
    for (int k = 0; k < 24; ++k) {
      int c = 1 + rand() % 78, rs = rand() % 2, r = 1 + rand() % 9;
      wd::seek((uint8_t)c);
      wd::side(rs);
      wait_us((uint64_t)(rand() % 200000));
      std::vector<uint8_t> v(512, (uint8_t)k);
      v[0] = (uint8_t)c;
      v[1] = (uint8_t)r;
      wd::Xfer w = wd::write_sector((uint8_t)c, (uint8_t)r, v);
      wd::Xfer b = wd::read_sector((uint8_t)c, (uint8_t)r, 512);
      if (error_bits(w.status) || b.data != v) {
        std::fprintf(stderr, "  write %d c%d rs%d r%d: wst=%02x rst=%02x\n", k,
                     c, rs, r, w.status, b.status);
        CHECK(false);
      }
    }
  }, ms(300000));
  for (int c = 0; c < 80; ++c)
    for (int h = 0; h < 2; ++h)
      if (count_valid_sectors(g_disk[0].at(c, h), false) != 9) {
        std::fprintf(stderr, "  damaged track c%d h%d\n", c, h);
        CHECK(false);
      }
}

TEST(write_sector_on_write_protected_disk_is_refused) {
  boot_with(Geometry::MZ_DD_DATA);
  g_disk[0].write_protect = true;
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::Xfer w = wd::write_sector(0, 3, std::vector<uint8_t>(512, 0x77));
    CHECK(w.status & wd::WPROT);
  });
  CHECK(cable().drive[0].writes.empty());
  CHECK(matches_pattern(decode_sector(g_disk[0].at(0, 0), 3, false).data, 0, 0, 3));
}

TEST(write_sector_with_wrong_track_register_writes_nothing) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(8);
    wd::side(1);
    wd::Xfer w = wd::write_sector(9, 3, std::vector<uint8_t>(512, 0x77));
    CHECK(w.status & wd::RNF);
  });
  CHECK(cable().drive[0].writes.empty());
  CHECK_EQ(count_valid_sectors(g_disk[0].at(8, 0), false), 9);
}

// A write must not need the sector length in advance: the boot track is
// 16 x 256, and writing it on a disk nobody has read used to write nothing.
TEST(write_256_byte_sector_on_unread_boot_track) {
  boot_with(Geometry::MZ_DD_MASTER);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(0); // IPL track: physical head 1
    std::vector<uint8_t> v(256, 0xC3);
    wd::Xfer w = wd::write_sector(0, 7, v);
    CHECK_EQ(error_bits(w.status), 0);
    CHECK(wd::read_sector(0, 7, 256).data == v);
  });
  CHECK_EQ(count_valid_sectors(g_disk[0].at(0, 1), false), 16);
}

TEST(write_deleted_data_mark) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::write_sector(0, 8, std::vector<uint8_t>(512, 0x42), /*deleted=*/true);
    wd::Xfer r = wd::read_sector(0, 8, 512);
    CHECK(r.status & wd::DELETED);
  });
  CHECK(decode_sector(g_disk[0].at(0, 0), 8, false).deleted);
}

// ---------------- Type III: WRITE TRACK / READ TRACK ----------------

TEST(write_track_formats_9x512) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
  g_disk[0].t[20][0] = Track{}; // unformatted
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(20);
    wd::side(1);
    wd::Xfer f = wd::write_track(wd::format_image(20, 0, 9, 2, false));
    CHECK_EQ(error_bits(f.status), 0);
    wd::Xfer r = wd::read_sector(20, 9, 512);
    CHECK_EQ(error_bits(r.status), 0);
    CHECK(r.data == std::vector<uint8_t>(512, 0xE5));
  });
  CHECK_EQ(count_valid_sectors(g_disk[0].at(20, 0), false), 9);
}

TEST(write_track_formats_16x256_interleaved) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(2);
    wd::side(0);
    std::vector<int> order = {1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15, 8, 16};
    wd::Xfer f = wd::write_track(wd::format_image(2, 1, 16, 1, false, 0xE5, order));
    CHECK_EQ(error_bits(f.status), 0);
    for (int r = 1; r <= 16; ++r)
      CHECK_EQ(error_bits(wd::read_sector(2, (uint8_t)r, 256).status), 0);
  });
  CHECK_EQ(count_valid_sectors(g_disk[0].at(2, 1), false), 16);
}

TEST(write_track_on_write_protected_disk_is_refused) {
  boot_with(Geometry::MZ_DD_DATA);
  g_disk[0].write_protect = true;
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::Xfer f = wd::write_track(wd::format_image(0, 0, 9, 2, false));
    CHECK(f.status & wd::WPROT);
  });
  CHECK(cable().drive[0].writes.empty());
}

TEST(write_track_hd_18_sectors) {
  g_disk[0] = make_disk(Geometry::HD_DATA);
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::density(true);
    wd::select(0);
    wd::restore();
    wd::seek(40);
    wd::side(1);
    wd::Xfer f = wd::write_track(wd::format_image(40, 0, 18, 2, true, 0x6D), 4000);
    CHECK_EQ(error_bits(f.status), 0);
    wd::Xfer r = wd::read_sector(40, 18, 512);
    CHECK(r.data == std::vector<uint8_t>(512, 0x6D));
  });
  CHECK_EQ(count_valid_sectors(g_disk[0].at(40, 0), true), 18);
}

TEST(read_track_shows_ids_and_gaps) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::Xfer t = wd::read_track();
    CHECK(t.data.size() > 5000);
    // Somewhere in it: an ID field for cylinder 0, head 0, sector 5, N=2.
    bool found = false;
    for (size_t i = 0; i + 4 < t.data.size(); ++i)
      if (t.data[i] == 0xFE && t.data[i + 1] == 0 && t.data[i + 2] == 0 &&
          t.data[i + 3] == 5 && t.data[i + 4] == 2)
        found = true;
    CHECK(found);
  });
}

// ---------------- HD ----------------

TEST(hd_declared_read) {
  g_disk[0] = make_disk(Geometry::HD_DATA);
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::density(true);
    wd::select(0);
    wd::restore();
    wd::seek(60);
    wd::side(0);
    for (int r : {1, 9, 18}) {
      wd::Xfer x = wd::read_sector(60, (uint8_t)r, 512);
      CHECK_EQ(error_bits(x.status), 0);
      CHECK(matches_pattern(x.data, 60, 1, r));
    }
  });
}

TEST(hd_write_roundtrip) {
  g_disk[0] = make_disk(Geometry::HD_DATA);
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::density(true);
    wd::select(0);
    wd::restore();
    wd::seek(33);
    wd::side(1);
    std::vector<uint8_t> v(512);
    for (int i = 0; i < 512; ++i)
      v[i] = (uint8_t)(i ^ 0x3C);
    CHECK_EQ(error_bits(wd::write_sector(33, 11, v).status), 0);
    CHECK(wd::read_sector(33, 11, 512).data == v);
  });
  CHECK_EQ(count_valid_sectors(g_disk[0].at(33, 0), true), 18);
}

// ---------------- Multi-drive ----------------

TEST(two_drives_keep_their_own_heads_and_media) {
  boot_with(Geometry::MZ_DD_DATA, Geometry::MZ_DD_DATA, /*two=*/true);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(40);
    wd::select(1);
    wd::restore();
    wd::select(0);
    out(0xD9, 40);
    wd::side(1);
    wd::Xfer a = wd::read_sector(40, 1, 512);
    CHECK(matches_pattern(a.data, 40, 0, 1));
    wd::select(1);
    out(0xD9, 0);
    wd::Xfer b = wd::read_sector(0, 1, 512);
    CHECK(b.data == std::vector<uint8_t>(512, 0x5A));
    CHECK_EQ(cable().drive[0].cyl, 40);
    CHECK_EQ(cable().drive[1].cyl, 0);
  }, ms(60000));
}

TEST(unit2_selected_reports_not_ready_then_unit0_still_works) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(2);
    out(0xD8, 0x00); // RESTORE on a unit that does not exist
    wait_ms(100);
    CHECK(wd::status() & wd::NOTREADY);
    wd::select(0);
    CHECK(!(wd::restore() & wd::BUSY));
    wd::side(1);
    wd::Xfer x = wd::read_sector(0, 1, 512, false, 2000);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 0, 0, 1));
  }, ms(30000));
}

// ---------------- Cross-core ordering (review findings) ----------------

// #2: ~HDSEL is applied at the top of a disk-task pass only. A host that
// changes side between a Type I and the following READ must still get the
// side it asked for. The two heads hold different data, so a wrong-side read
// is visible even though its CRC is clean.
TEST(side_change_after_seek_reaches_the_read) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1); // head 0 applied
    wd::read_sector(0, 1, 512);
    out(0xDB, 25);
    out(0xD8, 0x10); // SEEK 25 -- core0 is now stepping for ~50 ms
    wait_us(200);
    wd::side(0);     // switch to head 1 while the seek runs
    wd::Xfer x = wd::read_sector(25, 3, 512);
    CHECK_EQ(error_bits(x.status), 0);
    bool right = matches_pattern(x.data, 25, 1, 3);
    if (!right && matches_pattern(x.data, 25, 0, 3))
      std::fprintf(stderr, "  got head 0's sector: the side change was lost\n");
    CHECK(right);
  });
}

// #3: a new READ must never be answered from the previous read's capture.
TEST(read_after_seek_is_not_answered_from_the_old_capture) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::read_sector(0, 2, 512);
    wd::seek(1);
    wd::Xfer x = wd::read_sector(1, 2, 512);
    CHECK(matches_pattern(x.data, 1, 0, 2));
  });
}

// #3, the exact ordering: read A is still capturing (~240 ms) when the host
// gives up -- Force Interrupt, a SEEK, and read B on the new cylinder, all
// within microseconds, as CP/M does. B must be served from a capture taken
// AFTER its move, not from A's window.
TEST(read_issued_during_a_capture_waits_for_its_own_move) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    out(0xD9, 0);
    out(0xDA, 1);
    out(0xD8, 0x80); // read A, cylinder 0
    wait_ms(60);     // core0 is now inside A's capture
    wd::force_interrupt();
    out(0xDB, 6);
    out(0xD8, 0x10); // SEEK 6, no wait
    wd::Xfer b = wd::read_sector(6, 4, 512);
    CHECK_EQ(error_bits(b.status), 0);
    CHECK(matches_pattern(b.data, 6, 0, 4));
    FdcDebug d = dbg();
    CHECK_EQ(d.sec_cyl_mismatch, 0u); // no wrong-cylinder answer along the way
  });
}

// #3 with a side change instead of a move: B must not be decoded from A's
// window, which holds the other surface. Nothing would flag it -- same
// cylinder, clean CRC -- so the data itself is checked.
TEST(read_issued_during_a_capture_gets_its_own_side) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(9);
    wd::side(1); // head 0
    out(0xD9, 9);
    out(0xDA, 1);
    out(0xD8, 0x80); // read A on head 0
    wait_ms(60);
    wd::force_interrupt();
    wd::side(0);     // head 1
    wd::Xfer b = wd::read_sector(9, 5, 512);
    CHECK_EQ(error_bits(b.status), 0);
    bool right = matches_pattern(b.data, 9, 1, 5);
    if (!right && matches_pattern(b.data, 9, 0, 5))
      std::fprintf(stderr, "  got head 0's sector from read A's window\n");
    CHECK(right);
  });
}

// #1: a READ abandoned by Force Interrupt can complete on core0 after the host
// has moved on to a WRITE SECTOR; its result must not land in the write buffer.
TEST(abandoned_read_does_not_corrupt_following_write) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(6);
    wd::side(1);
    out(0xD9, 6);
    out(0xDA, 1);
    out(0xD8, 0x80); // READ sector 1...
    wait_us(500);
    wd::force_interrupt(); // ...abandoned while core0 is capturing
    std::vector<uint8_t> v(512, 0xA7);
    wd::Xfer w = wd::write_sector(6, 2, v);
    CHECK_EQ(error_bits(w.status), 0);
    CHECK(wd::read_sector(6, 2, 512).data == v);
  });
}

// #1, the exact ordering: a read of a sector that is not there takes the RNF
// path with its second-chance capture (~480 ms in all). The host gives up
// during it and starts a WRITE SECTOR, feeding bytes as the stale read
// completes. The stale completion must not drop the write's DRQ, raise RNF on
// it, or touch its buffer.
TEST(stale_read_completion_does_not_break_a_write_being_fed) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(11);
    wd::side(1);
    out(0xD9, 11);
    out(0xDA, 14); // no such sector: RNF after two captures
    out(0xD8, 0x80);
    wait_ms(300);  // inside the SECOND (second-chance) capture
    wd::force_interrupt();
    out(0xD9, 11);
    out(0xDA, 3);
    out(0xD8, 0xA0); // WRITE SECTOR 3, fed slowly across the stale completion
    std::vector<uint8_t> v(512);
    for (int i = 0; i < 512; ++i)
      v[i] = (uint8_t)(0x40 ^ i);
    size_t fed = 0;
    uint8_t st = 0xFF;
    for (uint64_t end = now() + ms(3000); now() < end;) {
      st = wd::status();
      if ((st & wd::DRQ) && fed < v.size()) {
        out(0xDB, v[fed++]);
        wait_us(1500); // ~770 ms to feed the sector
        continue;
      }
      if (!(st & wd::BUSY))
        break;
    }
    CHECK_EQ(fed, 512u);
    CHECK_EQ(error_bits(st), 0);
    CHECK(wd::read_sector(11, 3, 512).data == v);
  });
  CHECK_EQ(count_valid_sectors(g_disk[0].at(11, 0), false), 9);
}

// ---------------- ~INT ----------------

TEST(int_line_stays_released_without_eint) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    out(0xD9, 0);
    out(0xDA, 1);
    out(0xD8, 0x80);
    bool asserted = false;
    for (uint64_t end = now() + ms(2000); now() < end;) {
      if (pin_oe(12))
        asserted = true;
      uint8_t s = wd::status();
      if (s & wd::DRQ)
        in(0xDB);
      else if (!(s & wd::BUSY))
        break;
    }
    CHECK(!asserted);
  });
}

TEST(int_line_follows_drq_with_eint) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    out(0xDF, 0x01); // EINT on, as CP/M 4.1 does around data commands
    out(0xD9, 0);
    out(0xDA, 1);
    out(0xD8, 0x80);
    bool asserted = false;
    size_t n = 0;
    for (uint64_t end = now() + ms(2000); now() < end;) {
      if (pin_oe(12))
        asserted = true;
      uint8_t s = wd::status();
      if (s & wd::DRQ) {
        in(0xDB);
        ++n;
      } else if (!(s & wd::BUSY))
        break;
    }
    out(0xDF, 0x00);
    CHECK(asserted);
    CHECK_EQ(n, 512u);
    CHECK(!pin_oe(12));
  });
}

// ---------------- Review batch 2 ----------------

// #21: an ID with size code 4..7 (2-16 KB) must not let a WRITE SECTOR stream
// past the 1 KB transfer buffer into the rest of the controller state.
TEST(oversized_size_code_cannot_overrun_the_transfer_buffer) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
  TrackSpec t;
  for (int r = 1; r <= 3; ++r) {
    SectorSpec s;
    s.c = 15; s.h = 0; s.r = (uint8_t)r; s.n = 4; // 2048-byte IDs
    s.no_data = true;
    t.sectors.push_back(s);
  }
  g_disk[0].t[15][0] = encode_track(t);
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(15);
    wd::side(1);
    wd::Xfer a = wd::read_address(); // caches N=4 for this cylinder
    CHECK_EQ(a.data.size(), 6u);
    out(0xD9, 15);
    out(0xDA, 1);
    out(0xD8, 0xA0);
    size_t fed = 0;
    for (uint64_t end = now() + ms(500); now() < end && fed < 4096;) {
      if (wd::status() & wd::DRQ) {
        out(0xDB, 0x55);
        ++fed;
      }
    }
    FdcDebug d = dbg();
    CHECK(d.xfer_len <= 1024);
    CHECK(d.xfer_pos <= 1024);
    CHECK(fed <= 1024);
    wd::force_interrupt();
    wd::seek(3);
    wd::Xfer r = wd::read_sector(3, 2, 512);
    CHECK_EQ(error_bits(r.status), 0);
    CHECK(matches_pattern(r.data, 3, 0, 2));
  });
}

// #22: write refusal must use the selected unit's write protection, not the
// last drive core0 happened to sense.
TEST(write_to_writable_drive_right_after_switching_from_protected_one) {
  boot_with(Geometry::MZ_DD_DATA, Geometry::MZ_DD_DATA, /*two=*/true);
  g_disk[0].write_protect = true;
  run([] {
    wd::select(1);
    wd::restore();     // unit 1 sensed: writable
    wd::select(0);
    wd::restore();     // unit 0 sensed: protected
    wd::select(1);     // back to B, and straight into a write (no wait)
    out(0xD8, 0x50);   // STEP IN...
    wd::force_interrupt();
    wd::side(1);
    std::vector<uint8_t> v(512, 0x3C);
    wd::Xfer w = wd::write_sector(1, 4, v);
    CHECK(!(w.status & wd::WPROT));
    CHECK_EQ(error_bits(w.status), 0);
    CHECK(wd::read_sector(1, 4, 512).data == v);
  });
  CHECK(cable().drive[0].writes.empty());
}

// #20: a head that is physically on another cylinder (lost steps) must fail a
// Verify with SEEK ERROR -- and, as on the read path, the cylinder the disk
// reported must bring the head back to where the host asked.
TEST(verify_after_lost_steps_reports_seek_error_and_repositions) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(10);
    wait_ms(100);
    cable().drive[0].cyl = 12; // two steps "lost" behind the firmware's back
    wd::side(1);
    uint8_t st = wd::seek(10, /*verify=*/true);
    CHECK(st & wd::SEEKERR);
    wait_ms(200);
    CHECK_EQ(cable().drive[0].cyl, 10);
    wd::Xfer x = wd::read_sector(10, 5, 512);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 10, 0, 5));
  });
}

// #8, second half: a Type I sent to unit 2 must not be folded into drive 0's
// head target (fdc_drive_idx() clamps 2 -> 0), so drive 0 stays where it was.
TEST(type1_on_missing_unit_does_not_move_drive0) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(20);
    wait_ms(200);
    int steps = cable().drive[0].steps;
    wd::select(2);
    CHECK(!(wd::restore() & wd::BUSY));
    CHECK(wd::status() & wd::NOTREADY);
    wd::select(0);
    out(0xD9, 20);
    wd::side(1);
    wd::Xfer x = wd::read_sector(20, 1, 512);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 20, 0, 1));
    CHECK_EQ(cable().drive[0].steps, steps); // no recalibrate nobody asked for
  });
}

// #7: WRITE TRACK on a drive with no disk (no index pulse) must end the
// command with an error, not hang core0 waiting for an index forever.
TEST(write_track_on_empty_drive_fails_instead_of_hanging) {
  board_boot(); // drive present, no disk
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::Xfer f = wd::write_track(wd::format_image(0, 0, 9, 2, false), 5000);
    CHECK(!f.timed_out);
    CHECK(f.status & (wd::NOTREADY | wd::RNF));
    // ...and the controller still answers afterwards.
    CHECK(!(wd::restore() & wd::BUSY));
  });
}

// #7 follow-up: an empty-drive WRITE TRACK failure must not stick. With the
// motor still on, insert a disk: the next WRITE TRACK formats it.
TEST(write_track_works_after_disk_inserted_into_empty_drive) {
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::Xfer f1 = wd::write_track(wd::format_image(0, 0, 9, 2, false), 5000);
    CHECK(f1.status & (wd::NOTREADY | wd::RNF));
    g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
    g_disk[0].t[0][0] = Track{};
    insert(0, &g_disk[0]);
    wd::Xfer f2 = wd::write_track(wd::format_image(0, 0, 9, 2, false), 5000);
    CHECK_EQ(error_bits(f2.status), 0);
  });
  CHECK_EQ(count_valid_sectors(g_disk[0].at(0, 0), false), 9);
}

// #35: a READ is answered as soon as its sector has passed the head, not after
// a fixed full window. Over random rotational phases the average wait from
// the command to the first DRQ is about half a revolution (+ a sector), as on
// a real WD; it used to be the whole ~225 ms window every time.
TEST(read_latency_is_rotational_not_a_full_window) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::read_sector(0, 1, 512); // first read on the drive: measures the spindle
    srand(11);
    uint64_t sum = 0, worst = 0;
    const int N = 40;
    for (int i = 0; i < N; ++i) {
      wait_us((uint64_t)(rand() % 200000));
      out(0xD9, 0);
      out(0xDA, (uint8_t)(1 + i % 9));
      uint64_t t0 = now();
      out(0xD8, 0x80);
      uint64_t first = 0;
      size_t got = 0;
      for (uint64_t end = now() + ms(2000); now() < end;) {
        uint8_t s = wd::status();
        if (s & wd::DRQ) {
          if (!first)
            first = now() - t0;
          in(0xDB);
          ++got;
        } else if (!(s & wd::BUSY))
          break;
      }
      CHECK_EQ(got, 512u);
      sum += first;
      worst = std::max(worst, first);
    }
    double avg_ms = (double)sum / N / ms(1);
    double worst_ms = (double)worst / ms(1);
    std::printf("  READ -> first DRQ: avg %.0f ms, worst %.0f ms\n", avg_ms, worst_ms);
    CHECK(avg_ms < 150);
    // The worst case is inherent: a READ arriving just as its sector starts
    // to pass waits a revolution + a sector, i.e. the window itself.
    CHECK(worst_ms < 250);
  }, ms(60000));
}

// READ MULTIPLE over a whole track: each sector is captured as it arrives, so
// the track takes about one revolution, not a window per sector.
TEST(read_multiple_whole_track_takes_about_one_revolution) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(4);
    wd::side(1);
    wd::read_sector(4, 1, 512); // measure the spindle
    uint64_t t0 = now();
    wd::Xfer x = wd::read_sector(4, 1, 9 * 512, /*multi=*/true, 5000);
    double t_ms = (double)(now() - t0) / ms(1);
    std::printf("  READ MULTIPLE 9 x 512: %.0f ms\n", t_ms);
    CHECK_EQ(x.data.size(), 9u * 512u);
    CHECK_EQ(error_bits(x.status), 0);
    for (int r = 1; r <= 9 && x.data.size() == 9 * 512; ++r)
      CHECK(matches_pattern(std::vector<uint8_t>(x.data.begin() + (r - 1) * 512,
                                                 x.data.begin() + r * 512), 4, 0, r));
    CHECK(t_ms < 450);
  });
}

// A Type I issued while a READ is still capturing (no Force Interrupt first)
// does not end the read on this controller. The read must still complete --
// never wedge with BUSY up -- once it notices its flux is from before the
// move.
TEST(type1_during_a_read_does_not_wedge_it) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    out(0xD9, 0);
    out(0xDA, 3);
    out(0xD8, 0x80); // READ sector 3 on cylinder 0...
    wait_ms(40);
    out(0xDB, 5);
    out(0xD8, 0x10); // ...and a SEEK 5 on top of it, no Force Interrupt
    uint8_t st = wd::wait_ready(3000);
    CHECK(st != 0xFF); // BUSY dropped
    // ...and the controller is usable afterwards.
    wd::Xfer x = wd::read_sector(5, 2, 512);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 5, 0, 2));
  });
}

// ---------------- Read path: #5, #18, #19, #35 ----------------

// #5: a lapped DMA ring leaves a silent hole in the capture. The read must
// notice (the overrun flag), throw the capture away and capture again --
// never decode across the hole.
TEST(read_recaptures_after_a_dma_ring_overrun) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    for (int r = 1; r <= 9; ++r) {
      arm_capture_overrun(30000, 100000); // 100 ms lost, 30 ms into the capture
      wd::Xfer x = wd::read_sector(0, (uint8_t)r, 512);
      if (error_bits(x.status) || !matches_pattern(x.data, 0, 0, r)) {
        std::fprintf(stderr, "  sector %d: st=%02x n=%zu\n", r, x.status, x.data.size());
        CHECK(false);
      }
    }
  }, ms(60000));
}

// #18: a false ~INDEX glitch 160 ms after each real pulse must not be taken
// for a 160 ms revolution (which would shrink the window below one turn).
TEST(index_glitch_does_not_shrink_the_read_window) {
  g_disk[0] = make_filled_disk(Geometry::MZ_DD_DATA, 0xFF, 4);
  insert(0, &g_disk[0]);
  cable().drive[0].index_glitch_after_ticks = ms(160);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    srand(5);
    int bad = 0;
    for (int i = 0; i < 40; ++i) {
      wait_us((uint64_t)(rand() % 200000));
      if (error_bits(wd::read_sector(0, (uint8_t)(1 + i % 9), 512).status))
        ++bad;
    }
    CHECK_EQ(bad, 0);
    FdcDebug d = dbg();
    std::printf("  measured rev %u us, window %u us\n", d.rev_us, d.window_us);
    CHECK(d.rev_us == 0 || (d.rev_us > 195000 && d.rev_us < 205000));
    CHECK(d.window_us >= 200000 + 10000);
  }, ms(60000));
}

// #19: with no host density declaration the latch can be wrong, so the
// window's sector margin is the DD one (which also covers HD).
TEST(undeclared_density_uses_the_dd_sector_margin) {
  g_disk[0] = make_disk(Geometry::HD_DATA);
  insert(0, &g_disk[0]);
  board_boot();
  fdc_set_density_mode(3); // detect: probe media, no 0xDE declaration
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::Xfer x = wd::read_sector(0, 3, 512, false, 3000);
    if (error_bits(x.status))
      x = wd::read_sector(0, 3, 512, false, 3000); // after the latch flip
    CHECK_EQ(error_bits(x.status), 0);
    FdcDebug d = dbg();
    uint32_t rev = d.rev_us ? d.rev_us : 220000;
    std::printf("  window %u us over rev %u us\n", d.window_us, rev);
    CHECK(d.window_us >= rev + 20000); // DD margin: (512+128) x 16 x 2 us
  });
}

// #35: a sector that is not there is searched for over ~4 windows -- the WD's
// "until the fifth index pulse" -- before RNF. A Force Interrupt still ends
// the search at once.
TEST(missing_sector_searched_about_four_revolutions_before_rnf) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    uint64_t t0 = now();
    uint8_t st = wd::read_sector(0, 14, 512, false, 3000).status;
    double t_ms = (double)(now() - t0) / ms(1);
    std::printf("  RNF after %.0f ms\n", t_ms);
    CHECK(st & wd::RNF);
    CHECK(t_ms > 800 && t_ms < 1200);
    // Force Interrupt during the search ends it promptly.
    out(0xD9, 0);
    out(0xDA, 14);
    out(0xD8, 0x80);
    wait_ms(50);
    wd::force_interrupt();
    uint64_t t1 = now();
    CHECK(!(wd::wait_ready(100) & wd::BUSY));
    CHECK(now() - t1 < ms(20));
  });
}

// #5 during a READ MULTIPLE: an overrun restarts the running capture (and
// empties the buffer) while a sector is pending. That sector must wait for
// the new capture, not be reported RNF from the empty buffer.
TEST(overrun_during_read_multiple_keeps_the_pending_sector) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(6);
    wd::side(1);
    for (int at : {20000, 60000, 110000}) {
      arm_capture_overrun((uint64_t)at, 30000);
      wd::Xfer x = wd::read_sector(6, 1, 9 * 512, /*multi=*/true, 5000);
      CHECK_EQ(x.data.size(), 9u * 512u);
      CHECK_EQ(error_bits(x.status), 0);
      for (int r = 1; r <= 9 && x.data.size() == 9 * 512; ++r)
        CHECK(matches_pattern(std::vector<uint8_t>(x.data.begin() + (r - 1) * 512,
                                                   x.data.begin() + r * 512), 6, 0, r));
    }
  }, ms(60000));
}

// ---------------- Robustness batch: #15, #23, #25 ----------------

// #23: READ MULTIPLE's look-ahead past the end of the track is not a failure:
// not counted and not latched in firstfail. It keeps the WD's search time
// (without capturing), so a Force Interrupt always beats the RNF. The host
// here asks for one sector more than the track has and never sends FI.
TEST(end_of_track_lookahead_is_not_a_failure) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    FdcDebug d0 = dbg();
    uint64_t t0 = now();
    // 10 sectors wanted on a 9-sector track: the 10th is the look-ahead.
    wd::Xfer x = wd::read_sector(5, 1, 10 * 512, /*multi=*/true, 3000);
    double t_ms = (double)(now() - t0) / ms(1);
    std::printf("  READ MULTIPLE ended after %.0f ms, st=%02x, %zu bytes\n",
                t_ms, x.status, x.data.size());
    CHECK_EQ(x.data.size(), 9u * 512u);
    CHECK(x.status & wd::RNF); // the WD does report it, at the end...
    CHECK(t_ms > 800);         // ...after its usual search time, so a host's
                               // Force Interrupt always arrives first
    FdcDebug d1 = dbg();
    CHECK_EQ(d1.sec_rnf, d0.sec_rnf);
    FdcFirstFail ff{};
    fdc_firstfail_snapshot(&ff);
    CHECK(!ff.valid);
  });
}

// #15: a bench command that moves the head (as `scan` / `fdcread` do over
// USB) must not leave the next host read on the bench's cylinder.
TEST(bench_head_move_is_undone_before_the_next_host_read) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(10);
    wd::side(1);
    CHECK(matches_pattern(wd::read_sector(10, 1, 512).data, 10, 0, 1));
    CHECK(!fdc_host_busy());
    FloppyDrive *d = fdc_bench_drive(); // what the USB bench does...
    REQUIRE(d != nullptr);
    d->goto_track(30);
    d->side(1);
    fdc_bench_release();
    fdc_bench_done();                   // ...and what it now does after
    wd::side(1);
    wd::Xfer x = wd::read_sector(10, 2, 512); // no seek: head must be back
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 10, 0, 2));
    FdcDebug dd = dbg();
    CHECK_EQ(dd.sec_cyl_mismatch, 0u);
  });
}

TEST(host_busy_while_a_read_is_in_flight) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    out(0xD9, 0);
    out(0xDA, 1);
    out(0xD8, 0x80);
    wait_ms(5);
    CHECK(fdc_host_busy()); // a bench drive command would be refused now
    wd::wait_ready();
    while (wd::status() & wd::DRQ)
      in(0xDB);
    CHECK(!fdc_host_busy());
  });
}

// #25: with `drvlock 1`, a host that selects drive 1 must still have its
// steps applied to the drive actually being driven (unit 0).
TEST(drvlock_folds_steps_into_the_driven_unit) {
  boot_with(Geometry::MZ_DD_DATA);
  fdc_set_drive_lock(true);
  run([] {
    wd::select(1); // locked: unit 0 is driven
    wd::restore();
    wd::seek(20);
    wd::side(1);
    wd::Xfer x = wd::read_sector(20, 3, 512);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 20, 0, 3));
    CHECK_EQ(cable().drive[0].cyl, 20);
  });
}

// #15 (review): the head goes back only when the MZ-800 is about to use it --
// idle passes (which also run while a USB frame is arriving, e.g. between a
// greaseweazle SEEK and its WRITEFLUX) leave it where the bench put it. And
// a READ issued at once, with the bench having stopped the spindle, still
// reads the host's cylinder.
TEST(bench_head_is_restored_at_the_next_host_operation_not_when_idle) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(10);
    wd::side(1);
    wd::read_sector(10, 1, 512);
    FloppyDrive *d = fdc_bench_drive();
    REQUIRE(d != nullptr);
    d->goto_track(30);
    d->spin_motor(false); // as `scan` leaves it
    fdc_bench_release();
    fdc_bench_done();
    wait_ms(300); // idle: the head must stay on the bench cylinder
    CHECK_EQ(cable().drive[0].cyl, 30);
    wd::Xfer x = wd::read_sector(10, 7, 512); // straight in, no seek
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 10, 0, 7));
    CHECK_EQ(cable().drive[0].cyl, 10);
    CHECK_EQ(dbg().sec_cyl_mismatch, 0u);
  });
}

// ---------------- #32: head settle and spin-up ----------------

// A read issued right after a one-cylinder step: the simulated head rings for
// 12 ms and is marginal for 10 ms more (Drive::settle_*), as real heads are.
// Every read must still be clean -- a WD does not read until the head has
// settled. (In this model the early answer usually masks the ringing -- a
// later pass of the sector is taken -- so this passes without the settle
// wait too; it guards the combination.)
TEST(read_right_after_a_step_is_clean) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    srand(21);
    int bad = 0, n = 0;
    for (int i = 0; i < 72; ++i) {
      uint8_t c = (uint8_t)(10 + (i & 1)); // one step every time
      wd::seek(c);
      wait_us((uint64_t)(rand() % 3000)); // the host's own small delay
      uint8_t r = (uint8_t)(1 + (i * 5) % 9);
      wd::Xfer x = wd::read_sector(c, r, 512);
      ++n;
      if (error_bits(x.status) || !matches_pattern(x.data, c, 0, r)) {
        ++bad;
        if (bad <= 3)
          std::fprintf(stderr, "  i=%d c%u r%u st=%02x\n", i, c, r, x.status);
      }
    }
    std::printf("  %d of %d reads after a step failed\n", bad, n);
    CHECK_EQ(bad, 0);
  }, ms(120000));
}

// The first read after a cold spin-up: ~INDEX is there before the spindle is
// at speed (Drive::speed_stable_ticks), and data read then is garbage.
TEST(first_read_after_a_cold_spin_up_is_clean) {
  // Six cold starts in one power-up: the motor is switched off long enough
  // (past the 3 s deferred motor-off) for the spindle to stop each time.
  int bad = 0;
  boot_with(Geometry::MZ_DD_DATA);
  run([&bad] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    for (int k = 0; k < 6; ++k) {
      wd::select(0, /*motor=*/false);
      wait_ms(3600); // past the 3 s deferred motor-off: the spindle stops
      wait_us((uint64_t)(k * 37000));
      wd::select(0, /*motor=*/true);
      wd::Xfer x = wd::read_sector(0, (uint8_t)(1 + k), 512, false, 3000);
      if (error_bits(x.status) || !matches_pattern(x.data, 0, 0, 1 + k)) {
        ++bad;
        std::fprintf(stderr, "  spin-up %d: st=%02x n=%zu\n", k, x.status, x.data.size());
      }
    }
    CHECK_EQ(bad, 0);
  }, ms(120000));
}

// A WRITE right behind a STEP + FORCE INTERRUPT (CP/M style, no BUSY waits):
// the seek's completion must not clear the write's BUSY. When BUSY drops the
// sector must already be on the media -- the host takes BUSY low as "written"
// and moves on. (Found while adding the #32 gates: the seek completion
// cleared BUSY for any pending command but a read.)
TEST(write_behind_a_step_keeps_busy_until_on_media) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    for (int k = 0; k < 4; ++k) {
      size_t before = cable().drive[0].writes.size();
      out(0xD8, 0x50); // STEP IN...
      wd::force_interrupt();
      std::vector<uint8_t> v(512, (uint8_t)(0x40 + k));
      wd::Xfer w = wd::write_sector((uint8_t)(k + 1), 2, v); // ...and WRITE at once
      CHECK_EQ(error_bits(w.status), 0);
      CHECK(cable().drive[0].writes.size() > before); // on the media at BUSY low
      CHECK(wd::read_sector((uint8_t)(k + 1), 2, 512).data == v);
    }
  });
}

// A Force Interrupt during an armed WRITE TRACK's head wait (here: the 500 ms
// spin-up after a cold motor start) must not leave the track operation armed
// -- or no later command can drop BUSY. (CodeRabbit on #53.)
TEST(write_track_abandoned_during_the_head_wait_leaves_no_state) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::select(0, /*motor=*/false);
    wait_ms(3600); // spindle stops
    uint64_t motor_on = now();
    wd::select(0, /*motor=*/true);
    std::vector<uint8_t> img = wd::format_image(0, 0, 9, 2, false);
    out(0xD8, 0xF0); // WRITE TRACK
    for (size_t i = 0; i < img.size(); ++i) {
      if (wd::status() & wd::DRQ)
        out(0xDB, img[i]);
    }
    // spin_motor() returns ~300 ms + one index after motor-on; the image is
    // armed on the next pass; the write then waits for the 500 ms readiness
    // gate. Force Interrupt inside that last stretch.
    while (now() < motor_on + ms(470))
      wait_ms(1);
    wd::force_interrupt();
    size_t writes = cable().drive[0].writes.size();
    wait_ms(700);
    CHECK_EQ(cable().drive[0].writes.size(), writes); // nothing written
    CHECK(!(wd::seek(5) & wd::BUSY)); // and the controller completes commands
    CHECK(matches_pattern(wd::read_sector(5, 3, 512).data, 5, 0, 3));
  });
}

// ---------------- Weak bits (#51) ----------------
// A weak sector reads with a bad data CRC on one pass and clean on the next
// (the simulated disk alternates by revolution). A WD179x takes the first ID
// match it meets after the command and reports that pass's data field: CRC
// ERROR when that pass is bad, whatever a later pass would have given. A
// later good copy must never be handed over in its place: weak-bit copy
// protection re-reads such a sector and expects exactly the errors.
namespace {

void make_weak_disk(int cyl, int weak_sector) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  TrackSpec t;
  for (int r = 1; r <= 9; ++r) {
    SectorSpec s;
    s.c = (uint8_t)cyl; s.h = 0; s.r = (uint8_t)r; s.n = 2;
    s.data.resize(512);
    for (int i = 0; i < 512; ++i)
      s.data[i] = pattern_byte(cyl, 0, r, i);
    s.weak = (r == weak_sector);
    t.sectors.push_back(s);
  }
  g_disk[0].t[cyl][0] = encode_track(t);
  insert(0, &g_disk[0]);
  board_boot();
}

// Ticks from `t` until the head next reaches angle `a`.
uint64_t ticks_to_angle(const Drive &d, uint64_t t, uint32_t a) {
  uint32_t fwd = (a + REV_ANGLE - d.angle_at(t)) % REV_ANGLE;
  return d.ticks_for_angle(fwd);
}

// The pass of a sector whose ID the head first meets after `t` reads badly?
// Returns -1 when the ID is too close to `t` to tell (the capture starts a
// little after the command).
int first_pass_bad(const Drive &d, uint64_t t, uint32_t id_angle) {
  uint64_t dt = ticks_to_angle(d, t, id_angle);
  if (dt < ms(3) || dt > d.rev_ticks - ms(1))
    return -1;
  return d.rev_at(t + dt) % 2 == 0;
}

} // namespace

TEST(weak_sector_read_reports_the_pass_the_head_meets_first) {
  make_weak_disk(5, 5);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    wd::read_sector(5, 1, 512); // measure the spindle
    Drive &d = cable().drive[0];
    uint32_t a5 = g_disk[0].t[5][0].id_at[4];
    srand(51);
    int bad = 0, good = 0;
    for (int i = 0; i < 40; ++i) {
      wait_us((uint64_t)(rand() % 200000));
      int exp = first_pass_bad(d, now(), a5);
      if (exp < 0)
        continue;
      wd::Xfer x = wd::read_sector(5, 5, 512);
      if (exp) {
        ++bad;
        CHECK(x.status & wd::CRC);
      } else {
        ++good;
        CHECK_EQ(error_bits(x.status), 0);
        CHECK(matches_pattern(x.data, 5, 0, 5));
      }
    }
    std::printf("  first pass bad %d, good %d\n", bad, good);
    CHECK(bad >= 5);
    CHECK(good >= 5);
  }, ms(120000));
}

// READ MULTIPLE 4..6 with 5 weak: a WD reads sector 5 on the pass right after
// sector 4 -- not a copy of 5 that passed before 4 did, which a decode from
// the start of the capture buffer would find first.
TEST(weak_sector_in_read_multiple_is_read_on_the_pass_after_its_predecessor) {
  make_weak_disk(5, 5);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    wd::read_sector(5, 1, 512); // measure the spindle
    Drive &d = cable().drive[0];
    uint32_t a4 = g_disk[0].t[5][0].id_at[3];
    srand(52);
    int bad = 0, good = 0;
    for (int i = 0; i < 40; ++i) {
      wait_us((uint64_t)(rand() % 200000));
      // Sector 5 follows 4 within the same revolution on this track.
      int exp = first_pass_bad(d, now(), a4);
      if (exp < 0)
        continue;
      wd::Xfer x = wd::read_sector(5, 4, 3 * 512, /*multi=*/true, 5000);
      if (exp) {
        ++bad;
        CHECK(x.status & wd::CRC);
      } else {
        ++good;
        CHECK_EQ(error_bits(x.status), 0);
        CHECK_EQ(x.data.size(), 3u * 512u);
        for (int r = 4; r <= 6 && x.data.size() == 3 * 512; ++r)
          CHECK(matches_pattern(std::vector<uint8_t>(x.data.begin() + (r - 4) * 512,
                                                     x.data.begin() + (r - 3) * 512), 5, 0, r));
      }
    }
    std::printf("  first pass bad %d, good %d\n", bad, good);
    CHECK(bad >= 5);
    CHECK(good >= 5);
  }, ms(120000));
}

// A whole HD track by READ MULTIPLE from random phases. The records after the
// index are read on the capture's second pass, as on a WD, so the capture
// outlives one revolution -- more flux than the buffer holds at HD, which
// slides forward behind the last record read.
TEST(hd_read_multiple_whole_track_at_every_phase) {
  // Zero-filled -- the densest data there is (a transition every bit cell) --
  // but for each sector's number in its first byte, so every slice can be
  // told apart (a repeated or reordered record would otherwise pass).
  g_disk[0] = make_filled_disk(Geometry::HD_DATA, 0x00);
  {
    TrackSpec t;
    t.hd = true;
    for (int r = 1; r <= 18; ++r) {
      SectorSpec sp;
      sp.c = 40; sp.h = 1; sp.r = (uint8_t)r; sp.n = 2;
      sp.data.assign(512, 0x00);
      sp.data[0] = (uint8_t)r;
      t.sectors.push_back(sp);
    }
    g_disk[0].t[40][1] = encode_track(t);
  }
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::density(true);
    wd::select(0);
    wd::restore();
    wd::seek(40);
    wd::side(0);
    wd::read_sector(40, 1, 512); // measure the spindle
    srand(53);
    double worst = 0;
    for (int i = 0; i < 12; ++i) {
      wait_us((uint64_t)(rand() % 200000));
      uint64_t t0 = now();
      wd::Xfer x = wd::read_sector(40, 1, 18 * 512, /*multi=*/true, 5000);
      double t_ms = (double)(now() - t0) / ms(1);
      worst = std::max(worst, t_ms);
      CHECK_EQ(error_bits(x.status), 0);
      CHECK_EQ(x.data.size(), 18u * 512u);
      for (int r = 1; r <= 18 && x.data.size() == 18 * 512; ++r) {
        auto b = x.data.begin() + (r - 1) * 512;
        CHECK_EQ(*b, (uint8_t)r);
        CHECK(std::all_of(b + 1, b + 512, [](uint8_t v) { return v == 0; }));
      }
    }
    std::printf("  HD READ MULTIPLE 18 x 512: worst %.0f ms\n", worst);
    // Rotational latency to sector 1, then one revolution.
    CHECK(worst < 450);
  }, ms(120000));
}

// A field cut off for good -- the disk is pulled while its data field is under
// the head, so no recapture can complete it -- must not be delivered: its
// bytes past the cut are the decoder's zeros, not the disk's, and the CRC
// ERROR they would carry describes the cut.
TEST(field_cut_off_for_good_delivers_no_bytes) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(3);
    wd::side(1);
    wd::read_sector(3, 1, 512); // measure the spindle
    Drive &d = cable().drive[0];
    uint32_t a5 = d.disk->at(3, 0).id_at.empty() ? 0 : d.disk->at(3, 0).id_at[4];
    CHECK(a5 != 0);
    // Issue the READ ~20 ms before sector 5's ID, pull the disk ~6 ms after it
    // (inside the 16 ms data field).
    uint64_t to_id = ticks_to_angle(d, now(), a5);
    if (to_id < ms(25))
      to_id += d.rev_ticks;
    wait_us((to_id - ms(20)) / 24);
    out(0xD9, 3);
    out(0xDA, 5);
    out(0xD8, 0x80);
    wait_us(ms(26) / 24);
    d.disk = nullptr;
    size_t got = 0;
    uint8_t s = 0;
    for (uint64_t end = now() + ms(3000); now() < end;) {
      s = wd::status();
      if (s & wd::DRQ) {
        in(0xDB);
        ++got;
      } else if (!(s & wd::BUSY))
        break;
    }
    std::printf("  got %zu bytes, status %02x\n", got, s);
    CHECK_EQ(got, 0u);
    CHECK(!(s & wd::BUSY));
    CHECK(!(s & wd::CRC));
  }, ms(60000));
}

// ---------------- Drive select follows the spindle (#56) ----------------
// PC drives light their LED from ~DSn. Once the host has turned the motor off
// and the spindle has stopped, the drive is deselected; the next command
// selects it again. While deselected, ~TRK00/~WPT float, so STATUS must keep
// the last sense rather than read the open lines.
TEST(drive_is_deselected_after_the_motor_stops_and_reselected_on_use) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    CHECK_EQ(error_bits(wd::read_sector(0, 1, 512).status), 0);
    CHECK(!pin_level(PIN_DS0)); // selected (active low)
    wd::select(0, /*motor=*/false);
    wait_ms(1000);
    CHECK(!pin_level(PIN_DS0)); // still inside the motor-off delay
    wait_ms(3000);
    CHECK(pin_level(PIN_DS0)); // spindle stopped: released
    CHECK(!cable().drive[0].motor);
    // Deselected, ~TRK00 floats high; the held sense still says track 0.
    wait_ms(50);
    CHECK_EQ(dbg().at_track0, 1);
    // The host comes back: motor on, read -- selected again, and it works.
    wd::select(0);
    wait_ms(10);
    CHECK(!pin_level(PIN_DS0));
    wd::Xfer x = wd::read_sector(0, 2, 512);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 0, 0, 2));
    // (A seek with the motor bit still off is its own test, below.)
    wd::seek(5);
    CHECK(!pin_level(PIN_DS0));
    CHECK_EQ(cable().drive[0].cyl, 5);
  }, ms(60000));
}

// Write protect is read while selected and held across the deselected stretch:
// a protected disk must still refuse a write issued right after the reselect.
TEST(write_protect_survives_the_deselected_stretch) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  g_disk[0].write_protect = true;
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::read_sector(0, 1, 512);
    wd::select(0, false);
    wait_ms(4000);
    CHECK(pin_level(PIN_DS0));
    wait_ms(50);
    CHECK(dbg().flags & 0x02); // write protect still held
    wd::select(0);
    std::vector<uint8_t> d(512, 0xA5);
    uint8_t st = wd::write_sector(0, 3, d).status;
    CHECK(st & wd::WPROT);
    CHECK(cable().drive[0].writes.empty() || cable().drive[0].writes.back().refused_wp);
  }, ms(60000));
}

// Found by review of #57.
TEST(motor_on_landing_in_the_deselect_pass_does_not_latch_not_ready) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    CHECK_EQ(error_bits(wd::read_sector(0, 1, 512).status), 0);
    wd::select(0, /*motor=*/false);
    wait_ms(2900);
    int guard = 0;
    while (!pin_level(PIN_DS0) && guard++ < 400000)
      wait_us(1);
    wd::select(0); // motor on, right as the deselect pass is in select()'s delay
    wait_ms(2000);
    uint8_t st = wd::status();
    CHECK(!(st & wd::NOTREADY));
    wd::Xfer x = wd::read_sector(0, 2, 512);
    CHECK_EQ(error_bits(x.status), 0);
    wait_ms(500);
    CHECK(!(wd::status() & wd::NOTREADY));
  }, ms(60000));
}

TEST(seek_with_the_motor_bit_off_reselects_and_releases_again) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::read_sector(0, 1, 512);
    wd::select(0, false);
    wait_ms(4000);
    CHECK(pin_level(PIN_DS0));
    uint32_t steps0 = cable().drive[0].steps;
    wd::seek(5);
    CHECK_EQ(cable().drive[0].cyl, 5);
    wait_ms(5000);
    CHECK(pin_level(PIN_DS0));
    CHECK(!cable().drive[0].motor);
  }, ms(60000));
}

TEST(first_select_of_unit1_with_motor_off_still_stops_the_spindle) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  g_disk[1] = make_disk(Geometry::MZ_DD_DATA, 80, "B");
  insert(0, &g_disk[0]);
  insert(1, &g_disk[1]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::read_sector(0, 1, 512);
    wd::select(1, false); // first ever select of unit 1, motor bit off
    wait_ms(8000);
    CHECK(!cable().drive[0].motor);
  }, ms(60000));
}

TEST(switching_units_while_deselected_selects_nothing_until_used) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  g_disk[1] = make_disk(Geometry::MZ_DD_DATA, 80, "B");
  g_disk[1].write_protect = true;
  insert(0, &g_disk[0]);
  insert(1, &g_disk[1]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(7);
    wd::side(1);
    wd::read_sector(7, 1, 512);
    wd::select(0, false);
    wait_ms(4000);
    CHECK(pin_level(PIN_DS0));
    wd::select(1, false);
    wait_ms(100);
    wd::select(2, false);
    wait_ms(100);
    CHECK(wd::seek(3) & wd::NOTREADY); // unit 2 does not exist
    wd::select(1);
    wd::restore();
    CHECK_EQ(cable().drive[1].cyl, 0);
    CHECK_EQ(cable().drive[0].cyl, 7); // unit 0's head has not moved
    std::vector<uint8_t> d(512, 0xA5);
    uint8_t w = wd::write_sector(0, 3, d).status;
    CHECK(w & wd::WPROT);
    wd::select(0);
    wd::Xfer x = wd::read_sector(7, 2, 512);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 7, 0, 2));
  }, ms(60000));
}

// The held senses themselves: no FDC path samples ~TRK00/~WPT while the drive
// is deselected today, so this drives FloppyDrive directly.
TEST(deselected_drive_reports_its_last_senses) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  g_disk[0].write_protect = true;
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    CHECK(!fdc_host_busy());
    FloppyDrive *d = fdc_bench_drive();
    REQUIRE(d != nullptr);
    CHECK(d->get_track0_sense());
    CHECK(d->get_write_protect());
    d->select(false);
    CHECK(pin_level(PIN_DS0));
    CHECK(d->get_track0_sense());  // the lines float now; the values are held
    CHECK(d->get_write_protect());
    d->select(true);
    d->goto_track(3);
    CHECK(!d->get_track0_sense()); // and refreshed once selected again
    fdc_bench_release();
    fdc_bench_done();
  });
}

// Second review pass of #57: the spun/ready bookkeeping.
static uint8_t unit_flags(int unit) {
  int t = 0, tg = 0; uint8_t f = 0;
  fdc_drive_state(unit, &t, &tg, &f);
  return f;
}
// Control: empty unit 1 first selected with the motor bit ON -> NOT READY.
TEST(empty_unit1_first_selected_with_motor_on_is_not_ready) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0); wd::restore(); wd::side(1);
    wd::read_sector(0, 1, 512);
    wd::select(1);
    wait_ms(2500);
    CHECK(wd::status() & wd::NOTREADY);
  }, ms(60000));
}
// Empty unit 1 first selected with the motor bit OFF while the spindle line is
// still asserted (deferred-stop window), then motor on: NOT READY never earned.
TEST(empty_unit1_first_selected_with_motor_off_then_on_is_not_ready) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0); wd::restore(); wd::side(1);
    wd::read_sector(0, 1, 512);
    wd::select(1, false);
    wait_ms(100);
    wd::select(1);
    wait_ms(2500);
    CHECK(wd::status() & wd::NOTREADY);
    CHECK(!(unit_flags(1) & 0x01)); // g_ready[1] must be false
  }, ms(60000));
}
// Same, via a motor-off seek on a released drive (seek block spins it).
TEST(unit_first_spun_by_a_motor_off_seek_still_gets_its_catch_up_seek) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  g_disk[1] = make_disk(Geometry::MZ_DD_DATA, 80, "B");
  insert(0, &g_disk[0]); insert(1, &g_disk[1]);
  board_boot();
  run([] {
    wd::select(0); wd::restore(); wd::side(1);
    wd::read_sector(0, 1, 512);
    wd::select(0, false);
    wait_ms(4000);
    wd::select(1, false);
    wd::restore();
    wd::seek(5);
    CHECK_EQ(cable().drive[1].cyl, 5);
    wd::select(1);
    wait_ms(50);
    CHECK(unit_flags(1) & 0x02); // spun
    // Dropped-step catch-up at the next switch back to unit 1.
    wd::select(0);
    wait_ms(50);
    // fold a STEP-IN into unit 1's target while core0 has unit 0 selected
    out(0xDB, 30); out(0xD8, 0x10); wait_ms(10);
    out(0xDC, 0x85); out(0xD8, 0x50); out(0xDC, 0x84);
    wd::wait_ready();
    wait_ms(50);
    int t = 0, tg = 0; uint8_t f = 0;
    fdc_drive_state(1, &t, &tg, &f);
    wd::select(1);
    wait_ms(20);
    wait_ms(1500);
    fdc_drive_state(1, &t, &tg, &f);
    CHECK_EQ(cable().drive[1].cyl, tg);
  }, ms(60000));
}
// First select of unit 1 from an operation block (core0 busy), motor bit
// dropped before the next pass: is ~MOTOR_ON ever released?
TEST(first_select_from_an_operation_then_motor_off_stops_the_spindle) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  g_disk[1] = make_disk(Geometry::MZ_DD_DATA, 80, "B");
  insert(0, &g_disk[0]); insert(1, &g_disk[1]);
  board_boot();
  run([] {
    wd::select(0); wd::restore(); wd::side(1);
    wd::read_sector(0, 1, 512);
    out(0xDB, 40); out(0xD8, 0x10); wait_ms(10); // core0 busy stepping unit 0
    wd::select(1);
    wd::Xfer x = wd::read_address(3000);
    wd::select(1, false);
    wait_ms(8000);
    CHECK(!cable().drive[0].motor);
    CHECK(pin_level(PIN_DS1));
  }, ms(60000));
}

// Third review pass of #57: ~MOTOR_ON is one wire; its state must survive a
// detour through a unit that does not exist.
// 40d9789: the "physical line" is the current DRV's belief. A detour through a
// missing unit (DRV = null) loses it.
TEST(motor_stops_after_a_detour_through_a_missing_unit) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  g_disk[1] = make_disk(Geometry::MZ_DD_DATA, 80, "B");
  insert(0, &g_disk[0]); insert(1, &g_disk[1]);
  board_boot();
  run([] {
    wd::select(0); wd::restore(); wd::side(1);
    wd::read_sector(0, 1, 512);
    wd::select(1, false); // first select of unit 1, motor bit off, line on
    wait_ms(100);
    wd::select(2, false); // no such unit
    wait_ms(100);
    wd::select(0, false);
    wait_ms(8000);
    CHECK(pin_level(PIN_MOTOR)); // high = off
    CHECK(pin_level(PIN_DS0));
    CHECK(pin_level(PIN_DS1));
  }, ms(60000));
}

// Pre-existing: a missing unit selected with the motor bit off.
TEST(motor_stops_while_a_missing_unit_stays_selected) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0); wd::restore(); wd::side(1);
    wd::read_sector(0, 1, 512);
    wd::select(2, false);
    wait_ms(8000);
    CHECK(pin_level(PIN_MOTOR));
  }, ms(60000));
}

// Control for the above two: plain motor off -> stop + release.
TEST(motor_off_stops_the_spindle_and_releases_the_drive) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0); wd::restore(); wd::side(1);
    wd::read_sector(0, 1, 512);
    wd::select(0, false);
    wait_ms(8000);
    CHECK(pin_level(PIN_MOTOR));
    CHECK(pin_level(PIN_DS0));
  }, ms(60000));
}

// ---------------- Force Interrupt with nothing running (#58) ----------------
// WD179x: a Force Interrupt received while no command is executing leaves
// STATUS in the Type I layout (b2 TRK00, b6 WP). One that terminates a
// running command keeps that command's status -- the ROM's READ MULTIPLE +
// 0xD0 needs a clean 0x00 afterwards.
TEST(idle_force_interrupt_shows_type1_status) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  g_disk[0].write_protect = true;
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    CHECK_EQ(error_bits(wd::read_sector(0, 1, 512).status), 0);
    // Type II layout after the read: b2 is LOST DATA, clear.
    CHECK(!(wd::status() & 0x04));
    wd::force_interrupt();
    uint8_t st = wd::status();
    CHECK(!(st & wd::BUSY));
    CHECK(st & wd::TRK0);        // b2 = TRACK 00 now
    CHECK(st & wd::WPROT);       // b6 = WRITE PROTECT
  });
}

TEST(force_interrupt_ending_read_multiple_still_reads_clean) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    // read_sector(..., multi) ends the command with Force Interrupt once it
    // has its bytes, while the look-ahead is still BUSY.
    wd::Xfer x = wd::read_sector(0, 1, 3 * 512, /*multi=*/true, 5000);
    CHECK_EQ(x.data.size(), 3u * 512u);
    CHECK_EQ(x.status, 0);
    CHECK_EQ(wd::status(), 0); // on cylinder 0: a Type I layout would show TRK00
  });
}

TEST(force_interrupt_cutting_a_read_short_keeps_type2_status) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::read_sector(0, 1, 512); // measure the spindle
    out(0xD9, 0);
    out(0xDA, 5);
    out(0xD8, 0x80);
    wait_ms(20); // still searching: BUSY
    CHECK(wd::status() & wd::BUSY);
    wd::force_interrupt();
    uint8_t st = wd::status();
    CHECK(!(st & wd::BUSY));
    CHECK(!(st & wd::TRK0)); // Type II layout kept: b2 is LOST DATA, clear
  });
}

// A command that has ended by itself (CRC, RNF) keeps its error through the
// host's Force Interrupt -- the ROM and CP/M read STATUS only after it. Found
// by review of #59: BUSY drops here the moment the last byte is taken, so an
// "idle" test on BUSY alone turned these into 0x00.
static void crc_disk() {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
  TrackSpec t;
  for (int r = 1; r <= 9; ++r) {
    SectorSpec s;
    s.c = 3; s.h = 0; s.r = (uint8_t)r; s.n = 2;
    s.data.assign(512, 0x11);
    s.bad_data_crc = (r == 6);
    t.sectors.push_back(s);
  }
  g_disk[0].t[3][0] = encode_track(t);
  insert(0, &g_disk[0]);
  board_boot();
}
TEST(crc_on_the_last_sector_of_a_read_multiple_survives_force_interrupt) {
  crc_disk();
  run([] {
    wd::select(0); wd::restore(); wd::seek(3); wd::side(1);
    wd::Xfer x = wd::read_sector(3, 5, 2 * 512, true, 5000);
    std::printf("  multi 5..6 (6 bad): bytes=%zu status after FI=%02x\n", x.data.size(), x.status);
    CHECK(x.status & wd::CRC);
  });
}
TEST(crc_ending_a_read_multiple_survives_a_later_force_interrupt) {
  crc_disk();
  run([] {
    wd::select(0); wd::restore(); wd::seek(3); wd::side(1);
    wd::Xfer x = wd::read_sector(3, 5, 4 * 512, true, 5000);
    std::printf("  multi 5..8 (6 bad): bytes=%zu status at BUSY drop=%02x\n", x.data.size(), x.status);
    wd::force_interrupt();
    uint8_t st = wd::status();
    std::printf("    after FI=%02x\n", st);
    CHECK(st & wd::CRC);
  });
}
TEST(crc_of_a_single_read_survives_force_interrupt) {
  crc_disk();
  run([] {
    wd::select(0); wd::restore(); wd::seek(3); wd::side(1);
    uint8_t s0 = wd::read_sector(3, 6, 512).status;
    wd::force_interrupt(8);
    uint8_t st = wd::status();
    std::printf("  single CRC: before FI=%02x after FI=%02x\n", s0, st);
    CHECK(st & wd::CRC);
  });
}
TEST(rnf_survives_force_interrupt) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0); wd::restore(); wd::seek(7); wd::side(1);
    uint8_t s0 = wd::read_sector(7, 12, 512, true, 5000).status;
    wd::force_interrupt(8);
    uint8_t st = wd::status();
    std::printf("  RNF: before FI=%02x after FI=%02x\n", s0, st);
    CHECK(st & wd::RNF);
  });
}
TEST(write_rnf_survives_force_interrupt) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0); wd::restore(); wd::seek(8); wd::side(1);
    wd::Xfer w = wd::write_sector(9, 3, std::vector<uint8_t>(512, 0x77));
    wd::force_interrupt();
    uint8_t st = wd::status();
    std::printf("  write RNF: before FI=%02x after FI=%02x\n", w.status, st);
    CHECK(st & wd::RNF);
  });
}
TEST(write_refused_by_wp_then_force_interrupt_shows_wp) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  g_disk[0].write_protect = true;
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0); wd::restore(); wd::seek(8); wd::side(1);
    wd::read_sector(8, 1, 512);
    wd::Xfer w = wd::write_sector(8, 3, std::vector<uint8_t>(512, 0x77));
    wd::force_interrupt();
    uint8_t st = wd::status();
    std::printf("  write WP: before FI=%02x after FI=%02x\n", w.status, st);
    CHECK(st & wd::WPROT);
  });
}
TEST(idle_force_interrupt_after_a_clean_read_shows_no_stale_seek_error) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0); wd::restore(); wd::seek(10); wait_ms(100);
    cable().drive[0].cyl = 12;
    wd::side(1);
    uint8_t s1 = wd::seek(10, true);
    wait_ms(200);
    wd::Xfer x = wd::read_sector(10, 5, 512);
    wd::force_interrupt();
    uint8_t st = wd::status();
    std::printf("  stale seekerr: seek=%02x read=%02x after FI=%02x\n", s1, x.status, st);
    CHECK(!(st & 0x10));
  });
}
TEST(force_interrupt_right_after_the_last_byte_keeps_the_read_status) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0); wd::restore(); wd::side(1);
    out(0xD9, 0); out(0xDA, 1); wd::command(0x80);
    int n = 0;
    while (n < 512) { if (wd::status() & wd::DRQ) { in(0xDB); ++n; } }
    wd::force_interrupt(8);   // no STATUS poll between last byte and FI
    uint8_t st = wd::status();
    std::printf("  single read cyl0, FI right after last byte: %02x\n", st);
    CHECK_EQ(st, 0);
  });
}
TEST(read_address_then_force_interrupt) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0); wd::restore(); wd::side(1);
    wd::Xfer x = wd::read_address();
    wd::force_interrupt();
    uint8_t st = wd::status();
    std::printf("  read address: before=%02x after FI=%02x\n", x.status, st);
    CHECK(!(st & (wd::BUSY | wd::CRC | wd::RNF)));
  });
}

// ======================= re-review of #59 (repro tests) =======================
void sim_fire_gpio_irq(void);
namespace sim { void pin_set_input(int pin, bool level); }

// R1. CP/M BIOS write path (local image, routine at F9F8): seek (no step when
// already there), Force Interrupt, wait !BUSY, read STATUS and test bit 6
// (write protect); if set -> retry up to 5x WITHOUT recalibrating. Else WRITE,
// and on a bad status recalibrate (STEP+FI, READ ADDRESS) and retry.
static bool cpm_bios_write(uint8_t trk, uint8_t sec, int *precheck_wp, int *cmd_refused) {
  for (int tries = 0; tries < 5; ++tries) {
    wd::force_interrupt();
    wd::wait_ready();
    uint8_t st = wd::status();
    if (st & wd::WPROT) { ++*precheck_wp; continue; }
    wd::Xfer w = wd::write_sector(trk, sec, std::vector<uint8_t>(512, 0x5A));
    wait_ms(1);
    if (wd::status() == 0) return true;
    ++*cmd_refused;
    // recalibrate: STEP IN + FI, READ ADDRESS, Track := id; then step back
    wd::command(0x58); wd::force_interrupt(); wd::wait_ready();
    wd::Xfer a = wd::read_address();
    wd::wait_ready();
    if (a.data.size() >= 1) out(0xD9, a.data[0]);
    wd::command(0x78); wd::force_interrupt(); wd::wait_ready();
  }
  return false;
}
TEST(cpm_write_precheck_sees_live_write_protect_after_a_disk_swap) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  g_disk[0].write_protect = true;
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0); wd::restore(); wd::seek(2); wd::side(1);
    CHECK_EQ(error_bits(wd::read_sector(2, 1, 512).status), 0);
    g_disk[0].write_protect = false;       // user opens the tab, same disk back in
    CHECK_EQ(error_bits(wd::read_sector(2, 1, 512).status), 0); // directory read, no step
    int pre = 0, ref = 0;
    bool ok = cpm_bios_write(2, 3, &pre, &ref);
    std::printf("  CP/M write after WP removed: ok=%d precheck_wp=%d cmd_refused=%d\n", ok, pre, ref);
    CHECK(ok);
  });
}

// R2. Clean single read on cylinder 0, host polls until !BUSY (as CP/M's
// read does), then an idle Force Interrupt, then STATUS.
TEST(idle_force_interrupt_on_cylinder_0_reads_trk00_only) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0); wd::restore(); wd::side(1);
    wd::Xfer x = wd::read_sector(0, 1, 512);
    wd::force_interrupt();
    uint8_t st = wd::status();
    CHECK_EQ(x.status, 0);
    CHECK_EQ(st, wd::TRK0); // Type I view: track 0, nothing else
    wd::Xfer a = wd::read_address();
    wd::force_interrupt();
    CHECK_EQ(a.status, 0);
    CHECK_EQ(wd::status(), wd::TRK0);
  });
}

// R3. Bus RESET with the Type I-after-FI view active.
TEST(bus_reset_clears_the_status_register) {
  crc_disk();
  run([] {
    wd::select(0); wd::restore(); wd::seek(3); wd::side(1);
    uint8_t s0 = wd::read_sector(3, 6, 512).status;
    wd::force_interrupt();
    uint8_t s1 = wd::status();
    sim::pin_set_input(29, true);  sim_fire_gpio_irq();
    wait_us(65);
    sim::pin_set_input(29, false); sim_fire_gpio_irq();
    uint8_t s2 = wd::status();
    std::printf("  read=%02x after FI=%02x after bus RESET=%02x\n", s0, s1, s2);
    CHECK(!(s2 & wd::CRC));
  });
}

// R4. LOST DATA / refusal by the wren switch: Type II bits with no Type I twin.
TEST(lost_data_and_refused_write_bits_change_meaning_in_the_type1_view) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0); wd::restore(); wd::seek(5); wd::side(1);
    wd::read_sector(5, 1, 512);
    out(0xD9, 5); out(0xDA, 2); wd::command(0x80);
    uint8_t s0 = wd::wait_ready();           // never take a byte: LOST DATA
    wd::force_interrupt();
    uint8_t s1 = wd::status();
    CHECK(s0 & wd::LOST);   // the host saw LOST DATA in the final status...
    CHECK(!(s1 & wd::TRK0)); // ...and bit 2 is TRK00 afterwards (cylinder 5)
    fdc_set_write_enable(false);
    wd::Xfer w = wd::write_sector(5, 3, std::vector<uint8_t>(512, 1));
    wd::force_interrupt();
    CHECK(w.status & wd::WPROT);       // refused: reported once...
    CHECK(!(wd::status() & wd::WPROT)); // ...then bit 6 is the drive's own WP
    fdc_set_write_enable(true);
  });
}

// R5 (not this PR; Disk BASIC WRITE MULTIPLE ends with FI straight after the
// last byte of the last sector, A39FA).
TEST(write_multiple_cut_by_force_interrupt_after_the_last_byte_is_all_or_nothing) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0); wd::restore(); wd::seek(6); wd::side(1);
    wd::read_sector(6, 1, 512);
    out(0xD9, 6); out(0xDA, 4); wd::command(0xB4);
    int n = 0;
    while (n < 512) { if (wd::status() & wd::DRQ) { out(0xDB, 0xC3); ++n; } }
    uint8_t sp = wd::status();
    wd::force_interrupt();
    uint8_t st = wd::wait_ready();
    wait_ms(500);
    wd::Xfer r = wd::read_sector(6, 4, 512);
    int same = 0; for (uint8_t b : r.data) same += (b == 0xC3);
    (void)sp;
    CHECK_EQ(st, 0);
    // Whether the sector lands depends on whether core0 had begun the splice
    // when the Force Interrupt arrived (on a WD this sequence cuts the write
    // short, so no host relies on it). What must hold: all or nothing, with
    // valid CRC either way -- never a half-written sector.
    CHECK_EQ(error_bits(r.status), 0);
    CHECK(same == 512 || matches_pattern(r.data, 6, 0, 4));
  });
}

// R6. A: write-protected, B: not. Read A, select B (no step), CP/M pre-check.
TEST(write_protect_precheck_follows_the_selected_drive) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  g_disk[0].write_protect = true;
  g_disk[1] = make_disk(Geometry::MZ_DD_DATA, 80, "B");
  insert(0, &g_disk[0]);
  insert(1, &g_disk[1]);
  board_boot();
  run([] {
    wd::select(1); wd::restore(); wd::seek(2); wd::side(1);
    wd::read_sector(2, 1, 512);
    wd::select(0); wd::restore(); wd::seek(2); wd::side(1);
    for (int us : {0, 50, 100, 300, 1000}) {
      wd::select(0);
      wait_ms(5);
      CHECK_EQ(error_bits(wd::read_sector(2, 1, 512).status), 0);
      wd::select(1);
      out(0xD9, 2);
      if (us) wait_us((uint32_t)us);
      wd::force_interrupt();
      uint8_t st = wd::status();
      CHECK(!(st & wd::WPROT)); // B is writable: A's sense must not show
    }
  });
}

// #25: the write block snapshots the command generation before the drive
// select, side and bench-head restore, which can block for many ms. A Force
// Interrupt landing in that window must stop the splice: nothing reaches the
// media. Here a bench command left the head 60 cylinders away, so the restore
// takes >100 ms, and the host gives up 20 ms into it.
TEST(force_interrupt_during_the_pre_splice_restore_writes_nothing) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(10);
    wd::side(1);
    CHECK_EQ(error_bits(wd::read_sector(10, 1, 512).status), 0);
    // Motor off and stopped, THEN the bench moves the head: with the spindle
    // down the select catch-up is skipped, so the head is still away when the
    // host's write is armed, and the write block's own restore moves it.
    wd::select(0, false);
    wait_ms(4000);
    FloppyDrive *d = fdc_bench_drive();
    REQUIRE(d != nullptr);
    d->goto_track(70);
    fdc_bench_release();
    fdc_bench_done();
    wait_ms(50);
    CHECK_EQ(cable().drive[0].cyl, 70);
    wd::select(0); // motor on again
    size_t writes_before = cable().drive[0].writes.size();
    out(0xD9, 10);
    out(0xDA, 3);
    out(0xD8, 0xA0); // WRITE SECTOR
    size_t fed = 0;
    for (uint64_t end = now() + ms(3000); now() < end && fed < 512;) {
      if (wd::status() & wd::DRQ) {
        out(0xDB, 0x5A);
        ++fed;
      }
    }
    CHECK_EQ(fed, 512u);
    // Wait until the restore is under way, then give up.
    for (uint64_t end = now() + ms(1000); now() < end && cable().drive[0].cyl == 70;)
      wait_us(100);
    wait_ms(5);
    CHECK(cable().drive[0].cyl > 10 && cable().drive[0].cyl < 70);
    wd::force_interrupt();
    wait_ms(1500);
    CHECK(!(wd::status() & wd::BUSY));
    CHECK_EQ(cable().drive[0].writes.size(), writes_before);
    wd::Xfer x = wd::read_sector(10, 3, 512); // the sector is untouched
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 10, 0, 3));
  });
}

namespace {
void bench_strand_head_at_70() {
  wd::select(0);
  wd::restore();
  wd::seek(10);
  wd::side(1);
  CHECK_EQ(error_bits(wd::read_sector(10, 1, 512).status), 0);
  wd::select(0, false);
  wait_ms(4000);
  FloppyDrive *d = fdc_bench_drive();
  REQUIRE(d != nullptr);
  d->goto_track(70);
  fdc_bench_release();
  fdc_bench_done();
  wait_ms(50);
  CHECK_EQ(cable().drive[0].cyl, 70);
  wd::select(0);
}
void wait_restore_under_way() {
  for (uint64_t end = now() + ms(2000); now() < end && cable().drive[0].cyl == 70;)
    wait_us(100);
  wait_ms(5);
  CHECK(cable().drive[0].cyl > 10 && cable().drive[0].cyl < 70);
}
}

TEST(read_for_the_other_side_issued_during_the_pre_capture_restore_gets_it) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    bench_strand_head_at_70();
    out(0xD9, 10);
    out(0xDA, 1);
    out(0xD8, 0x80); // read A on head 0
    wait_restore_under_way();
    wd::force_interrupt();
    wd::side(0); // head 1
    wd::Xfer b = wd::read_sector(10, 5, 512);
    std::fprintf(stderr, "  status %02x n=%zu h0=%d h1=%d\n", b.status, b.data.size(),
                 (int)matches_pattern(b.data, 10, 0, 5), (int)matches_pattern(b.data, 10, 1, 5));
    CHECK_EQ(error_bits(b.status), 0);
    CHECK(matches_pattern(b.data, 10, 1, 5));
  });
}

TEST(read_behind_a_step_issued_during_the_pre_capture_restore_waits_for_it) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    bench_strand_head_at_70();
    out(0xD9, 10);
    out(0xDA, 1);
    out(0xD8, 0x80); // read A
    wait_restore_under_way();
    wd::force_interrupt();
    out(0xD8, 0x58); // STEP IN with update, no wait
    wd::Xfer b = wd::read_sector(11, 5, 512);
    std::fprintf(stderr, "  status %02x n=%zu c11=%d c10=%d\n", b.status, b.data.size(),
                 (int)matches_pattern(b.data, 11, 0, 5), (int)matches_pattern(b.data, 10, 0, 5));
    CHECK_EQ(error_bits(b.status), 0);
    CHECK(matches_pattern(b.data, 11, 0, 5));
  });
}

// ---- re-review of #60: armed WRITE TRACK superseded before core0 writes ----
namespace {
// Feed a padded image until the firmware stops asking (buffer-full arming on
// core1), returning right after the last accepted byte.
void feed_track_until_armed(const std::vector<uint8_t> &img) {
  out(0xD8, 0xF0);
  size_t i = 0;
  for (uint64_t end = now() + ms(3000); now() < end;) {
    if (!(wd::status() & wd::DRQ)) {
      if (i > 0) break;
      continue;
    }
    out(0xDB, i < img.size() ? img[i] : 0x4E);
    ++i;
    if (dbg().trk_armed) break;
  }
  CHECK(dbg().trk_armed);
}
void report(const char *what) {
  FdcDebug d = dbg();
  std::fprintf(stderr, "  %s: status %02x trk_op=%d armed=%d\n", what, wd::status(),
               (int)d.trk_op, (int)d.trk_armed);
}
}

// A: Force Interrupt immediately after the last image byte, motor running.
TEST(force_interrupt_during_the_track_write_leaves_the_controller_usable) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    CHECK(matches_pattern(wd::read_sector(5, 3, 512).data, 5, 0, 3));
    size_t writes = cable().drive[0].writes.size();
    feed_track_until_armed(wd::format_image(5, 0, 9, 2, false));
    out(0xD8, 0xD0);
    wait_ms(700);
    report("after FI");
    // core0 is already in the track write (waiting for the index) by the time
    // the Force Interrupt lands: a write under way cannot be recalled, on a
    // WD either. What must hold is that the controller stays usable.
    CHECK(cable().drive[0].writes.size() <= writes + 1);
    CHECK_EQ(dbg().trk_op, 0);
    CHECK(!(wd::seek(6) & wd::BUSY));
    CHECK(matches_pattern(wd::read_sector(6, 3, 512).data, 6, 0, 3));
    CHECK(matches_pattern(wd::read_sector(6, 4, 512).data, 6, 0, 4));
  });
}

// B: motor bit off -> the track block spins up in-block (blocks ~300 ms+);
// Force Interrupt inside that.
TEST(force_interrupt_superseding_an_armed_write_track_formats_nothing) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    wd::select(0, false);
    wait_ms(3600);
    size_t writes = cable().drive[0].writes.size();
    feed_track_until_armed(wd::format_image(5, 0, 9, 2, false));
    wait_ms(100);
    out(0xD8, 0xD0);
    wait_ms(1500);
    report("after FI");
    CHECK_EQ(cable().drive[0].writes.size(), writes);
    CHECK_EQ(dbg().trk_op, 0);
    wd::select(0);
    CHECK(!(wd::seek(6) & wd::BUSY));
    CHECK(matches_pattern(wd::read_sector(6, 3, 512).data, 6, 0, 3));
  });
}

// C: Type I (STEP IN) in the same window.
TEST(step_superseding_an_armed_write_track_completes_and_formats_nothing) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    wd::select(0, false);
    wait_ms(3600);
    size_t writes = cable().drive[0].writes.size();
    feed_track_until_armed(wd::format_image(5, 0, 9, 2, false));
    wait_ms(100);
    out(0xD8, 0x50);
    uint8_t st = wd::wait_ready(3000);
    report("after STEP");
    std::fprintf(stderr, "  st=%02x cyl=%d writes+%zu\n", st, cable().drive[0].cyl,
                 cable().drive[0].writes.size() - writes);
    CHECK(!(st & wd::BUSY));
    CHECK_EQ(dbg().trk_op, 0);
    wd::select(0);
    CHECK(matches_pattern(wd::read_sector(6, 3, 512).data, 6, 0, 3));
  });
}

// D: a READ SECTOR in the same window.
TEST(read_superseding_an_armed_write_track_is_served) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    wd::select(0, false);
    wait_ms(3600);
    size_t writes = cable().drive[0].writes.size();
    feed_track_until_armed(wd::format_image(5, 0, 9, 2, false));
    wait_ms(100);
    wd::select(0);
    wd::Xfer r = wd::read_sector(5, 3, 512);
    report("after READ");
    std::fprintf(stderr, "  st=%02x n=%zu to=%d writes+%zu\n", r.status, r.data.size(),
                 (int)r.timed_out, cable().drive[0].writes.size() - writes);
    CHECK(!r.timed_out);
    CHECK_EQ(dbg().trk_op, 0);
  });
}

// Third review pass of #60: commands issued over a format in flight.

// 7700cce: a second WRITE TRACK issued while the first is physically being
// written shares its trk_seq (WRITE TRACK only bumps trk_seq when it arms), so
// the post-write clear takes it for the finished one.
TEST(write_track_issued_during_the_previous_track_write_survives) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    CHECK(matches_pattern(wd::read_sector(5, 3, 512).data, 5, 0, 3));
    size_t writes = cable().drive[0].writes.size();
    feed_track_until_armed(wd::format_image(5, 0, 9, 2, false, 0x11));
    wait_ms(30); // core0 is in the track write (index wait / laying it down)
    wd::Xfer f = wd::write_track(wd::format_image(5, 0, 9, 2, false, 0x22), 3000);
    std::fprintf(stderr, "  timed_out=%d status=%02x writes+%zu\n", (int)f.timed_out,
                 f.status, cable().drive[0].writes.size() - writes);
    CHECK(!f.timed_out);
    CHECK_EQ(error_bits(f.status), 0);
    wd::Xfer r = wd::read_sector(5, 4, 512);
    CHECK(r.data == std::vector<uint8_t>(512, 0x22));
  });
}

// Same with the Force Interrupt a timing-out host would send first.
TEST(write_track_retry_after_force_interrupt_during_the_track_write) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    CHECK(matches_pattern(wd::read_sector(5, 3, 512).data, 5, 0, 3));
    feed_track_until_armed(wd::format_image(5, 0, 9, 2, false, 0x11));
    wait_ms(30);
    wd::force_interrupt();
    wd::Xfer f = wd::write_track(wd::format_image(5, 0, 9, 2, false, 0x22), 3000);
    CHECK(!f.timed_out);
    CHECK_EQ(error_bits(f.status), 0);
    wd::Xfer r = wd::read_sector(5, 4, 512);
    CHECK(r.data == std::vector<uint8_t>(512, 0x22));
  });
}




namespace {
// A host at Z80 speed (~30 us per polled byte) that sends its image padded,
// like FORMAT4, until DRQ drops or BUSY clears.
wd::Xfer slow_write_track(const std::vector<uint8_t> &img, uint32_t timeout_ms,
                              bool pad, uint32_t byte_us) {
  wd::Xfer x;
  out(0xD8, 0xF0);
  size_t i = 0;
  uint64_t end = now() + ms(timeout_ms);
  while (now() < end) {
    uint8_t s = wd::status();
    if ((s & wd::DRQ) && (pad || i < img.size())) {
      out(0xDB, i < img.size() ? img[i] : 0x4E);
      ++i;
      wait_us(byte_us);
      continue;
    }
    if (!(s & wd::BUSY)) { x.status = s; x.data.resize(i); return x; }
  }
  x.timed_out = true;
  x.data.resize(i);
  x.status = wd::status();
  return x;
}
}

TEST(slow_fed_write_track_issued_during_the_previous_track_write_survives) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    CHECK(matches_pattern(wd::read_sector(5, 3, 512).data, 5, 0, 3));
    size_t writes = cable().drive[0].writes.size();
    feed_track_until_armed(wd::format_image(5, 0, 9, 2, false, 0x11));
    wait_ms(30);
    wd::Xfer f = slow_write_track(wd::format_image(5, 0, 9, 2, false, 0x22), 3000, true, 30);
    std::fprintf(stderr, "  timed_out=%d status=%02x fed=%zu writes+%zu\n", (int)f.timed_out,
                 f.status, f.data.size(), cable().drive[0].writes.size() - writes);
    CHECK(!f.timed_out);
    CHECK_EQ(error_bits(f.status), 0);
    wd::force_interrupt();
    wd::Xfer r = wd::read_sector(5, 4, 512);
    CHECK(r.data == std::vector<uint8_t>(512, 0x22));
  });
}

TEST(slow_fed_write_track_retry_after_force_interrupt_during_the_track_write) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    CHECK(matches_pattern(wd::read_sector(5, 3, 512).data, 5, 0, 3));
    size_t writes = cable().drive[0].writes.size();
    feed_track_until_armed(wd::format_image(5, 0, 9, 2, false, 0x11));
    wait_ms(30);
    wd::force_interrupt();
    wd::Xfer f = slow_write_track(wd::format_image(5, 0, 9, 2, false, 0x22), 3000, false, 30);
    std::fprintf(stderr, "  timed_out=%d status=%02x fed=%zu writes+%zu\n", (int)f.timed_out,
                 f.status, f.data.size(), cable().drive[0].writes.size() - writes);
    CHECK(!f.timed_out);
    CHECK_EQ(error_bits(f.status), 0);
    wd::force_interrupt();
    wd::Xfer r = wd::read_sector(5, 4, 512);
    CHECK(r.data == std::vector<uint8_t>(512, 0x22));
  });
}

// DATA-port handler: `trk_op == 2 && drq` does not look at trk_armed. A WRITE
// SECTOR issued while an armed (buffer-full) format is being laid down has its
// first data byte taken as a track byte, which RE-ARMS the old image under the
// new command's generation: the drop no longer sees it as superseded.
TEST(write_sector_during_a_track_write_does_not_rearm_the_format) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    CHECK(matches_pattern(wd::read_sector(5, 3, 512).data, 5, 0, 3));
    size_t writes = cable().drive[0].writes.size();
    feed_track_until_armed(wd::format_image(5, 0, 9, 2, false, 0x11));
    wait_ms(30);
    wd::force_interrupt();
    wd::Xfer w = wd::write_sector(5, 2, std::vector<uint8_t>(512, 0x33), false, 3000);
    wait_ms(1000);
    size_t full = 0;
    for (size_t k = writes; k < cable().drive[0].writes.size(); ++k)
      if (cable().drive[0].writes[k].full_track) ++full;
    std::fprintf(stderr, "  timed_out=%d status=%02x fed=%zu writes+%zu full=%zu\n",
                 (int)w.timed_out, w.status, w.data.size(),
                 cable().drive[0].writes.size() - writes, full);
    CHECK(full <= 1);
    CHECK(!w.timed_out);
    wd::force_interrupt();
    wd::Xfer r = wd::read_sector(5, 2, 512);
    std::fprintf(stderr, "  readback st=%02x first=%02x\n", r.status, r.data.empty() ? 0 : r.data[0]);
    // The host was told the sector write completed without error: it must be there.
    if (!w.timed_out && error_bits(w.status) == 0)
      CHECK(r.data == std::vector<uint8_t>(512, 0x33));
  });
}

// The dangerous variant: the host also moves the head. The re-armed image is
// then laid down on a cylinder the host never formatted.
TEST(seek_and_write_sector_during_a_track_write_do_not_format_the_new_cylinder) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    CHECK(matches_pattern(wd::read_sector(5, 3, 512).data, 5, 0, 3));
    size_t writes = cable().drive[0].writes.size();
    feed_track_until_armed(wd::format_image(5, 0, 9, 2, false, 0x11));
    wait_ms(30);
    wd::force_interrupt();
    out(0xDB, 6);
    out(0xD8, 0x10); // SEEK 6, not waited for (CP/M style)
    wd::Xfer w = wd::write_sector(6, 2, std::vector<uint8_t>(512, 0x33), false, 3000);
    wait_ms(1000);
    for (size_t k = writes; k < cable().drive[0].writes.size(); ++k)
      std::fprintf(stderr, "  write: cyl=%d head=%d full=%d\n", cable().drive[0].writes[k].cyl,
                   cable().drive[0].writes[k].head, (int)cable().drive[0].writes[k].full_track);
    for (size_t k = writes; k < cable().drive[0].writes.size(); ++k)
      CHECK(!(cable().drive[0].writes[k].full_track && cable().drive[0].writes[k].cyl == 6));
    std::fprintf(stderr, "  ws timed_out=%d status=%02x fed=%zu\n", (int)w.timed_out, w.status, w.data.size());
    wd::force_interrupt();
    wd::Xfer r = wd::read_sector(6, 4, 512);
    std::fprintf(stderr, "  cyl 6 sec 4 readback st=%02x n=%zu\n", r.status, r.data.size());
    CHECK(matches_pattern(r.data, 6, 0, 4));
  });
}

// ======== Integration review: #57 x #59 x #60 combined scenarios ========
namespace {
void integ_release_drive0() {
  wd::select(0);
  wd::restore();
  wd::side(1);
  CHECK_EQ(error_bits(wd::read_sector(0, 1, 512).status), 0);
  wd::select(0, /*motor=*/false);
  wait_ms(4000);
  CHECK(pin_level(PIN_DS0)); // released
  CHECK(!cable().drive[0].motor);
}
size_t integ_feed(uint8_t cmd, const std::vector<uint8_t> &d, uint32_t budget_ms) {
  out(0xD8, cmd);
  size_t fed = 0;
  for (uint64_t end = now() + ms(budget_ms); now() < end && fed < d.size();) {
    if (wd::status() & wd::DRQ)
      out(0xDB, d[fed++]);
  }
  return fed;
}
} // namespace

// (a) released -> motor-on + READ at once -> Force Interrupt during the
// spin-up -> new READ.
TEST(integ_a_read_abandoned_during_spinup_of_a_released_drive) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    integ_release_drive0();
    wd::select(0);
    out(0xD9, 0);
    out(0xDA, 2);
    out(0xD8, 0x80);
    wait_ms(100); // inside spin_motor()'s 300 ms
    wd::force_interrupt();
    CHECK(!(wd::status() & wd::BUSY));
    wd::Xfer x = wd::read_sector(0, 3, 512);
    CHECK(!x.timed_out);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 0, 0, 3));
    CHECK(!pin_level(PIN_DS0));
    CHECK(cable().drive[0].motor);
    // idle FI after the status was read: Type I view, TRK00 set, no errors
    wd::force_interrupt();
    uint8_t st = wd::status();
    CHECK(st & wd::TRK0);
    CHECK(!(st & (wd::BUSY | wd::CRC | wd::SEEKERR | wd::NOTREADY)));
    wd::select(0, false);
    wait_ms(4000);
    CHECK(pin_level(PIN_DS0));
    CHECK(!cable().drive[0].motor);
  });
}

// (a2) the same, Force Interrupt at many offsets through spin-up and capture.
TEST(integ_a2_force_interrupt_swept_through_spinup_of_a_released_drive) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    integ_release_drive0();
    for (int off = 0; off < 800; off += 37) {
      wd::select(0);
      out(0xD9, 0);
      out(0xDA, 2);
      out(0xD8, 0x80);
      wait_ms(off);
      wd::force_interrupt();
      wait_us(50);
      CHECK(!(wd::status() & wd::BUSY));
      wd::Xfer x = wd::read_sector(0, 4, 512);
      CHECK(!x.timed_out);
      CHECK_EQ(error_bits(x.status), 0);
      CHECK(matches_pattern(x.data, 0, 0, 4));
      CHECK(!pin_level(PIN_DS0));
      wd::select(0, false);
      wait_ms(4000);
      CHECK(pin_level(PIN_DS0));
      CHECK(!cable().drive[0].motor);
    }
  }, ms(200000));
}

// (b) WRITE SECTOR armed while released (motor bit off), then Force Interrupt.
TEST(integ_b_write_armed_on_released_drive_then_force_interrupt) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    integ_release_drive0();
    size_t w0 = cable().drive[0].writes.size();
    out(0xD9, 0);
    out(0xDA, 3);
    CHECK_EQ(integ_feed(0xA0, std::vector<uint8_t>(512, 0x5A), 3000), 512u);
    wd::force_interrupt();
    CHECK(!(wd::status() & wd::BUSY));
    wait_ms(1500);
    CHECK(!(wd::status() & wd::BUSY));
    CHECK_EQ(cable().drive[0].writes.size(), w0);
    wait_ms(5000);
    CHECK(pin_level(PIN_DS0));
    CHECK(!cable().drive[0].motor);
    wd::select(0);
    wd::Xfer x = wd::read_sector(0, 3, 512);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 0, 0, 3));
  });
}

// (b2) half-fed write on a released drive, Force Interrupt, released again.
TEST(integ_b2_half_fed_write_on_released_drive_then_force_interrupt) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    integ_release_drive0();
    size_t w0 = cable().drive[0].writes.size();
    out(0xD9, 0);
    out(0xDA, 3);
    CHECK_EQ(integ_feed(0xA0, std::vector<uint8_t>(100, 0x5A), 3000), 100u);
    wd::force_interrupt();
    wait_ms(100);
    CHECK(!(wd::status() & wd::BUSY));
    wait_ms(5000);
    CHECK_EQ(cable().drive[0].writes.size(), w0);
    CHECK(pin_level(PIN_DS0));
    CHECK(!cable().drive[0].motor);
  });
}

// (b3) write with the motor bit off on a released drive, left to run.
TEST(integ_b3_write_with_motor_bit_off_on_released_drive_terminates) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    integ_release_drive0();
    wd::Xfer w = wd::write_sector(0, 3, std::vector<uint8_t>(512, 0x5A), false, 8000);
    CHECK(!w.timed_out);
    std::printf("    b3: status %02X writes %zu\n", w.status, cable().drive[0].writes.size());
    wait_ms(6000);
    CHECK(!(wd::status() & wd::BUSY));
    CHECK(pin_level(PIN_DS0));
    CHECK(!cable().drive[0].motor);
  });
}

// (c) released drive, bench moves the head, host READ with the motor bit OFF.
TEST(integ_c_bench_move_while_released_then_read_with_motor_bit_off) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(10);
    wd::side(1);
    CHECK_EQ(error_bits(wd::read_sector(10, 1, 512).status), 0);
    wd::select(0, false);
    wait_ms(4000);
    CHECK(pin_level(PIN_DS0));
    FloppyDrive *d = fdc_bench_drive();
    REQUIRE(d != nullptr);
    d->goto_track(70);
    fdc_bench_release();
    fdc_bench_done();
    wait_ms(50);
    CHECK_EQ(cable().drive[0].cyl, 70);
    CHECK(pin_level(PIN_DS0)); // idle passes neither move nor select
    wd::Xfer x = wd::read_sector(10, 2, 512, false, 5000);
    std::printf("    c: status %02X cyl %d\n", x.status, cable().drive[0].cyl);
    CHECK(!x.timed_out);
    // motor bit off = NOT READY in STATUS by design; nothing else may be set
    CHECK_EQ(error_bits(x.status) & ~wd::NOTREADY, 0);
    CHECK(matches_pattern(x.data, 10, 0, 2));
    CHECK_EQ(cable().drive[0].cyl, 10);
    wait_ms(5000);
    CHECK(pin_level(PIN_DS0));
    CHECK(!cable().drive[0].motor);
    // and with the motor on
    wd::select(0);
    x = wd::read_sector(10, 3, 512);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 10, 0, 3));
  });
}

// (d) CRC error, status polled, idle Force Interrupt, motor off 4 s, motor on
// + READ of a good sector.
TEST(integ_d_latched_crc_does_not_survive_release_and_next_read) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
  TrackSpec t;
  for (int r = 1; r <= 9; ++r) {
    SectorSpec s;
    s.c = 3; s.h = 0; s.r = (uint8_t)r; s.n = 2;
    s.data.assign(512, 0x11);
    s.bad_data_crc = (r == 6);
    t.sectors.push_back(s);
  }
  g_disk[0].t[3][0] = encode_track(t);
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(3);
    wd::side(1);
    uint8_t st = wd::read_sector(3, 6, 512).status;
    CHECK(st & wd::CRC);
    CHECK(wd::status() & wd::CRC);
    wd::force_interrupt(); // idle: Type I view, CRC stays latched
    st = wd::status();
    CHECK(st & wd::CRC);
    CHECK(!(st & wd::BUSY));
    wd::select(0, false);
    wait_ms(4000);
    CHECK(pin_level(PIN_DS0));
    st = wd::status(); // still the Type I view with the latched CRC, released
    CHECK(st & wd::CRC);
    CHECK(st & wd::NOTREADY); // motor bit off = NOT READY, by design
    wd::select(0);
    wd::Xfer x = wd::read_sector(3, 5, 512);
    CHECK_EQ(x.status, 0);
    CHECK_EQ(x.data.size(), 512u);
    CHECK(!pin_level(PIN_DS0));
    wd::force_interrupt();
    st = wd::status();
    CHECK(!(st & (wd::CRC | wd::SEEKERR | wd::BUSY)));
    // FI with motor-on + READ at once on a released drive, after a CRC
    wd::read_sector(3, 6, 512);
    wd::select(0, false);
    wait_ms(4000);
    wd::select(0);
    out(0xD9, 3);
    out(0xDA, 5);
    out(0xD8, 0x80);
    wait_ms(50);
    wd::force_interrupt();
    st = wd::status();
    CHECK(!(st & (wd::CRC | wd::BUSY | wd::RNF)));
    x = wd::read_sector(3, 5, 512);
    CHECK_EQ(x.status, 0);
  });
}

// (e) WRITE TRACK with the motor bit off on a released drive: the image is
// armed, the track block spins the drive up (blocking), and the host gives up
// with a Force Interrupt during that spin-up.
TEST(integ_e_write_track_abandoned_during_its_own_spinup) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    integ_release_drive0();
    size_t w0 = cable().drive[0].writes.size();
    std::vector<uint8_t> img = wd::format_image(0, 0, 9, 2, false);
    CHECK_EQ(integ_feed(0xF0, img, 3000), img.size());
    // silence arms the image; then the track block's spin_motor() blocks
    for (uint64_t end = now() + ms(500); now() < end && !cable().drive[0].motor;)
      wait_us(100);
    CHECK(cable().drive[0].motor);
    wait_ms(50);
    wd::force_interrupt();
    wait_ms(2000);
    FdcDebug d = dbg();
    std::printf("    e: trk_op %d armed %d writes +%zu\n", d.trk_op, d.trk_armed,
                cable().drive[0].writes.size() - w0);
    CHECK_EQ(cable().drive[0].writes.size(), w0);
    CHECK_EQ(d.trk_op, 0);
    CHECK(!(wd::seek(5) & wd::BUSY));
    wd::Xfer x = wd::read_sector(5, 3, 512);
    CHECK(!x.timed_out);
    CHECK(matches_pattern(x.data, 5, 0, 3));
    wd::select(0, false);
    wait_ms(5000);
    CHECK(pin_level(PIN_DS0)); // released again
  });
}

// (e2) WRITE TRACK on an empty drive; the host gives up before the ~1.3 s
// index wait is over.
TEST(integ_e2_write_track_on_empty_drive_abandoned_early) {
  board_boot(); // no disk
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::Xfer f = wd::write_track(wd::format_image(0, 0, 9, 2, false), 400);
    std::printf("    e2: timed_out %d status %02X trk_op %d\n", f.timed_out, f.status, dbg().trk_op);
    wait_ms(3000);
    CHECK_EQ(dbg().trk_op, 0);
    CHECK(!(wd::restore() & wd::BUSY));
    g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
    insert(0, &g_disk[0]);
    wd::select(0, false);
    wait_ms(4000);
    wd::select(0);
    wd::Xfer x = wd::read_sector(0, 3, 512);
    CHECK(!x.timed_out);
    CHECK_EQ(error_bits(x.status), 0);
    wd::select(0, false);
    wait_ms(5000);
    CHECK(pin_level(PIN_DS0));
  });
}
// (e3) RESTORE (Type I) instead of Force Interrupt as the host's give-up.
TEST(integ_e3_write_track_on_empty_drive_then_restore) {
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    std::vector<uint8_t> img = wd::format_image(0, 0, 9, 2, false);
    out(0xD8, 0xF0);
    size_t fed = 0;
    for (uint64_t end = now() + ms(3000); now() < end && fed < img.size();)
      if (wd::status() & wd::DRQ)
        out(0xDB, img[fed++]);
    wait_ms(300);
    uint8_t st = wd::restore();
    wait_ms(3000);
    std::printf("    integ_e3: restore st %02X trk_op %d\n", st, dbg().trk_op);
    CHECK_EQ(dbg().trk_op, 0);
    CHECK(!(wd::status() & wd::BUSY));
  });
}

// ---------------- Recalibrate that misses TRK00 (#31) ----------------
// These drives lose STEP pulses until the spindle is at speed. A recalibrate
// issued right after the spin-up's first index never reaches TRK00 and used to
// leave the head stranded, position unknown: the host's next read then hit the
// wrong cylinder (the #31 capture: head_target=0, phys_track=?). The failure is
// now latched in full and the move retried with the spindle up.
namespace {
FdcRecalDiag recal() {
  FdcRecalDiag r{};
  fdc_recaldiag_snapshot(&r);
  return r;
}
} // namespace

TEST(cold_recalibrate_that_misses_trk00_is_retried_with_the_spindle_up) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  insert(0, &g_disk[0]);
  board_boot();
  // Longer than one whole recalibrate attempt (100 step-outs and 20 step-ins,
  // about 1 s), so the first attempt really fails; over before the retry.
  cable().drive[0].steps_need_motor_for_ticks = ms(1800);
  run([] {
    wd::select(0);
    uint8_t st = wd::restore();
    CHECK(st & wd::TRK0);
    CHECK_EQ(cable().drive[0].cyl, 0);
    CHECK(cable().drive[0].steps_lost > 0); // the first attempt really missed
    FdcRecalDiag r = recal();
    CHECK_EQ(r.fails, 1u);
    CHECK_EQ(r.retry_ok, 1u);
    CHECK_EQ(r.first_valid, 1);
    CHECK_EQ(r.first_site, 1);   // RESTORE
    CHECK_EQ(r.first_drive, 0);
    CHECK(r.first_retry >= 1 && r.first_retry <= 2);
    CHECK(r.first_state & 0x04); // the motor line was on
    wd::side(1);
    wd::Xfer x = wd::read_sector(0, 1, 512);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 0, 0, 1));
  });
}

// The #31 shape: drive 1 used for the first time, CP/M-style, its steps lost
// at first. The head must still end up where the host asked.
TEST(first_seek_on_unit1_with_lost_steps_lands_on_the_target) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  g_disk[1] = make_disk(Geometry::MZ_DD_DATA, 80, "B");
  insert(0, &g_disk[0]);
  insert(1, &g_disk[1]);
  board_boot();
  cable().drive[1].cyl = 9;
  cable().drive[1].steps_need_motor_for_ticks = ms(1800);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::read_sector(0, 1, 512);
    // ~MOTOR_ON is one wire: drive 1's spindle has been turning all along.
    // The quirk only bites from a cold start of the line, so stop it first.
    wd::select(0, false);
    wait_ms(4000);
    CHECK(!cable().drive[1].motor);
    wd::select(1);
    wd::restore();
    wd::seek(2);
    CHECK_EQ(cable().drive[1].cyl, 2);
    wd::Xfer x = wd::read_sector(2, 3, 512);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 2, 0, 3));
    CHECK_EQ(dbg().sec_cyl_mismatch, 0u);
    FdcRecalDiag r = recal();
    CHECK(r.fails >= 1);
    CHECK_EQ(r.retry_fail, 0u);
    CHECK_EQ(r.first_drive, 1);
  });
}

// A drive that never finds TRK00 (no disk turning it, a dead sensor) is
// latched as such, and the controller does not wedge.
TEST(recalibrate_that_never_finds_trk00_is_latched_and_does_not_wedge) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  insert(0, &g_disk[0]);
  board_boot();
  cable().drive[0].steps_need_motor_for_ticks = ms(1000000);
  run([] {
    wd::select(0);
    wd::restore();
    uint8_t st = wd::wait_ready(8000);
    CHECK(!(st & wd::BUSY));
    CHECK(!(st & wd::TRK0));
    FdcRecalDiag r = recal();
    CHECK_EQ(r.fails, 1u);
    CHECK_EQ(r.retry_fail, 1u);
    CHECK_EQ(r.first_retry, 0xFF);
    // Not cleared with firstfail's latch (a failure latched between the two
    // reports would be lost); it has its own clear.
    fdc_firstfail_reset();
    CHECK_EQ(recal().first_valid, 1);
    fdc_recaldiag_reset();
    CHECK_EQ(recal().first_valid, 0);
    CHECK_EQ(recal().fails, 0u);
  }, ms(60000));
}

// A resync inside the READ path that has to recalibrate (target 0)
// and whose recalibrate is retried blocks core0 for seconds. `live` was taken
// before it; a Force Interrupt + new READ meanwhile must not be ended by the
// stale read's publish.
TEST(resync_retry_does_not_end_a_newer_read) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  insert(0, &g_disk[0]);
  board_boot();
  cable().drive[0].cyl = 5;
  cable().drive[0].steps_need_motor_for_ticks = ms(2600);
  run([] {
    wd::select(0);
    wd::side(1);
    // No RESTORE: position unknown, head over cyl 5, host thinks cyl 0.
    wd::Xfer x1 = wd::read_sector(0, 1, 512, false, 1200);
    wd::Xfer x2 = wd::read_sector(0, 2, 512, false, 6000);
    CHECK(x1.timed_out);  // the host gave up (and sent Force Interrupt)
    CHECK(!x2.timed_out); // the newer READ finished on its own
    // ...and either delivered its sector or reported an error -- never
    // "complete, no error" with no data.
    CHECK(!(x2.data.empty() && error_bits(x2.status) == 0));
  });
}

// a failed RESTORE -- what does the host see, and how long is BUSY?
TEST(failed_restore_reports_seek_error_and_recalibrates_once) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  insert(0, &g_disk[0]);
  board_boot();
  cable().drive[0].cyl = 5;
  cable().drive[0].steps_need_motor_for_ticks = ms(1000000);
  run([] {
    wd::select(0);
    wd::side(1);
    for (int v = 0; v < 2; ++v) {
      uint64_t t0 = now();
      out(0xD8, v ? 0x04 : 0x00); // RESTORE, verify flag v
      wait_us(30);
      uint8_t st = wd::wait_ready(20000);
      uint64_t busy_ms = (now() - t0) / ms(1);
      CHECK(st & wd::SEEKERR); // a WD sets Seek Error when TRK00 never shows
      CHECK(!(st & wd::BUSY));
      CHECK(busy_ms < 5000);   // one recalibrate with its retries, not two
    }
  }, ms(120000));
}

// the retry spins up with the host's motor bit off. The deferred stop
// and the idle ~DSn release must still follow.
TEST(recalibrate_retry_with_motor_off_still_stops_and_releases) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  insert(0, &g_disk[0]);
  board_boot();
  cable().drive[0].cyl = 5;
  cable().drive[0].steps_need_motor_for_ticks = ms(1500);
  run([] {
    wd::select(0, false);
    wd::restore();
    wait_ms(6000);
    CHECK(!cable().drive[0].motor);
    CHECK(pin_level(PIN_DS0)); // released (active low)
  }, ms(60000));
}

// ---- second review of #64 ----

// A STEP (no Force Interrupt) written while a READ is blocked in a resync
// recalibrate. Does BUSY ever drop?
TEST(step_during_a_read_resync_does_not_wedge) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  insert(0, &g_disk[0]);
  board_boot();
  cable().drive[0].cyl = 5;
  cable().drive[0].steps_need_motor_for_ticks = ms(2600);
  run([] {
    wd::select(0);
    wd::side(1);
    out(0xD9, 0);
    out(0xDA, 1);
    wd::command(0x80);
    wait_ms(1500); // inside the resync's recalibrate
    uint64_t t0 = now();
    wd::command(0x50); // STEP IN, update
    uint8_t st = wd::wait_ready(15000);
    CHECK(st != 0xFF);
  }, ms(60000));
}

// Same, but the read is blocked in the multi-window search for a sector that
// is not there (no resync): is the wedge pre-existing?
TEST(step_during_a_missing_sector_search_does_not_wedge) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::side(1);
    wd::restore();
    out(0xD9, 0);
    out(0xDA, 30);
    wd::command(0x80);
    wait_ms(400);
    uint64_t t0 = now();
    wd::command(0x50);
    uint8_t st = wd::wait_ready(15000);
    CHECK(st != 0xFF);
  }, ms(60000));
}

// Seek Error from a failed RESTORE must not stick: a later RESTORE that finds
// TRK00 clears it.
TEST(seek_error_of_a_failed_restore_clears_on_the_next_good_one) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  insert(0, &g_disk[0]);
  board_boot();
  cable().drive[0].cyl = 5;
  cable().drive[0].steps_need_motor_for_ticks = ms(6000);
  run([] {
    wd::select(0);
    wd::side(1);
    wd::command(0x00);
    uint8_t st = wd::wait_ready(20000);
    CHECK(st & wd::SEEKERR);
    wait_ms(3000);
    wd::command(0x00);
    st = wd::wait_ready(20000);
    CHECK(st & wd::TRK0);
    CHECK(!(st & wd::SEEKERR));
  }, ms(120000));
}

// Force Interrupt during a RESTORE that is failing: what does STATUS say once
// core0 finishes? A WD aborted by FI does not report Seek Error.
TEST(force_interrupt_during_a_failed_restore_reports_no_seek_error) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  insert(0, &g_disk[0]);
  board_boot();
  cable().drive[0].cyl = 5;
  cable().drive[0].steps_need_motor_for_ticks = ms(1000000);
  run([] {
    wd::select(0);
    wd::side(1);
    wd::command(0x00);
    wait_ms(300);
    wd::force_interrupt();
    uint8_t s1 = wd::status();
    wait_ms(6000);
    uint8_t s2 = wd::status();
    CHECK(!(s2 & wd::SEEKERR));
  }, ms(60000));
}

// CP/M-style burst: RESTORE, then STEP IN (no V) before core0 has run either;
// the recalibrate fails. Does the STEP (V=0) report Seek Error?
TEST(step_folded_after_a_failed_restore_reports_no_seek_error) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  insert(0, &g_disk[0]);
  board_boot();
  cable().drive[0].cyl = 5;
  cable().drive[0].steps_need_motor_for_ticks = ms(1000000);
  run([] {
    wd::select(0);
    wd::side(1);
    out(0xD8, 0x00);
    out(0xD8, 0xD0);
    out(0xD8, 0x50);
    wait_us(30);
    uint8_t st = wd::wait_ready(20000);
    CHECK(!(st & wd::SEEKERR));
  }, ms(60000));
}

// A STEP (no Force Interrupt) landing on a READ ADDRESS in flight: the Type I
// handler clears read_addr but leaves the read running. The READ ADDRESS must
// end -- not come back as a READ SECTOR -- and the STEP must complete.
TEST(step_during_a_read_address_ends_it_and_completes) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    wd::read_sector(5, 1, 512);
    for (int k = 0; k < 3; ++k) {
      out(0xD8, 0xC0); // READ ADDRESS
      wait_us(200 + 300 * k);
      out(0xD8, 0x50); // STEP IN, no Force Interrupt
      uint8_t st = wd::wait_ready(3000);
      CHECK(!(st & wd::BUSY));
      CHECK(!(st & wd::DRQ));
    }
    CHECK_EQ(cable().drive[0].cyl, 8);
    wd::Xfer x = wd::read_sector(8, 2, 512); // and the controller still works
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 8, 0, 2));
  });
}

// A STEP landing while the READ ADDRESS is still inside its ID search (the
// rd_fresh block), not in the decode block: does it come back as a READ
// SECTOR? Watch DRQ and the counters, not just the final BUSY.
TEST(step_during_read_address_search_never_raises_drq) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    wd::read_sector(5, 1, 512);
    for (int k = 0; k < 4; ++k) {
      FdcDebug d0{};
      fdc_debug_snapshot(&d0);
      out(0xD8, 0xC0); // READ ADDRESS
      wait_us(200 + 300 * k);
      out(0xD8, 0x50); // STEP IN, no Force Interrupt
      // After a STEP, STATUS is in the Type I layout, where bit 1 is INDEX:
      // watch the controller's own DRQ flag (rd_flags b2) instead.
      bool saw_drq = false;
      uint8_t st = 0;
      for (int i = 0; i < 200000; ++i) {
        st = wd::status();
        if (dbg().rd_flags & 0x04)
          saw_drq = true;
        if (!(st & wd::BUSY))
          break;
      }
      FdcDebug d1{};
      fdc_debug_snapshot(&d1);
      CHECK(!saw_drq);
      CHECK(!(st & wd::BUSY));
      CHECK_EQ(d1.lost_data_events, d0.lost_data_events);
      CHECK_EQ(d1.sec_ok, d0.sec_ok);
      CHECK_EQ(d1.sec_rnf, d0.sec_rnf);
    }
  });
}

// CP/M's recalibrate idiom: READ ADDRESS, then a STEP that does not update
// the Track register. If the READ ADDRESS turns into a READ SECTOR of the
// last sector number against the (now wrong) Track register, it reports a
// wrong-cylinder RNF -- the #31 signature -- for a command that never read.
TEST(step_no_update_during_read_address_search_counts_no_cyl_mismatch) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    wd::read_sector(5, 1, 512);
    FdcDebug d0{};
    fdc_debug_snapshot(&d0);
    uint64_t t0 = now();
    out(0xD8, 0xC0); // READ ADDRESS
    wait_us(200);
    out(0xD8, 0x40); // STEP IN, no update, no Force Interrupt
    uint8_t st = wd::wait_ready(5000);
    uint64_t dt = now() - t0;
    FdcDebug d1{};
    fdc_debug_snapshot(&d1);
    CHECK(!(st & wd::BUSY));
    CHECK_EQ(d1.sec_rnf, d0.sec_rnf);
    CHECK_EQ(d1.sec_cyl_mismatch, d0.sec_cyl_mismatch);
    CHECK(dt < ms(100));
  });
}

// A failed RESTORE's Seek Error, then a RESTORE that succeeds but is followed
// at once by a Force Interrupt (the host does not wait for BUSY). The publish
// is skipped for the second RESTORE -- does the first one's Seek Error stay?
TEST(seek_error_does_not_survive_a_good_restore_followed_by_force_interrupt) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  insert(0, &g_disk[0]);
  board_boot();
  cable().drive[0].cyl = 5;
  cable().drive[0].steps_need_motor_for_ticks = ms(6000);
  run([] {
    wd::select(0);
    wd::side(1);
    wd::command(0x00);
    uint8_t st = wd::wait_ready(20000);
    CHECK(st & wd::SEEKERR);
    wait_ms(3000);
    out(0xD8, 0x00); // RESTORE
    wait_us(20);
    out(0xD8, 0xD0); // Force Interrupt right behind it
    wait_ms(8000);   // core0 finishes the (now good) recalibrate
    st = wd::status();
    CHECK(!(st & wd::BUSY));
    CHECK(st & wd::TRK0);
    CHECK(!(st & wd::SEEKERR));
  }, ms(120000));
}

// Same, CP/M style: a STEP IN + Force Interrupt burst after the failed
// RESTORE. A STEP without V never reports Seek Error.
TEST(seek_error_does_not_survive_a_step_force_interrupt_burst) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  insert(0, &g_disk[0]);
  board_boot();
  cable().drive[0].cyl = 5;
  cable().drive[0].steps_need_motor_for_ticks = ms(6000);
  run([] {
    wd::select(0);
    wd::side(1);
    wd::command(0x00);
    uint8_t st = wd::wait_ready(20000);
    CHECK(st & wd::SEEKERR);
    wait_ms(3000);
    out(0xD8, 0x50); // STEP IN
    wait_us(20);
    out(0xD8, 0xD0);
    wait_ms(8000);
    st = wd::status();
    CHECK(!(st & wd::BUSY));
    CHECK(!(st & wd::SEEKERR));
  }, ms(120000));
}

// ---- fourth review of #64 ----

// EINT armed, a READ ADDRESS raises DRQ (and ~INT), the host takes two bytes
// and then issues a STEP. core1 ends the READ ADDRESS (reading/drq cleared),
// but the ~INT refresh in the command handler ran BEFORE that clear, so the
// line must not stay asserted with nothing pending.
TEST(step_mid_read_address_transfer_releases_int) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(5);
    wd::side(1);
    out(0xDF, 0x01); // EINT on
    out(0xD8, 0xC0); // READ ADDRESS
    bool drq = false;
    for (uint64_t end = now() + ms(500); now() < end;) {
      if (wd::status() & wd::DRQ) { drq = true; break; }
    }
    CHECK(drq);
    CHECK(pin_oe(12));
    in(0xDB);
    in(0xDB);
    out(0xD8, 0x50); // STEP IN, no Force Interrupt
    wait_us(50);
    bool held = pin_oe(12);
    wd::wait_ready(3000);
    out(0xDF, 0x00);
    CHECK(!held); // ~INT asserted with no DRQ pending
  });
}


// A second RESTORE arriving while the first one's failed recalibrate is still
// retrying: the first one's Seek Error must not be reported as the second's
// (CodeRabbit on #64). Here the second finds TRK00.
TEST(failed_restore_seek_error_is_not_reported_for_a_later_restore) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  insert(0, &g_disk[0]);
  board_boot();
  cable().drive[0].cyl = 5;
  cable().drive[0].steps_need_motor_for_ticks = ms(1000000); // A cannot win
  run([] {
    wd::select(0);
    wd::side(1);
    out(0xD8, 0x00); // RESTORE A
    wait_ms(1500);   // inside A's retries
    out(0xD8, 0x00); // RESTORE B
    // Once A has given up for good, let the steps land: B then succeeds.
    for (uint64_t end = now() + ms(10000); now() < end && recal().retry_fail == 0;)
      wait_us(200);
    CHECK_EQ(recal().retry_fail, 1u);
    cable().drive[0].steps_need_motor_for_ticks = 0;
    // While B runs, A's failure must not show as B's Seek Error, nor be
    // latched as the first failure.
    bool seek_err_seen = false;
    uint8_t st = 0;
    for (uint64_t end = now() + ms(8000); now() < end;) {
      st = wd::status();
      if (st & wd::SEEKERR)
        seek_err_seen = true;
      if (!(st & wd::BUSY))
        break;
      wait_us(200);
    }
    FdcFirstFail ff{};
    fdc_firstfail_snapshot(&ff);
    CHECK(!seek_err_seen);
    CHECK_EQ(ff.count, 0u);
    CHECK(!(st & wd::BUSY));
    CHECK(st & wd::TRK0);
    CHECK(!(st & wd::SEEKERR));
    CHECK_EQ(cable().drive[0].cyl, 0);
  }, ms(60000));
}

// ---------------- core0 publish vs a host command (#61) ----------------
// core0 checks that its work still belongs to the live command, then
// publishes. A command landing between the two used to be clobbered. Each
// test issues host commands AT a race point between a check and its publish
// (sim::set_race_point); with the publish lock they wait until the publish is
// done, as core1 waits on the lock. Run with sim_publock_model(false) each of
// these fails.
namespace {
// Poll like the ROM: take every DRQ byte until BUSY drops.
wd::Xfer drain(uint32_t timeout_ms = 3000) {
  wd::Xfer x;
  for (uint64_t end = now() + ms(timeout_ms); now() < end;) {
    uint8_t st = wd::status();
    if (st & wd::DRQ)
      x.data.push_back(in(0xDB));
    else if (!(st & wd::BUSY)) {
      x.status = st;
      return x;
    }
  }
  x.timed_out = true;
  x.status = wd::status();
  return x;
}
struct RaceOnce {
  int id;
  std::function<void()> fn;
  bool fired = false;
};
void arm_race(RaceOnce &r) {
  set_race_point([&r](int id) {
    if (id == r.id && !r.fired) {
      r.fired = true;
      r.fn();
    }
  });
}
} // namespace

// A STEP completes; a READ the host issues right as the STEP drops BUSY must
// keep its own BUSY and deliver its sector.
TEST(read_issued_as_a_seek_drops_busy_is_not_ended_by_it) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(4);
    wd::side(1);
    wd::read_sector(4, 1, 512);
    RaceOnce r{1, [] { race_out(0xDA, 3); race_out(0xD8, 0x80); }};
    arm_race(r);
    out(0xD8, 0x50); // STEP IN, update -> Track 5
    wd::Xfer x = drain();
    set_race_point(nullptr);
    CHECK(r.fired);
    CHECK_EQ(x.data.size(), 512u);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 5, 0, 3));
  });
}

// Force Interrupt right as core0 raises DRQ for a sector: STATUS afterwards is
// 0x00, no DRQ (the ROM requires a clean STATUS after 0xD0).
TEST(force_interrupt_as_drq_is_raised_leaves_no_drq) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::read_sector(0, 1, 512);
    RaceOnce r{2, [] { race_out(0xD8, 0xD0); }};
    arm_race(r);
    out(0xDA, 2);
    out(0xD8, 0x80);
    wait_ms(400);
    set_race_point(nullptr);
    CHECK(r.fired);
    uint8_t st = wd::status();
    CHECK(!(st & wd::DRQ));
    CHECK(!(st & wd::BUSY));
    // and the controller is fine
    CHECK(matches_pattern(wd::read_sector(0, 3, 512).data, 0, 0, 3));
  });
}

// A READ ends with RNF; a new READ the host issues (after Force Interrupt)
// right at that moment must survive.
TEST(read_issued_as_an_rnf_completes_survives) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::read_sector(0, 1, 512);
    RaceOnce r{3, [] { race_out(0xD8, 0xD0); race_out(0xDA, 4); race_out(0xD8, 0x80); }};
    arm_race(r);
    out(0xDA, 14); // not on the track
    out(0xD8, 0x80);
    wd::Xfer x;
    for (uint64_t end = now() + ms(3000); now() < end && !r.fired;)
      wait_ms(5);
    x = drain();
    set_race_point(nullptr);
    CHECK(r.fired);
    CHECK_EQ(x.data.size(), 512u);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 0, 0, 4));
  });
}

// A WRITE completes; a READ issued right then must survive.
TEST(read_issued_as_a_write_completes_survives) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(6);
    wd::side(1);
    wd::read_sector(6, 1, 512);
    RaceOnce r{4, [] { race_out(0xD8, 0xD0); race_out(0xDA, 5); race_out(0xD8, 0x80); }};
    arm_race(r);
    std::vector<uint8_t> d(512, 0x3C);
    out(0xDA, 2);
    out(0xD8, 0xA0);
    size_t fed = 0;
    for (uint64_t end = now() + ms(2000); now() < end && fed < 512;)
      if (wd::status() & wd::DRQ) {
        out(0xDB, d[fed]);
        ++fed;
      }
    for (uint64_t end = now() + ms(3000); now() < end && !r.fired;)
      wait_ms(5);
    wd::Xfer x = drain();
    set_race_point(nullptr);
    CHECK(r.fired);
    CHECK_EQ(x.data.size(), 512u);
    CHECK(matches_pattern(x.data, 6, 0, 5));
  });
}

// A READ nobody drains times out (LOST DATA); a new READ the host issues right
// then must survive.
TEST(read_issued_as_lost_data_fires_survives) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::read_sector(0, 1, 512);
    RaceOnce r{13, [] { race_out(0xD8, 0xD0); race_out(0xDA, 6); race_out(0xD8, 0x80); }};
    arm_race(r);
    out(0xDA, 2);
    out(0xD8, 0x80); // never drained
    for (uint64_t end = now() + ms(3000); now() < end && !r.fired;)
      wait_ms(1);
    wd::Xfer x = drain();
    set_race_point(nullptr);
    CHECK(r.fired);
    CHECK_EQ(x.data.size(), 512u);
    CHECK(matches_pattern(x.data, 0, 0, 6));
  });
}

// READ TRACK decoded; a Force Interrupt right as it publishes: no DRQ after.
TEST(force_interrupt_as_read_track_publishes_leaves_no_drq) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::side(1);
    wd::read_sector(0, 1, 512);
    RaceOnce r{8, [] { race_out(0xD8, 0xD0); }};
    arm_race(r);
    out(0xD8, 0xE0); // READ TRACK
    for (uint64_t end = now() + ms(2000); now() < end && !r.fired;)
      wait_ms(5);
    wait_ms(20);
    set_race_point(nullptr);
    CHECK(r.fired);
    uint8_t st = wd::status();
    CHECK(!(st & wd::DRQ));
    CHECK(!(st & wd::BUSY));
  });
}

// ---- review of #65 ----
namespace {
bool feed(const std::vector<uint8_t> &d, size_t from, uint32_t timeout_ms = 3000) {
  size_t fed = from;
  for (uint64_t end = now() + ms(timeout_ms); now() < end && fed < d.size();)
    if (wd::status() & wd::DRQ)
      out(0xDB, d[fed++]);
  return fed == d.size();
}
} // namespace

// core0 judged a READ live, then fills xfer[]/xfer_len/xfer_pos BEFORE the
// locked re-check. A WRITE SECTOR the host issues in that window (after a
// Force Interrupt) has its first byte overwritten and xfer_pos reset.
TEST(write_issued_as_a_read_fills_the_buffer_survives) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(6);
    wd::side(1);
    wd::read_sector(6, 1, 512);
    std::vector<uint8_t> d(512);
    for (size_t i = 0; i < d.size(); ++i)
      d[i] = (uint8_t)(i * 7 + 1);
    RaceOnce r{20, [&d] {
      race_out(0xD8, 0xD0);
      race_out(0xDA, 5);
      race_out(0xD8, 0xA0);  // WRITE SECTOR 5
      race_out(0xDB, d[0]);  // first byte: a DATA write is not blocked
    }};
    arm_race(r);
    out(0xDA, 2);
    out(0xD8, 0x80); // READ SECTOR 2
    for (uint64_t end = now() + ms(3000); now() < end && !r.fired;)
      wait_ms(1);
    CHECK(r.fired);
    CHECK(feed(d, 1));
    uint8_t st = wd::wait_ready();
    set_race_point(nullptr);
    CHECK_EQ(error_bits(st), 0);
    CHECK(wd::read_sector(6, 5, 512).data == d);
  });
}

// The WRITE TRACK abandon branch (cmd_gen moved during the head wait) checks
// trk_op/trk_armed/trk_seq and clears them outside the lock: a new WRITE
// TRACK landing between is zeroed, BUSY stays up with no DRQ.
TEST(write_track_issued_as_an_abandoned_one_is_dropped_survives) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA);
  g_disk[0].t[20][0] = Track{};
  insert(0, &g_disk[0]);
  board_boot();
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(20);
    wd::side(1);
    std::vector<uint8_t> img = wd::format_image(20, 0, 9, 2, false);
    int stage = 0;
    set_race_point([&stage](int id) {
      if (id == 21 && stage == 0) {
        stage = 1;
        race_out(0xD8, 0xD0); // supersede the armed format
      } else if (id == 22 && stage == 1) {
        stage = 2;
        race_out(0xD8, 0xF0); // the host's retry: a new WRITE TRACK
      }
    });
    out(0xD8, 0xF0);
    CHECK(feed(img, 0));
    for (uint64_t end = now() + ms(3000); now() < end && stage < 2;)
      wait_ms(1);
    set_race_point(nullptr);
    CHECK_EQ(stage, 2);
    CHECK(feed(img, 0, 1000)); // the new format collects its image
    uint8_t st = wd::wait_ready(3000);
    CHECK(!(st & wd::BUSY));
    CHECK_EQ(error_bits(st), 0);
  });
}

// The write path sets err_rnf from a `live` computed outside the lock: a
// command landing between (Force Interrupt + WRITE TRACK) inherits the dead
// write's RNF, and a successful format reports RNF.
TEST(format_issued_as_a_write_fails_reports_no_rnf) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(6);
    wd::side(1);
    wd::read_sector(6, 1, 512);
    RaceOnce r{23, [] { race_out(0xD8, 0xD0); race_out(0xD8, 0xF0); }};
    arm_race(r);
    std::vector<uint8_t> d(512, 0x3C);
    out(0xDA, 14); // not on the track: the splice fails RNF
    out(0xD8, 0xA0);
    CHECK(feed(d, 0));
    for (uint64_t end = now() + ms(3000); now() < end && !r.fired;)
      wait_ms(1);
    set_race_point(nullptr);
    CHECK(r.fired);
    CHECK(feed(wd::format_image(6, 0, 9, 2, false), 0));
    uint8_t st = wd::wait_ready(3000);
    CHECK(!(st & wd::BUSY));
    CHECK_EQ(error_bits(st), 0);
  });
}

// ---- second review of #65 ----
// A Type I does not end a READ SECTOR (core1 leaves `reading` set). One that
// lands after core0 judged the read live but before the locked publish makes
// the publish fail -- and the found path then marks the read done anyway
// (S.rd_done = seq), unlike the RNF path's finish_stale(). The read is still
// pending, nothing will ever serve or end it: BUSY forever.
TEST(type1_between_live_and_read_publish_does_not_wedge_the_read) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(6);
    wd::side(1);
    wd::read_sector(6, 1, 512);
    RaceOnce r{20, [] {
      race_out(0xDB, 6);    // SEEK to where the head already is
      race_out(0xD8, 0x10);
    }};
    arm_race(r);
    out(0xDA, 2);
    out(0xD8, 0x80); // READ SECTOR 2
    for (uint64_t end = now() + ms(3000); now() < end && !r.fired;)
      wait_ms(1);
    set_race_point(nullptr);
    CHECK(r.fired);
    // The read must finish one way or the other: DRQ with data, or an error.
    bool done = false;
    for (uint64_t end = now() + ms(3000); now() < end && !done;) {
      uint8_t st = wd::status();
      if (st & wd::DRQ) {
        for (int i = 0; i < 512 && (wd::status() & wd::DRQ); ++i)
          in(0xDB);
      }
      if (!(st & wd::BUSY))
        done = true;
    }
    CHECK(done);
  });
}

// ---- final review: the three PRs combined (#64 recal retry x #65 lock x #66) ----
namespace {
void xrv_bus_reset() {
  sim::pin_set_input(29, true);  sim_fire_gpio_irq();
  wait_us(65);
  sim::pin_set_input(29, false); sim_fire_gpio_irq();
}
// Head over cyl 5, position unknown, steps lost for `lost_ms` from motor-on:
// a READ of cyl 0 hits a wrong-cylinder ID, and its resync recalibrate fails
// once and is retried (#64) -- core0 blocked for seconds.
void xrv_boot_stranded(uint64_t lost_ms) {
  g_disk[0] = make_disk(Geometry::MZ_DD_DATA, 80, "A");
  insert(0, &g_disk[0]);
  board_boot();
  cable().drive[0].cyl = 5;
  cable().drive[0].steps_need_motor_for_ticks = ms(lost_ms);
}
} // namespace

// Force Interrupt + a new READ while the resync's recalibrate is retrying:
// the new READ is served, CRC-clean, from cylinder 0 once the retry lands.
TEST(combined_fi_and_read_during_resync_retry_delivers_the_new_sector) {
  xrv_boot_stranded(2600);
  run([] {
    wd::select(0);
    wd::side(1);
    out(0xD9, 0);
    out(0xDA, 1);
    wd::command(0x80);
    wait_ms(1500); // inside the resync's recalibrate retry
    CHECK(wd::status() & wd::BUSY);
    wd::force_interrupt();
    wd::Xfer x = wd::read_sector(0, 2, 512, false, 10000);
    CHECK(!x.timed_out);
    CHECK_EQ(x.data.size(), 512u);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 0, 0, 2));
    CHECK_EQ(cable().drive[0].cyl, 0);
    CHECK_EQ(recal().retry_ok, 1u);
  }, ms(60000));
}

// A bus RESET while the resync recalibrate retries: STATUS is clean at once,
// nothing from the dead READ is published afterwards, and the boot ROM's
// RESTORE + READ after the reset work.
TEST(combined_bus_reset_during_resync_retry_then_rom_restore_and_read) {
  xrv_boot_stranded(2600);
  run([] {
    wd::select(0);
    wd::side(1);
    out(0xD9, 0);
    out(0xDA, 1);
    wd::command(0x80);
    wait_ms(1500);
    xrv_bus_reset();
    uint8_t s0 = wd::status();
    CHECK(!(s0 & (wd::BUSY | wd::DRQ)));
    // Watch the dead READ's completion: nothing may come up.
    bool drq = false;
    for (uint64_t end = now() + ms(4000); now() < end;) {
      if (dbg().rd_flags & 0x04)
        drq = true;
      wait_us(500);
    }
    CHECK(!drq);
    CHECK(!(wd::status() & wd::BUSY));
    // the ROM after reset
    wd::select(0);
    uint8_t st = wd::restore();
    CHECK(st & wd::TRK0);
    CHECK(!(st & wd::SEEKERR));
    wd::side(1);
    wd::Xfer x = wd::read_sector(0, 3, 512, false, 5000);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 0, 0, 3));
  }, ms(60000));
}

// CP/M's recalibrate idiom (FI, READ ADDRESS, STEP) while the previous READ's
// resync recalibrate is still retrying. Everything must end; no phantom DRQ;
// the head lands where the folded target says (0 + 1).
TEST(combined_read_address_and_step_during_resync_retry) {
  xrv_boot_stranded(2600);
  run([] {
    wd::select(0);
    wd::side(1);
    out(0xD9, 0);
    out(0xDA, 1);
    wd::command(0x80);
    wait_ms(1500);
    out(0xD8, 0xD0);
    out(0xD8, 0xC0); // READ ADDRESS
    wait_us(100);
    out(0xD8, 0x50); // STEP IN, update -> Track 1
    bool drq = false;
    uint8_t st = 0xFF;
    for (uint64_t end = now() + ms(10000); now() < end;) {
      st = wd::status();
      if (dbg().rd_flags & 0x04)
        drq = true;
      if (!(st & wd::BUSY))
        break;
      wait_us(200);
    }
    CHECK(!(st & wd::BUSY));
    CHECK(!drq);
    CHECK(!(st & wd::SEEKERR));
    CHECK_EQ(cable().drive[0].cyl, 1);
    wd::Xfer x = wd::read_sector(1, 2, 512);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 1, 0, 2));
  }, ms(60000));
}

// A STEP (no FI) between `live` and the locked publish (#65): by design a Type
// I does not end a READ SECTOR, so the READ is left pending (finish_stale) and
// recaptured after the move. It must end (BUSY drops), never wedge, and the
// controller must serve the next READ from the new cylinder. (Its own result
// is published under the STEP's Type I STATUS layout -- see the report.)
TEST(combined_step_at_read_publish_race_does_not_wedge) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(6);
    wd::side(1);
    wd::read_sector(6, 1, 512);
    RaceOnce r{20, [] { race_out(0xD8, 0x50); }}; // STEP IN, update -> 7
    arm_race(r);
    out(0xDF, 0x01); // EINT on, as CP/M runs
    out(0xDA, 2);
    out(0xD8, 0x80);
    bool int_seen = false, drq_seen = false;
    uint8_t st = 0xFF;
    for (uint64_t end = now() + ms(5000); now() < end;) {
      st = wd::status();
      if (r.fired && pin_oe(12)) int_seen = true;
      if (r.fired && (dbg().rd_flags & 0x04)) drq_seen = true;
      if (r.fired && !(st & wd::BUSY)) break;
      wait_us(200);
    }
    out(0xDF, 0x00);
    set_race_point(nullptr);
    std::printf("    xrv20: st=%02X int_seen=%d drq_seen(after STEP)=%d lost=%u\n", st, int_seen, drq_seen, dbg().lost_data_events);
    CHECK(r.fired);
    CHECK(!(st & wd::BUSY));
    CHECK_EQ(cable().drive[0].cyl, 7);
    wd::Xfer x = wd::read_sector(7, 3, 512);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 7, 0, 3));
  });
}

// Same race on the RNF publish (point 3): the READ of a missing sector is
// retried after the STEP and ends; the controller is usable afterwards.
TEST(combined_step_at_rnf_publish_race_does_not_wedge) {
  boot_with(Geometry::MZ_DD_DATA);
  run([] {
    wd::select(0);
    wd::restore();
    wd::seek(6);
    wd::side(1);
    wd::read_sector(6, 1, 512);
    RaceOnce r{3, [] { race_out(0xD8, 0x50); }};
    arm_race(r);
    out(0xDA, 14);
    out(0xD8, 0x80);
    uint8_t st = wd::wait_ready(10000);
    set_race_point(nullptr);
    CHECK(r.fired);
    CHECK(st != 0xFF);
    CHECK_EQ(cable().drive[0].cyl, 7);
    wd::Xfer x = wd::read_sector(7, 3, 512);
    CHECK_EQ(error_bits(x.status), 0);
    CHECK(matches_pattern(x.data, 7, 0, 3));
  }, ms(60000));
}

// A failed RESTORE, and a READ the host issues during its retries (CP/M does
// not wait for BUSY). The READ owns BUSY; it must end, and the RESTORE's Seek
// Error must not be published over it.
TEST(combined_read_during_failed_restore_retries_ends) {
  xrv_boot_stranded(1000000);
  run([] {
    wd::select(0);
    wd::side(1);
    wd::command(0x00); // RESTORE
    wait_ms(1500);
    out(0xDA, 1);
    out(0xD8, 0x80);
    uint8_t st = wd::wait_ready(30000);
    CHECK(st != 0xFF);
    CHECK(error_bits(st) != 0); // the head is nowhere known: an error, not data
  }, ms(120000));
}

// RESET during a failed RESTORE's retries, then the boot ROM's own RESTORE
// once the drive can step: TRK00, no Seek Error from the dead one.
TEST(combined_reset_during_failed_restore_then_rom_restore_is_clean) {
  xrv_boot_stranded(1000000);
  run([] {
    wd::select(0);
    wd::side(1);
    wd::command(0x00);
    wait_ms(1500);
    xrv_bus_reset();
    CHECK(!(wd::status() & (wd::BUSY | wd::SEEKERR)));
    wd::select(0);
    out(0xD8, 0x0B); // the ROM's RESTORE (h, r=11, no V)
    for (uint64_t end = now() + ms(10000); now() < end && recal().retry_fail == 0;)
      wait_us(200);
    cable().drive[0].steps_need_motor_for_ticks = 0;
    bool seek_err_seen = false;
    uint8_t st = 0xFF;
    for (uint64_t end = now() + ms(15000); now() < end;) {
      st = wd::status();
      if (st & wd::SEEKERR)
        seek_err_seen = true;
      if (!(st & wd::BUSY))
        break;
      wait_us(200);
    }
    CHECK(!seek_err_seen);
    CHECK(!(st & wd::BUSY));
    CHECK(st & wd::TRK0);
    CHECK_EQ(cable().drive[0].cyl, 0);
  }, ms(120000));
}
