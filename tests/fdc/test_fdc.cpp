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
