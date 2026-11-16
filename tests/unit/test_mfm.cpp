// MFM codec: the firmware's encoder and decoder against each other and against
// the independent encoder in tests/sim/disk_image.cpp.
#include <cstring>
#include <vector>

#include "disk_image.hpp"
#include "mfm_decode.hpp"
#include "mfm_encode.hpp"
#include "testfw.hpp"

using namespace sim;

namespace {

// One revolution of a Track as 8-bit deltas starting at the index, repeated.
std::vector<uint8_t> deltas(const Track &t, int revs = 2) {
  std::vector<uint8_t> d;
  uint64_t last = 0;
  for (int r = 0; r < revs; ++r)
    for (uint32_t p : t.pos) {
      uint64_t a = (uint64_t)r * REV_ANGLE + p;
      uint64_t x = a - last;
      d.push_back((uint8_t)(x > 255 ? 255 : x));
      last = a;
    }
  return d;
}

TrackSpec uniform(int c, int h, int ns, int n, bool hd, uint8_t fill = 0) {
  TrackSpec t;
  t.hd = hd;
  for (int r = 1; r <= ns; ++r) {
    SectorSpec s;
    s.c = (uint8_t)c; s.h = (uint8_t)h; s.r = (uint8_t)r; s.n = (uint8_t)n;
    s.data.resize((size_t)128 << n);
    for (size_t i = 0; i < s.data.size(); ++i)
      s.data[i] = fill ? fill : pattern_byte(c, h, r, (int)i);
    t.sectors.push_back(s);
  }
  return t;
}

void check_all_sectors(const TrackSpec &spec, uint32_t jitter = 0) {
  Track t = encode_track(spec, jitter);
  std::vector<uint8_t> f = deltas(t);
  float bit = spec.hd ? 1.0f : 2.0f;
  for (const SectorSpec &s : spec.sectors) {
    std::vector<uint8_t> out(1024);
    SectorRead r = mfm_read_sector(f.data(), f.size(), bit, s.r, s.c, out.data(), out.size());
    CHECK(r.found);
    CHECK(r.has_data && r.data_crc_ok);
    CHECK(memcmp(out.data(), s.data.data(), s.data.size()) == 0);
  }
}

} // namespace

TEST(decode_independent_dd_9x512) { check_all_sectors(uniform(5, 1, 9, 2, false)); }
TEST(decode_independent_dd_16x256) { check_all_sectors(uniform(0, 1, 16, 1, false)); }
TEST(decode_independent_hd_18x512) { check_all_sectors(uniform(70, 0, 18, 2, true)); }
TEST(decode_independent_dd_dense_ff) { check_all_sectors(uniform(2, 0, 9, 2, false, 0xFF)); }
TEST(decode_with_flux_jitter_dd) { check_all_sectors(uniform(9, 0, 9, 2, false), 5); }
TEST(decode_with_flux_jitter_hd) { check_all_sectors(uniform(9, 0, 18, 2, true), 3); }

TEST(firmware_encoder_roundtrip_dd_and_hd) {
  for (bool hd : {false, true}) {
    int ns = hd ? 18 : 9;
    std::vector<uint8_t> data((size_t)ns * 512);
    for (size_t i = 0; i < data.size(); ++i)
      data[i] = (uint8_t)(i * 7 + 1);
    std::vector<uint8_t> flux(200000);
    size_t n = mfm_encode_track(flux.data(), flux.size(), data.data(), ns, 2,
                                hd ? 1.0f : 2.0f, 12, 1);
    REQUIRE(n > 0);
    for (int r = 1; r <= ns; ++r) {
      std::vector<uint8_t> out(512);
      SectorRead sr = mfm_read_sector(flux.data(), n, hd ? 1.0f : 2.0f, (uint8_t)r, 12, out.data(), 512);
      CHECK(sr.found && sr.data_crc_ok);
      CHECK(memcmp(out.data(), &data[(size_t)(r - 1) * 512], 512) == 0);
    }
  }
}

TEST(format_image_encoder_writes_missing_clock_syncs) {
  // A WD format image: 0xF5 -> A1 sync, 0xF7 -> CRC. Decodes as a real track.
  std::vector<uint8_t> img;
  auto rep = [&](int n, uint8_t b) { img.insert(img.end(), (size_t)n, b); };
  rep(80, 0x4E); rep(12, 0); rep(3, 0xF6); img.push_back(0xFC); rep(50, 0x4E);
  for (int r = 1; r <= 9; ++r) {
    rep(12, 0); rep(3, 0xF5); img.push_back(0xFE);
    img.push_back(3); img.push_back(0); img.push_back((uint8_t)r); img.push_back(2);
    img.push_back(0xF7); rep(22, 0x4E); rep(12, 0); rep(3, 0xF5); img.push_back(0xFB);
    rep(512, (uint8_t)(0x30 + r)); img.push_back(0xF7); rep(84, 0x4E);
  }
  std::vector<uint8_t> flux(100000);
  size_t n = mfm_encode_format_image(flux.data(), flux.size(), img.data(), img.size(), 2.0f);
  REQUIRE(n > 0);
  SectorInfo ids[16];
  uint8_t scratch[1024];
  int ns = mfm_scan_track(flux.data(), n, 2.0f, ids, 16, scratch, sizeof scratch);
  CHECK_EQ(ns, 9);
  for (int i = 0; i < ns; ++i)
    CHECK(ids[i].id_crc_ok && ids[i].has_data && ids[i].data_crc_ok);
}

TEST(wrong_cylinder_id_is_reported_not_returned) {
  Track t = encode_track(uniform(7, 0, 9, 2, false));
  std::vector<uint8_t> f = deltas(t);
  std::vector<uint8_t> out(512);
  SectorRead r = mfm_read_sector(f.data(), f.size(), 2.0f, 3, 8, out.data(), 512);
  CHECK(!r.found || !r.has_data);
  CHECK(r.cyl_mismatch);
  CHECK_EQ(r.seen_cyl, 7);
}

TEST(id_without_data_field_is_not_paired_with_a_later_orphan_data_field) {
  // ID(1) with no data field, then a data field with no ID (0x22s): the
  // situation a missed sync leaves in a capture. A WD gives up on a data mark
  // that does not follow within ~43 bytes of the ID; the orphan must never be
  // delivered as sector 1.
  TrackSpec spec = uniform(1, 0, 4, 2, false);
  spec.sectors[0].no_data = true;
  spec.sectors[1].no_id = true;
  spec.sectors[1].data.assign(512, 0x22);
  Track t = encode_track(spec);
  std::vector<uint8_t> f = deltas(t, 1);
  std::vector<uint8_t> out(512);
  SectorRead r = mfm_read_sector(f.data(), f.size(), 2.0f, 1, 1, out.data(), 512);
  CHECK(!(r.found && r.has_data && r.data_crc_ok));
}

TEST(scan_does_not_swallow_the_sector_after_an_id_without_data) {
  TrackSpec spec = uniform(1, 0, 3, 2, false);
  spec.sectors[0].no_data = true;
  Track t = encode_track(spec);
  std::vector<uint8_t> f = deltas(t, 1);
  SectorInfo ids[8];
  uint8_t scratch[1024];
  int n = mfm_scan_track(f.data(), f.size(), 2.0f, ids, 8, scratch, sizeof scratch);
  bool saw2 = false;
  for (int i = 0; i < n; ++i)
    saw2 |= ids[i].sec == 2 && ids[i].data_crc_ok;
  CHECK(saw2);
}

TEST(read_address_finds_first_id) {
  Track t = encode_track(uniform(44, 1, 9, 2, false));
  std::vector<uint8_t> f = deltas(t, 1);
  uint8_t id[6];
  REQUIRE(mfm_read_address(f.data(), f.size(), 2.0f, id));
  CHECK_EQ(id[0], 44);
  CHECK_EQ(id[1], 1);
  CHECK_EQ(id[2], 1);
  CHECK_EQ(id[3], 2);
}

TEST(verify_track_checks_cylinder) {
  Track t = encode_track(uniform(30, 0, 9, 2, false));
  std::vector<uint8_t> f = deltas(t, 1);
  CHECK(mfm_verify_track(f.data(), f.size(), 2.0f, 30));
  CHECK(!mfm_verify_track(f.data(), f.size(), 2.0f, 31));
}

// #4 via a dropout: the capture clamps long intervals to 255, so a flux gap
// between an ID and an orphan data field must not shrink below the DAM
// window in the sum. A saturated interval inside the window rejects the pair.
TEST(flux_dropout_between_id_and_orphan_data_does_not_pair_them) {
  TrackSpec spec = uniform(1, 0, 4, 2, false);
  spec.sectors[0].no_data = true;
  spec.sectors[1].no_id = true;
  spec.sectors[1].data.assign(512, 0x22);
  Track t = encode_track(spec);
  std::vector<uint8_t> f = deltas(t, 1);
  // Find sector 1's ID (it ends ~ after the first few hundred transitions of
  // the index area); replace a long stretch of the gap after it with one
  // saturated interval, as a 1.2 ms dropout is captured: time removed, 255 left.
  std::vector<uint8_t> out(512);
  SectorRead before = mfm_read_sector(f.data(), f.size(), 2.0f, 1, 1, out.data(), 512);
  CHECK(!(before.found && before.has_data)); // no pairing without the dropout
  // Collapse the whole stretch between ID 1 and the orphan's sync region into
  // a couple of saturated bytes: the clamped sum now "fits" the window.
  size_t id_end = 0;
  {
    // locate the first 0xFE ID mark by decoding progressively larger prefixes
    for (size_t n = 200; n < f.size(); n += 50) {
      uint8_t id[6];
      if (mfm_read_address(f.data(), n, 2.0f, id)) { id_end = n; break; }
    }
  }
  REQUIRE(id_end > 0);
  // The orphan's data sync is the first A1 sync after ID 1 (which has no data
  // field): MFM 0x4489 shows up as 4T,3T,4T,3T intervals (192,144,... ticks).
  auto near = [](uint8_t v, int want) { return v > want - 20 && v < want + 20; };
  size_t sync = 0;
  for (size_t i = id_end + 20; i + 4 < f.size(); ++i)
    if (near(f[i], 192) && near(f[i + 1], 144) && near(f[i + 2], 192) &&
        near(f[i + 3], 144)) {
      sync = i;
      break;
    }
  REQUIRE(sync > id_end);
  // Keep ~25 bytes (25 x 16 cells x 48 ticks) before the sync, and replace
  // everything between the ID and there with ONE saturated interval: a long
  // dropout, as the capture records it.
  size_t cut_from = id_end + 20, cut_to = sync;
  uint32_t kept = 0;
  while (cut_to > cut_from && kept < 25u * 16u * 48u)
    kept += f[--cut_to];
  REQUIRE(cut_to > cut_from + 50); // a real gap was removed
  std::vector<uint8_t> g(f.begin(), f.begin() + (long)cut_from);
  g.push_back(255);
  g.insert(g.end(), f.begin() + (long)cut_to, f.end());
  SectorRead r = mfm_read_sector(g.data(), g.size(), 2.0f, 1, 1, out.data(), 512);
  CHECK(!(r.found && r.has_data && r.data_crc_ok));
}
