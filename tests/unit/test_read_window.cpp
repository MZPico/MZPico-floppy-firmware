// The read window, offline: one revolution of real-format flux, a capture
// opened at every rotational phase, the firmware decoder on what it caught.
//
// This is the simulation that found the fixed-transition-count bug (16e5310):
// a count is data-dependent -- ~38k transitions per revolution of ordinary DD
// data, ~47k on an empty directory (0x00 stored as 0xFF) -- so the old 45700
// window was less than one revolution exactly on directory tracks. The rule
// the firmware now implements is TIME: one revolution plus one sector span.
#include <algorithm>
#include <vector>

#include "disk_image.hpp"
#include "mfm_decode.hpp"
#include "testfw.hpp"

using namespace sim;

namespace {

struct Result { int reads = 0, ok = 0, crc = 0, rnf = 0; };

// Capture `window_ticks` of flux starting at angle phase, decode sector r.
Result sweep(const Track &t, int nsec, bool hd, uint64_t window_ticks,
             size_t count_limit = 0) {
  Result res;
  std::vector<uint8_t> buf;
  std::vector<uint8_t> out(1024);
  for (uint32_t ph = 0; ph < REV_ANGLE; ph += REV_ANGLE / 97) {
    buf.clear();
    // Transitions after `ph`, wrapping, until the window (or count) closes.
    auto it = std::upper_bound(t.pos.begin(), t.pos.end(), ph);
    uint64_t last = ph, elapsed = 0;
    size_t i = (size_t)(it - t.pos.begin());
    uint64_t base = 0;
    while (true) {
      if (i == t.pos.size()) { i = 0; base += REV_ANGLE; }
      uint64_t a = base + t.pos[i];
      elapsed = a - ph;
      if (count_limit ? buf.size() >= count_limit : elapsed > window_ticks) break;
      uint64_t d = a - last;
      buf.push_back((uint8_t)(d > 255 ? 255 : d));
      last = a;
      ++i;
    }
    for (int r = 1; r <= nsec; ++r) {
      SectorRead sr = mfm_read_sector(buf.data(), buf.size(), hd ? 1.0f : 2.0f,
                                      (uint8_t)r, MFM_ANY_CYL, out.data(), out.size());
      ++res.reads;
      if (!sr.found) ++res.rnf;
      else if (!sr.has_data || !sr.data_crc_ok) ++res.crc;
      else ++res.ok;
    }
  }
  return res;
}

TrackSpec filled(int ns, int n, bool hd, int fill) {
  TrackSpec t;
  t.hd = hd;
  for (int r = 1; r <= ns; ++r) {
    SectorSpec s;
    s.c = 0; s.h = 0; s.r = (uint8_t)r; s.n = (uint8_t)n;
    s.data.resize((size_t)128 << n);
    for (size_t i = 0; i < s.data.size(); ++i)
      s.data[i] = fill >= 0 ? (uint8_t)fill : pattern_byte(0, 0, r, (int)i);
    t.sectors.push_back(s);
  }
  return t;
}

// The firmware's rule: revolution + one sector span ((512+128) bytes x 16 cells).
uint64_t firmware_window(bool hd) {
  double cell_us = hd ? 1.0 : 2.0;
  return us(200000) + (uint64_t)(640 * 16 * cell_us * TICKS_PER_US);
}

} // namespace

TEST(timed_window_reads_every_sector_at_every_phase_dense_dd) {
  Track t = encode_track(filled(9, 2, false, 0xFF));
  Result r = sweep(t, 9, false, firmware_window(false));
  CHECK_EQ(r.ok, r.reads);
}

TEST(timed_window_reads_every_sector_at_every_phase_random_dd) {
  Track t = encode_track(filled(9, 2, false, -1));
  Result r = sweep(t, 9, false, firmware_window(false));
  CHECK_EQ(r.ok, r.reads);
}

TEST(timed_window_reads_every_sector_at_every_phase_boot_16x256) {
  Track t = encode_track(filled(16, 1, false, 0xFF));
  Result r = sweep(t, 16, false, firmware_window(false));
  CHECK_EQ(r.ok, r.reads);
}

TEST(timed_window_reads_every_sector_at_every_phase_hd) {
  Track t = encode_track(filled(18, 2, true, 0xFF));
  Result r = sweep(t, 18, true, firmware_window(true));
  CHECK_EQ(r.ok, r.reads);
}

// Documents WHY the window is a sector span longer than a revolution: a bare
// revolution misses the sector straddling the window start.
TEST(one_revolution_alone_is_not_enough) {
  Track t = encode_track(filled(9, 2, false, -1));
  Result r = sweep(t, 9, false, us(200000));
  CHECK(r.ok < r.reads);
}

// Documents the bug the timed window fixed: the old 45700-transition window
// fails on a dense (empty-directory) track.
TEST(legacy_transition_count_window_fails_on_dense_track) {
  Track t = encode_track(filled(9, 2, false, 0xFF));
  Result r = sweep(t, 9, false, 0, 45700);
  CHECK(r.ok < r.reads);
}
