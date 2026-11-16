#include "disk_image.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

#include "mfm_decode.hpp"

namespace sim {
namespace {

uint16_t crc16_ccitt(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (int i = 0; i < 8; ++i)
    crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  return crc;
}

// MFM cell writer: data bit d is preceded by a clock bit that is 1 only
// between two zero data bits. A '1' cell is a flux transition.
class CellWriter {
public:
  CellWriter(uint32_t cell_ticks, uint32_t jitter, uint32_t seed)
      : cell_(cell_ticks), jitter_(jitter), rng_(seed ? seed : 1) {}

  void cell(bool one) {
    t_ += cell_;
    if (one) {
      int64_t j = 0;
      if (jitter_) {
        rng_ = rng_ * 1103515245u + 12345u;
        j = (int64_t)((rng_ >> 8) % (2 * jitter_ + 1)) - (int64_t)jitter_;
      }
      int64_t a = (int64_t)t_ + j;
      if (a >= 0 && a < (int64_t)REV_ANGLE)
        pos_.push_back((uint32_t)a);
    }
  }
  void byte(uint8_t b) {
    for (int i = 7; i >= 0; --i) {
      bool d = (b >> i) & 1;
      cell(!prev_ && !d);
      cell(d);
      prev_ = d;
    }
  }
  // 16 raw cells, msb first (sync marks with a missing clock bit).
  void raw16(uint16_t w) {
    for (int i = 15; i >= 0; --i)
      cell((w >> i) & 1);
    prev_ = w & 1;
  }
  uint64_t t() const { return t_; }
  std::vector<uint32_t> &pos() { return pos_; }

private:
  uint64_t t_ = 0;
  uint32_t cell_, jitter_, rng_;
  bool prev_ = false;
  std::vector<uint32_t> pos_;
};

} // namespace

Track encode_track(const TrackSpec &t, uint32_t jitter, uint32_t seed) {
  Track out;
  if (t.unformatted)
    return out;
  // DD (250 kbit/s): 4 us per data bit = 2 us per MFM cell = 48 ticks.
  const uint32_t cell = t.hd ? 24 : 48;
  CellWriter w(cell, jitter, seed);
  auto gap = [&](int n) {
    for (int i = 0; i < n; ++i)
      w.byte(0x4E);
  };
  auto zeros = [&](int n) {
    for (int i = 0; i < n; ++i)
      w.byte(0x00);
  };
  int ns = (int)t.sectors.size();
  int gap3 = t.gap3;
  if (gap3 < 0) {
    int sz = ns ? (128 << (t.sectors[0].n & 3)) : 512;
    if (t.hd)
      gap3 = 84;
    else
      gap3 = (sz <= 256 && ns > 10) ? 32 : 84;
  }
  gap(80);  // gap 4a
  zeros(12);
  for (int i = 0; i < 3; ++i)
    w.raw16(0x5224); // C2 with missing clock
  w.byte(0xFC);      // index address mark
  gap(50);           // gap 1
  for (const SectorSpec &s : t.sectors) {
    uint16_t crc;
    if (!s.no_id) {
      zeros(12);
      crc = 0xFFFF;
      for (int i = 0; i < 3; ++i) {
        w.raw16(0x4489); // A1 with missing clock
        crc = crc16_ccitt(crc, 0xA1);
      }
      const uint8_t id[5] = {0xFE, s.c, s.h, s.r, s.n};
      for (uint8_t b : id) {
        w.byte(b);
        crc = crc16_ccitt(crc, b);
      }
      if (s.bad_id_crc)
        crc ^= 0x5A5A;
      w.byte(crc >> 8);
      w.byte(crc & 0xFF);
    } else {
      gap(22); // where the ID field would have been
    }
    gap(22); // gap 2
    if (!s.no_data) {
      zeros(12);
      crc = 0xFFFF;
      for (int i = 0; i < 3; ++i) {
        w.raw16(0x4489);
        crc = crc16_ccitt(crc, 0xA1);
      }
      uint8_t mark = s.deleted ? 0xF8 : 0xFB;
      w.byte(mark);
      crc = crc16_ccitt(crc, mark);
      size_t len = (size_t)128 << (s.n & 3);
      for (size_t i = 0; i < len; ++i) {
        uint8_t b = i < s.data.size() ? s.data[i] : 0;
        w.byte(b);
        crc = crc16_ccitt(crc, b);
      }
      if (s.bad_data_crc)
        crc ^= 0xA5A5;
      w.byte(crc >> 8);
      w.byte(crc & 0xFF);
    } else {
      zeros(12);
    }
    gap(gap3);
  }
  while (w.t() + 16 * cell < REV_ANGLE)
    gap(1); // gap 4b to the index
  out.pos = std::move(w.pos());
  return out;
}

uint8_t pattern_byte(int c, int h, int r, int i) {
  return (uint8_t)(c * 7 + h * 101 + r * 13 + i * 31 + (i >> 8) * 3);
}

static TrackSpec track_for(Geometry g, int c, int h, int fill) {
  TrackSpec t;
  int ns = 9, n = 2;
  if (g == Geometry::HD_DATA) {
    t.hd = true;
    ns = 18;
  }
  if (g == Geometry::MZ_DD_MASTER && c == 0 && h == 1) {
    ns = 16; // the 16 x 256 IPL track
    n = 1;
  }
  for (int r = 1; r <= ns; ++r) {
    SectorSpec s;
    s.c = (uint8_t)c;
    s.h = (uint8_t)h;
    s.r = (uint8_t)r;
    s.n = (uint8_t)n;
    s.data.resize((size_t)128 << n);
    for (size_t i = 0; i < s.data.size(); ++i)
      s.data[i] = fill >= 0 ? (uint8_t)fill : pattern_byte(c, h, r, (int)i);
    t.sectors.push_back(s);
  }
  return t;
}

Disk make_disk(Geometry g, int cylinders, const std::string &name) {
  Disk d;
  d.name = name;
  d.hd_media = g == Geometry::HD_DATA;
  for (int c = 0; c < cylinders && c < MAX_CYL; ++c)
    for (int h = 0; h < 2; ++h)
      d.t[c][h] = encode_track(track_for(g, c, h, -1), 0, (uint32_t)(c * 2 + h + 1));
  return d;
}

Disk make_filled_disk(Geometry g, uint8_t fill, int cylinders) {
  Disk d;
  d.hd_media = g == Geometry::HD_DATA;
  for (int c = 0; c < cylinders && c < MAX_CYL; ++c)
    for (int h = 0; h < 2; ++h)
      d.t[c][h] = encode_track(track_for(g, c, h, fill));
  return d;
}

bool load_edsk(const std::string &path, Disk *out, std::string *err) {
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    *err = "cannot open " + path;
    return false;
  }
  std::vector<uint8_t> b((std::istreambuf_iterator<char>(f)), {});
  if (b.size() < 256 || memcmp(b.data(), "EXTENDED CPC DSK", 16) != 0) {
    *err = "not an EXTENDED CPC DSK image";
    return false;
  }
  int ntracks = b[0x30], nsides = b[0x31];
  size_t off = 256;
  for (int ti = 0; ti < ntracks * nsides; ++ti) {
    size_t tsize = (size_t)b[0x34 + ti] * 256;
    if (!tsize)
      continue;
    if (off + tsize > b.size() || memcmp(&b[off], "Track-Info", 10) != 0) {
      *err = "bad Track-Info block";
      return false;
    }
    const uint8_t *ti_blk = &b[off];
    int cyl = ti_blk[0x10], side = ti_blk[0x11];
    int nsec = ti_blk[0x15];
    TrackSpec t;
    t.gap3 = ti_blk[0x16];
    size_t data_off = off + 256;
    for (int s = 0; s < nsec; ++s) {
      const uint8_t *si = ti_blk + 0x18 + s * 8;
      SectorSpec sp;
      sp.c = si[0];
      sp.h = si[1];
      sp.r = si[2];
      sp.n = si[3];
      uint8_t st1 = si[4], st2 = si[5];
      size_t len = (size_t)si[6] | ((size_t)si[7] << 8);
      sp.deleted = (st2 & 0x40) != 0;
      sp.bad_data_crc = (st1 & 0x20) || (st2 & 0x20);
      size_t want = (size_t)128 << (sp.n & 3);
      sp.data.assign(b.begin() + (long)data_off,
                     b.begin() + (long)(data_off + std::min(len, want)));
      t.sectors.push_back(sp);
      data_off += len;
    }
    if (!t.sectors.empty() && (t.sectors[0].n & 3) <= 2 && nsec > 12)
      t.hd = nsec > 16 && t.sectors[0].n == 2;
    if (cyl < MAX_CYL && side < 2)
      out->t[cyl][side] = encode_track(t);
    off += tsize;
  }
  out->name = path;
  return true;
}

// Two revolutions of flux as 8-bit deltas, starting at the index.
static std::vector<uint8_t> flux_two_revs(const Track &t) {
  std::vector<uint8_t> d;
  if (t.pos.empty())
    return d;
  uint64_t last = 0;
  for (int rev = 0; rev < 2; ++rev)
    for (uint32_t p : t.pos) {
      uint64_t a = (uint64_t)rev * REV_ANGLE + p;
      uint64_t delta = a - last;
      d.push_back((uint8_t)(delta > 255 ? 255 : delta));
      last = a;
    }
  return d;
}

DecodedSector decode_sector(const Track &t, int r, bool hd) {
  DecodedSector o;
  std::vector<uint8_t> f = flux_two_revs(t);
  if (f.empty())
    return o;
  std::vector<uint8_t> buf(1024);
  SectorRead sr = mfm_read_sector(f.data(), f.size(), hd ? 1.0f : 2.0f,
                                  (uint8_t)r, MFM_ANY_CYL, buf.data(), buf.size());
  o.found = sr.found;
  o.crc_ok = sr.found && sr.has_data && sr.data_crc_ok;
  o.deleted = sr.deleted;
  o.c = sr.cyl;
  o.h = sr.head;
  o.n = sr.size_n;
  if (sr.has_data)
    o.data.assign(buf.begin(), buf.begin() + (128 << (sr.size_n & 3)));
  return o;
}

int count_valid_sectors(const Track &t, bool hd) {
  std::vector<uint8_t> f = flux_two_revs(t);
  if (f.empty())
    return 0;
  // One revolution's worth is enough to see every sector once.
  f.resize(f.size() / 2);
  SectorInfo ids[40];
  static uint8_t scratch[1024];
  int n = mfm_scan_track(f.data(), f.size(), hd ? 1.0f : 2.0f, ids, 40, scratch,
                         sizeof(scratch));
  int ok = 0;
  for (int i = 0; i < n; ++i)
    ok += ids[i].id_crc_ok && ids[i].has_data && ids[i].data_crc_ok;
  return ok;
}

} // namespace sim
