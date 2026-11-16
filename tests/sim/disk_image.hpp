// Test media: build a simulated Disk from a sector-level description.
//
// The MFM encoder here is written from the format definition (IBM System/34
// double density), independently of the firmware's encoder in mfm_impl.h, so
// a test that formats a disk with it and reads it through the firmware is a
// check of the firmware's decoder rather than of the encoder against itself.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "sim_drive.hpp"

namespace sim {

struct SectorSpec {
  uint8_t c, h, r, n;           // ID field
  std::vector<uint8_t> data;    // 128 << n bytes (shorter is zero-padded)
  bool deleted = false;         // F8 data mark instead of FB
  bool bad_id_crc = false;
  bool bad_data_crc = false;
  bool no_data = false;         // ID field with no data field after it
  bool no_id = false;           // data field with no ID field before it
};

struct TrackSpec {
  std::vector<SectorSpec> sectors;
  bool hd = false;   // 500 kbit/s cells
  int gap3 = -1;     // -1: chosen from the geometry
  bool unformatted = false;
};

// Encode one track into transition angles (one revolution, index at 0).
Track encode_track(const TrackSpec &t, uint32_t jitter_ticks = 0,
                   uint32_t seed = 1);

// Geometries.
//   MZ-800 CP/M DD data disk: 80 cyl x 2 heads x 9 x 512 (N=2), DD.
//   MZ-800 master disk: as above, but cyl 0 / physical head 1 carries the
//     16 x 256 boot track (see the side-select inversion notes).
//   HD data disk: 80 x 2 x 18 x 512 at 500 kbit/s.
// Sector payloads are deterministic functions of (c, h, r) so a test can
// predict every byte: pattern_byte().
enum class Geometry { MZ_DD_DATA, MZ_DD_MASTER, HD_DATA };
uint8_t pattern_byte(int c, int h, int r, int i);
Disk make_disk(Geometry g, int cylinders = 80, const std::string &name = "");
// Fill a whole disk with one data byte (e.g. 0xFF: an empty CP/M directory as
// stored through the inverted bus -- the densest possible flux).
Disk make_filled_disk(Geometry g, uint8_t fill, int cylinders = 80);

// EDSK ("EXTENDED CPC DSK File") image loader. Sector status bytes are
// honoured: ST2 0x40 = deleted mark, ST1/ST2 0x20 = data CRC error.
bool load_edsk(const std::string &path, Disk *out, std::string *err);

// Decode helper for assertions: read sector R off a track as a perfect
// reader would (via the firmware decoder on two revolutions of flux).
struct DecodedSector {
  bool found = false, crc_ok = false, deleted = false;
  uint8_t c = 0, h = 0, n = 0;
  std::vector<uint8_t> data;
};
DecodedSector decode_sector(const Track &t, int r, bool hd);
int count_valid_sectors(const Track &t, bool hd);

} // namespace sim
