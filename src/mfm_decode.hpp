// Generic MFM track scanner — reports every sector physically present on a
// track with its real ID field, size, mark type, and CRC status (like the WD's
// READ ADDRESS + READ SECTOR combined). Unlike Adafruit's decode_track_mfm it
// does NOT assume fixed 512-byte sectors or sequential IDs, so it handles
// mixed/odd formats (e.g. the MZ-800 CP/M disk: 9x512 tracks + a 16x256 boot
// track). Built on the low-level primitives in flux/mfm_impl.h.
#pragma once
#include <cstddef>
#include <cstdint>

struct SectorInfo {
  uint8_t cyl;      // IDAM cylinder byte
  uint8_t head;     // IDAM head byte
  uint8_t sec;      // IDAM sector-id byte
  uint8_t n;        // IDAM size code (size = 128 << n)
  uint16_t size;    // decoded sector size in bytes
  uint8_t dam;      // data address mark (0xFB normal, 0xF8 deleted); 0 = none
  bool id_crc_ok;   // ID field CRC valid
  bool has_data;    // a data field was found after the ID
  bool data_crc_ok; // data field CRC valid
  bool deleted;     // data mark was a deleted-DAM (0xF8)
};

// Scan `flux` (raw 1-byte deltas from capture_track with store_greaseweazle=
// false) for one track. nominal_bit_time_us = 2.0 for DD (250 kbit/s), 1.0 for
// HD. Fills `out` (up to max_sectors) with sectors in physical order; sector
// data bytes are validated for CRC via `scratch` (must be >= largest sector).
// Returns the number of sectors found.
int mfm_scan_track(const uint8_t *flux, size_t n_flux, float nominal_bit_time_us,
                   SectorInfo *out, int max_sectors, uint8_t *scratch,
                   size_t scratch_size);
