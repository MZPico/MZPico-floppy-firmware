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

// Result of locating one sector by ID (the WD READ SECTOR operation).
struct SectorRead {
  bool found;        // an IDAM with sec == want_sec and a valid ID CRC was found
  bool has_data;     // a data field (DAM/DDAM) followed the ID
  bool data_crc_ok;  // data-field CRC valid
  bool deleted;      // the data mark was a deleted-DAM (0xF8)
  uint8_t cyl, head; // IDAM cylinder/head bytes of the matched sector (debug)
  uint16_t size;     // data-field size actually decoded (128 << N)
};

// Locate the sector whose IDAM sector-id == `want_sec` (with a valid ID CRC) in
// one track's `flux` and decode its data field into `out` (up to out_size). This
// is the per-command READ SECTOR primitive: it re-scans the same captured flux
// for each requested sector, so a multi-sector read needs only one capture.
// Returns .found=false (→ RNF) if no matching good-ID sector exists.
SectorRead mfm_read_sector(const uint8_t *flux, size_t n_flux,
                           float nominal_bit_time_us, uint8_t want_sec,
                           uint8_t *out, size_t out_size);

// WD Type I SEEK VERIFY primitive: after a seek, a real WD1793 (with the
// command's Verify bit set) reads back the *first* ID field it finds
// (regardless of sector number) and checks its cylinder byte against the
// track just sought to, setting Seek Error if it doesn't match (or none is
// found). Returns true iff an IDAM with a valid CRC and cyl == want_cyl was
// found anywhere in `flux`.
bool mfm_verify_track(const uint8_t *flux, size_t n_flux,
                      float nominal_bit_time_us, uint8_t want_cyl);

// WD Type III READ ADDRESS primitive: return the *first* ID field found in
// `flux` (whatever sector it belongs to), as the six bytes a real WD179x hands
// the host over DRQ -- track, side, sector, size code, CRC1, CRC2. The CRC
// bytes are the ones actually recorded on the disk, not a recomputation, so a
// host that checks them sees what the chip would have given it. `out` must have
// room for 6 bytes. Returns false if no ID field with a valid CRC was found
// (-> Record Not Found).
bool mfm_read_address(const uint8_t *flux, size_t n_flux,
                      float nominal_bit_time_us, uint8_t *out);
