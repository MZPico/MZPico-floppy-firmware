// MFM track encoder — sectors -> flux, the write-side counterpart of
// mfm_decode. Wraps the encoder in flux/mfm_impl.h with correct timing/settings.
// Produces a uniform-format track (all sectors 128<<size_code bytes, IDs 1..N);
// mixed/interleaved layouts and the in-place single-sector write-splice come
// with the Phase-3 WD bridge. See docs/wd-fidelity.md.
#pragma once
#include <cstddef>
#include <cstdint>

// Encode `n_sectors` sectors (each 128<<size_code bytes, taken from `sectors`)
// into MFM flux written to `flux` (capacity max_flux, raw 1-byte deltas matching
// capture_track/store_greaseweazle=false). nominal_bit_time_us = 2.0 DD / 1.0 HD.
// Returns the flux length up to the end of the last sector (excludes trailing
// gap padding). max_flux must exceed one track's worth of flux.
size_t mfm_encode_track(uint8_t *flux, size_t max_flux, const uint8_t *sectors,
                        int n_sectors, int size_code, float nominal_bit_time_us,
                        uint8_t cyl, uint8_t head);

// Encode ONE data field -- 12x 0x00 presync, the 3-byte 0xA1 sync, the data
// address mark, `n_data` data bytes, the CRC, and `trailing_gap` bytes of 0x4E
// -- into flux. This is exactly what a WD179x emits between turning WGATE on
// and off again during WRITE SECTOR, and it is the payload of the in-place
// splice: everything before it (the ID field) and after it (the rest of Gap 3)
// stays as recorded, so the track's format is preserved.
//
// The trailing gap bytes exist to carry the write splice itself. The head
// cannot switch from writing to not-writing instantly, so the last thing under
// WGATE has to be expendable -- ending the flux on the CRC would put the
// turn-off transient on top of it.
//
// Returns the number of flux bytes produced (0 if it would not fit).
size_t mfm_encode_data_field(uint8_t *flux, size_t max_flux,
                             const uint8_t *data, size_t n_data, bool deleted,
                             float nominal_bit_time_us, int trailing_gap);
