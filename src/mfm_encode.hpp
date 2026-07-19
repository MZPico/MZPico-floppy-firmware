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
