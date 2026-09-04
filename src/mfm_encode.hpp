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

// Encode a WD WRITE TRACK image -- the byte stream a host shifts in to format a
// track -- into flux. Most bytes are written literally; four are commands to
// the WD's write channel, and this is where they are obeyed (MFM values):
//
//   0xF5  write 0xA1 with a missing clock bit, and preset the CRC
//   0xF6  write 0xC2 with a missing clock bit (the index address mark's sync)
//   0xF7  write the two accumulated CRC bytes
//   else  write the byte, accumulating it into the CRC
//
// The missing clock bits are the whole point: they make the sync marks
// unambiguous in a stream where any data pattern is otherwise legal, and they
// cannot be produced by writing an ordinary 0xA1 or 0xC2. Only a format can
// create them, which is why formatting is a distinct operation and not just a
// sequence of sector writes.
//
// On the CRC preset: a real IBM track has three 0xF5s in a row and the CRC that
// follows must cover all three 0xA1s. So the preset happens once, at the START
// of a run of 0xF5s, not on each one -- resetting per byte would leave the CRC
// covering only the last 0xA1 and every ID field would be unreadable.
//
// F8..FB and FE are NOT treated as CRC-initialising here. That rule belongs to
// FM/single density, where there is no 0xA1 sync to do the job; in MFM the
// address marks are ordinary bytes that simply accumulate. MZ-800 media is MFM
// throughout.
//
// Returns flux bytes produced. If the image needs more flux than `max_flux`,
// the buffer is left FULL (the first max_flux transitions of the image),
// max_flux is returned, and *out_overflow (if given) is set. That is NOT an
// error for a track write: the write is terminated by the index pulse, so a
// buffer holding at least one revolution behaves exactly like a real WD being
// fed a longer image than the track can take -- the index cuts it off. The
// caller that must not truncate (a bench format validating exact geometry)
// checks the flag.
size_t mfm_encode_format_image(uint8_t *flux, size_t max_flux,
                               const uint8_t *image, size_t n_image,
                               float nominal_bit_time_us,
                               bool *out_overflow = nullptr);

// Fill `flux` from `n_flux` onward with encoded 0x4E gap bytes, up to
// `max_flux`. A track image shorter than one revolution would otherwise leave
// the tail of the OLD track intact past the end of the new one -- a track that
// is half one format and half another, with the seam wherever the host happened
// to stop. Padding lets the write run past the index pulse that terminates it.
// Returns the new total flux length.
size_t mfm_encode_gap_fill(uint8_t *flux, size_t n_flux, size_t max_flux,
                           float nominal_bit_time_us);
