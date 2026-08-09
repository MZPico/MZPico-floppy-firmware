// WRITE SECTOR — the in-place MFM write splice (Milestone 3.3).
//
// This is the one operation in the bridge that cannot be done by capturing a
// window and thinking about it afterwards. A read may take a whole revolution
// of flux, decode it at leisure and answer from the buffer; a write has to put
// new flux on the disk in the ~700 us Gap 2 window that follows one specific ID
// field, which means the ID has to be recognised WHILE it passes the head.
//
// So the search here is a live MFM decoder: it pulls one flux transition at a
// time straight out of the capture FIFO, tracks the MFM sync/parity state, and
// decodes each ID field as it arrives. Per transition the work is a subtract, a
// compare and a shift -- roughly 0.2 us against a ~4 us DD transition interval
// -- so unlike the batch decoder it can keep up with the disk indefinitely.
// (See fdc_capture_window() for why the *buffered* decoder must never be
// interleaved with capture: it is O(n^2) and rescans from the start.)
//
// What gets written is only the data field: 12 bytes of zeros, the 3-byte 0xA1
// sync, the data address mark, the data, its CRC and a couple of gap bytes to
// absorb the write-off splice. The ID field, Gap 2 and the rest of Gap 3 are
// left exactly as recorded, so the track keeps whatever format it was written
// with -- sector size, interleave, numbering and all. That is also what makes
// the operation safe on an unknown disk: nothing outside one sector's data
// field is ever overwritten.
//
// KNOWN ASSUMPTION: Gap 2 is the IBM System-34 standard 22 bytes. That is the
// one number here that is taken on faith rather than read off the disk, because
// it has to be known BEFORE the data field it precedes can be observed. It
// holds for MZ-800 CP/M media and for anything else a WD formatted, and the
// splice is biased late so that a longer real gap is harmless; a materially
// SHORTER one would put the write on top of the ID field's CRC. Measuring it
// per track (during a read, where there is time) is the fix if that ever turns
// up.
#pragma once
#include <cstddef>
#include <cstdint>

#include "mfm_decode.hpp" // MFM_ANY_CYL

struct SpliceResult {
  bool wrote;        // the data field was written to the disk
  bool id_found;     // an ID for the requested sector passed the head
  bool cyl_mismatch; // that ID carried a different cylinder byte (see below)
  bool size_mismatch;// ...or a different sector size than the host's data
  bool late;         // the ID was decoded too late to hit its Gap 2 window
  bool encode_fail;  // the data field did not fit the flux buffer
  bool no_flux;      // no transitions at all (no disk / motor not up)
  uint8_t seen_cyl, seen_head; // ID bytes of the mismatching candidate
  uint8_t size_n;              // size code from the matched ID
  uint16_t ids_seen;           // ID fields decoded during the search
  uint32_t search_us;          // time from capture start to the splice/timeout
};

// Locate sector `want_sec` on the current track and overwrite its data field
// with `data` (`n_data` bytes).
//
// `want_cyl` is the WD ID-field track compare, and it matters far more here
// than it does on a read: a read from the wrong cylinder returns garbage the
// host can retry, a WRITE to the wrong cylinder destroys a track that was fine.
// Pass MFM_ANY_CYL only for bench use on a disk you are willing to lose.
//
// The sector's recorded size must match `n_data` exactly; a size mismatch
// aborts without writing (writing 512 bytes into a 256-byte sector would run
// straight through the following ID field).
//
// The caller is responsible for the head being on the right cylinder/side and
// for the disk not being write-protected. Runs with interrupts disabled for the
// duration of the search.
SpliceResult mfm_write_sector(uint8_t want_sec, int want_cyl,
                              const uint8_t *data, uint16_t n_data,
                              bool deleted, float nominal_bit_time_us,
                              uint32_t timeout_us);
