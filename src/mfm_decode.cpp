#include "mfm_decode.hpp"

#include <cmath>
#include <cstring>

// Silence mfm_impl's debug/assert hooks before including it.
#define DEBUG_PRINTF(...) ((void)0)
#define DEBUG_ASSERT(x) ((void)0)
#include "flux/mfm_impl.h"

// RP2040 flux sample frequency (see arch_rp2 / getSampleFrequency).
static constexpr uint32_t GW_SF = 24000000UL;

static const uint8_t MFM_IO_DDAM = 0xF8; // deleted data address mark

// The data field belongs to the ID before it only if its sync mark follows
// within the WD's search window: a WD179x gives up on the data address mark 43
// bytes after the ID field's CRC (MFM) and reports RNF. A standard track puts
// it 37 bytes on (Gap 2 + presync + the sync itself) and our write splice ~2
// bytes later, so 46 bytes (with slack for bit-cell drift) separates a real
// data field from whatever comes next. Without the limit an ID whose own data
// sync was lost was paired with the NEXT data field on the track -- right
// cylinder and sector, CRC-valid, another sector's content (issue #4).
//
// Returns true with `io` just past the data sync. On false, io->pos is put
// back where it was, so the caller's next sync search finds what really comes
// next -- typically the following sector's ID, which a scan must not swallow.
static constexpr uint32_t MFM_DAM_WINDOW_BYTES = 46;
static bool find_data_sync(mfm_io_t *io) {
  size_t start = io->pos;
  bool ok = skip_triple_sync_mark(io);
  if (ok) {
    uint32_t ticks = 0;
    uint32_t limit = MFM_DAM_WINDOW_BYTES * 16u * io->T1_nom;
    // A byte of 255 is a SATURATED interval: the capture clamps longer gaps
    // (a dropout, a damaged stretch) to 255, so its true length is unknown and
    // the sum would under-count. No legal MFM interval comes near it (4T is
    // 192 ticks at DD), so treat one inside the window as "too far".
    for (size_t i = start; i < io->pos && ticks <= limit; ++i) {
      if (io->pulses[i] == 255) {
        ticks = limit + 1;
        break;
      }
      ticks += io->pulses[i];
    }
    ok = ticks <= limit;
  }
  if (!ok)
    io->pos = start;
  return ok;
}

int mfm_scan_track(const uint8_t *flux, size_t n_flux, float nominal_bit_time_us,
                   SectorInfo *out, int max_sectors, uint8_t *scratch,
                   size_t scratch_size) {
  mfm_io_t io;
  memset(&io, 0, sizeof(io));
  io.pulses = const_cast<uint8_t *>(flux);
  io.n_pulses = n_flux;
  io.pos = 0;
  // Same thresholds Adafruit's set_timings() uses (2/3/4 T flux classes).
  io.T1_nom = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 1e-6f);
  io.T2_max = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 2.5e-6f);
  io.T3_max = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 3.5e-6f);

  int n = 0;
  while (!mfm_io_eof(&io) && n < max_sectors) {
    if (!skip_triple_sync_mark(&io)) {
      break; // no more sync marks -> end of track data
    }

    uint8_t mark = 0;
    uint8_t idam[mfm_io_idam_size];
    uint8_t crcbuf[mfm_io_crc_size];
    uint16_t crc = receive_crc(&io, &mark, (size_t)1, idam, (size_t)sizeof(idam),
                               crcbuf, (size_t)sizeof(crcbuf), (uint8_t *)NULL);
    if (mark != MFM_IO_IDAM) {
      continue; // that sync wasn't an ID address mark
    }

    SectorInfo *s = &out[n];
    s->cyl = idam[0];
    s->head = idam[1];
    s->sec = idam[2];
    s->n = idam[3] & 7;
    s->size = (uint16_t)(128u << s->n);
    s->id_crc_ok = (crc == 0);
    s->dam = 0;
    s->has_data = false;
    s->data_crc_ok = false;
    s->deleted = false;

    // The data field follows after Gap 2 + its own sync mark.
    size_t after_id = io.pos;
    if (find_data_sync(&io)) {
      size_t dsize = s->size;
      uint8_t dmark = 0;
      if (dsize <= scratch_size) {
        uint16_t dcrc =
            receive_crc(&io, &dmark, (size_t)1, scratch, dsize, crcbuf,
                        (size_t)sizeof(crcbuf), (uint8_t *)NULL);
        if (dmark == MFM_IO_DAM || dmark == MFM_IO_DDAM) {
          s->dam = dmark;
          s->has_data = true;
          s->deleted = (dmark == MFM_IO_DDAM);
          s->data_crc_ok = (dcrc == 0);
        }
      }
      if (!s->has_data)
        io.pos = after_id; // that sync was the next ID: leave it to the loop
    }
    n++;
  }
  return n;
}

SectorRead mfm_read_sector(const uint8_t *flux, size_t n_flux,
                           float nominal_bit_time_us, uint8_t want_sec,
                           int want_cyl, uint8_t *out, size_t out_size) {
  SectorRead r;
  memset(&r, 0, sizeof(r));

  mfm_io_t io;
  memset(&io, 0, sizeof(io));
  io.pulses = const_cast<uint8_t *>(flux);
  io.n_pulses = n_flux;
  io.pos = 0;
  io.T1_nom = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 1e-6f);
  io.T2_max = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 2.5e-6f);
  io.T3_max = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 3.5e-6f);

  while (!mfm_io_eof(&io)) {
    if (!skip_triple_sync_mark(&io))
      break; // no more sync marks -> end of track

    uint8_t mark = 0;
    uint8_t idam[mfm_io_idam_size];
    uint8_t crcbuf[mfm_io_crc_size];
    uint16_t crc = receive_crc(&io, &mark, (size_t)1, idam, (size_t)sizeof(idam),
                               crcbuf, (size_t)sizeof(crcbuf), (uint8_t *)NULL);
    if (mark != MFM_IO_IDAM)
      continue; // that sync wasn't an ID address mark

    if (crc != 0)
      continue; // corrupt ID field -> keep looking
    if (idam[2] != want_sec)
      continue; // not our sector
    if (want_cyl != MFM_ANY_CYL && idam[0] != (uint8_t)want_cyl) {
      // Right sector number, WRONG cylinder: the head is not where the host
      // believes. A real WD reports RNF here rather than handing over another
      // track's data, which is what lets the host notice and re-seek.
      r.cyl_mismatch = true;
      r.seen_cyl = idam[0];
      r.seen_head = idam[1];
      continue;
    }

    // Matched a good ID for the requested sector. Decode its data field.
    // Clear any mismatch noted from an EARLIER candidate: a wrong-cylinder copy
    // of this sector followed by a correct one is a successful read, and leaving
    // the flag set would report a positioning failure alongside found=true.
    r.cyl_mismatch = false;
    r.seen_cyl = r.seen_head = 0;
    r.found = true;
    r.cyl = idam[0];
    r.head = idam[1];
    uint8_t nsz = idam[3] & 7;
    uint16_t size = (uint16_t)(128u << nsz);
    r.size = size;
    r.size_n = nsz;

    if (find_data_sync(&io) && size <= out_size) {
      uint8_t dmark = 0;
      uint16_t dcrc = receive_crc(&io, &dmark, (size_t)1, out, size, crcbuf,
                                  (size_t)sizeof(crcbuf), (uint8_t *)NULL);
      if (dmark == MFM_IO_DAM || dmark == MFM_IO_DDAM) {
        r.has_data = true;
        r.deleted = (dmark == MFM_IO_DDAM);
        r.data_crc_ok = (dcrc == 0);
      }
    }
    return r;
  }
  return r; // .found == false -> Record Not Found
}

bool mfm_read_address(const uint8_t *flux, size_t n_flux,
                      float nominal_bit_time_us, uint8_t *out) {
  mfm_io_t io;
  memset(&io, 0, sizeof(io));
  io.pulses = const_cast<uint8_t *>(flux);
  io.n_pulses = n_flux;
  io.pos = 0;
  io.T1_nom = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 1e-6f);
  io.T2_max = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 2.5e-6f);
  io.T3_max = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 3.5e-6f);

  while (!mfm_io_eof(&io)) {
    if (!skip_triple_sync_mark(&io))
      break;

    uint8_t mark = 0;
    uint8_t idam[mfm_io_idam_size];
    uint8_t crcbuf[mfm_io_crc_size];
    uint16_t crc = receive_crc(&io, &mark, (size_t)1, idam, (size_t)sizeof(idam),
                               crcbuf, (size_t)sizeof(crcbuf), (uint8_t *)NULL);
    if (mark != MFM_IO_IDAM || crc != 0)
      continue; // not an ID field, or its CRC is bad -> keep looking

    out[0] = idam[0]; // track
    out[1] = idam[1]; // side
    out[2] = idam[2]; // sector
    out[3] = idam[3]; // size code N
    out[4] = crcbuf[0];
    out[5] = crcbuf[1];
    return true;
  }
  return false;
}

bool mfm_verify_track(const uint8_t *flux, size_t n_flux,
                      float nominal_bit_time_us, uint8_t want_cyl) {
  mfm_io_t io;
  memset(&io, 0, sizeof(io));
  io.pulses = const_cast<uint8_t *>(flux);
  io.n_pulses = n_flux;
  io.pos = 0;
  io.T1_nom = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 1e-6f);
  io.T2_max = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 2.5e-6f);
  io.T3_max = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 3.5e-6f);

  while (!mfm_io_eof(&io)) {
    if (!skip_triple_sync_mark(&io))
      break;

    uint8_t mark = 0;
    uint8_t idam[mfm_io_idam_size];
    uint8_t crcbuf[mfm_io_crc_size];
    uint16_t crc = receive_crc(&io, &mark, (size_t)1, idam, (size_t)sizeof(idam),
                               crcbuf, (size_t)sizeof(crcbuf), (uint8_t *)NULL);
    if (mark != MFM_IO_IDAM || crc != 0)
      continue; // not an ID field, or its CRC is bad -> keep looking

    if (idam[0] == want_cyl)
      return true; // any sector's ID on the right cylinder verifies the seek
  }
  return false;
}

size_t mfm_read_track_raw(const uint8_t *flux, size_t n_flux,
                          float nominal_bit_time_us, uint8_t *out,
                          size_t out_size) {
  mfm_io_t io;
  memset(&io, 0, sizeof(io));
  io.pulses = const_cast<uint8_t *>(flux);
  io.n_pulses = n_flux;
  io.pos = 0;
  io.T1_nom = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 1e-6f);
  io.T2_max = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 2.5e-6f);
  io.T3_max = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 3.5e-6f);

  size_t n = 0;
  // Bit assembly, same rules as receive_crc() but free-running: `state` says
  // whether the next symbol carries the data bit or the clock bit, `tmp`/
  // `weight` shift a byte together. The sync-mark detector runs in parallel so
  // a mark can force both the phase and the byte boundary back into agreement
  // with what is actually on the disk.
  unsigned tmp = 0, weight = 0x8000;
  mfm_state_t state = mfm_io_even;
  uint32_t mark = 0;
  bool resync = false; // the next symbol is the one after a sync mark
  // KNOWN COSMETIC QUIRK: data-field sync runs are still reported as five
  // 0xA1s on real media where the disk has three; ID fields report three. Two
  // fixes were tried on hardware and neither moved the data-field case -- so
  // the extra pair is neither a decoded byte (the swallow below fixed exactly
  // that, and fixed the ID fields) nor a second match on the same run (the
  // since_fire guard covers that). Left alone deliberately: everything either
  // side of the run decodes correctly, and READ TRACK is a diagnostic whose
  // consumers care about the address mark and the field behind it. Anyone
  // resuming this should instrument the symbol stream rather than guess again.
  //
  // The detector does not always fire on the last 0xA1 of a run: it works on a
  // sliding window of symbols that, unlike the sector decoder's, is primed by
  // whatever preceded the sync, so it can match a byte or two early. When it
  // does, the remaining physical 0xA1s decode again as ordinary bytes and the
  // run is reported as five sync bytes instead of three. Swallow decoded 0xA1s
  // until the first byte that is not one -- which is the address mark, the
  // thing the caller is actually looking for.
  bool eat_a1 = false;
  unsigned since_fire = 4; // bytes decoded since the last mark (starts "far")

  while (!mfm_io_eof(&io) && n < out_size) {
    mfm_io_symbol_t s = mfm_io_read_symbol(&io);

    mark = ((mark << 2) | (uint32_t)s) & mfm_io_triple_mark_mask;
    if (mark == mfm_io_triple_mark_magic) {
      // Three 0xA1s with missing clock bits just went past. They are really on
      // the disk, so emit all three -- and more importantly, take the phase.
      // One run of 0xA1s can match more than once: the window slides by one
      // symbol at a time and the three sync bytes are identical, so a match at
      // the second 0xA1 is followed by another at the third. Emitting on each
      // reported five sync bytes where the disk has three. A second match
      // within a few bytes is therefore the SAME run -- take its (better)
      // phase, but do not announce it again.
      if (since_fire >= 4) {
        for (int k = 0; k < 3 && n < out_size; ++k)
          out[n++] = 0xA1;
      }
      since_fire = 0;
      tmp = 0;
      weight = 0x8000;
      resync = true;
      eat_a1 = true; // see below
      mark = 0;      // require a full fresh pattern before matching again
      continue;
    }

    if (resync) {
      // Exactly receive_crc()'s rule for the first symbol after a mark: on
      // arrival we are still waiting to parse the final '1' data bit of the
      // sync, so this symbol sets the parity and may carry a leading 0 -- it
      // does NOT contribute a bit under the normal rule.
      //
      // Skipping this is what made the decoder latch clock bits as data: the
      // gaps came back as 0x84 instead of 0x4E, which is not even a rotation
      // of it, and no address mark was ever recognisable.
      resync = false;
      switch (s) {
      case mfm_io_pulse_100:
        state = mfm_io_odd;
        weight >>= 1; // a leading 0 data bit
        break;
      case mfm_io_pulse_1000:
        state = mfm_io_even;
        weight >>= 1; // likewise
        break;
      default:
        state = mfm_io_even;
        break;
      }
      continue;
    }

    if (state)
      tmp |= weight;
    weight >>= 1;
    if (s == mfm_io_pulse_1000) {
      weight >>= 1; // the second bit of a 1000 is always a 0
    } else if (s == mfm_io_pulse_100) {
      if (state)
        weight >>= 1;
      state = (mfm_state_t)!state;
    }

    if (weight <= 0x80) {
      uint8_t b = (uint8_t)(tmp >> 8);
      tmp <<= 8;
      weight <<= 8;
      if (since_fire < 0xFFFF)
        since_fire++;
      if (eat_a1) {
        if (b == 0xA1)
          continue; // a sync byte already reported at the mark
        eat_a1 = false;
      }
      out[n++] = b;
    }
  }
  return n;
}
