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
    if (skip_triple_sync_mark(&io)) {
      size_t dsize = s->size;
      if (dsize <= scratch_size) {
        uint8_t dmark = 0;
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
    }
    n++;
  }
  return n;
}
