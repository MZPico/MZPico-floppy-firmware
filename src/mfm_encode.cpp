#include "mfm_encode.hpp"

#include <cmath>
#include <cstring>

#define DEBUG_PRINTF(...) ((void)0)
#define DEBUG_ASSERT(x) ((void)0)
#include "flux/mfm_impl.h"

static constexpr uint32_t GW_SF = 24000000UL;

size_t mfm_encode_track(uint8_t *flux, size_t max_flux, const uint8_t *sectors,
                        int n_sectors, int size_code, float nominal_bit_time_us,
                        uint8_t cyl, uint8_t head) {
  mfm_io_t io;
  memset(&io, 0, sizeof(io));
  io.T1_nom = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 1e-6f);
  io.T2_max = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 2.5e-6f);
  io.T3_max = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 3.5e-6f);
  io.pulses = flux;
  io.n_pulses = max_flux;
  io.sectors = const_cast<uint8_t *>(sectors);
  io.n_sectors = (size_t)n_sectors;
  io.n = (uint8_t)(size_code & 7);
  io.cylinder = cyl;
  io.head = head;
  io.sector_validity = nullptr;
  io.settings = &standard_mfm; // IBM System-34 MFM gaps/marks
  io.encode_compact = false;

  // Returns length up to the end of the encoded sectors (before gap fill).
  return encode_track_mfm(&io);
}

size_t mfm_encode_data_field(uint8_t *flux, size_t max_flux,
                             const uint8_t *data, size_t n_data, bool deleted,
                             float nominal_bit_time_us, int trailing_gap) {
  mfm_io_t io;
  memset(&io, 0, sizeof(io));
  io.T1_nom = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 1e-6f);
  io.pulses = flux;
  io.n_pulses = max_flux;
  io.settings = &standard_mfm;
  io.flux_byte = mfm_io_flux_byte;
  io.encode_raw = mfm_io_encode_raw_mfm;
  io.pos = 0;
  io.y = 0;
  // Start mid-cell rather than at 0: with pulse_len 0 the very first transition
  // lands one bit time after WGATE goes active, i.e. right on top of the
  // turn-on transient. One extra cell of lead-in costs nothing (it falls inside
  // the presync zeros) and keeps the first recorded transition clean.
  io.pulse_len = 1;

  for (int i = 0; i < standard_mfm.gap_presync; i++)
    mfm_io_encode_byte(&io, 0x00);
  mfm_io_encode_raw_buf(&io, mfm_io_sync_bytes_mfm,
                        sizeof(mfm_io_sync_bytes_mfm));

  mfm_io_crc_preload(&io); // preload EXCLUDES the three 0xA1s (they are in it)
  mfm_io_encode_byte_crc(&io, deleted ? 0xF8 : (uint8_t)MFM_IO_DAM);
  mfm_io_encode_buf_crc(&io, data, n_data);
  mfm_io_encode_crc(&io);

  for (int i = 0; i < trailing_gap; i++)
    mfm_io_encode_byte(&io, standard_mfm.gap_byte);

  // mfm_io_flux_put silently drops pulses once pos reaches n_pulses, so an
  // overflow would look like a short-but-valid field and be written to the
  // disk. Report it as a failure instead.
  return mfm_io_eof(&io) ? 0 : io.pos;
}
