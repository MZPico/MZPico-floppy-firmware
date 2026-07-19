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
