#include "mfm_splice.hpp"

#include "board_config.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include "flux/arch_rp2.h"
#include "mfm_encode.hpp"

#include "hardware/sync.h"
#include "pico/time.h"

#include <cmath>
#include <cstring>

#define DEBUG_PRINTF(...) ((void)0)
#define DEBUG_ASSERT(x) ((void)0)
#include "flux/mfm_impl.h"

namespace {

// RP2040 flux sample frequency (arch_rp2 clkdiv -> 24 MHz).
constexpr uint32_t GW_SF = 24000000UL;

// Encoded data field, largest case: 12 presync + 3 sync + mark + 1024 data +
// 2 CRC + gap, at up to 8 flux transitions per byte.
constexpr size_t SPLICE_FLUX_MAX = 9216;
uint8_t g_splice_flux[SPLICE_FLUX_MAX];

// Gap bytes written after the data CRC, under WGATE, purely to carry the
// write-off splice (see mfm_splice.hpp).
constexpr int SPLICE_TRAILING_GAP = 3;

// A DD track always has transitions (even the gaps are 0x4E); this much silence
// means no disk, no motor, or the head is off the media.
constexpr uint32_t SPLICE_SYMBOL_TIMEOUT_US = 10000;

// Debug: raw deltas (24 MHz ticks, clamped to a byte) the live hunt saw. The
// bench cannot reproduce the HD blind hunts (no MZ bus to switch), so this
// carries the ACTUAL garbage stream off a failed hunt to be looked at -- good
// signal, noise, a wrong clock, zeros and a stuck line all look different in
// the raw deltas. Recorded during every hunt; latched on a blind failure.
constexpr int HUNT_DBG_N = 512;
static uint8_t g_hunt_dbg[HUNT_DBG_N]; // ring: holds the LAST HUNT_DBG_N symbols
static uint32_t g_hunt_dbg_wr = 0;     // total symbols this hunt (ring wr index)
static uint8_t g_hunt_fail[HUNT_DBG_N];
static uint16_t g_hunt_fail_n = 0;
static uint32_t g_hunt_fail_total = 0; // total symbols the failed hunt processed
static bool g_hunt_fail_valid = false;
static uint8_t g_hunt_ok[HUNT_DBG_N];
static uint16_t g_hunt_ok_n = 0;
static bool g_hunt_ok_valid = false;
// Sync-search forensics for the last blind hunt: symbol histogram and how many
// of the 14-symbol 0xA1 triple-sync the matcher ever got right in a row.
static uint32_t g_sym_2t, g_sym_3t, g_sym_4t;
static uint8_t g_sync_best; // best partial match, in symbols (max 14)
static uint32_t g_sym_2t_f, g_sym_3t_f, g_sym_4t_f;
static uint8_t g_sync_best_f;
// Pin state during the hunt. The blind-hunt flux is a clean ~1 MHz tone that
// cannot be the disk (the same track post-mortems fine 240 ms later), so
// something DRIVES GPIO20 while the splice's state is active. Measure the
// pins rather than reason about them: raw GPIO input word, output-enable
// word, and GPIO20's pad function, at hunt start and end.
struct HuntPins { uint32_t in_start, in_end, oe_start, oe_end; uint8_t f20, f17, f18; };
static HuntPins g_hunt_pins_cur, g_hunt_pins_fail;
// Pulse widths (low, high alternating, u16-clamped counts) of the first 512
// pulses of the last BLIND hunt and of the FIRST successful hunt (reference).
constexpr int PWMON_N = 1024;
static uint16_t g_pw_fail[PWMON_N];
static uint16_t g_pw_fail_n = 0;
static bool g_pw_fail_valid = false;
static uint16_t g_pw_ok[PWMON_N];
static uint16_t g_pw_ok_n = 0;
static bool g_pw_ok_valid = false;
static void hunt_dbg_linearise(uint8_t *dst, uint16_t *dst_n) {
  uint32_t total = g_hunt_dbg_wr;
  uint32_t cnt = total < HUNT_DBG_N ? total : HUNT_DBG_N;
  uint32_t first = total - cnt; // absolute index of the oldest kept symbol
  for (uint32_t k = 0; k < cnt; ++k)
    dst[k] = g_hunt_dbg[(first + k) % HUNT_DBG_N];
  *dst_n = (uint16_t)cnt;
}
static void pw_latch(uint16_t *dst, uint16_t *dst_n, uint32_t n) {
  const uint32_t *src = rp2040_pwmon_buf();
  if (n > PWMON_N)
    n = PWMON_N;
  for (uint32_t k = 0; k < n; ++k)
    dst[k] = (uint16_t)(src[k] > 0xFFFF ? 0xFFFF : src[k]);
  *dst_n = (uint16_t)n;
}
static inline void snap_pins(uint32_t *in, uint32_t *oe) {
  *in = sio_hw->gpio_in;
  *oe = sio_hw->gpio_oe;
}

// Live MFM symbol source: the buffered decoder's mfm_io_read_symbol() reads
// pulses[pos++] out of an array, this one takes the next transition off the
// capture FIFO as the disk produces it. Everything downstream (the sync-mark
// shift register, the parity/bit assembly) is the same algorithm, which is why
// they agree on what is on the disk.
struct LiveIo {
  uint16_t T2_max, T3_max;
  bool dead;        // a symbol never arrived -> the stream is over
  bool saw_long;    // a 3T or 4T interval has appeared at these thresholds
  uint32_t nsym;    // symbols consumed this hunt (for the wrong-density bail)
};

inline int live_symbol(LiveIo *l) {
  int d = rp2040_flux_capture_next(SPLICE_SYMBOL_TIMEOUT_US);
  g_hunt_dbg[g_hunt_dbg_wr % HUNT_DBG_N] =
      (d < 0) ? 0 : (d > 255 ? 255 : (uint8_t)d);
  g_hunt_dbg_wr++;
  if (d < 0) {
    l->dead = true;
    return mfm_io_pulse_10; // harmless filler; callers check `dead`
  }
  l->nsym++;
  if (d > (int)l->T3_max) {
    g_sym_4t++;
    l->saw_long = true;
    return mfm_io_pulse_1000;
  }
  if (d > (int)l->T2_max) {
    g_sym_3t++;
    l->saw_long = true;
    return mfm_io_pulse_100;
  }
  g_sym_2t++;
  return mfm_io_pulse_10;
}

// Live counterpart of skip_triple_sync_mark().
bool live_find_sync(LiveIo *l, uint32_t deadline_us) {
  uint32_t state = 0;
  while (state != mfm_io_triple_mark_magic) {
    state = ((state << 2) | (uint32_t)live_symbol(l)) & mfm_io_triple_mark_mask;
    // How many trailing 2-bit symbols of `state` agree with the target: the
    // depth the matcher reached before a symbol broke the run.
    uint32_t diff = (state ^ mfm_io_triple_mark_magic) & mfm_io_triple_mark_mask;
    uint8_t match = 0;
    while (match < 14 && (diff & 3u) == 0) { diff >>= 2; match++; }
    if (match > g_sync_best)
      g_sync_best = match;
    if (l->dead)
      return false;
    // Wrong bit clock: at the correct density every gap and ID carries 3T/4T
    // intervals, so ~12k symbols (>2 sectors) with NONE of them means the
    // thresholds are for the other density. Bail now so the caller can retry
    // at the other density in milliseconds instead of the full search timeout.
    if (!l->saw_long && l->nsym > 12288)
      return false;
    if ((int32_t)(time_us_32() - deadline_us) >= 0)
      return false;
  }
  return true;
}

// Live counterpart of receive_crc(), for one contiguous run of bytes. The
// symbol-to-bit rules -- including the special treatment of the first symbol
// after a sync mark -- are copied from receive_crc(); see the commentary there.
uint16_t live_receive(LiveIo *l, uint8_t *buf, size_t n) {
  unsigned tmp = 0, weight = 0x8000;
  uint16_t crc = mfm_io_crc_preload_value;

#define PUT_BIT(x)                                                             \
  do {                                                                         \
    if (x)                                                                     \
      tmp |= weight;                                                           \
    weight >>= 1;                                                              \
  } while (0)

  int s = live_symbol(l);
  mfm_state_t state = mfm_io_even;
  switch (s) {
  case mfm_io_pulse_100:
    state = mfm_io_odd;
    /* fallthrough */
  case mfm_io_pulse_1000:
    PUT_BIT(0);
    break;
  default:
    break;
  }

  while (n && !l->dead) {
    s = live_symbol(l);
    PUT_BIT(state);
    if (s == mfm_io_pulse_1000)
      PUT_BIT(0);
    if (s == mfm_io_pulse_100) {
      if (state)
        PUT_BIT(0);
      state = (mfm_state_t)!state;
    }
    if (weight <= 0x80) {
      *buf = tmp >> 8;
      crc = mfm_io_crc16_table[*buf ^ (uint8_t)(crc >> 8)] ^ (crc << 8);
      tmp <<= 8;
      weight <<= 8;
      buf++;
      n--;
    }
  }
#undef PUT_BIT
  return crc;
}

} // namespace

void mfm_splice_debug_pw(uint8_t which, const uint16_t **out, uint16_t *n,
                         uint8_t *valid) {
  *out = which ? g_pw_ok : g_pw_fail;
  *n = which ? g_pw_ok_n : g_pw_fail_n;
  *valid = (which ? g_pw_ok_valid : g_pw_fail_valid) ? 1 : 0;
}

void mfm_splice_debug_pins(uint32_t *in_start, uint32_t *in_end,
                           uint32_t *oe_start, uint32_t *oe_end,
                           uint8_t *f20, uint8_t *f17, uint8_t *f18) {
  *in_start = g_hunt_pins_fail.in_start;
  *in_end = g_hunt_pins_fail.in_end;
  *oe_start = g_hunt_pins_fail.oe_start;
  *oe_end = g_hunt_pins_fail.oe_end;
  *f20 = g_hunt_pins_fail.f20;
  *f17 = g_hunt_pins_fail.f17;
  *f18 = g_hunt_pins_fail.f18;
}

uint32_t mfm_splice_debug_flux_total(void) { return g_hunt_fail_total; }
void mfm_splice_debug_sync(uint32_t *n2t, uint32_t *n3t, uint32_t *n4t,
                           uint8_t *best) {
  *n2t = g_sym_2t_f; *n3t = g_sym_3t_f; *n4t = g_sym_4t_f; *best = g_sync_best_f;
}
void mfm_splice_debug_flux(uint8_t which, uint8_t *out, uint16_t *n,
                           uint8_t *valid) {
  const uint8_t *src = which ? g_hunt_ok : g_hunt_fail;
  uint16_t cnt = which ? g_hunt_ok_n : g_hunt_fail_n;
  for (uint16_t k = 0; k < cnt; ++k)
    out[k] = src[k];
  *n = cnt;
  *valid = (which ? g_hunt_ok_valid : g_hunt_fail_valid) ? 1 : 0;
}

SpliceResult mfm_write_sector(uint8_t want_sec, int want_cyl,
                              const uint8_t *data, uint16_t n_data,
                              bool deleted, float nominal_bit_time_us,
                              uint32_t timeout_us) {
  SpliceResult r;
  memset(&r, 0, sizeof(r));

  // Encode first, outside the timed window: this is ~1 ms of CRC and bit
  // twiddling and there is no room for it once an ID has been matched.
  size_t n_flux =
      mfm_encode_data_field(g_splice_flux, SPLICE_FLUX_MAX, data, n_data,
                            deleted, nominal_bit_time_us, SPLICE_TRAILING_GAP);
  if (n_flux == 0) {
    r.encode_fail = true;
    return r;
  }

  // Gap 2 is 22 bytes of 0x4E between the ID CRC and the data field's presync
  // zeros. A WD179x lets the whole of it pass, then turns WGATE on and starts
  // writing the 12 zeros -- which is why a rewritten sector lands in exactly
  // the same place as the formatter put it. One MFM byte is 8 data bits at two
  // cells each.
  const uint32_t byte_us = (uint32_t)lroundf(16.0f * nominal_bit_time_us);
  //
  // The two ways of being wrong are not symmetric. Starting LATE eats into the
  // presync zeros -- and we write 12 of our own anyway, so the reader still
  // locks and the field simply sits a byte or two further into Gap 3, which is
  // 54 bytes long. Starting EARLY writes over the tail of the ID field's CRC
  // and destroys the sector's identity permanently. So the bias is deliberately
  // positive: two byte times of "rather late than early", well inside what the
  // gap absorbs.
  const uint32_t gap_us = (22u + 2u) * byte_us;

  // Claim the write state machine BEFORE the search: allocating and loading a
  // PIO program inside the Gap 2 window would eat most of it.
  if (!rp2040_flux_write_prepare(PIN_WDATA))
    return r;

  LiveIo l;
  g_hunt_dbg_wr = 0;
  g_sym_2t = g_sym_3t = g_sym_4t = 0;
  g_sync_best = 0;
  uint32_t irq = save_and_disable_interrupts();
  uint32_t t_start = time_us_32();
  snap_pins(&g_hunt_pins_cur.in_start, &g_hunt_pins_cur.oe_start);
  g_hunt_pins_cur.f20 = (uint8_t)gpio_get_function(PIN_RDATA);
  g_hunt_pins_cur.f17 = (uint8_t)gpio_get_function(PIN_WGATE);
  g_hunt_pins_cur.f18 = (uint8_t)gpio_get_function(PIN_WDATA);
  bool pwmon_on = false;

  if (!rp2040_flux_capture_start(PIN_INDEX, PIN_RDATA)) {
    restore_interrupts(irq);
    rp2040_flux_write_release();
    r.no_flux = true;
    return r;
  }
  pwmon_on = rp2040_pwmon_start(PIN_RDATA); // pulse-shape monitor, DMA-fed

  l.T2_max = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 2.5e-6f);
  l.T3_max = (uint16_t)lroundf(GW_SF * nominal_bit_time_us * 3.5e-6f);
  l.dead = false;
  l.saw_long = false;
  l.nsym = 0;

  uint32_t deadline = t_start + timeout_us;

  while (!l.dead && (int32_t)(time_us_32() - deadline) < 0) {
    if (!live_find_sync(&l, deadline))
      break;
    r.syncs++;

    uint8_t idam[1 + mfm_io_idam_size + mfm_io_crc_size]; // mark + ID + CRC
    uint16_t crc = live_receive(&l, idam, sizeof(idam));
    if (l.dead)
      break;
    // Timestamp the end of the ID field immediately: everything from here is
    // measured against it, and the checks below cost microseconds.
    uint32_t t_id = time_us_32();

    if (idam[0] != MFM_IO_IDAM || crc != 0) {
      r.bad_idam++;
      continue; // not an ID field, or unreadable -> wait for the next one
    }
    r.ids_seen++;
    if (idam[3] != want_sec)
      continue;

    if (want_cyl != MFM_ANY_CYL && idam[1] != (uint8_t)want_cyl) {
      // Right sector, wrong cylinder. On a read this is a retryable RNF; here
      // it is the difference between rewriting a sector and destroying an
      // unrelated track, so it always aborts.
      r.cyl_mismatch = true;
      r.seen_cyl = idam[1];
      r.seen_head = idam[2];
      continue;
    }

    r.id_found = true;
    r.seen_cyl = idam[1];
    r.seen_head = idam[2];
    r.size_n = idam[4] & 7;
    if ((uint16_t)(128u << r.size_n) != n_data) {
      // Writing more than the sector holds would run straight over the next ID
      // field; writing less would leave the tail of the old data in place under
      // a CRC computed for the new. Neither is recoverable, so refuse.
      r.size_mismatch = true;
      break;
    }

    // Flux still queued in the FIFO is disk that has ALREADY gone past the
    // head, so it has to come off the gap we are about to wait out. Normally
    // this is zero or one transition -- the search consumes them as fast as
    // they arrive -- but it is exactly the error that would put the splice in
    // the wrong place, and it costs one drain to remove.
    uint32_t backlog_us = rp2040_flux_capture_backlog() / (GW_SF / 1000000u);
    rp2040_flux_capture_disable(); // silence the reader for the write

    if (backlog_us + 64 >= gap_us) {
      // Far enough behind that the data field is already under the head. Do not
      // guess -- a misplaced splice lands on top of the next ID field.
      r.late = true;
      break;
    }
    uint32_t wait_us = gap_us - backlog_us;
    while ((time_us_32() - t_id) < wait_us) {
    }

    r.wrote = rp2040_flux_write_now(PIN_WGATE, g_splice_flux, n_flux);
    break;
  }

  r.no_flux = l.dead && r.ids_seen == 0;
  snap_pins(&g_hunt_pins_cur.in_end, &g_hunt_pins_cur.oe_end);
  uint32_t pw_n = pwmon_on ? rp2040_pwmon_stop() : 0;
  r.overrun = rp2040_flux_capture_overrun();
  r.pio_overflow = rp2040_flux_capture_pio_overflow();
  rp2040_flux_capture_stop();
  if (r.ids_seen == 0 && !r.no_flux) { // blind: latch the stream we just saw
    g_hunt_pins_fail = g_hunt_pins_cur;
    pw_latch(g_pw_fail, &g_pw_fail_n, pw_n);
    g_pw_fail_valid = pw_n > 0;
    hunt_dbg_linearise(g_hunt_fail, &g_hunt_fail_n);
    g_hunt_fail_total = g_hunt_dbg_wr;
    g_sym_2t_f = g_sym_2t; g_sym_3t_f = g_sym_3t; g_sym_4t_f = g_sym_4t;
    g_sync_best_f = g_sync_best;
    g_hunt_fail_valid = true;
  } else if (r.id_found && !g_hunt_ok_valid) { // first good hunt: keep for A/B
    pw_latch(g_pw_ok, &g_pw_ok_n, pw_n);
    g_pw_ok_valid = pw_n > 0;
    hunt_dbg_linearise(g_hunt_ok, &g_hunt_ok_n);
    g_hunt_ok_valid = true;
  }
  restore_interrupts(irq);
  rp2040_flux_write_release();
  r.search_us = time_us_32() - t_start;
  return r;
}
