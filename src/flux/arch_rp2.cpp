// Derived from Adafruit_Floppy (https://github.com/adafruit/Adafruit_Floppy),
// MIT License, Copyright (c) 2021 Limor Fried for Adafruit Industries.
// See LICENSES/MIT-Adafruit_Floppy.txt. Changes: Apache-2.0, see LICENSE.
// RP2040 PIO flux capture/write — bare Pico-SDK port of Adafruit_Floppy's
// arch_rp2.cpp (MIT). Changes vs. the Arduino original:
//   - <Arduino.h>/<Adafruit_Floppy.h> removed; only free functions kept.
//   - millis()            -> to_ms_since_boot(get_absolute_time())
//   - noInterrupts()/interrupts() -> save_and_disable_interrupts()/restore
//   - pinMode()/digitalWrite() on WGATE -> gpio_set_dir()/gpio_put()
//   - Dropped mfm_io_sample_flux() and the Adafruit_FloppyBase methods
//     (MFM sampling is not needed for the flux-level greaseweazle protocol).
#include "arch_rp2.h"
#include "greasepack.h"

#include <string.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/time.h"

static inline uint32_t millis() {
  return to_ms_since_boot(get_absolute_time());
}

// ---- PIO programs (unchanged from Adafruit_Floppy) ----

static const uint16_t fluxread[] = {
    0x0041, // wait_one:  jmp x--, wait_one_next
    0x00c3, // wait_one_next: jmp pin wait_zero
    0x0000, //            jmp wait_one
    0x0044, // wait_zero: jmp x--, wait_zero_next
    0x01c3, // wait_zero_next: jmp pin wait_zero [1]
    0x4001, //            in pins, 1
    0x402f, //            in x, 15
    0x0040, //            jmp x--, wait_one
};
static const pio_program_t fluxread_struct = {
    .instructions = fluxread,
    .length = sizeof(fluxread) / sizeof(fluxread[0]),
    .origin = -1};

static const uint16_t fluxwrite[] = {
    0xe000, // loop_flux: set pins, 0
    0x6030, //            out x, 16
    0xae42, //            nop [14]
    0xe001, //            set pins, 1
    0x0044, // loop_high: jmp x--, loop_high
    0x0000, //            jmp loop_flux
};
static const pio_program_t fluxwrite_struct = {
    .instructions = fluxwrite,
    .length = sizeof(fluxwrite) / sizeof(fluxwrite[0]),
    .origin = -1};

typedef struct floppy_singleton {
  PIO pio;
  const pio_program_t *program;
  unsigned sm;
  uint16_t offset;
  uint16_t half;
  // A second sample is pending in `half`. Its own flag, because 0 is a valid
  // sample ({counter[14:0], index} with ~INDEX low and the counter's low 15
  // bits 0): testing `half` for zero dropped it and merged two intervals
  // (2T+2T read as 4T), bit-slipping whatever was under the head (#24;
  // inherited from Adafruit_Floppy).
  bool half_valid;
} floppy_singleton_t;

static floppy_singleton_t g_reader, g_writer;

static const PIO pio_instances[2] = {pio0, pio1};

static bool allocate_pio_set_program(floppy_singleton_t *info,
                                     const pio_program_t *program) {
  memset(info, 0, sizeof(*info));
  for (size_t i = 0; i < NUM_PIOS; i++) {
    PIO pio = pio_instances[i];
    if (!pio_can_add_program(pio, program)) {
      continue;
    }
    int sm = pio_claim_unused_sm(pio, false);
    if (sm != -1) {
      info->pio = pio;
      info->sm = sm;
      info->offset = pio_add_program(pio, program);
      return true;
    }
  }
  return false;
}

// ---- Capture ----

static bool init_capture(int index_pin, int read_pin) {
  if (g_reader.pio) {
    return true;
  }
  if (!allocate_pio_set_program(&g_reader, &fluxread_struct)) {
    return false;
  }

  gpio_pull_up(index_pin);

  pio_sm_config c = {0, 0, 0};
  sm_config_set_wrap(&c, g_reader.offset,
                     g_reader.offset + fluxread_struct.length - 1);
  sm_config_set_jmp_pin(&c, read_pin);
  sm_config_set_in_pins(&c, index_pin);
  sm_config_set_in_shift(&c, true, true, 32);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
  pio_sm_set_pins_with_mask(g_reader.pio, g_reader.sm, 1u << read_pin,
                            1u << read_pin);
  float div = (float)clock_get_hz(clk_sys) / (3 * 24e6);
  sm_config_set_clkdiv(&c, div); // 72MHz capture clock / 24MHz sample rate

  pio_sm_init(g_reader.pio, g_reader.sm, g_reader.offset, &c);
  return true;
}

static void start_common() {
  pio_sm_exec(g_reader.pio, g_reader.sm, g_reader.offset);
  pio_sm_restart(g_reader.pio, g_reader.sm);
}

// ---- DMA-fed capture ring ------------------------------------------------
//
// The PIO reader pushes one 16-bit counter sample per flux transition, two
// per 32-bit word. Before this, the CPU drained the RX FIFO itself -- 8 words
// deep, ~24 us of flux at DD and ~12 us at HD -- and every consumer that
// decoded between drains lived on that cushion. The read path learned early
// to capture-then-decode for exactly that reason; the WRITE SECTOR splice
// cannot (it has to decode the ID as it passes to hit Gap 2), and at
// 500 kbit/s its live decoder fell behind the FIFO on ~8% of hunts and,
// once behind, never saw an intact ID again: zero IDs for 2.3 revolutions
// on flux that a capture-then-decode of the same track read perfectly
// (measured with the failed-write post-mortem).
//
// A DMA channel now drains the FIFO into this ring with hardware address
// wrap, paced by the PIO's RX DREQ, so the cushion is the ring (16 KB =
// 8192 samples, ~16-25 ms at HD) instead of the FIFO. Consumers keep their
// unpack logic; only the source of words changed. Overrun is now only
// possible after tens of milliseconds of not consuming, and is flagged.
#define FLUX_RING_BYTES_LOG2 14 // 16 KB, the alignment the DMA ring mode needs
#define FLUX_RING_WORDS (1u << (FLUX_RING_BYTES_LOG2 - 2))
static uint32_t g_ring[FLUX_RING_WORDS]
    __attribute__((aligned(1u << FLUX_RING_BYTES_LOG2)));
static int g_ring_dma = -1;       // claimed channel, -1 = none
static uint32_t g_ring_consumed = 0; // ABSOLUTE words consumed (monotonic)
static bool g_ring_overrun = false;

// Words the DMA has written since the capture started, as an absolute count.
// Configured with 0xFFFFFFFF transfers, so remaining counts down from there.
static inline uint32_t ring_written(void) {
  return 0xFFFFFFFFu - dma_hw->ch[g_ring_dma].transfer_count;
}

// Words available to the reader, absolute-difference so it CANNOT alias: if
// this reaches FLUX_RING_WORDS the DMA has lapped us and the oldest unread
// words have been overwritten. The old index-masked version wrapped a full
// lap back to a small number and reported "caught up" while handing out
// overwritten, out-of-sequence samples -- garbage deltas with no flag, which no consumer could detect.
static inline uint32_t ring_unread(void) {
  return ring_written() - g_ring_consumed;
}

static void ring_start(void) {
  if (g_ring_dma < 0)
    g_ring_dma = dma_claim_unused_channel(true);
  dma_channel_config c = dma_channel_get_default_config(g_ring_dma);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_ring(&c, true, FLUX_RING_BYTES_LOG2); // wrap the WRITE side
  channel_config_set_dreq(&c, pio_get_dreq(g_reader.pio, g_reader.sm, false));
  g_ring_consumed = 0;
  g_ring_overrun = false;
  // 2^32-1 transfers is ~8 hours at HD rates: effectively endless for any
  // capture this firmware runs. Stopped explicitly by ring_stop().
  dma_channel_configure(g_ring_dma, &c, g_ring, &g_reader.pio->rxf[g_reader.sm],
                        0xFFFFFFFFu, true);
}

static void ring_stop(void) {
  if (g_ring_dma >= 0) {
    dma_channel_abort(g_ring_dma);
    dma_channel_unclaim(g_ring_dma);
    g_ring_dma = -1;
  }
}

static bool data_available() {
  return g_reader.half_valid || (g_ring_dma >= 0 && ring_unread() != 0);
}

static uint16_t read_fifo() {
  if (g_reader.half_valid) {
    uint16_t result = g_reader.half;
    g_reader.half_valid = false;
    return result;
  }
  // Callers only get here after data_available(); spin for the word anyway so
  // the contract ("blocking read") holds if one ever does not.
  while (g_ring_dma >= 0 && ring_unread() == 0) {
  }
  // A lap: the DMA has written a whole ring past us, so the word we are about
  // to read has been overwritten with much newer flux and every delta from
  // here is out of sequence. Flag it, and resync to just behind the DMA so the
  // stream at least becomes coherent again rather than staying scrambled.
  if (ring_unread() >= FLUX_RING_WORDS) {
    g_ring_overrun = true;
    g_ring_consumed = ring_written() - (FLUX_RING_WORDS / 2);
  }
  uint32_t value = g_ring[g_ring_consumed & (FLUX_RING_WORDS - 1)];
  g_ring_consumed++;
  g_reader.half = value >> 16;
  g_reader.half_valid = true;
  return value & 0xffff;
}

static void disable_capture(void) {
  pio_sm_set_enabled(g_reader.pio, g_reader.sm, false);
  ring_stop();
}

static void free_capture(void) {
  if (!g_reader.pio) {
    return;
  }
  disable_capture();
  pio_sm_unclaim(g_reader.pio, g_reader.sm);
  pio_remove_program(g_reader.pio, &fluxread_struct, g_reader.offset);
  memset(&g_reader, 0, sizeof(g_reader));
}

// How long to wait for the FIRST flux transition after enabling the reader.
// Any magnetized disk that is actually turning produces transitions
// continuously, so one revolution (~200 ms at 300 rpm) plus margin is
// generous; only "no disk", "spindle not turning" or virgin AC-erased media
// can exhaust it -- and for all of those, failing is the correct answer.
static constexpr uint32_t FLUX_FIRST_SAMPLE_TIMEOUT_US = 400000;

static uint8_t *capture_foreground(int index_pin, uint8_t *start, uint8_t *end,
                                   int32_t *falling_index_offset,
                                   bool store_greaseweazle,
                                   uint32_t capture_counts,
                                   uint32_t max_wait_time) {
  uint8_t *ptr = start;
  if (falling_index_offset) {
    *falling_index_offset = -1;
  }
  start_common();

  // wait for a falling edge of index pin, then enable the capture peripheral
  if (max_wait_time) {
    uint32_t start_time = millis();
    while (!gpio_get(index_pin)) {
      if (millis() - start_time > max_wait_time) {
        disable_capture();
        return ptr;
      }
    }
    while (gpio_get(index_pin)) {
      if (millis() - start_time > max_wait_time) {
        disable_capture();
        return ptr;
      }
    }
  }

  uint32_t total_counts = 0;

  uint32_t irq = save_and_disable_interrupts();
  pio_sm_clear_fifos(g_reader.pio, g_reader.sm);
  g_reader.half = 0;
  g_reader.half_valid = false;
  ring_start();
  pio_sm_set_enabled(g_reader.pio, g_reader.sm, true);
  // Bounded prime -- same reasoning as in rp2040_flux_capture_start(): the
  // fluxread program only pushes on transitions, so an unconditional blocking
  // FIFO read hangs here forever, interrupts off, if the drive produces none.
  uint32_t prime_t0 = time_us_32();
  while (!data_available()) {
    if (time_us_32() - prime_t0 > FLUX_FIRST_SAMPLE_TIMEOUT_US) {
      restore_interrupts(irq);
      disable_capture();
      return ptr;
    }
  }
  int last = read_fifo();
  bool last_index = gpio_get(index_pin);
  // Flux can also STOP mid-capture (disk pulled, motor stopped). Bound the gap
  // between transitions, or this loop spins forever with interrupts off.
  uint32_t last_data_us = time_us_32();
  while (ptr != end) {
    bool now_index = gpio_get(index_pin);
    if (!now_index && last_index) {
      if (falling_index_offset) {
        *falling_index_offset = ptr - start;
        if (!capture_counts) {
          break;
        }
      }
    }
    last_index = now_index;

    if (!data_available()) {
      if (time_us_32() - last_data_us > FLUX_FIRST_SAMPLE_TIMEOUT_US)
        break;
      continue;
    }
    last_data_us = time_us_32();

    int data = read_fifo();
    int delta = last - data;
    if (delta < 0)
      delta += 65536;
    delta /= 2;

    last = data;
    total_counts += delta;
    if (store_greaseweazle) {
      ptr = greasepack(ptr, end, delta);
    } else {
      *ptr++ = delta > 255 ? 255 : delta;
    }
    if (capture_counts != 0 && total_counts >= capture_counts) {
      break;
    }
  }
  restore_interrupts(irq);

  disable_capture();
  return ptr;
}

// ---- Incremental (streaming) capture ----

// Persists across drain() calls, unlike capture_foreground's local `last` --
// each drain() picks up the delta computation exactly where the previous one
// left off, so there is no gap/discontinuity at poll boundaries.
static int g_stream_last;

bool rp2040_flux_capture_start(int index_pin, int rdpin) {
  if (!init_capture(index_pin, rdpin)) {
    return false;
  }
  start_common();
  uint32_t irq = save_and_disable_interrupts();
  pio_sm_clear_fifos(g_reader.pio, g_reader.sm);
  g_reader.half = 0;
  g_reader.half_valid = false;
  ring_start();
  pio_sm_set_enabled(g_reader.pio, g_reader.sm, true);
  restore_interrupts(irq);
  // Prime g_stream_last with the first sample -- BOUNDED. read_fifo() blocks
  // on the RX FIFO and the fluxread program only pushes on flux transitions,
  // so an unconditional read here hung the whole firmware (interrupts off, no
  // USB, replug to recover) whenever the selected drive produced no flux:
  // `streamscan` with no disk in the drive did exactly that. It also fired
  // before mfm_splice's own no-flux timeout could ever run. Waiting with
  // interrupts enabled is fine -- the delta arithmetic in drain() only cares
  // about counter values, not when we picked the first one up.
  uint32_t t0 = time_us_32();
  while (!data_available()) {
    if (time_us_32() - t0 > FLUX_FIRST_SAMPLE_TIMEOUT_US) {
      disable_capture();
      return false;
    }
  }
  g_stream_last = read_fifo();
  return true;
}

size_t rp2040_flux_capture_drain(volatile uint8_t *dest, size_t dest_cap,
                                 size_t already_written,
                                 bool store_greaseweazle) {
  uint8_t *ptr = (uint8_t *)dest + already_written;
  uint8_t *end = (uint8_t *)dest + dest_cap;

  uint32_t irq = save_and_disable_interrupts();
  while (ptr != end && data_available()) {
    int data = read_fifo();
    int delta = g_stream_last - data;
    if (delta < 0)
      delta += 65536;
    delta /= 2;
    g_stream_last = data;

    if (store_greaseweazle) {
      ptr = greasepack(ptr, end, delta);
    } else {
      *ptr++ = delta > 255 ? 255 : delta;
    }
  }
  restore_interrupts(irq);

  return (size_t)(ptr - (uint8_t *)dest);
}

void rp2040_flux_capture_stop(void) {
  disable_capture();
  free_capture();
}

// ---- Live capture (write splice) ----

int rp2040_flux_capture_next(uint32_t timeout_us) {
  uint32_t t0 = time_us_32();
  while (!data_available()) {
    if ((time_us_32() - t0) > timeout_us) {
      return -1;
    }
  }
  int data = read_fifo();
  int delta = g_stream_last - data;
  if (delta < 0)
    delta += 65536;
  delta /= 2;
  g_stream_last = data;
  return delta;
}

bool rp2040_flux_capture_overrun(void) { return g_ring_overrun; }

uint32_t rp2040_flux_capture_backlog(void) {
  uint32_t sum = 0;
  while (data_available()) {
    int d = rp2040_flux_capture_next(0);
    if (d < 0) {
      break;
    }
    sum += (uint32_t)d;
  }
  return sum;
}

void rp2040_flux_capture_disable(void) {
  if (g_reader.pio) {
    disable_capture();
  }
}

// ---- Write ----

static bool init_write(int wrdata_pin) {
  if (g_writer.pio) {
    return true;
  }
  const pio_program_t *program = &fluxwrite_struct;
  if (!allocate_pio_set_program(&g_writer, program)) {
    return false;
  }
  // AFTER the allocation: allocate_pio_set_program() memsets the whole struct,
  // so assigning before it left `program` null and free_write() handed
  // pio_remove_program() a null program on every write -- on the RP2040 that
  // reads a garbage length out of the boot ROM at address 0 and clears that
  // many bits of the PIO's instruction-space bookkeeping, which can let a
  // later program load over another one (e.g. the bus SMs').
  g_writer.program = program;

  uint32_t wrdata_bit = 1u << wrdata_pin;
  pio_gpio_init(g_writer.pio, wrdata_pin);
  pio_sm_set_pindirs_with_mask(g_writer.pio, g_writer.sm, wrdata_bit,
                               wrdata_bit);
  pio_sm_set_pins_with_mask(g_writer.pio, g_writer.sm, wrdata_bit, wrdata_bit);
  pio_sm_set_pins_with_mask(g_writer.pio, g_writer.sm, 0, wrdata_bit);
  pio_sm_set_pins_with_mask(g_writer.pio, g_writer.sm, wrdata_bit, wrdata_bit);

  pio_sm_config c{};
  sm_config_set_wrap(&c, g_writer.offset,
                     g_writer.offset + program->length - 1);
  sm_config_set_set_pins(&c, wrdata_pin, 1);
  sm_config_set_out_shift(&c, true, true, 16);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  float div = (float)clock_get_hz(clk_sys) / (24e6);
  sm_config_set_clkdiv(&c, div); // 24MHz output clock

  pio_sm_init(g_writer.pio, g_writer.sm, g_writer.offset, &c);
  return true;
}

#define OVERHEAD (20) // minimum pulse length due to PIO overhead, about 0.833us

static void disable_write() {
  pio_sm_set_enabled(g_writer.pio, g_writer.sm, false);
}

// How long a track write waits for the index pulse it starts on. One
// revolution is <= ~220 ms on any drive seen; no index in this long means no
// disk, an absent drive or a stopped spindle. The waits used to be unbounded,
// so a host WRITE TRACK on an empty drive hung core0 for good (issue #7).
static constexpr uint32_t WRITE_INDEX_TIMEOUT_US = 600000;

static bool write_foreground(int index_pin, int wrgate_pin, uint8_t *pulses,
                             uint8_t *pulse_end, bool store_greaseweazle,
                             bool use_index) {
  if (use_index) {
    uint32_t t0 = time_us_32();
    while (!gpio_get(index_pin)) { /* not during an index pulse */
      if (time_us_32() - t0 > WRITE_INDEX_TIMEOUT_US)
        return false;
    }
    while (gpio_get(index_pin)) { /* wait for falling edge of index */
      if (time_us_32() - t0 > WRITE_INDEX_TIMEOUT_US)
        return false;
    }
  }

  gpio_set_dir(wrgate_pin, GPIO_OUT);
  gpio_put(wrgate_pin, 0); // WGATE active low

  uint32_t irq = save_and_disable_interrupts();
  pio_sm_set_enabled(g_writer.pio, g_writer.sm, false);
  pio_sm_clear_fifos(g_writer.pio, g_writer.sm);
  pio_sm_exec(g_writer.pio, g_writer.sm, g_writer.offset);
  // Refills stop at the end of the buffer: past it greaseunpack() returns
  // 0xffff, and queueing those wrote a few long garbage pulses after the
  // image (#25).
  // Lengths of the two newest pulses queued, in 24 MHz ticks, as the PIO
  // will play them (it takes 16 bits).
  unsigned last_ticks = 0, prev_ticks = 0;
  while (pulses != pulse_end && !pio_sm_is_tx_fifo_full(g_writer.pio, g_writer.sm)) {
    unsigned value = greaseunpack(&pulses, pulse_end, store_greaseweazle);
    if (value == 0xffff && pulses == pulse_end)
      break; // a truncated greaseweazle record, not a pulse
    prev_ticks = last_ticks;
    last_ticks = value > 0xffff ? 0xffff : value;
    value = (value < OVERHEAD) ? 1 : value - OVERHEAD;
    pio_sm_put_blocking(g_writer.pio, g_writer.sm, value);
  }
  pio_sm_set_enabled(g_writer.pio, g_writer.sm, true);

  bool old_index_state = false;
  bool at_index = false;
  while (pulses != pulse_end) {
    bool index_state = gpio_get(index_pin);
    if (old_index_state && !index_state) {
      at_index = true;
      break; // falling edge of index pin
    }
    while (pulses != pulse_end && !pio_sm_is_tx_fifo_full(g_writer.pio, g_writer.sm)) {
      unsigned value = greaseunpack(&pulses, pulse_end, store_greaseweazle);
      if (value == 0xffff && pulses == pulse_end)
        break;
      prev_ticks = last_ticks;
      last_ticks = value > 0xffff ? 0xffff : value;
      value = (value < OVERHEAD) ? 1 : value - OVERHEAD;
      pio_sm_put_blocking(g_writer.pio, g_writer.sm, value);
    }
    old_index_state = index_state;
  }
  if (!at_index) {
    // The image ran out before the index. Disabling the state machine now
    // would drop whatever is still queued -- the last bytes of the image --
    // so let the FIFO drain and the last pulse reach the head first, as the
    // splice does. Never past the index, though: that is the start of the
    // track, so both waits end at its falling edge (#25).
    while (!pio_sm_is_tx_fifo_empty(g_writer.pio, g_writer.sm)) {
      bool index_state = gpio_get(index_pin);
      if (old_index_state && !index_state) {
        at_index = true;
        break;
      }
      old_index_state = index_state;
    }
    // FIFO empty means the last word has just been pulled, not that it is
    // out: the pulse before it has only just started, and the last one
    // follows it. Wait both lengths (the output clock is 24 MHz, so ticks/24
    // is us; a greaseweazle interval can be far longer than MFM's 8 us, and
    // is clamped to the 16 bits the PIO plays, so the wait is bounded at
    // ~5.5 ms even with no index to end it).
    uint32_t tail_us = (prev_ticks + last_ticks) / 24u + 16u;
    uint32_t t_tail = time_us_32();
    while (!at_index && time_us_32() - t_tail < tail_us) {
      bool index_state = gpio_get(index_pin);
      if (old_index_state && !index_state)
        at_index = true;
      old_index_state = index_state;
    }
  }
  restore_interrupts(irq);

  pio_sm_set_enabled(g_writer.pio, g_writer.sm, false);
  gpio_set_dir(wrgate_pin, GPIO_IN); // release WGATE (external/internal pull-up)
  gpio_pull_up(wrgate_pin);
  return true;
}

static void free_write() {
  if (!g_writer.pio) {
    return;
  }
  disable_write();
  pio_sm_unclaim(g_writer.pio, g_writer.sm);
  pio_remove_program(g_writer.pio, g_writer.program, g_writer.offset);
  memset(&g_writer, 0, sizeof(g_writer));
}

// ---- Pre-armed write (splice) ----

bool rp2040_flux_write_prepare(int wrdata_pin) {
  return init_write(wrdata_pin);
}

void rp2040_flux_write_release(void) { free_write(); }

bool rp2040_flux_write_now(int wrgate_pin, const uint8_t *pulses, size_t n) {
  if (!g_writer.pio) {
    return false;
  }
  const uint8_t *p = pulses;
  const uint8_t *end = pulses + n;

  gpio_set_dir(wrgate_pin, GPIO_OUT);
  gpio_put(wrgate_pin, 0); // WGATE active low

  pio_sm_set_enabled(g_writer.pio, g_writer.sm, false);
  pio_sm_clear_fifos(g_writer.pio, g_writer.sm);
  pio_sm_exec(g_writer.pio, g_writer.sm, g_writer.offset);
  while (p != end && !pio_sm_is_tx_fifo_full(g_writer.pio, g_writer.sm)) {
    unsigned value = *p++;
    value = (value < OVERHEAD) ? 1 : value - OVERHEAD;
    pio_sm_put_blocking(g_writer.pio, g_writer.sm, value);
  }
  pio_sm_set_enabled(g_writer.pio, g_writer.sm, true);

  while (p != end) {
    while (p != end && !pio_sm_is_tx_fifo_full(g_writer.pio, g_writer.sm)) {
      unsigned value = *p++;
      value = (value < OVERHEAD) ? 1 : value - OVERHEAD;
      pio_sm_put_blocking(g_writer.pio, g_writer.sm, value);
    }
  }

  // Let the FIFO drain and the last pulse actually reach the head before WGATE
  // drops -- unlike the greaseweazle path (which ends at an index pulse, in the
  // middle of a gap) a splice ends a few bytes past the data CRC, and cutting
  // the tail short there truncates real data. The longest single pulse is 4T,
  // 8 us at DD; 32 us covers it with margin.
  while (!pio_sm_is_tx_fifo_empty(g_writer.pio, g_writer.sm)) {
  }
  busy_wait_us_32(32);

  pio_sm_set_enabled(g_writer.pio, g_writer.sm, false);
  gpio_set_dir(wrgate_pin, GPIO_IN); // release WGATE (pull-up)
  gpio_pull_up(wrgate_pin);
  return true;
}

// ---- Public API ----

uint32_t rp2040_flux_capture(int index_pin, int rdpin, volatile uint8_t *pulses,
                             volatile uint8_t *pulse_end,
                             int32_t *falling_index_offset,
                             bool store_greaseweazle, uint32_t capture_counts,
                             uint32_t index_wait_ms) {
  if (!init_capture(index_pin, rdpin)) {
    return 0;
  }
  uint32_t result =
      capture_foreground(index_pin, (uint8_t *)pulses, (uint8_t *)pulse_end,
                         falling_index_offset, store_greaseweazle,
                         capture_counts, index_wait_ms) -
      (uint8_t *)pulses;
  free_capture();
  return result;
}

bool rp2040_flux_write(int index_pin, int wrgate_pin, int wrdata_pin,
                       uint8_t *pulses, uint8_t *pulse_end,
                       bool store_greaseweazle, bool use_index) {
  if (!init_write(wrdata_pin)) {
    return false;
  }
  bool ok = write_foreground(index_pin, wrgate_pin, pulses, pulse_end,
                             store_greaseweazle, use_index);
  free_write();
  return ok;
}
