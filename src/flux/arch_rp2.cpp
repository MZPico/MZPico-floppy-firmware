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

// Flux capture with a minimum-low-duration DEGLITCH (assembled by pioasm from
// the source in the commit message; the Adafruit original captured the instant
// the pin went low).
//
// Why: the RDATA rise is a slow RC curve (scope: sharp fall, ~1 us exponential
// recovery). At HD the line spends a large part of every 2 us interval near
// the input threshold, where core1's ground bounce as it switches the bus
// re-crosses it, and instant-capture records a spurious extra edge per pulse
// -- intervals halved, the exact huntflux signature of the blind HD write
// hunts. DD's 4 us interval leaves the line settled on a flat top, hence
// immune. A real drive pulse stays low >= ~150-300 ns; a threshold re-crossing
// is a dip of tens of ns. So after seeing low, require it to STAY low for 2
// iterations (3 instr each @ 72 MHz = ~83 ns) before capturing; if it goes
// high again first it was a glitch. x keeps decrementing throughout, so the
// capture offset is constant and intervals are unchanged.
static const uint16_t fluxread[] = {
    0x0041, //  0 wait_one:       jmp x--, wait_one_next
    0x00c3, //  1 wait_one_next:  jmp pin, wait_zero
    0x0000, //  2                 jmp wait_one
    0x0044, //  3 wait_zero:      jmp x--, wait_zero_next
    0x01c3, //  4 wait_zero_next: jmp pin, wait_zero [1]
    0xe042, //  5                 set y, 2        ; low seen: debounce
    0x0047, //  6 deb:            jmp x--, deb_chk ; keep the counter running
    0x00c3, //  7 deb_chk:        jmp pin, wait_zero ; high again = glitch
    0x0086, //  8                 jmp y--, deb
    0x4001, //  9                 in pins, 1      ; stayed low: capture
    0x402f, // 10                 in x, 15
    0x0040, // 11                 jmp x--, wait_one
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

static const uint16_t fluxwrite_apple2[] = {
    0x6030, 0xe001, 0xb042, 0x0043, 0x6030, 0xe000, 0xb042, 0x0047,
};
static const pio_program_t fluxwrite_apple2_struct = {
    .instructions = fluxwrite_apple2,
    .length = sizeof(fluxwrite_apple2) / sizeof(fluxwrite_apple2[0]),
    .origin = -1};

typedef struct floppy_singleton {
  PIO pio;
  const pio_program_t *program;
  unsigned sm;
  uint16_t offset;
  uint16_t half;
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

// ---- RDATA pulse-width monitor (diagnostic) --------------------------------
//
// A spare state machine on the read pin measures every pulse: the LOW
// duration, then the HIGH duration that follows, pushed alternately as
// counts of 3 PIO cycles at clkdiv 1 (width_ns = count * 3e9 / clk_sys, ~15 ns
// resolution). A one-shot DMA drops the first PWMON_WORDS samples (512
// pulses) of a capture into a buffer at zero cost to whoever is decoding, so
// the pulse SHAPE of the blind HD write hunts -- a 2x-density train the scope
// could not trigger on -- is recorded for exactly the event that fails.
// Assembled by pioasm from:
//   wait_low:  jmp pin, wait_low
//              mov x, ~null
//   low_loop:  jmp x--, low_chk
//   low_chk:   jmp pin, low_done
//              jmp low_loop
//   low_done:  mov isr, ~x ; push noblock
//              mov x, ~null
//   high_loop: jmp x--, high_chk
//   high_chk:  jmp pin, high_loop [1]
//              mov isr, ~x ; push noblock
static const uint16_t pwmon_prog[] = {
    0x00c0, 0xa02b, 0x0043, 0x00c5, 0x0002, 0xa0c9,
    0x8000, 0xa02b, 0x0049, 0x01c8, 0xa0c9, 0x8000,
};
static const pio_program_t pwmon_struct = {
    .instructions = pwmon_prog,
    .length = sizeof(pwmon_prog) / sizeof(pwmon_prog[0]),
    .origin = -1};
static floppy_singleton_t g_pwmon;
#define PWMON_WORDS 1024 // 512 pulses x (low, high)
static uint32_t g_pwmon_buf[PWMON_WORDS];
static int g_pwmon_dma = -1;

bool rp2040_pwmon_start(int read_pin) {
  if (!g_pwmon.pio) {
    if (!allocate_pio_set_program(&g_pwmon, &pwmon_struct))
      return false;
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, g_pwmon.offset,
                       g_pwmon.offset + pwmon_struct.length - 1);
    sm_config_set_jmp_pin(&c, read_pin);
    sm_config_set_clkdiv(&c, 1.0f);
    sm_config_set_in_shift(&c, false, false, 32); // manual push only
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(g_pwmon.pio, g_pwmon.sm, g_pwmon.offset, &c);
  }
  pio_sm_set_enabled(g_pwmon.pio, g_pwmon.sm, false);
  pio_sm_clear_fifos(g_pwmon.pio, g_pwmon.sm);
  pio_sm_restart(g_pwmon.pio, g_pwmon.sm);
  pio_sm_exec(g_pwmon.pio, g_pwmon.sm, pio_encode_jmp(g_pwmon.offset));
  if (g_pwmon_dma < 0)
    g_pwmon_dma = dma_claim_unused_channel(true);
  dma_channel_config dc = dma_channel_get_default_config(g_pwmon_dma);
  channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
  channel_config_set_read_increment(&dc, false);
  channel_config_set_write_increment(&dc, true);
  channel_config_set_dreq(&dc, pio_get_dreq(g_pwmon.pio, g_pwmon.sm, false));
  dma_channel_configure(g_pwmon_dma, &dc, g_pwmon_buf,
                        &g_pwmon.pio->rxf[g_pwmon.sm], PWMON_WORDS, true);
  pio_sm_set_enabled(g_pwmon.pio, g_pwmon.sm, true);
  return true;
}

uint32_t rp2040_pwmon_stop(void) {
  if (!g_pwmon.pio)
    return 0;
  pio_sm_set_enabled(g_pwmon.pio, g_pwmon.sm, false);
  uint32_t remaining = 0;
  if (g_pwmon_dma >= 0) {
    remaining = dma_hw->ch[g_pwmon_dma].transfer_count;
    dma_channel_abort(g_pwmon_dma);
  }
  return PWMON_WORDS - remaining; // words actually captured
}

const uint32_t *rp2040_pwmon_buf(void) { return g_pwmon_buf; }

// ---- Capture ----

static bool init_capture(int index_pin, int read_pin) {
  if (g_reader.pio) {
    return true;
  }
  if (!allocate_pio_set_program(&g_reader, &fluxread_struct)) {
    return false;
  }

  gpio_pull_up(index_pin);
  // The RP2040 pad's own Schmitt trigger: on by default, but set it explicitly
  // so no pad reconfiguration can ever drop it -- it is the first line of
  // defence against threshold re-crossing on the slow RDATA rise (see the
  // deglitch note on the capture program).
  gpio_set_input_hysteresis_enabled(read_pin, true);
  gpio_set_input_hysteresis_enabled(index_pin, true);

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
// overwritten, out-of-sequence samples -- garbage deltas with no flag, which
// is exactly the HD write-splice signature (RXSTALL clear, ring flag clear).
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
  // Clear the PIO's sticky RX-FIFO-overflow flag so it reflects only THIS
  // capture. If it is set at the end, the DMA (or, before it, the CPU) could
  // not drain the RX FIFO fast enough and the PIO dropped samples at the
  // source -- which no amount of ring buffering downstream can recover, and
  // which is what a starved capture pipeline looks like as opposed to a
  // corrupted RDATA signal (the PIO captures that one just fine, flag clear).
  // RXSTALL is the RP2040 sticky flag for "the SM tried to push to a full RX
  // FIFO" -- i.e. a flux sample was dropped because nothing drained fast
  // enough. Write-1-to-clear so it reflects only this capture.
  g_reader.pio->fdebug = (1u << (PIO_FDEBUG_RXSTALL_LSB + g_reader.sm));
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
  return g_reader.half || (g_ring_dma >= 0 && ring_unread() != 0);
}

static uint16_t read_fifo() {
  if (g_reader.half) {
    uint16_t result = g_reader.half;
    g_reader.half = 0;
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
      continue;
    }

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

bool rp2040_flux_capture_pio_overflow(void) {
  return g_reader.pio &&
         (g_reader.pio->fdebug & (1u << (PIO_FDEBUG_RXSTALL_LSB + g_reader.sm)));
}

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

static bool init_write(int wrdata_pin, bool is_apple2) {
  if (g_writer.pio) {
    return true;
  }
  const pio_program_t *program =
      is_apple2 ? &fluxwrite_apple2_struct : &fluxwrite_struct;
  g_writer.program = program;

  if (!allocate_pio_set_program(&g_writer, program)) {
    return false;
  }

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

static void write_foreground(int index_pin, int wrgate_pin, uint8_t *pulses,
                             uint8_t *pulse_end, bool store_greaseweazle,
                             bool use_index) {
  if (use_index) {
    while (!gpio_get(index_pin)) { /* not during an index pulse */
    }
    while (gpio_get(index_pin)) { /* wait for falling edge of index */
    }
  }

  gpio_set_dir(wrgate_pin, GPIO_OUT);
  gpio_put(wrgate_pin, 0); // WGATE active low

  uint32_t irq = save_and_disable_interrupts();
  pio_sm_set_enabled(g_writer.pio, g_writer.sm, false);
  pio_sm_clear_fifos(g_writer.pio, g_writer.sm);
  pio_sm_exec(g_writer.pio, g_writer.sm, g_writer.offset);
  while (!pio_sm_is_tx_fifo_full(g_writer.pio, g_writer.sm)) {
    unsigned value = greaseunpack(&pulses, pulse_end, store_greaseweazle);
    value = (value < OVERHEAD) ? 1 : value - OVERHEAD;
    pio_sm_put_blocking(g_writer.pio, g_writer.sm, value);
  }
  pio_sm_set_enabled(g_writer.pio, g_writer.sm, true);

  bool old_index_state = false;
  while (pulses != pulse_end) {
    bool index_state = gpio_get(index_pin);
    if (old_index_state && !index_state) {
      break; // falling edge of index pin
    }
    while (!pio_sm_is_tx_fifo_full(g_writer.pio, g_writer.sm)) {
      unsigned value = greaseunpack(&pulses, pulse_end, store_greaseweazle);
      value = (value < OVERHEAD) ? 1 : value - OVERHEAD;
      pio_sm_put_blocking(g_writer.pio, g_writer.sm, value);
    }
    old_index_state = index_state;
  }
  restore_interrupts(irq);

  pio_sm_set_enabled(g_writer.pio, g_writer.sm, false);
  gpio_set_dir(wrgate_pin, GPIO_IN); // release WGATE (external/internal pull-up)
  gpio_pull_up(wrgate_pin);
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
  return init_write(wrdata_pin, false);
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
  // the tail short there truncates real data. The longest single pulse is ~4 us
  // at DD; 32 us covers it with margin.
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
                       bool store_greaseweazle, bool is_apple2,
                       bool use_index) {
  if (!init_write(wrdata_pin, is_apple2)) {
    return false;
  }
  write_foreground(index_pin, wrgate_pin, pulses, pulse_end, store_greaseweazle,
                   use_index);
  free_write();
  return true;
}
