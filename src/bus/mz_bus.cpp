#include "mz_bus.hpp"
#include "board_bus.h"
#include "fdc.hpp"

#include "bus.pio.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/multicore.h"

// Bus servicing on pio1; the flux engine (arch_rp2) takes pio0 first, so core1
// (bus) and core0 (flux/disk) never contend on the same PIO peripheral.
#define BUS_PIO pio1
#define SM_READ 0
#define SM_WRITE 1

BusHelloState g_bus_hello;
BusDiag g_bus_diag;

static void init_bus_gpio(void) {
  // Shared C0..C7 — inputs by default.
  for (int i = 0; i < DATA_BUS_COUNT; ++i) {
    gpio_init(DATA_BUS_BASE + i);
    gpio_set_dir(DATA_BUS_BASE + i, GPIO_IN);
    gpio_set_slew_rate(DATA_BUS_BASE + i, GPIO_SLEW_RATE_FAST);
  }
  // Control inputs (active low), pulled up.
  const int in_pins[] = {IORQ_PIN, RD_PIN, WR_PIN};
  for (int p : in_pins) {
    gpio_init(p);
    gpio_set_dir(p, GPIO_IN);
    gpio_pull_up(p);
  }
  // RESET is the exception: active HIGH (the board's pin table names it `RESET`,
  // not `~RESET`). Pull DOWN, so a board on the bench with no MZ-800 attached
  // reads "not in reset" instead of sitting in permanent reset. See
  // fdc_poll_bus_reset().
  gpio_init(RESET_PIN);
  gpio_set_dir(RESET_PIN, GPIO_IN);
  gpio_pull_down(RESET_PIN);
  // EXWAIT / INT — released (input; bus pull-up). We only drive low to assert.
  gpio_init(EXWAIT_PIN);
  gpio_set_dir(EXWAIT_PIN, GPIO_IN);
  gpio_set_slew_rate(EXWAIT_PIN, GPIO_SLEW_RATE_FAST);
  gpio_init(INT_PIN);
  gpio_put(INT_PIN, 0); // park the output latch low: asserting = enabling the
                        // driver (see assert_int_line()), releasing = Hi-Z
  gpio_set_dir(INT_PIN, GPIO_IN);
  gpio_set_slew_rate(INT_PIN, GPIO_SLEW_RATE_FAST);

  // Buffer gates EN0/EN1/RD1: preset OFF / read direction as plain outputs,
  // then hand all three to the bus PIO (pio1) side-set. From here on core1
  // never touches them (bus.pio owns the gate sequencing; only the bench
  // self-tests take them back).
  const int gate_pins[] = {EN0_PIN, EN1_PIN, RD1_PIN};
  for (int p : gate_pins) {
    gpio_init(p);
    gpio_put(p, 1); // EN0=1 addr buffer off, EN1=1 data buffer off, RD1=1 read
    gpio_set_dir(p, GPIO_OUT);
    gpio_set_slew_rate(p, GPIO_SLEW_RATE_FAST);
    gpio_set_function(p, GPIO_FUNC_PIO1);
  }
}

static uint g_off_read, g_off_write;

static void setup_bus_pio(void) {
  g_off_read = pio_add_program(BUS_PIO, &bus_read_program);
  g_off_write = pio_add_program(BUS_PIO, &bus_write_program);
  bus_read_init(BUS_PIO, SM_READ, g_off_read);
  bus_write_init(BUS_PIO, SM_WRITE, g_off_write);
}

// Flush stale captures and restart both bus SMs from their entry points. The
// write SM pushes two words per cycle, so a half-captured transaction left
// behind by a bench test that halted the SMs would desync the pop pairing for
// good; clearing the FIFOs and restarting puts both sides back in step.
static void restart_bus_sms(void) {
  pio_sm_set_enabled(BUS_PIO, SM_READ, false);
  pio_sm_set_enabled(BUS_PIO, SM_WRITE, false);
  pio_sm_clear_fifos(BUS_PIO, SM_READ);
  pio_sm_clear_fifos(BUS_PIO, SM_WRITE);
  pio_sm_restart(BUS_PIO, SM_READ);
  pio_sm_restart(BUS_PIO, SM_WRITE);
  pio_sm_exec(BUS_PIO, SM_READ, pio_encode_jmp(g_off_read));
  pio_sm_exec(BUS_PIO, SM_WRITE, pio_encode_jmp(g_off_write));
  pio_sm_set_enabled(BUS_PIO, SM_READ, true);
  pio_sm_set_enabled(BUS_PIO, SM_WRITE, true);
}

static inline bool port_is_ours(uint8_t addr) {
  return addr >= FDC_PORT_BASE && addr <= FDC_PORT_LAST;
}

// Listen loop. The bus PIO owns every gate: the write SM delivers the port
// address AND the PIO-captured data byte (two FIFO words), the read SM delivers
// the address and waits for a go-word before it turns the data buffer toward
// the bus. core1 therefore never samples C0..C7 against the end of a cycle and
// never drives the MZ-800 data bus for a port it does not own.
//
// Hard real-time on reads only: from the PIO push, the go-word and EXWAIT must
// land before the Z80 samples /WAIT. Everything that is not strictly needed to
// hit that window -- bit-activity accumulators, per-port counters -- runs AFTER
// the bus access, never before it.
RAM_FUNC static void listen_loop(void) {
  while (true) {
    // Drain BOTH FIFOs each pass. Under a tight OUT/IN loop, handling only one
    // entry per FIFO per iteration can still accumulate backlog and eventually
    // overflow one FIFO.
    bool first = true;
    while (!pio_sm_is_rx_fifo_empty(BUS_PIO, SM_WRITE)) {
      uint8_t addr = pio_sm_get(BUS_PIO, SM_WRITE) >> 24;
      // Second word: the data byte, sampled by the PIO ~80 ns into the cycle.
      // Valid whether or not core1 is late -- no cycle-end race any more.
      uint8_t raw = pio_sm_get_blocking(BUS_PIO, SM_WRITE) >> 24;
      if (!first)
        g_bus_diag.wr_backlog++;
      first = false;
      if (port_is_ours(addr)) {
        uint8_t idx = addr - FDC_PORT_BASE;
        // Stretch the cycle so the register write has landed before the ROM's
        // next access (a STATUS poll typically follows a command within a few
        // T-states). If the cycle is already over this is a no-op; the byte is
        // ours regardless - count it as telemetry only.
        set_exwait();
        if (iorq_is_high())
          g_bus_diag.wr_late_at_pop++;
        fdc_reg_write(addr, raw); // FDC applies per-port inversion + decode
        release_exwait();
        // --- off the critical path from here ---
        g_bus_hello.dat_low |= (uint8_t)~raw; // write-data bit activity
        g_bus_hello.dat_high |= raw;
        g_bus_hello.last_raw[idx] = raw; // raw, un-inverted
        g_bus_hello.wr[idx]++;
      }
      g_bus_hello.addr_low |= (uint8_t)~addr; // bit-activity over ALL writes
      g_bus_hello.addr_high |= addr;
    }

    first = true;
    while (!pio_sm_is_rx_fifo_empty(BUS_PIO, SM_READ)) {
      uint8_t addr = pio_sm_get(BUS_PIO, SM_READ) >> 24;
      if (!first)
        g_bus_diag.rd_backlog++;
      first = false;
      if (port_is_ours(addr)) {
        uint8_t idx = addr - FDC_PORT_BASE;
        set_exwait();
        // If the cycle is already over the read SM has left its verdict loop:
        // a go-word now would only sit in the FIFO (the SM discards it at its
        // next wrap, but there is nothing to serve). Count it and move on.
        if (iorq_is_high()) {
          release_exwait();
          g_bus_diag.rd_late_at_pop++;
          g_bus_hello.addr_low |= (uint8_t)~addr;
          g_bus_hello.addr_high |= addr;
          continue;
        }
        // Go-word: the read SM turns U2 toward the bus (Pico C -> bus D).
        pio_sm_put(BUS_PIO, SM_READ, 1);
        uint8_t bus_byte = fdc_reg_read(addr); // inversion handled by the FDC
        write_data_bus(bus_byte);
        acquire_data_bus_for_writing();
        bus_settle();     // U2 propagation before we free the Z80
        release_exwait(); // data is valid on the bus before we free the Z80
        while (!iorq_is_high()) { /* hold until the Z80 finishes the cycle */
        }
        // The SM blocks U2 on IORQ high and holds the address buffer off for
        // the guard delay; drop our own drive inside that window (see bus.pio).
        release_data_bus();
        // --- off the critical path from here ---
        g_bus_hello.reg[idx] = bus_byte;
        g_bus_hello.rd[idx]++;
      }
      g_bus_hello.addr_low |= (uint8_t)~addr; // bit-activity over ALL reads
      g_bus_hello.addr_high |= addr;
    }
  }
}

void bus_diag_reset(void) {
  for (int k = 0; k < 8; ++k) {
    g_bus_hello.reg[k] = 0;
    g_bus_hello.last_raw[k] = 0;
    g_bus_hello.wr[k] = 0;
    g_bus_hello.rd[k] = 0;
  }
  g_bus_hello.addr_low = 0;
  g_bus_hello.addr_high = 0;
  g_bus_hello.dat_low = 0;
  g_bus_hello.dat_high = 0;
  g_bus_diag.wr_late_at_pop = 0;
  g_bus_diag.wr_late_at_sample = 0; // no longer incremented: the PIO samples
  g_bus_diag.rd_late_at_pop = 0;
  g_bus_diag.wr_backlog = 0;
  g_bus_diag.rd_backlog = 0;
}

static void core1_entry(void) {
  // Arm the bus-RESET interrupt from CORE1, so it is delivered here rather than
  // on core0 -- core0 spends up to ~250 ms at a time inside a flux capture with
  // interrupts disabled and would service it far too late.
  //
  // An interrupt, not a poll. Polling this line from the listen loop cost bus
  // cycles: the loop has ~560 ns to answer a Z80 I/O cycle, and measured on
  // hardware, even a couple of instructions per pass was enough to make the
  // machine report "no floppy driver detected". An edge interrupt costs
  // *nothing* until the line actually moves, and when it does move the machine
  // is being reset and is not issuing I/O anyway.
  fdc_bus_reset_irq_init();
  listen_loop();
}

// Bench pin-READ test: sample the live C0..C7 inputs (bit0 = GPIO0 = C0).
uint8_t bus_read_pins(bool isolate) {
  if (isolate) {
    // Remove the PIO/core1 from the picture: stop the SMs and make C0..C7 plain
    // SIO inputs with no pull, so an external jumper fully owns each line.
    pio_sm_set_enabled(BUS_PIO, SM_READ, false);
    pio_sm_set_enabled(BUS_PIO, SM_WRITE, false);
    // Critically: take the buffer gates back from the PIO and turn BOTH '245s
    // OFF. Otherwise the PIO's last side-set may leave the address buffer (U1)
    // enabled, driving C0..C7 (24 mA) and overriding any jumper on the line.
    const int gate_pins[] = {EN0_PIN, EN1_PIN, RD1_PIN};
    for (int p : gate_pins) {
      gpio_set_function(p, GPIO_FUNC_SIO);
      gpio_set_dir(p, GPIO_OUT);
      gpio_put(p, 1); // EN0=1 addr buffer off, EN1=1 data buffer off
    }
    for (int i = 0; i < DATA_BUS_COUNT; ++i) {
      gpio_set_function(DATA_BUS_BASE + i, GPIO_FUNC_SIO);
      gpio_set_dir(DATA_BUS_BASE + i, GPIO_IN);
      gpio_disable_pulls(DATA_BUS_BASE + i);
    }
  }
  return read_data_bus();
}

// Bench pin self-test: characterise each C0..C7 pad with its own internal pull.
void bus_pin_selftest(uint8_t *pu, uint8_t *pd) {
  // Force everything off the bus first (buffers OFF, SMs halted, pads as inputs).
  bus_read_pins(true);
  volatile uint32_t d; // ~ms settle for the ~50k internal pull vs pin C

  for (int i = 0; i < DATA_BUS_COUNT; ++i)
    gpio_pull_up(DATA_BUS_BASE + i);
  for (d = 0; d < 200000; ++d) {}
  *pu = read_data_bus();

  for (int i = 0; i < DATA_BUS_COUNT; ++i)
    gpio_pull_down(DATA_BUS_BASE + i);
  for (d = 0; d < 200000; ++d) {}
  *pd = read_data_bus();

  for (int i = 0; i < DATA_BUS_COUNT; ++i)
    gpio_disable_pulls(DATA_BUS_BASE + i);
}

// Bench pin-drive test. The gate pins are normally PIO-owned; take all three
// back as SIO (and halt the bus SMs) so we can statically drive/measure C0..C7.
void bus_drive_test(uint8_t value, uint8_t mode) {
  const int gate_pins[] = {EN0_PIN, EN1_PIN, RD1_PIN};

  if (mode == 0) { // release: restore normal bus operation
    release_data_bus();
    // All three gates go back to the bus PIO side-set (parked off / read dir),
    // and the SMs restart from a clean slate: a halted write SM may have left
    // half a transaction in its FIFO.
    for (int p : gate_pins) {
      gpio_put(p, 1);
      gpio_set_function(p, GPIO_FUNC_PIO1);
    }
    restart_bus_sms();
    return;
  }

  pio_sm_set_enabled(BUS_PIO, SM_READ, false);
  pio_sm_set_enabled(BUS_PIO, SM_WRITE, false);
  for (int p : gate_pins) { // take the gates as plain GPIO
    gpio_set_function(p, GPIO_FUNC_SIO);
    gpio_set_dir(p, GPIO_OUT);
    gpio_put(p, 1); // EN0 off (addr buffer off); EN1 off; RD1 read
  }
  write_data_bus(value);
  acquire_data_bus_for_writing(); // Pico C0..C7 drive `value`
  if (mode >= 2) {                // push through the data '245 to the bus
    gpio_put(RD1_PIN, 0);         // direction Pico->bus
    gpio_put(EN1_PIN, 0);         // data buffer on
  }
}

void bus_init(void) {
  // STATUS starts as NOT READY (the FDC reports NOT READY until the ROM commands
  // the motor on), so the MZ-800 boot ROM sees "no drive" rather than hanging.
  init_bus_gpio();
  setup_bus_pio();
  multicore_launch_core1(core1_entry);
}
