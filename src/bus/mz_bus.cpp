#include "mz_bus.hpp"
#include "board_bus.h"

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

static void init_bus_gpio(void) {
  // Shared C0..C7 — inputs by default.
  for (int i = 0; i < DATA_BUS_COUNT; ++i) {
    gpio_init(DATA_BUS_BASE + i);
    gpio_set_dir(DATA_BUS_BASE + i, GPIO_IN);
    gpio_set_slew_rate(DATA_BUS_BASE + i, GPIO_SLEW_RATE_FAST);
  }
  // Control inputs (active low), pulled up.
  const int in_pins[] = {IORQ_PIN, RD_PIN, WR_PIN, RESET_PIN};
  for (int p : in_pins) {
    gpio_init(p);
    gpio_set_dir(p, GPIO_IN);
    gpio_pull_up(p);
  }
  // EXWAIT / INT — released (input; bus pull-up). We only drive low to assert.
  gpio_init(EXWAIT_PIN);
  gpio_set_dir(EXWAIT_PIN, GPIO_IN);
  gpio_set_slew_rate(EXWAIT_PIN, GPIO_SLEW_RATE_FAST);
  gpio_init(INT_PIN);
  gpio_set_dir(INT_PIN, GPIO_IN);

  // Data buffer gate (core1-controlled, NOT the PIO): start OFF (EN1=1),
  // direction=read (RD1=1) so we never touch the MZ-800 data bus until we
  // decide to, and only for ports we own.
  gpio_init(EN1_PIN);
  gpio_put(EN1_PIN, 1);
  gpio_set_dir(EN1_PIN, GPIO_OUT);
  gpio_set_slew_rate(EN1_PIN, GPIO_SLEW_RATE_FAST);
  gpio_init(RD1_PIN);
  gpio_put(RD1_PIN, 1);
  gpio_set_dir(RD1_PIN, GPIO_OUT);
  gpio_set_slew_rate(RD1_PIN, GPIO_SLEW_RATE_FAST);

  // EN0 (address buffer /OE) is driven by the bus PIO (pio1) side-set. Preset
  // high (buffer off) before handing it to the PIO.
  gpio_init(EN0_PIN);
  gpio_put(EN0_PIN, 1);
  gpio_set_dir(EN0_PIN, GPIO_OUT);
  gpio_set_slew_rate(EN0_PIN, GPIO_SLEW_RATE_FAST);
  gpio_set_function(EN0_PIN, GPIO_FUNC_PIO1);
}

static void setup_bus_pio(void) {
  uint off_read = pio_add_program(BUS_PIO, &bus_read_program);
  uint off_write = pio_add_program(BUS_PIO, &bus_write_program);
  bus_read_init(BUS_PIO, SM_READ, off_read);
  bus_write_init(BUS_PIO, SM_WRITE, off_write);
}

static inline bool port_is_ours(uint8_t addr) {
  return addr >= FDC_PORT_BASE && addr <= 0xDF;
}

// Listen loop: the PIO side-set only exposes the port address (EN0/address
// buffer). core1 owns the data buffer (EN1/RD1) and enables it ONLY for ports we
// own, so we never drive or load the MZ-800 data bus on other devices' I/O.
RAM_FUNC static void listen_loop(void) {
  while (true) {
    if (!pio_sm_is_rx_fifo_empty(BUS_PIO, SM_READ)) {
      uint8_t addr = pio_sm_get(BUS_PIO, SM_READ) >> 24;
      g_bus_hello.addr_low |= (uint8_t)~addr; // bit-activity over ALL reads
      g_bus_hello.addr_high |= addr;
      if (port_is_ours(addr)) {
        uint8_t idx = addr - FDC_PORT_BASE;
        uint8_t v = g_bus_hello.reg[idx];
        set_exwait();
        write_data_bus((uint8_t)~v); // ROM CPLs on read
        acquire_data_bus_for_writing();
        data_dir_drive();
        data_buffer_on();
        for (volatile int k = 0; k < 4; ++k) { /* let the '245 output settle */
        }
        release_exwait(); // data is valid on the bus before we free the Z80
        while (!iorq_is_high()) { /* hold until the Z80 finishes the cycle */
        }
        data_buffer_off();
        release_data_bus();
        data_dir_read();
        g_bus_hello.rd[idx]++;
      }
    } else if (!pio_sm_is_rx_fifo_empty(BUS_PIO, SM_WRITE)) {
      uint8_t addr = pio_sm_get(BUS_PIO, SM_WRITE) >> 24;
      g_bus_hello.addr_low |= (uint8_t)~addr; // bit-activity over ALL writes
      g_bus_hello.addr_high |= addr;
      if (port_is_ours(addr)) {
        uint8_t idx = addr - FDC_PORT_BASE;
        set_exwait();
        data_dir_read();
        data_buffer_on();
        for (volatile int k = 0; k < 4; ++k) { /* buffer/settle delay */
        }
        uint8_t raw = read_data_bus();
        data_buffer_off();
        release_exwait();
        g_bus_hello.dat_low |= (uint8_t)~raw; // write-data bit activity
        g_bus_hello.dat_high |= raw;
        g_bus_hello.last_raw[idx] = raw;      // raw, un-inverted
        g_bus_hello.reg[idx] = (uint8_t)~raw; // ROM CPLs on write
        g_bus_hello.wr[idx]++;
      }
    }
  }
}

static void core1_entry(void) { listen_loop(); }

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
    // EN1/RD1 are core1/SIO-owned: leave them as outputs, data buffer OFF.
    gpio_put(EN1_PIN, 1);
    gpio_put(RD1_PIN, 1);
    // EN0 goes back to the bus PIO side-set.
    gpio_put(EN0_PIN, 1);
    gpio_set_function(EN0_PIN, GPIO_FUNC_PIO1);
    pio_sm_set_enabled(BUS_PIO, SM_READ, true);
    pio_sm_set_enabled(BUS_PIO, SM_WRITE, true);
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
  // Default STATUS (0xD8) = NOT READY (bit7) so the MZ-800 boot ROM sees "no
  // drive" and boots to a prompt rather than hanging on a bogus FDC.
  g_bus_hello.reg[0] = 0x80;

  init_bus_gpio();
  setup_bus_pio();
  multicore_launch_core1(core1_entry);
}
