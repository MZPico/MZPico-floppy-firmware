#include "machine.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

#include "fdc.hpp"
#include "sim_drive.hpp"

namespace mz800 {

static constexpr double CPU_HZ = 3546895.0;

static uint8_t rd(void *u, uint16_t a) { return static_cast<Machine *>(u)->mem[a]; }
static void wr(void *u, uint16_t a, uint8_t v) {
  if (a >= 0x1000) // 0000h-0FFFh is the monitor ROM: writes are lost
    static_cast<Machine *>(u)->mem[a] = v;
}
static uint8_t port_in(z80 *, uint8_t port) {
  if (port >= 0xD8 && port <= 0xDF)
    return fdc_reg_read(port);
  return 0xFF;
}
static void port_out(z80 *, uint8_t port, uint8_t v) {
  if (port >= 0xD8 && port <= 0xDF)
    fdc_reg_write(port, v);
}

Machine::Machine() {
  std::memset(mem, 0, sizeof mem);
  z80_init(&cpu_);
  cpu_.read_byte = rd;
  cpu_.write_byte = wr;
  cpu_.port_in = port_in;
  cpu_.port_out = port_out;
  cpu_.userdata = this;
  // Every monitor entry point holds a RET, so a program that calls one we do
  // not trap at least returns; the trap list below catches the ones we know.
  for (int a = 0; a < 0x1000; ++a)
    mem[a] = 0xC9;
}

bool Machine::load_mzf(const std::string &path, std::string *err) {
  std::ifstream f(path, std::ios::binary);
  std::vector<uint8_t> b((std::istreambuf_iterator<char>(f)), {});
  if (b.size() < 128) {
    *err = "not an MZF file: " + path;
    return false;
  }
  uint16_t size = (uint16_t)(b[18] | (b[19] << 8));
  uint16_t load = (uint16_t)(b[20] | (b[21] << 8));
  uint16_t exec = (uint16_t)(b[22] | (b[23] << 8));
  if (b.size() < 128u + size) {
    *err = "MZF truncated";
    return false;
  }
  std::memcpy(&mem[load], &b[128], size);
  cpu_.pc = exec;
  cpu_.sp = 0x10F0; // the monitor's stack
  // Monitor "L"+jump leaves a return address of 00ADh (monitor warm start).
  cpu_.sp -= 2;
  mem[cpu_.sp] = 0xAD;
  mem[cpu_.sp + 1] = 0x00;
  cycles_base_ = sim::now();
  return true;
}

void Machine::putch(uint8_t c) {
  if (c == 0x0D) {
    screen_ += '\n';
    column_ = 0;
  } else if (c >= 0x20 && c < 0x7F) {
    screen_ += (char)c;
    ++column_;
  } else {
    return; // cursor control etc.
  }
  if (echo) {
    std::fputc(c == 0x0D ? '\n' : c, stdout);
    if (c == 0x0D)
      std::fflush(stdout);
  }
}

void Machine::ret() {
  cpu_.pc = (uint16_t)(mem[cpu_.sp] | (mem[(uint16_t)(cpu_.sp + 1)] << 8));
  cpu_.sp += 2;
  cpu_.cyc += 10;
}

void Machine::trap() {
  switch (cpu_.pc) {
  case 0x0000:
  case 0x00AD:
    finished_ = true;
    return;
  case 0x0012:
    putch(cpu_.a);
    cpu_.cyc += 400;
    break;
  case 0x0015: {
    uint16_t p = (uint16_t)((cpu_.d << 8) | cpu_.e);
    for (int n = 0; n < 256 && mem[p] != 0x0D; ++n, ++p)
      putch(mem[p]);
    cpu_.cyc += 2000;
    break;
  }
  case 0x0006:
    putch(0x0D);
    break;
  case 0x0009:
    if (column_)
      putch(0x0D);
    break;
  case 0x001B:
    if (!keys_.empty()) {
      cpu_.a = keys_.front();
      keys_.pop_front();
    } else {
      cpu_.a = 0;
      if (++idle_getky_ > 200000) {
        error_ = "program waits for a key the script does not provide";
        finished_ = true;
        return;
      }
    }
    cpu_.cyc += 300;
    break;
  default: {
    char b[80];
    std::snprintf(b, sizeof b, "program called monitor address %04Xh, not emulated",
                  cpu_.pc);
    error_ = b;
    finished_ = true;
    return;
  }
  }
  ret();
}

uint64_t Machine::run(uint64_t now) {
  if (finished_)
    return UINT64_MAX;
  // Execute until the Z80 is ahead of virtual time, then come back when its
  // clock is due again. The FDC side (core0) interleaves at ~1 us.
  while (true) {
    uint64_t t = cycles_base_ +
                 (uint64_t)((double)cpu_.cyc * sim::TICKS_PER_US * 1e6 / CPU_HZ);
    if (t > now)
      return t;
    if (cpu_.pc < 0x1000) {
      trap();
      if (finished_)
        return UINT64_MAX;
      continue;
    }
    // ~INT: GPIO12 output-enabled = line pulled low (board_bus.h).
    if (sim::pin_oe(12) && cpu_.iff1)
      z80_gen_int(&cpu_, 0xFF);
    z80_step(&cpu_);
  }
}

} // namespace mz800
