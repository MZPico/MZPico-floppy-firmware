// A minimal MZ-800 for running MZ-800 programs against the firmware's FDC:
// a Z80 at 3.546895 MHz, 64 KB of RAM, the FDC ports D8h-DFh wired to
// fdc_reg_read/fdc_reg_write (= core1), everything else open bus.
//
// There is no monitor ROM (it is Sharp's and is not redistributable): the
// handful of MZ-700/800 monitor entry points that test programs use are
// trapped and implemented here --
//   0012h PRNT  print the character in A (0Dh = newline)
//   0015h MSG   print the 0Dh-terminated string at DE
//   0006h LETNL newline        0009h NL  newline unless at column 0
//   001Bh GETKY next scripted key in A, 0 if none
//   0000h / 00ADh  monitor (re)entry: the program has finished
// A program that needs more of the monitor stops with an error naming the
// call, rather than running off into empty memory.
#pragma once
#include <cstdint>
#include <deque>
#include <string>

#include "sim_core.hpp"

extern "C" {
#include "z80.h"
}

namespace mz800 {

class Machine : public sim::Host {
public:
  Machine();
  bool load_mzf(const std::string &path, std::string *err);
  void keys(const std::string &k) {
    for (char c : k)
      keys_.push_back((uint8_t)c);
  }
  uint64_t run(uint64_t now) override;

  bool finished() const { return finished_; }
  const std::string &error() const { return error_; }
  const std::string &screen() const { return screen_; }
  bool echo = true;

  uint8_t mem[65536];

private:
  void trap();
  void ret();
  void putch(uint8_t c);

  z80 cpu_;
  uint64_t cycles_base_ = 0; // virtual tick at cpu cycle 0
  std::deque<uint8_t> keys_;
  std::string screen_, error_;
  bool finished_ = false;
  int column_ = 0;
  uint32_t idle_getky_ = 0;
  static constexpr uint32_t KEY_HOLD = 40, KEY_GAP = 2000; // GETKY polls
  uint32_t key_phase_ = KEY_HOLD; // start released: keys come after a prompt
};

} // namespace mz800
