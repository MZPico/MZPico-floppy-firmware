// Host-simulation stand-in for hardware/structs/sio.h. The firmware writes the
// SIO set/clear aliases directly (board_bus.h); here each member is a proxy
// whose assignment applies the same set/clear semantics to the simulated pins.
#pragma once
#include <stdint.h>

void sim_sio_write(int kind, uint32_t mask);
uint32_t sim_sio_gpio_in(void);

struct sim_sio_wr {
  int kind;
  sim_sio_wr &operator=(uint32_t mask) {
    sim_sio_write(kind, mask);
    return *this;
  }
};
struct sim_sio_in {
  operator uint32_t() const { return sim_sio_gpio_in(); }
};
struct sim_sio_hw_t {
  sim_sio_in gpio_in;
  sim_sio_wr gpio_set{0}, gpio_clr{1}, gpio_oe_set{2}, gpio_oe_clr{3};
};
extern sim_sio_hw_t *const sio_hw;
