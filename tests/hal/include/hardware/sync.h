// Host-simulation stand-in for hardware/sync.h. There is no preemption in the
// simulation (see tests/sim/sim_core.hpp), so masking interrupts is a no-op.
#pragma once
#include <stdint.h>
static inline uint32_t save_and_disable_interrupts(void) { return 0; }
static inline void restore_interrupts(uint32_t) {}
static inline void __dmb(void) {}
static inline void tight_loop_contents(void) {}
