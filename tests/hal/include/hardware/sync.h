// Host-simulation stand-in for hardware/sync.h. There is no preemption in the
// simulation (see tests/sim/sim_core.hpp), so masking interrupts is a no-op.
#pragma once
#include <stdint.h>
static inline uint32_t save_and_disable_interrupts(void) { return 0; }
static inline void restore_interrupts(uint32_t) {}
static inline void __dmb(void) {}
static inline void tight_loop_contents(void) {}

// Spin locks (#61). The simulation runs core0 as a coroutine and core1's
// register handlers synchronously, so a real spin would never end. The model
// instead: while core0 holds the lock, a host register write is DEFERRED until
// the unlock (tests/sim/board.cpp) -- what a core1 command write blocking on
// the lock does on hardware. sim_publock_model(false) turns the lock into a
// no-op, to show that a race test really exercises the window.
typedef struct spin_lock_sim { volatile int held; } spin_lock_t;
spin_lock_t *spin_lock_instance(unsigned num);
int spin_lock_claim_unused(bool required);
void spin_lock_unsafe_blocking(spin_lock_t *lock);
void spin_unlock_unsafe(spin_lock_t *lock);
bool sim_publock_held(void);
void sim_publock_model(bool on);
// A named point between a check and the publish it guards; a test can run
// host register writes "at exactly this instant" (sim::set_race_point).
void sim_race_point(int id);
