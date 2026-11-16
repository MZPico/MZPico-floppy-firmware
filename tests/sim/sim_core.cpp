#include "sim_core.hpp"

#include <cstdio>
#include <cstdlib>
#include <ucontext.h>
#include <vector>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/structs/sio.h"
#include "pico/time.h"

namespace sim {
namespace {

constexpr uint64_t NEVER = UINT64_MAX;
constexpr uint32_t QUANTUM = TICKS_PER_US; // core0 yields at least this often
constexpr size_t STACK = 1u << 20;

struct Coro {
  ucontext_t ctx;
  std::vector<char> stack;
  bool done = false;
};

ucontext_t g_main;
Coro g_core0, g_script;
enum Ctx { MAIN, CORE0, SCRIPT } g_cur = MAIN;

uint64_t g_clock = 0;       // global virtual time
uint64_t g_core0_wake = 0;  // core0 runs again at this time
uint64_t g_core0_debt = 0;  // CPU time core0 has used since it last yielded
uint64_t g_script_wake = 0;
void (*g_core0_main)() = nullptr;
const std::function<void()> *g_script_fn = nullptr;
const char *g_hang = nullptr;
PinBoard *g_board = nullptr;

bool g_out[32], g_dir_out[32], g_oe[32], g_forced[32], g_forced_level[32];

void core0_trampoline() {
  if (g_core0_main)
    g_core0_main();
  g_core0.done = true;
  g_cur = MAIN;
  swapcontext(&g_core0.ctx, &g_main);
}

void script_trampoline() {
  (*g_script_fn)();
  g_script.done = true;
  g_cur = MAIN;
  swapcontext(&g_script.ctx, &g_main);
}

void coro_init(Coro &c, void (*fn)()) {
  c.stack.assign(STACK, 0);
  c.done = false;
  getcontext(&c.ctx);
  c.ctx.uc_stack.ss_sp = c.stack.data();
  c.ctx.uc_stack.ss_size = c.stack.size();
  c.ctx.uc_link = nullptr;
  makecontext(&c.ctx, fn, 0);
}

void core0_yield_to_main() {
  g_cur = MAIN;
  swapcontext(&g_core0.ctx, &g_main);
  g_cur = CORE0;
}

// Scheduler: always advance to the earliest pending wake.
bool schedule(Host *host, const std::function<bool()> &done, uint64_t limit) {
  uint64_t host_wake = 0;
  bool host_finished = false;
  static bool core0_started = false;
  if (!core0_started) { // core0 boots once and keeps running across runs
    coro_init(g_core0, core0_trampoline);
    g_core0_wake = g_clock;
    g_core0_debt = 0;
    core0_started = true;
  }
  while (true) {
    if (g_hang)
      return false;
    if (done && done())
      return true;
    if (host_finished)
      return true;
    uint64_t t = g_core0.done ? host_wake : std::min(g_core0_wake, host_wake);
    if (t > g_clock)
      g_clock = t;
    if (g_clock >= limit) {
      std::fprintf(stderr, "sim: virtual time limit reached (%.3f s)\n",
                   (double)g_clock / ms(1000));
      return false;
    }
    if (!g_core0.done && g_core0_wake <= g_clock) {
      g_cur = CORE0;
      swapcontext(&g_main, &g_core0.ctx);
      g_cur = MAIN;
      continue; // core0 set its next wake before yielding
    }
    if (host_wake <= g_clock) {
      host_wake = host->run(g_clock);
      if (host_wake == NEVER)
        host_finished = true;
    }
  }
}

struct ScriptHost : Host {
  uint64_t run(uint64_t) override {
    if (g_script.done)
      return NEVER;
    g_cur = SCRIPT;
    swapcontext(&g_main, &g_script.ctx);
    g_cur = MAIN;
    return g_script.done ? NEVER : g_script_wake;
  }
};

} // namespace

uint64_t now() { return g_cur == CORE0 ? g_clock + g_core0_debt : g_clock; }
bool in_core0() { return g_cur == CORE0; }

void core0_cost(uint32_t ticks) {
  if (g_cur != CORE0)
    return;
  g_core0_debt += ticks;
  if (g_core0_debt >= QUANTUM) {
    g_core0_wake = g_clock + g_core0_debt;
    g_core0_debt = 0;
    core0_yield_to_main();
  }
}

void core0_sleep_until(uint64_t t) {
  if (g_cur != CORE0)
    return;
  uint64_t n = g_clock + g_core0_debt;
  g_core0_debt = 0;
  g_core0_wake = t > n ? t : n;
  core0_yield_to_main();
}

void wait_ticks(uint64_t t) {
  if (g_cur != SCRIPT) {
    std::fprintf(stderr, "sim: wait_ticks outside a script\n");
    std::abort();
  }
  g_script_wake = g_clock + (t ? t : 1);
  g_cur = MAIN;
  swapcontext(&g_script.ctx, &g_main);
  g_cur = SCRIPT;
}

bool run_script(const std::function<void()> &fn, uint64_t limit_ticks) {
  g_script_fn = &fn;
  coro_init(g_script, script_trampoline);
  g_script_wake = g_clock;
  ScriptHost h;
  bool ok = schedule(&h, nullptr, g_clock + limit_ticks);
  return ok && g_script.done;
}

bool run_host(Host *host, const std::function<bool()> &done,
              uint64_t limit_ticks) {
  return schedule(host, done, g_clock + limit_ticks);
}

void set_core0_main(void (*fn)()) { g_core0_main = fn; }
const char *core0_hang_reason() { return g_hang; }
void core0_report_hang(const char *why) {
  g_hang = why;
  std::fprintf(stderr, "sim: core0 hang: %s\n", why);
  // Park core0 for good; the scheduler sees g_hang and stops.
  while (true)
    core0_sleep_until(NEVER - 1);
}

void set_pin_board(PinBoard *b) { g_board = b; }
bool pin_level(int pin) { return g_out[pin]; }
bool pin_is_output(int pin) { return g_dir_out[pin]; }
bool pin_oe(int pin) { return g_oe[pin]; }
void pin_set_input(int pin, bool level) {
  g_forced[pin] = true;
  g_forced_level[pin] = level;
}

} // namespace sim

// ---------------- Pico SDK shims ----------------
using namespace sim;

static constexpr uint32_t COST_CALL = 2; // ~80 ns per HAL call on core0

extern "C" {

uint32_t time_us_32(void) {
  core0_cost(COST_CALL);
  return (uint32_t)(now() / TICKS_PER_US);
}
uint64_t time_us_64(void) {
  core0_cost(COST_CALL);
  return now() / TICKS_PER_US;
}
absolute_time_t get_absolute_time(void) { return time_us_64(); }
void sleep_us(uint64_t u) { core0_sleep_until(now() + sim::us(u)); }
void sleep_ms(uint32_t m) { core0_sleep_until(now() + sim::ms(m)); }
void busy_wait_us_32(uint32_t u) { core0_sleep_until(now() + sim::us(u)); }

bool gpio_get(unsigned pin) {
  core0_cost(COST_CALL);
  if (g_forced[pin])
    return g_forced_level[pin];
  if (g_dir_out[pin])
    return g_out[pin];
  return g_board ? g_board->input((int)pin, now()) : true;
}
void gpio_put(unsigned pin, bool value) {
  core0_cost(COST_CALL);
  bool changed = g_out[pin] != value;
  g_out[pin] = value;
  if (changed && g_dir_out[pin] && g_board)
    g_board->output((int)pin, value, now());
}
void gpio_init(unsigned pin) {
  g_dir_out[pin] = false;
  g_out[pin] = false;
}
void gpio_set_dir(unsigned pin, bool out) {
  bool was = g_dir_out[pin];
  g_dir_out[pin] = out;
  // Releasing an output to an input lets the line float to its pull-up: the
  // drive sees a high. Taking it as an output drives the latched level.
  if (g_board && was != out)
    g_board->output((int)pin, out ? g_out[pin] : true, now());
}
void gpio_pull_up(unsigned) {}
void gpio_pull_down(unsigned) {}
void gpio_disable_pulls(unsigned) {}
void gpio_set_function(unsigned, enum gpio_function) {}

static uint32_t g_irq_mask[32];
static irq_handler_t g_irq_handler;
void gpio_set_irq_enabled(unsigned pin, uint32_t events, bool enabled) {
  g_irq_mask[pin] = enabled ? events : 0;
}
uint32_t gpio_get_irq_event_mask(unsigned pin) {
  bool lv = g_forced[pin] ? g_forced_level[pin] : true;
  return lv ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL;
}
void gpio_acknowledge_irq(unsigned, uint32_t) {}
void irq_set_exclusive_handler(unsigned, irq_handler_t h) { g_irq_handler = h; }
void irq_set_enabled(unsigned, bool) {}

} // extern "C"

// Fire the bank-0 GPIO interrupt (bus RESET edges) as core1 would take it.
void sim_fire_gpio_irq(void) {
  if (g_irq_handler)
    g_irq_handler();
}

// SIO set/clear aliases (board_bus.h): 0 set, 1 clr, 2 oe_set, 3 oe_clr.
void sim_sio_write(int kind, uint32_t mask) {
  for (int p = 0; p < 32; ++p) {
    if (!(mask & (1u << p)))
      continue;
    switch (kind) {
    case 0: g_out[p] = true; break;
    case 1: g_out[p] = false; break;
    case 2: g_oe[p] = true; break;
    case 3: g_oe[p] = false; break;
    }
  }
}
uint32_t sim_sio_gpio_in(void) {
  // IORQ (GPIO8) reads high: in the simulation every bus cycle is complete by
  // the time anything looks at the line.
  return 1u << 8;
}
static sim_sio_hw_t g_sio;
sim_sio_hw_t *const sio_hw = &g_sio;
