// Simulated drive mechanics + the rp2040 flux capture/write API (arch_rp2.h)
// implemented on top of it. Replaces src/flux/arch_rp2.cpp in host builds.
#include "sim_drive.hpp"

#include <algorithm>
#include <cstdio>

#include "board_config.h"
#include "flux/arch_rp2.h"
#include "hardware/gpio.h"

namespace sim {

static FloppyCable g_cable;
void arm_capture_overrun(uint64_t at_us, uint64_t hole_us);
FloppyCable &cable() { return g_cable; }

uint32_t Drive::angle_at(uint64_t now) const {
  uint64_t t = (now + rev_ticks - (phase_ticks % rev_ticks)) % rev_ticks;
  return (uint32_t)(t * REV_ANGLE / rev_ticks);
}

Drive *FloppyCable::selected() {
  // ~DS0 = GPIO24, ~DS1 = GPIO25, active low, driven by FloppyDrive::select.
  bool s0 = pin_is_output(PIN_DS0) && !pin_level(PIN_DS0);
  bool s1 = pin_is_output(PIN_DS1) && !pin_level(PIN_DS1);
  if (s0 && s1)
    return &drive[0]; // both asserted: the cable is shorted; pick one
  if (s0)
    return &drive[0];
  if (s1)
    return &drive[1];
  return nullptr;
}

int FloppyCable::head() const {
  return (pin_is_output(PIN_HDSEL) && !pin_level(PIN_HDSEL)) ? 1 : 0;
}

bool FloppyCable::input(int pin, uint64_t now) {
  Drive *d = selected();
  switch (pin) {
  case PIN_INDEX: {
    if (!(d && d->spinning(now)))
      return true;
    uint32_t a = d->angle_at(now);
    if (a < INDEX_ANGLE)
      return false;
    if (d->index_glitch_after_ticks) {
      uint64_t g = d->index_glitch_after_ticks * REV_ANGLE / d->rev_ticks;
      if (a >= g && a < g + us(20) * REV_ANGLE / d->rev_ticks)
        return false;
    }
    return true;
  }
  case PIN_TRK00:
    return !(d && d->cyl == 0);
  case PIN_WPT:
    return !(d && d->disk && d->disk->write_protect);
  default:
    return true; // pulled up
  }
}

void FloppyCable::output(int pin, bool level, uint64_t now) {
  if (pin == PIN_MOTOR) {
    bool on = !level; // ~MOTOR_ON reaches every drive on the cable
    for (Drive &d : drive) {
      if (on && !d.motor)
        d.motor_on_since = now;
      d.motor = on;
    }
    return;
  }
  if (pin == PIN_STEP) {
    if (!level) {
      step_low_since = now;
      return;
    }
    // Trailing (rising) edge of the ~STEP pulse moves the head.
    Drive *d = selected();
    if (!d)
      return;
    bool in = !pin_level(PIN_DIR); // ~DIR low = toward the spindle
    int c = d->cyl + (in ? 1 : -1);
    d->cyl = std::clamp(c, 0, MAX_CYL - 1);
    d->steps++;
    d->last_step_at = now;
  }
}

uint64_t FloppyCable::next_transition(uint64_t t) {
  Drive *d = selected();
  if (!d || !d->spinning(t + 1))
    return UINT64_MAX;
  if (d->reading_noise(t)) {
    // Ringing head or a spindle below speed: transitions at no MFM spacing
    // in particular (deterministic in t, so a run is reproducible).
    uint64_t h = (t + 1) * 0x9E3779B97F4A7C15ull;
    return t + 30 + (h >> 56); // 30..285 ticks
  }
  const Track &tr = d->disk->at(d->cyl, head());
  if (tr.pos.empty())
    return UINT64_MAX;
  uint32_t a = d->angle_at(t);
  auto it = std::upper_bound(tr.pos.begin(), tr.pos.end(), a);
  uint64_t da;
  if (it != tr.pos.end())
    da = *it - a;
  else
    da = (uint64_t)REV_ANGLE - a + tr.pos.front();
  uint64_t dt = d->ticks_for_angle(da);
  uint64_t next = t + (dt ? dt : 1);
  if (!tr.weak.empty()) {
    uint32_t an = (it != tr.pos.end()) ? *it : tr.pos.front();
    for (const auto &w : tr.weak) {
      if (an < w.first || an >= w.second)
        continue;
      uint64_t rev = d->rev_at(next);
      bool bad = (rev % 2) == 0;
      if (d->weak_passes.empty() || d->weak_passes.back().rev != rev)
        d->weak_passes.push_back({rev, bad, next});
      if (bad && next > t + 41) {
        // Weak flux: the transition wanders by up to ~a DD cell, enough to
        // flip bits (deterministic in time, so a run is reproducible).
        uint64_t h = (next + 3) * 0x9E3779B97F4A7C15ull;
        int64_t j = (int64_t)(h >> 58) - 32; // -32..31 ticks
        next = (uint64_t)((int64_t)next + j);
      }
      break;
    }
  }
  if (d->reading_marginal(next) && next > t + d->marginal_jitter_ticks + 1) {
    // Shift this transition by a deterministic pseudo-random amount. Only
    // forward-safe shifts (never before t), so the sequence stays ordered.
    uint64_t h = (next + 7) * 0xD6E8FEB86659FD93ull;
    int64_t j = (int64_t)(h >> 59) - 16; // -16..15
    j = j * (int64_t)d->marginal_jitter_ticks / 16;
    next = (uint64_t)((int64_t)next + j);
  }
  return next;
}

void FloppyCable::write_flux(uint64_t t0, const uint8_t *deltas, size_t n,
                             bool full_track) {
  Drive *d = selected();
  if (!d || !d->disk || !d->spinning(t0))
    return;
  WriteEvent ev{};
  ev.t0 = t0;
  ev.cyl = d->cyl;
  ev.head = head();
  ev.full_track = full_track;
  uint64_t dur = 0;
  for (size_t i = 0; i < n; ++i)
    dur += deltas[i];
  if (full_track && dur > d->rev_ticks)
    dur = d->rev_ticks; // terminated by the index pulse
  ev.t1 = t0 + dur;
  ev.a0 = d->angle_at(t0);
  ev.a1 = d->angle_at(t0 + dur);
  ev.transitions = n;
  if (d->disk->write_protect) {
    ev.refused_wp = true; // the drive's own WP interlock
    d->writes.push_back(ev);
    return;
  }
  Track &tr = d->disk->at(d->cyl, ev.head);
  // A splice is not a clean cut on real media: the write current ramps up
  // after WGATE asserts, so the first stretch records no transitions, and the
  // tunnel-erase gap trails the write head, so a stretch after WGATE drops is
  // erased too. Both leave a flux-free interval longer than any MFM symbol at
  // each end of a rewritten data field -- which a reader must tolerate.
  uint64_t junction = full_track ? 0 : d->splice_junction_ticks;
  // Erase [a0, a0+span) (with wrap), then lay down the new transitions.
  uint64_t span = (dur + junction) * REV_ANGLE / d->rev_ticks;
  auto erased = [&](uint32_t p) {
    uint64_t rel = (p + (uint64_t)REV_ANGLE - ev.a0) % REV_ANGLE;
    return span >= REV_ANGLE || rel < span;
  };
  std::vector<uint32_t> out;
  out.reserve(tr.pos.size() + n);
  for (uint32_t p : tr.pos)
    if (!erased(p))
      out.push_back(p);
  uint64_t t = 0;
  for (size_t i = 0; i < n; ++i) {
    t += deltas[i];
    if (t > dur)
      break;
    if (t < junction)
      continue; // write current still ramping up
    out.push_back((uint32_t)((ev.a0 + t * REV_ANGLE / d->rev_ticks) % REV_ANGLE));
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  tr.pos.swap(out);
  d->writes.push_back(ev);
}

} // namespace sim

// ---------------- arch_rp2.h on the simulated drive ----------------
using namespace sim;

namespace {
bool g_cap_active = false;
uint64_t g_cap_last = 0; // time of the last transition handed out
uint64_t g_cap_start = 0;
bool g_ovr_armed = false, g_ovr_flag = false;
uint64_t g_ovr_at = 0, g_ovr_hole = 0;
constexpr uint64_t FIRST_SAMPLE_TIMEOUT = 400000; // us, as arch_rp2.cpp
} // namespace

extern "C" {

bool rp2040_flux_capture_start(int, int) {
  core0_cost(48);
  uint64_t t = cable().next_transition(now());
  if (t == UINT64_MAX || t > now() + sim::us(FIRST_SAMPLE_TIMEOUT)) {
    core0_sleep_until(now() + sim::us(FIRST_SAMPLE_TIMEOUT));
    g_cap_active = false;
    return false;
  }
  core0_sleep_until(t);
  g_cap_last = t;
  g_cap_active = true;
  g_cap_start = t;
  g_ovr_flag = false; // cleared per capture, as ring_start() does
  return true;
}

size_t rp2040_flux_capture_drain(volatile uint8_t *dest, size_t dest_cap,
                                 size_t already, bool) {
  core0_cost(4);
  size_t n = already;
  if (!g_cap_active)
    return n;
  uint64_t lim = now();
  while (n < dest_cap) {
    uint64_t t = cable().next_transition(g_cap_last);
    if (t > lim)
      break;
    if (g_ovr_armed) {
      // Lapped: nothing between `at` and `at + hole` is ever delivered, the
      // consumer resumes after it, and the flag is up from then on. (Withheld
      // per transition, so frequent drains cannot have delivered it first.)
      uint64_t hs = g_cap_start + sim::us(g_ovr_at);
      uint64_t he = hs + sim::us(g_ovr_hole);
      if (t >= hs) {
        if (lim < he)
          break; // still inside the hole: deliver nothing yet
        if (t < he) {
          g_cap_last = he;
          g_ovr_armed = false;
          g_ovr_flag = true;
          continue;
        }
        g_ovr_armed = false;
        g_ovr_flag = true;
      }
    }
    uint64_t d = t - g_cap_last;
    dest[n++] = (uint8_t)(d > 255 ? 255 : d);
    g_cap_last = t;
  }
  return n;
}

void rp2040_flux_capture_stop(void) { g_cap_active = false; }
void rp2040_flux_capture_disable(void) { g_cap_active = false; }
bool rp2040_flux_capture_overrun(void) { return g_ovr_flag; }

int rp2040_flux_capture_next(uint32_t timeout_us) {
  core0_cost(2);
  if (!g_cap_active)
    return -1;
  uint64_t t = cable().next_transition(g_cap_last);
  uint64_t n = now();
  if (t == UINT64_MAX || (t > n && t - n > sim::us(timeout_us))) {
    core0_sleep_until(n + sim::us(timeout_us) + 1);
    return -1;
  }
  if (t > n)
    core0_sleep_until(t);
  uint64_t d = t - g_cap_last;
  g_cap_last = t;
  return (int)(d > 65535 ? 65535 : d);
}

uint32_t rp2040_flux_capture_backlog(void) {
  uint32_t sum = 0;
  uint64_t lim = now();
  while (g_cap_active) {
    uint64_t t = cable().next_transition(g_cap_last);
    if (t > lim)
      break;
    sum += (uint32_t)(t - g_cap_last);
    g_cap_last = t;
  }
  return sum;
}

bool rp2040_flux_write_prepare(int) { return true; }
void rp2040_flux_write_release(void) {}

bool rp2040_flux_write_now(int wrgate_pin, const uint8_t *pulses, size_t n) {
  gpio_set_dir(wrgate_pin, GPIO_OUT);
  gpio_put(wrgate_pin, 0);
  uint64_t t0 = now();
  uint64_t dur = 0;
  for (size_t i = 0; i < n; ++i)
    dur += pulses[i];
  cable().write_flux(t0, pulses, n, false);
  core0_sleep_until(t0 + dur + sim::us(32));
  gpio_set_dir(wrgate_pin, GPIO_IN);
  return true;
}

// Whole-track write (WRITE TRACK / gw write): optionally from the index
// falling edge, ending at the next one or when the pulses run out. The index
// waits are bounded at 600 ms, as in arch_rp2.cpp (issue #7); the guard below
// reports a hang should that bound ever go missing again.
bool rp2040_flux_write(int index_pin, int wrgate_pin, int, uint8_t *pulses,
                       uint8_t *pulse_end, bool, bool, bool use_index) {
  uint64_t start = now();
  auto timed_out = [&] {
    if (now() - start > sim::ms(10000))
      core0_report_hang("rp2040_flux_write: index wait never ends (no ~INDEX)");
    return now() - start > sim::us(600000);
  };
  if (use_index) {
    while (!gpio_get(index_pin))
      if (timed_out())
        return false;
    while (gpio_get(index_pin))
      if (timed_out())
        return false;
  }
  gpio_set_dir(wrgate_pin, GPIO_OUT);
  gpio_put(wrgate_pin, 0);
  uint64_t t0 = now();
  size_t n = (size_t)(pulse_end - pulses);
  uint64_t dur = 0;
  for (size_t i = 0; i < n; ++i)
    dur += pulses[i];
  Drive *d = cable().selected();
  uint64_t rev = d ? d->rev_ticks : REV_ANGLE;
  bool to_index = use_index && dur >= rev;
  cable().write_flux(t0, pulses, n, to_index);
  core0_sleep_until(t0 + (to_index ? rev : dur));
  gpio_set_dir(wrgate_pin, GPIO_IN);
  return true;
}

// Batch capture (greaseweazle/bench paths). Not used by the FDC bridge's
// live paths; implemented plainly for completeness.
uint32_t rp2040_flux_capture(int index_pin, int rdpin, volatile uint8_t *pulses,
                             volatile uint8_t *pulse_end,
                             int32_t *falling_index_offset, bool,
                             uint32_t capture_counts, uint32_t index_wait_ms) {
  if (falling_index_offset)
    *falling_index_offset = -1;
  if (index_wait_ms) {
    uint64_t lim = now() + sim::ms(index_wait_ms);
    while (!gpio_get(index_pin) && now() < lim) {
    }
    while (gpio_get(index_pin) && now() < lim) {
    }
  }
  if (!rp2040_flux_capture_start(index_pin, rdpin))
    return 0;
  size_t cap = (size_t)(pulse_end - pulses);
  uint64_t t_end = now() + capture_counts;
  size_t n = 0;
  while (now() < t_end && n < cap)
    n = rp2040_flux_capture_drain(pulses, cap, n, false);
  rp2040_flux_capture_stop();
  return (uint32_t)n;
}

} // extern "C"

void sim::arm_capture_overrun(uint64_t at_us, uint64_t hole_us) {
  g_ovr_armed = true;
  g_ovr_at = at_us;
  g_ovr_hole = hole_us;
}
