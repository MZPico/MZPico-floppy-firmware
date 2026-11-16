// A simulated Shugart floppy drive (and the disk in it), seen through the same
// 34-pin interface the board wires to GPIO14..25.
//
// The disk surface is stored as flux transition POSITIONS around the track
// (angle units: one revolution = REV_ANGLE, i.e. 200 ms at 24 MHz), not as
// bytes. A read returns whatever transitions pass under the head in the
// capture window -- rotational latency, index position, a sector straddling
// the window start: all of it is real, because the disk really turns. A write
// replaces the transitions under the head for the write's duration, so a
// splice that lands early or late damages its neighbours exactly as on media.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "sim_core.hpp"

namespace sim {

constexpr uint32_t REV_ANGLE = 4800000; // 200 ms of 24 MHz ticks
constexpr uint32_t INDEX_ANGLE = 48000; // index pulse ~2 ms long
constexpr int MAX_CYL = 84;

struct Track {
  std::vector<uint32_t> pos; // sorted transition angles in [0, REV_ANGLE)
};

struct Disk {
  Track t[MAX_CYL][2];
  bool write_protect = false;
  bool hd_media = false; // informational; flux timing is what decides
  std::string name;
  Track &at(int cyl, int head) { return t[cyl][head]; }
};

struct WriteEvent {
  uint64_t t0, t1;     // virtual time span the write gate was active
  int cyl, head;
  uint32_t a0, a1;     // angle span overwritten
  size_t transitions;
  bool full_track;     // a WRITE TRACK (index to index)
  bool refused_wp;     // the drive ignored it: write-protected media
};

struct Drive {
  Disk *disk = nullptr;
  int cyl = 7;                // head position; starts off track 0 on purpose
  uint64_t rev_ticks = REV_ANGLE; // time per revolution (218.7 ms drives exist)
  uint64_t phase_ticks = 0;   // rotational phase offset (angle 0 at t=phase)
  uint64_t spinup_ticks = ms(250);
  uint64_t motor_on_since = 0;
  bool motor = false;
  int steps = 0;              // STEP pulses obeyed (test assertions)
  // Flux-free stretch at each end of a spliced write (write-current ramp at
  // WGATE on, tunnel erase after WGATE off). 20 us is longer than the 255-tick
  // (10.6 us) capture clamp, as observed on hardware.
  uint64_t splice_junction_ticks = us(20);
  // A false ~INDEX pulse this long after each real one (0 = none), 20 us
  // wide: a coupled glitch, not a pulse (issue #18).
  uint64_t index_glitch_after_ticks = 0;
  std::vector<WriteEvent> writes;

  bool spinning(uint64_t now) const {
    return motor && disk && now >= motor_on_since + spinup_ticks;
  }
  uint32_t angle_at(uint64_t now) const;
  // Rounded UP, so that angle_at(t + ticks_for_angle(da)) >= angle_at(t) + da
  // and a transition is never handed out twice.
  uint64_t ticks_for_angle(uint64_t a) const {
    return (a * rev_ticks + REV_ANGLE - 1) / REV_ANGLE;
  }
};

// Pin-level board: up to two drives on one cable.
class FloppyCable : public PinBoard {
public:
  Drive drive[2];
  bool input(int pin, uint64_t now) override;
  void output(int pin, bool level, uint64_t now) override;

  // The drive(s) whose ~DSn is asserted right now (nullptr if none).
  Drive *selected();
  int head() const; // ~HDSEL: low = head 1
  // First transition strictly after `t` under the selected head, or UINT64_MAX.
  uint64_t next_transition(uint64_t t);
  // Overwrite the selected drive's current track from t0 with `deltas`.
  void write_flux(uint64_t t0, const uint8_t *deltas, size_t n, bool full_track);

  uint64_t step_low_since = 0;
};

FloppyCable &cable();

// One-shot DMA ring overrun (issue #5): the next capture that runs past
// `at_us` loses `hole_us` of flux, as when the consumer is lapped and jumps
// ahead, and rp2040_flux_capture_overrun() reports it.
void arm_capture_overrun(uint64_t at_us, uint64_t hole_us);

} // namespace sim
