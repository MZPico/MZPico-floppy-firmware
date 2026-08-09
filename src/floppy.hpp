// FloppyDrive — PC/Shugart drive control, Pico-SDK port of Adafruit_Floppy's
// Adafruit_Floppy class (MIT). Logic/polarities preserved exactly; Arduino
// GPIO/delay calls replaced with Pico-SDK equivalents. Flux capture/write
// delegate to src/flux/arch_rp2.
#pragma once
#include <cstddef>
#include <cstdint>

#define STEP_OUT true  // outward (toward track 0)
#define STEP_IN false  // inward

class FloppyDrive {
public:
  FloppyDrive(int densitypin, int indexpin, int selectpin, int motorpin,
              int directionpin, int steppin, int wrdatapin, int wrgatepin,
              int track0pin, int protectpin, int rddatapin, int sidepin,
              int readypin);

  bool begin();
  void end();
  void soft_reset();

  void select(bool selected);
  bool spin_motor(bool motor_on);
  // Assert/deassert the motor line only — no spin-up delay, no index wait.
  // For FDC Type I stepping, which does not need the motor fully up.
  void motor_raw(bool motor_on);
  bool goto_track(int track_num);
  void step(bool dir, uint8_t times);
  bool side(int head);
  int track() const { return _track; }
  // Overwrite the cached head position with one that has been OBSERVED rather
  // than assumed -- the cylinder byte of an ID field actually read off the
  // disk. _track is otherwise derived from the moves we asked for, so it has no
  // way back to reality after a lost step except a seek to TRK00. Pass -1 to
  // mean "unknown", which forces the next goto_track() to recalibrate.
  void set_track_belief(int t) { _track = t; }
  // Cross-check the cached position against the TRK00 sense on every seek (see
  // goto_track). Runtime-disableable: it can force a full recalibrate on its
  // own initiative, which is exactly the kind of behaviour that must be
  // removable without a reflash when seek timing is under suspicion.
  void set_track0_crosscheck(bool on) { _track0_crosscheck = on; }
  int get_side() const { return _side; }

  bool read_index();
  bool get_write_protect();
  bool get_track0_sense();
  bool get_ready_sense();
  bool set_density(bool high_density);

  bool drive_is_selected() const { return _is_drive_selected; }
  bool motor_is_spinning() const { return _is_motor_spinning; }

  size_t capture_track(volatile uint8_t *pulses, size_t max_pulses,
                       int32_t *falling_index_offset, bool store_greaseweazle,
                       uint32_t capture_ms, uint32_t index_wait_ms);
  bool write_track(uint8_t *pulses, size_t n_pulses, bool store_greaseweazle,
                   bool use_index);
  uint32_t getSampleFrequency() const;

  // greaseweazle GETPARAMS/SETPARAMS delay knobs (usec/msec)
  uint16_t select_delay_us = 10;
  uint16_t step_delay_us = 12000;
  uint16_t settle_delay_ms = 15;
  uint16_t motor_delay_ms = 1000;
  uint16_t watchdog_delay_ms = 1000;

private:
  const int _densitypin, _indexpin, _selectpin, _motorpin, _directionpin,
      _steppin, _wrdatapin, _wrgatepin, _track0pin, _protectpin, _rddatapin,
      _sidepin, _readypin;

  int _track = -1;
  int _side = -1;
  bool _is_drive_selected = false;
  bool _is_motor_spinning = false;
  bool _is_index_seen = false;
  bool _track0_crosscheck = true;
};
