// Derived from Adafruit_Floppy (https://github.com/adafruit/Adafruit_Floppy),
// MIT License, Copyright (c) 2021 Limor Fried for Adafruit Industries.
// See LICENSES/MIT-Adafruit_Floppy.txt. Changes: Apache-2.0, see LICENSE.
#include "floppy.hpp"

#include "flux/arch_rp2.h"

#include "hardware/gpio.h"
#include "pico/time.h"

// Small Arduino-compat shims so the ported logic reads like the original.
static inline uint32_t millis() {
  return to_ms_since_boot(get_absolute_time());
}
static inline void out_high(int pin) { gpio_put(pin, 1); }
static inline void out_low(int pin) { gpio_put(pin, 0); }
static inline void write_pin(int pin, bool v) { gpio_put(pin, v); }
static inline bool read_pin(int pin) { return gpio_get(pin); }

static void init_out(int pin, bool level) {
  if (pin < 0)
    return;
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_OUT);
  gpio_put(pin, level);
}
static void init_in_pullup(int pin) {
  if (pin < 0)
    return;
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);
  gpio_pull_up(pin);
}
static void init_in_nopull(int pin) {
  if (pin < 0)
    return;
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);
  gpio_disable_pulls(pin); // RDATA has an external pull-up
}

FloppyDrive::FloppyDrive(int densitypin, int indexpin, int selectpin,
                         int motorpin, int directionpin, int steppin,
                         int wrdatapin, int wrgatepin, int track0pin,
                         int protectpin, int rddatapin, int sidepin,
                         int readypin)
    : _densitypin(densitypin), _indexpin(indexpin), _selectpin(selectpin),
      _motorpin(motorpin), _directionpin(directionpin), _steppin(steppin),
      _wrdatapin(wrdatapin), _wrgatepin(wrgatepin), _track0pin(track0pin),
      _protectpin(protectpin), _rddatapin(rddatapin), _sidepin(sidepin),
      _readypin(readypin) {}

bool FloppyDrive::begin() {
  soft_reset();
  return true;
}

void FloppyDrive::soft_reset() {
  // Index: input with pull-up (active low)
  init_in_pullup(_indexpin);

  // Write data idle high, write gate released (input, pulled up)
  init_out(_wrdatapin, true);
  init_in_pullup(_wrgatepin);

  // Read data: input, no internal pull (external pull-up on the board bodge)
  init_in_nopull(_rddatapin);

  // Deselect drive (select active low)
  init_out(_selectpin, true);
  // Motor off (active low -> high = off)
  init_out(_motorpin, true);
  // Direction: low = inward
  init_out(_directionpin, false);
  // Step idle high (pulses low)
  init_out(_steppin, true);
  // Side 0 (head 0 = logic high)
  init_out(_sidepin, true);

  init_in_pullup(_track0pin);
  init_in_pullup(_protectpin);
  init_in_pullup(_readypin);

  // Density low
  init_out(_densitypin, false);

  select_delay_us = 10;
  step_delay_us = 12000;
  settle_delay_ms = 15;
  motor_delay_ms = 1000;
  watchdog_delay_ms = 1000;

  _is_drive_selected = false;
  _is_motor_spinning = false;
  _is_index_seen = false;
}

void FloppyDrive::end() {
  gpio_set_dir(_rddatapin, GPIO_IN);
  if (_wrdatapin >= 0)
    gpio_set_dir(_wrdatapin, GPIO_IN);
  if (_wrgatepin >= 0)
    gpio_set_dir(_wrgatepin, GPIO_IN);
  if (_indexpin >= 0)
    gpio_set_dir(_indexpin, GPIO_IN);
  _is_drive_selected = false;
  _is_motor_spinning = false;
  _is_index_seen = false;
}

bool FloppyDrive::read_index() {
  if (_indexpin < 0)
    return true;
  return read_pin(_indexpin);
}

void FloppyDrive::select(bool selected) {
  write_pin(_selectpin, !selected); // selected = logic 0
  _is_drive_selected = selected;
  sleep_us(select_delay_us);
}

bool FloppyDrive::side(int head) {
  if (head != 0 && head != 1)
    return false;
  if (_sidepin < 0 && head != 0)
    return false;
  _side = head;
  write_pin(_sidepin, !head); // head 0 = logic 1, head 1 = logic 0
  return true;
}

bool FloppyDrive::s_spinup_pending = false;
uint32_t FloppyDrive::s_motor_on_us = 0;

bool FloppyDrive::head_ready(uint32_t settle_us, uint32_t spinup_us) {
  uint32_t now = time_us_32();
  // settle_us == 0 means "don't care about settling for this call" -- it must
  // not clear the pending settle a later read or write still has to wait for.
  if (settle_us && _settle_pending && now - _last_step_us >= settle_us)
    _settle_pending = false;
  if (s_spinup_pending && now - s_motor_on_us >= spinup_us)
    s_spinup_pending = false;
  return (!settle_us || !_settle_pending) && !s_spinup_pending;
}

bool FloppyDrive::spin_motor(bool motor_on) {
  if (motor_on != _is_motor_spinning) {
    write_pin(_motorpin, !motor_on); // motor on = logic 0
    _is_motor_spinning = motor_on;
    if (motor_on) {
      s_motor_on_us = time_us_32();
      s_spinup_pending = true;
    }
  }
  if (!motor_on) {
    _is_index_seen = false;
    return true;
  }
  if (_is_index_seen)
    return true;

  sleep_ms(motor_delay_ms);

  uint32_t index_stamp = millis();
  bool timedout = false;
  while (read_index()) {
    if ((millis() - index_stamp) > 1000) {
      timedout = true;
      break;
    }
  }
  _is_index_seen = !timedout;
  return _is_index_seen;
}

void FloppyDrive::motor_raw(bool motor_on) {
  if (motor_on && !_is_motor_spinning) {
    s_motor_on_us = time_us_32();
    s_spinup_pending = true;
  }
  write_pin(_motorpin, !motor_on); // motor on = logic 0
  _is_motor_spinning = motor_on;
  if (!motor_on)
    _is_index_seen = false;
}

void FloppyDrive::step(bool dir, uint8_t times) {
  write_pin(_directionpin, dir);
  sleep_us(10);
  uint32_t half = (step_delay_us / 1000UL) + 1;
  while (times--) {
    out_high(_steppin);
    sleep_ms(half);
    out_low(_steppin);
    sleep_ms(half);
    out_high(_steppin);
    _last_step_us = time_us_32();
    _settle_pending = true;
  }
  sleep_ms(half); // one more for good measure
}

bool FloppyDrive::goto_track(int track_num) {
  static const int FLOPPY_MAX_TRACK = 80; // FLOPPY_IBMPC_HD_TRACKS

  if ((_track < 0) || track_num == 0) {
    uint8_t max_steps = 100;
    while (max_steps--) {
      if (!read_pin(_track0pin)) { // track0 active low
        _track = 0;
        break;
      }
      step(STEP_OUT, 1);
    }
    if (read_pin(_track0pin)) {
      // never saw track 0 stepping out; try stepping in a bit
      max_steps = 20;
      while (max_steps--) {
        if (!read_pin(_track0pin)) {
          _track = 0;
          break;
        }
        step(STEP_IN, 1);
      }
      if (read_pin(_track0pin)) {
        _track = -1;
        return false;
      }
    }
  }
  sleep_ms(settle_delay_ms);

  if (track_num > FLOPPY_MAX_TRACK - 1)
    track_num = FLOPPY_MAX_TRACK - 1;

  // Cross-check the belief against the one position the drive can actually
  // report. _track is otherwise pure bookkeeping -- it is set from the number
  // we *asked* for, so a single lost STEP pulse (or a step the head could not
  // make) becomes a permanent off-by-one, and the early return below then keeps
  // us from ever re-issuing the move that would fix it. TRK00 disagreeing with
  // the belief is proof the belief is wrong, and re-seeking to 0 is the only
  // resync a floppy offers.
  //
  // This mattered less when the bridge was read-only: the WD ID track compare
  // turned a mispositioned head into an RNF and the host retried. A write
  // cannot be retried after the fact, so relying on the last line of defence
  // for a condition the first line should prevent is no longer acceptable.
  if (_track0_crosscheck && _track >= 0 && _track0pin >= 0) {
    bool at_zero = !read_pin(_track0pin);
    if (at_zero != (_track == 0)) {
      _track = -1; // belief is provably wrong -> force the TRK00 recalibrate
      return goto_track(track_num);
    }
  }

  if (_track == track_num)
    return true;

  int steps = track_num - _track;
  if (steps > 0) {
    step(STEP_IN, steps);
  } else {
    step(STEP_OUT, -steps);
  }
  sleep_ms(settle_delay_ms);
  _track = track_num;
  return true;
}

bool FloppyDrive::get_write_protect() {
  if (_protectpin < 0)
    return false;
  return !read_pin(_protectpin);
}

bool FloppyDrive::get_ready_sense() {
  if (_readypin < 0)
    return true;
  return !read_pin(_readypin);
}

bool FloppyDrive::get_track0_sense() {
  if (_track0pin < 0)
    return track() != 0;
  return !read_pin(_track0pin);
}

bool FloppyDrive::set_density(bool high_density) {
  if (_densitypin < 0)
    return !high_density;
  write_pin(_densitypin, high_density);
  return true;
}

uint32_t FloppyDrive::getSampleFrequency() const { return 24000000UL; }

size_t FloppyDrive::capture_track(volatile uint8_t *pulses, size_t max_pulses,
                                  int32_t *falling_index_offset,
                                  bool store_greaseweazle, uint32_t capture_ms,
                                  uint32_t index_wait_ms) {
  for (size_t i = 0; i < max_pulses; i++)
    pulses[i] = 0;
  return rp2040_flux_capture(_indexpin, _rddatapin, pulses, pulses + max_pulses,
                             falling_index_offset, store_greaseweazle,
                             capture_ms * (getSampleFrequency() / 1000),
                             index_wait_ms);
}

bool FloppyDrive::write_track(uint8_t *pulses, size_t n_pulses,
                              bool store_greaseweazle, bool use_index) {
  return rp2040_flux_write(_indexpin, _wrgatepin, _wrdatapin, pulses,
                           pulses + n_pulses, store_greaseweazle,
                           /*is_apple2=*/false, use_index);
}
