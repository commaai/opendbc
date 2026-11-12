#ifndef SAFETY_LONGITUDINAL_H
#define SAFETY_LONGITUDINAL_H

#include "opendbc/safety/declarations.h"

// the host reads RX every 10 ms, it can only react to an RX frame once it read it
#define HOST_READ_TIMEOUT 20000U  // us

static bool host_unread(uint32_t frame, uint32_t ts) {
  // the frame counters wrap
  bool unread = (frame - can_rx_frames_read()) < 0x80000000U;
  return unread && (safety_get_ts_elapsed(safety_ts, ts) < HOST_READ_TIMEOUT);
}

static void host_read_expire(uint32_t *ts) {
  // keep an expired restriction expired, the timer wraps
  if (safety_get_ts_elapsed(safety_ts, *ts) > HOST_READ_TIMEOUT) {
    *ts = safety_ts - HOST_READ_TIMEOUT;
  }
}

bool get_longitudinal_allowed(void) {
  return safety.controls_allowed && (!safety.gas_pressed_prev || host_unread(safety.gas_press_frame, safety.gas_press_ts));
}

// Safety checks for longitudinal actuation
static bool longitudinal_accel_checks(int desired_accel, const LongitudinalLimits limits) {
  bool accel_valid = get_longitudinal_allowed() && !safety_max_limit_check(desired_accel, limits.max_accel, limits.min_accel);
  bool accel_inactive = desired_accel == limits.inactive_accel;
  return !(accel_valid || accel_inactive);
}

static bool longitudinal_speed_checks(int desired_speed, const LongitudinalLimits limits) {
  return !get_longitudinal_allowed() && (desired_speed != limits.inactive_speed);
}

static bool longitudinal_gas_checks(int desired_gas, const LongitudinalLimits limits) {
  bool gas_valid = get_longitudinal_allowed() && !safety_max_limit_check(desired_gas, limits.max_gas, limits.min_gas);
  bool gas_inactive = desired_gas == limits.inactive_gas;
  return !(gas_valid || gas_inactive);
}

static bool longitudinal_brake_checks(int desired_brake, const LongitudinalLimits limits) {
  const bool inactive_brake_violation = !get_longitudinal_allowed() && (desired_brake != 0);
  const bool brake_limit_violation = desired_brake > limits.max_brake;
  return inactive_brake_violation || brake_limit_violation;
}

#endif  // SAFETY_LONGITUDINAL_H
