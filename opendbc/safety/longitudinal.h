#include "opendbc/safety/declarations.h"

bool get_longitudinal_gas_allowed(void) {
  return controls_allowed;
}

bool get_longitudinal_brake_allowed(void) {
  // Braking commands already in flight when the driver presses the gas can't react to it, so allow braking
  // for a short grace period after the gas press. openpilot responds on the next frame it sees the gas pressed
  const uint32_t GAS_PRESS_BRAKE_GRACE_US = 20000U;  // 20 ms, two openpilot control frames
  bool gas_override = gas_pressed && (safety_get_ts_elapsed(microsecond_timer_get(), gas_pressed_ts) > GAS_PRESS_BRAKE_GRACE_US);
  return controls_allowed && !gas_override;
}

// Safety checks for longitudinal actuation
bool longitudinal_accel_checks(int desired_accel, const LongitudinalLimits limits) {
  bool longitudinal_allowed = (desired_accel >= limits.zero_accel) ? get_longitudinal_gas_allowed() : get_longitudinal_brake_allowed();
  bool accel_valid = longitudinal_allowed && !safety_max_limit_check(desired_accel, limits.max_accel, limits.min_accel);
  bool accel_inactive = desired_accel == limits.inactive_accel;
  return !(accel_valid || accel_inactive);
}

bool longitudinal_speed_checks(int desired_speed, const LongitudinalLimits limits) {
  return !get_longitudinal_brake_allowed() && (desired_speed != limits.inactive_speed);
}

bool longitudinal_gas_checks(int desired_gas, const LongitudinalLimits limits) {
  bool longitudinal_allowed = (desired_gas >= limits.zero_gas) ? get_longitudinal_gas_allowed() : get_longitudinal_brake_allowed();
  bool gas_valid = longitudinal_allowed && !safety_max_limit_check(desired_gas, limits.max_gas, limits.min_gas);
  bool gas_inactive = desired_gas == limits.inactive_gas;
  return !(gas_valid || gas_inactive);
}

bool longitudinal_brake_checks(int desired_brake, const LongitudinalLimits limits) {
  bool violation = false;
  violation |= !get_longitudinal_brake_allowed() && (desired_brake != 0);
  violation |= desired_brake > limits.max_brake;
  return violation;
}
