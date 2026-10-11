#include "opendbc/safety/declarations.h"

// ISO 11270
static const float ISO_LATERAL_ACCEL = 3.0f;  // m/s^2

static const float EARTH_G = 9.81f;
static const float AVERAGE_ROAD_ROLL = 0.06f;  // ~3.4 degrees, 6% superelevation

// check that commanded torque value isn't too far from measured
static bool dist_to_meas_check(int val, int val_last, const struct sample_t *val_meas,
                        const int MAX_RATE_UP, const int MAX_RATE_DOWN, const int MAX_ERROR) {

  // *** val rate limit check ***
  int highest_allowed_rl = safety_max(val_last, 0) + MAX_RATE_UP;
  int lowest_allowed_rl = safety_min(val_last, 0) - MAX_RATE_UP;

  // if we've exceeded the meas val, we must start moving toward 0
  int highest_allowed = safety_min(highest_allowed_rl, safety_max(val_last - MAX_RATE_DOWN, safety_max(val_meas->max, 0) + MAX_ERROR));
  int lowest_allowed = safety_max(lowest_allowed_rl, safety_min(val_last + MAX_RATE_DOWN, safety_min(val_meas->min, 0) - MAX_ERROR));

  // check for violation
  return safety_max_limit_check(val, highest_allowed, lowest_allowed);
}

// check that commanded value isn't fighting against driver
static bool driver_limit_check(int val, int val_last, const struct sample_t *val_driver,
                        const int MAX_VAL, const int MAX_RATE_UP, const int MAX_RATE_DOWN,
                        const int MAX_ALLOWANCE, const int DRIVER_FACTOR) {

  // torque delta/rate limits
  int highest_allowed_rl = safety_max(val_last, 0) + MAX_RATE_UP;
  int lowest_allowed_rl = safety_min(val_last, 0) - MAX_RATE_UP;

  // driver
  int driver_max_limit = MAX_VAL + ((MAX_ALLOWANCE + val_driver->max) * DRIVER_FACTOR);
  int driver_min_limit = -MAX_VAL + ((-MAX_ALLOWANCE + val_driver->min) * DRIVER_FACTOR);

  // if we've exceeded the applied torque, we must start moving toward 0
  int highest_allowed = safety_min(highest_allowed_rl, safety_max(val_last - MAX_RATE_DOWN,
                                             safety_max(driver_max_limit, 0)));
  int lowest_allowed = safety_max(lowest_allowed_rl, safety_min(val_last + MAX_RATE_DOWN,
                                           safety_min(driver_min_limit, 0)));

  // check for violation
  return safety_max_limit_check(val, highest_allowed, lowest_allowed);
}

// real time check, mainly used for steer torque rate limiter
static bool rt_torque_rate_limit_check(int val, int val_last, const int MAX_RT_DELTA) {

  // *** torque real time rate limit check ***
  int highest_val = safety_max(val_last, 0) + MAX_RT_DELTA;
  int lowest_val = safety_min(val_last, 0) - MAX_RT_DELTA;

  // check for violation
  return safety_max_limit_check(val, highest_val, lowest_val);
}

// Safety checks for torque-based steering commands
bool steer_torque_cmd_checks(int desired_torque, bool steer_req, const TorqueSteeringLimits limits) {
  bool violation = false;
  uint32_t ts = microsecond_timer_get();

  if (controls_allowed) {
    // Some safety models support variable torque limit based on vehicle speed
    int max_torque = limits.max_torque;
    if (limits.dynamic_max_torque) {
      const float fudged_speed = (float)(((double)vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.0);
      const float max_torque_interp = safety_interpolate(limits.max_torque_lookup, fudged_speed) + 1.0f;
      max_torque = safety_clamp((int)max_torque_interp, -limits.max_torque, limits.max_torque);
    }

    // *** global torque limit check ***
    if (safety_max_limit_check(desired_torque, max_torque, -max_torque)) {
      violation = true;
    }

    // *** torque rate limit check ***
    if (limits.type == TorqueDriverLimited) {
      if (driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
                             max_torque, limits.max_rate_up, limits.max_rate_down,
                             limits.driver_torque_allowance, limits.driver_torque_multiplier)) {
        violation = true;
      }
    } else {
      if (dist_to_meas_check(desired_torque, desired_torque_last, &torque_meas,
                             limits.max_rate_up, limits.max_rate_down, limits.max_torque_error)) {
        violation = true;
      }
    }
    desired_torque_last = desired_torque;

    // *** torque real time rate limit check ***
    if (rt_torque_rate_limit_check(desired_torque, rt_torque_last, limits.max_rt_delta)) {
      violation = true;
    }

    // every RT_INTERVAL set the new limits
    uint32_t ts_elapsed = safety_get_ts_elapsed(ts, ts_torque_check_last);
    if (ts_elapsed > MAX_RT_INTERVAL) {
      rt_torque_last = desired_torque;
      ts_torque_check_last = ts;
    }
  }

  bool torque_requested = desired_torque != 0;
  if (!controls_allowed && torque_requested) {
    violation = true;
  }

  // certain safety modes set their steer request bit low for one or more frame at a
  // predefined max frequency to avoid steering faults in certain situations
  bool steer_req_mismatch = !steer_req && torque_requested;
  if (!limits.has_steer_req_tolerance) {
    if (steer_req_mismatch) {
      violation = true;
    }

  } else {
    if (steer_req_mismatch) {
      if (invalid_steer_req_count == 0) {
        // disallow torque cut if not enough recent matching steer_req messages
        if (valid_steer_req_count < limits.min_valid_request_frames) {
          violation = true;
        }

        // or we've cut torque too recently in time
        uint32_t ts_elapsed = safety_get_ts_elapsed(ts, ts_steer_req_mismatch_last);
        if (ts_elapsed < limits.min_valid_request_rt_interval) {
          violation = true;
        }
      } else {
        // or we're cutting more frames consecutively than allowed
        if (invalid_steer_req_count >= limits.max_invalid_request_frames) {
          violation = true;
        }
      }

      valid_steer_req_count = 0;
      ts_steer_req_mismatch_last = ts;
      invalid_steer_req_count = safety_min(invalid_steer_req_count + 1, limits.max_invalid_request_frames);
    } else {
      valid_steer_req_count = safety_min(valid_steer_req_count + 1, limits.min_valid_request_frames);
      invalid_steer_req_count = 0;
    }
  }

  // reset to 0 if either controls is not allowed or there's a violation
  if (violation || !controls_allowed) {
    valid_steer_req_count = 0;
    invalid_steer_req_count = 0;
    desired_torque_last = 0;
    rt_torque_last = 0;
    ts_torque_check_last = ts;
    ts_steer_req_mismatch_last = ts;
  }

  return violation;
}

static bool rt_angle_rate_limit_check(AngleSteeringLimits limits) {
  bool violation = false;
  uint32_t ts = microsecond_timer_get();

  // *** angle real time rate limit check ***
  const float rt_interval_msgs = (float)limits.frequency * (float)MAX_RT_INTERVAL;
  const double max_rt_msgs_buffered = ((rt_interval_msgs / 1e6) * 1.2) + 1.0;  // 1.2x buffer
  int max_rt_msgs = (int)max_rt_msgs_buffered;
  if ((int)rt_angle_msgs > max_rt_msgs) {
    violation = true;
  }

  rt_angle_msgs += 1U;

  // every RT_INTERVAL reset message counter
  uint32_t ts_elapsed = safety_get_ts_elapsed(ts, ts_angle_check_last);
  if (ts_elapsed >= MAX_RT_INTERVAL) {
    rt_angle_msgs = 0;
    ts_angle_check_last = ts;
  }

  return violation;
}

static bool rt_curvature_rate_limit_check(CurvatureSteeringLimits limits) {
  bool violation = false;
  uint32_t ts = microsecond_timer_get();

  // *** curvature real time rate limit check ***
  const float rt_interval_msgs = (float)limits.frequency * (float)MAX_RT_INTERVAL;
  const double max_rt_msgs_buffered = ((rt_interval_msgs / 1e6) * 1.2) + 1.0;  // 1.2x buffer
  int max_rt_msgs = (int)max_rt_msgs_buffered;
  uint32_t rt_msgs = curvature_state.rt_msgs + curvature_state.rt_msgs_prev;
  if ((int)rt_msgs > max_rt_msgs) {
    violation = true;
  }
  curvature_state.rt_msgs += 1U;

  //roll the window every half interval
  if (safety_get_ts_elapsed(ts, curvature_state.ts_check_last) >= (MAX_RT_INTERVAL / 2U)) {
    curvature_state.rt_msgs_prev = curvature_state.rt_msgs;
    curvature_state.rt_msgs = 0U;
    curvature_state.ts_check_last = ts;
  }

  return violation;
}

static bool steer_angle_cmd_inactive_check(int desired_angle, int max_angle) {
  const int max_inactive_angle = safety_clamp(angle_meas.max, -max_angle, max_angle) + 1;
  const int min_inactive_angle = safety_clamp(angle_meas.min, -max_angle, max_angle) - 1;
  return safety_max_limit_check(desired_angle, max_inactive_angle, min_inactive_angle);
}

// Safety checks for angle-based steering commands
bool steer_angle_cmd_checks(int desired_angle, bool steer_control_enabled, const AngleSteeringLimits limits) {
  bool violation = false;

  if (controls_allowed && steer_control_enabled) {
    // convert floating point angle rate limits to integers in the scale of the desired angle on CAN,
    // add 1 to not false trigger the violation. also fudge the speed by 1 m/s so rate limits are
    // always slightly above openpilot's in case we read an updated speed in between angle commands
    // TODO: this speed fudge can be much lower, look at data to determine the lowest reasonable offset
    const float fudged_speed = (float)(((double)vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.0);
    int delta_angle_up = safety_float_offset_to_int(safety_interpolate(limits.angle_rate_up_lookup, fudged_speed) * limits.angle_deg_to_can, 1.0);
    int delta_angle_down = safety_float_offset_to_int(safety_interpolate(limits.angle_rate_down_lookup, fudged_speed) * limits.angle_deg_to_can, 1.0);

    // allow down limits at zero since small floats from openpilot will be rounded to 0
    // TODO: openpilot should be cognizant of this and not send small floats
    int highest_desired_angle = desired_angle_last + ((desired_angle_last > 0) ? delta_angle_up : delta_angle_down);
    int lowest_desired_angle = desired_angle_last - ((desired_angle_last >= 0) ? delta_angle_down : delta_angle_up);

    // check for violation;
    if (safety_max_limit_check(desired_angle, highest_desired_angle, lowest_desired_angle)) {
      violation = true;
    }
  }
  desired_angle_last = desired_angle;

  // Angle should be close to current angle while not steering
  if (!steer_control_enabled) {
    if (steer_angle_cmd_inactive_check(desired_angle, limits.max_angle)) {
      violation = true;
    }
  }

  // No angle control allowed when controls are not allowed
  if (!controls_allowed) {
    if (steer_control_enabled) {
      violation = true;
    }
  }

  // reset to current angle if either controls is not allowed or there's a violation
  if (violation || !controls_allowed) {
    desired_angle_last = safety_clamp(angle_meas.values[0], -limits.max_angle, limits.max_angle);
  }

  return violation;
}

// Safety checks for curvature-based steering commands
bool steer_curvature_cmd_checks(int desired_curvature, int steer_power, bool steer_control_enabled, const CurvatureSteeringLimits limits) {
  // Highway curves are rolled in the direction of the turn, add tolerance to compensate
  static const float MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL + (EARTH_G * AVERAGE_ROAD_ROLL);  // ~3.6 m/s^2
  // Lower than ISO 11270 lateral jerk limit, which is 5.0 m/s^3
  static const float MAX_LATERAL_JERK = 3.0f + (EARTH_G * AVERAGE_ROAD_ROLL);  // ~3.6 m/s^3

  const float fudged_speed = safety_maxf((float)(((double)vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.0), 1.0f);
  bool violation = false;

  speed_mismatch_check((float)((float)vehicle_speed_2.values[0] / VEHICLE_SPEED_FACTOR));

  if (controls_allowed && steer_control_enabled) {
    // *** absolute curvature cap ***
    if (safety_max_limit_check(desired_curvature, limits.max_curvature, -limits.max_curvature)) {
      violation = true;
    }

    // *** ISO lateral accel limit ***
    const float max_curvature = MAX_LATERAL_ACCEL / (fudged_speed * fudged_speed);
    const int max_curvature_can = safety_float_offset_to_int(max_curvature * limits.curvature_to_can, 1.0);
    if (safety_max_limit_check(desired_curvature, max_curvature_can, -max_curvature_can)) {
      violation = true;
    }

    // *** ISO lateral jerk limit ***
    const float max_curvature_rate_sec = MAX_LATERAL_JERK / (fudged_speed * fudged_speed);
    const float max_curvature_delta = max_curvature_rate_sec / (float)limits.frequency;
    const int max_curvature_delta_can = safety_float_offset_to_int(max_curvature_delta * limits.curvature_to_can, 1.0);

    int highest_desired_curvature = curvature_state.desired_last + max_curvature_delta_can;
    int lowest_desired_curvature = curvature_state.desired_last - max_curvature_delta_can;

    // *** curvature error from measured ***
    // ensure we start moving in direction of meas while respecting relaxed rate limits if error is exceeded
    if ((limits.max_curvature_error != 0) && (((double)vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR) > limits.curvature_error_min_speed)) {
      // flipped fudge to avoid false positives
      const float fudged_speed_error = (float)(((double)vehicle_speed.max / VEHICLE_SPEED_FACTOR) + 1.0);
      const float max_curvature_rate_sec_relaxed = MAX_LATERAL_JERK / (fudged_speed_error * fudged_speed_error);
      const int max_curvature_delta_relaxed_can = safety_float_offset_to_int(max_curvature_rate_sec_relaxed / (float)limits.frequency * limits.curvature_to_can, -1.0);

      // openpilot clips its command to the lateral accel limit and to what the EPS accepts,
      // so requiring more curvature than either can never be met
      const int max_curvature_accel_can = safety_float_offset_to_int(MAX_LATERAL_ACCEL / (fudged_speed_error * fudged_speed_error) * limits.curvature_to_can, -1.0);
      const int max_curvature_relaxed_can = safety_min(max_curvature_accel_can, limits.max_curvature);

      // the minimum and maximum curvature allowed based on the measured curvature
      const int lowest_desired_curvature_error = curvature_state.meas.min - limits.max_curvature_error - 1;
      const int highest_desired_curvature_error = curvature_state.meas.max + limits.max_curvature_error + 1;

      if (curvature_state.desired_last < lowest_desired_curvature_error) {
        // demand winding up: never require more than the relaxed step, never past the error band edge,
        // and never past what openpilot can reach (lat accel or max_curvature).
        const int required = safety_min(safety_min(curvature_state.desired_last + max_curvature_delta_relaxed_can,
                                                   lowest_desired_curvature_error), max_curvature_relaxed_can);
        lowest_desired_curvature = safety_max(lowest_desired_curvature, required);  // can't widen the rate limit window

      } else if (curvature_state.desired_last > highest_desired_curvature_error) {
        const int required = safety_max(safety_max(curvature_state.desired_last - max_curvature_delta_relaxed_can,
                                                   highest_desired_curvature_error), -max_curvature_relaxed_can);
        highest_desired_curvature = safety_min(highest_desired_curvature, required);

      } else {
        // already inside error boundary, don't allow commanding outside it
        highest_desired_curvature = safety_min(highest_desired_curvature, highest_desired_curvature_error);
        lowest_desired_curvature = safety_max(lowest_desired_curvature, lowest_desired_curvature_error);
      }
    }
    if (safety_max_limit_check(desired_curvature, highest_desired_curvature, lowest_desired_curvature)) {
      violation = true;
    }

    // *** real time rate limit check ***
    if (rt_curvature_rate_limit_check(limits)) {
      violation = true;
    }
  }
  curvature_state.desired_last = desired_curvature;

  // Curvature must be 0 while not steering
  if (!steer_control_enabled) {
    if (desired_curvature != 0) {
      violation = true;
    }
  }

  // *** steer power check ***
  // allow power winddown after disengage to prevent EPS fault
  if (limits.max_steer_power != 0) {
    if (safety_max_limit_check(steer_power, limits.max_steer_power, 0)) {
      violation = true;
    }
    if ((steer_power != 0) && !steer_control_enabled) {
      violation = true;
    }
    if (!controls_allowed && (steer_power != 0) && (steer_power >= curvature_state.steer_power_last)) {
      violation = true;
    }
    curvature_state.steer_power_last = steer_power;
  } else {
    // No curvature control allowed when controls are not allowed
    if (!controls_allowed && steer_control_enabled) {
      violation = true;
    }
  }

  if (violation) {
    curvature_state.desired_last = 0;
  }

  return violation;
}

static float get_curvature_factor(const float speed, const AngleSteeringParams params) {
  // Matches VehicleModel.curvature_factor()
  const float slip_term = params.slip_factor * (speed * speed);
  return (float)((1.0 / (1.0 - slip_term)) / params.wheelbase);
}

static float get_angle_from_curvature(const float curvature, const float curvature_factor, const AngleSteeringParams params) {
  // Matches VehicleModel.get_steer_from_curvature()
  static const float RAD_TO_DEG = 57.29577951308232f;
  return curvature * params.steer_ratio / curvature_factor * RAD_TO_DEG;
}

bool steer_angle_cmd_checks_vm(int desired_angle, bool steer_control_enabled, const AngleSteeringLimits limits,
                               const AngleSteeringParams params) {
  // This check uses a simple vehicle model to allow for constant lateral acceleration and jerk limits across all speeds.
  // TODO: remove the inaccurate breakpoint angle limiting function above and always use this one

  // Highway curves are rolled in the direction of the turn, add tolerance to compensate
  static const float MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL + (EARTH_G * AVERAGE_ROAD_ROLL);  // ~3.6 m/s^2
  // Lower than ISO 11270 lateral jerk limit, which is 5.0 m/s^3
  static const float MAX_LATERAL_JERK = 3.0f + (EARTH_G * AVERAGE_ROAD_ROLL);  // ~3.6 m/s^3

  const float fudged_speed = safety_maxf((float)(((double)vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.0), 1.0f);
  const float curvature_factor = get_curvature_factor(fudged_speed, params);

  bool violation = false;

  if (controls_allowed && steer_control_enabled) {
    // *** ISO lateral jerk limit ***
    // calculate maximum angle rate per second
    const float max_curvature_rate_sec = MAX_LATERAL_JERK / (fudged_speed * fudged_speed);
    const float max_angle_rate_sec = get_angle_from_curvature(max_curvature_rate_sec, curvature_factor, params);

    // finally get max angle delta per frame
    const float max_angle_delta = max_angle_rate_sec / (float)limits.frequency;
    const int max_angle_delta_can = safety_float_offset_to_int(max_angle_delta * limits.angle_deg_to_can, 1.0);

    // NOTE: symmetric up and down limits
    const int highest_desired_angle = desired_angle_last + max_angle_delta_can;
    const int lowest_desired_angle = desired_angle_last - max_angle_delta_can;

    if (safety_max_limit_check(desired_angle, highest_desired_angle, lowest_desired_angle)) {
      violation = true;
    }

    // *** ISO lateral accel limit ***
    const float max_curvature = MAX_LATERAL_ACCEL / (fudged_speed * fudged_speed);
    const float max_angle = get_angle_from_curvature(max_curvature, curvature_factor, params);
    const int max_angle_can = safety_float_offset_to_int(max_angle * limits.angle_deg_to_can, 1.0);

    if (safety_max_limit_check(desired_angle, max_angle_can, -max_angle_can)) {
      violation = true;
    }

    // *** angle real time rate limit check ***
    if (rt_angle_rate_limit_check(limits)) {
      violation = true;
    }
  }
  desired_angle_last = desired_angle;

  // Angle should either be 0 or same as current angle while not steering
  if (!steer_control_enabled) {
    if (steer_angle_cmd_inactive_check(desired_angle, limits.max_angle)) {
      violation = true;
    }
  }

  // No angle control allowed when controls are not allowed
  if (!controls_allowed) {
    if (steer_control_enabled) {
      violation = true;
    }
  }

  // reset to current angle if either controls is not allowed or there's a violation
  if (violation || !controls_allowed) {
    desired_angle_last = safety_clamp(angle_meas.values[0], -limits.max_angle, limits.max_angle);
  }

  return violation;
}
