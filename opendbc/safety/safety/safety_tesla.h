#pragma once

#include "safety_declarations.h"

static bool tesla_longitudinal = false;
static bool tesla_stock_aeb = false;

// Only rising edges while controls are not allowed are considered for these systems:
// TODO: Only LKAS (non-emergency) is currently supported since we've only seen it
static bool tesla_stock_lkas = false;
static bool tesla_stock_lkas_prev = false;

// Only Summon is currently supported due to Autopark not setting Autopark state properly
static bool tesla_autopark = false;
static bool tesla_autopark_prev = false;

static void tesla_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if (bus == 0) {
    // Steering angle: (0.1 * val) - 819.2 in deg.
    if (addr == 0x370) {
      // Store it 1/10 deg to match steering request
      const int angle_meas_new = (((GET_BYTE(to_push, 4) & 0x3FU) << 8) | GET_BYTE(to_push, 5)) - 8192U;
      update_sample(&angle_meas, angle_meas_new);

      const int hands_on_level = GET_BYTE(to_push, 4) >> 6;  // EPAS3S_handsOnLevel
      const int eac_status = GET_BYTE(to_push, 6) >> 5;  // EPAS3S_eacStatus
      const int eac_error_code = GET_BYTE(to_push, 2) >> 4;  // EPAS3S_eacErrorCode

      // Disengage on normal user override, or if high angle rate fault from user overriding extremely quickly
      steering_disengage = (hands_on_level >= 3) || ((eac_status == 0) && (eac_error_code == 9));
    }

    // Vehicle speed
    if (addr == 0x257) {
      // Vehicle speed: ((val * 0.08) - 40) / MS_TO_KPH
      float speed = ((((GET_BYTE(to_push, 2) << 4) | (GET_BYTE(to_push, 1) >> 4)) * 0.08) - 40) / 3.6;
      UPDATE_VEHICLE_SPEED(speed);
    }

    // Gas pressed
    if (addr == 0x118) {
      gas_pressed = (GET_BYTE(to_push, 4) != 0U);
    }

    // Brake pressed
    if (addr == 0x39d) {
      brake_pressed = (GET_BYTE(to_push, 2) & 0x03U) == 2U;
    }

    // Cruise and Autopark/Summon state
    if (addr == 0x286) {
      // Autopark state
      int autopark_state = (GET_BYTE(to_push, 3) >> 1) & 0x0FU;  // DI_autoparkState
      bool tesla_autopark_now = (autopark_state == 3) ||  // ACTIVE
                                (autopark_state == 4) ||  // COMPLETE
                                (autopark_state == 9);    // SELFPARK_STARTED

      // Only consider rising edges while controls are not allowed
      if (tesla_autopark_now && !tesla_autopark_prev && !controls_allowed) {
        tesla_autopark = true;
      }
      if (!tesla_autopark_now) {
        tesla_autopark = false;
      }
      tesla_autopark_prev = tesla_autopark_now;

      // Cruise state
      int cruise_state = (GET_BYTE(to_push, 1) >> 4) & 0x07U;
      bool cruise_engaged = (cruise_state == 2) ||  // ENABLED
                            (cruise_state == 3) ||  // STANDSTILL
                            (cruise_state == 4) ||  // OVERRIDE
                            (cruise_state == 6) ||  // PRE_FAULT
                            (cruise_state == 7);    // PRE_CANCEL
      cruise_engaged = cruise_engaged && !tesla_autopark;

      vehicle_moving = cruise_state != 3; // STANDSTILL
      pcm_cruise_check(cruise_engaged);
    }
  }

  if (bus == 2) {
    // DAS_control
    if (addr == 0x2b9) {
      // "AEB_ACTIVE"
      tesla_stock_aeb = (GET_BYTE(to_push, 2) & 0x03U) == 1U;
    }

    // DAS_steeringControl
    if (addr == 0x488) {
      int steering_control_type = GET_BYTE(to_push, 2) >> 6;
      bool tesla_stock_lkas_now = steering_control_type == 2;  // "LANE_KEEP_ASSIST"

      // Only consider rising edges while controls are not allowed
      if (tesla_stock_lkas_now && !tesla_stock_lkas_prev && !controls_allowed) {
        tesla_stock_lkas = true;
      }
      if (!tesla_stock_lkas_now) {
        tesla_stock_lkas = false;
      }
      tesla_stock_lkas_prev = tesla_stock_lkas_now;
    }
  }
}


static bool tesla_tx_hook(const CANPacket_t *to_send) {
  const AngleSteeringLimits TESLA_STEERING_LIMITS = {
    .max_angle = 3600,  // 360 deg, EPAS faults above this
    .angle_deg_to_can = 10,
    .angle_rate_up_lookup = {
      {0., 5., 25.},
      {2.5, 1.5, 0.2}
    },
    .angle_rate_down_lookup = {
      {0., 5., 25.},
      {5., 2.0, 0.3}
    },
  };

  const LongitudinalLimits TESLA_LONG_LIMITS = {
    .max_accel = 425,       // 2 m/s^2
    .min_accel = 288,       // -3.48 m/s^2
    .inactive_accel = 375,  // 0. m/s^2
  };

  bool tx = true;
  int addr = GET_ADDR(to_send);
  bool violation = false;

  // Don't send any messages when Autopark is active
  if (tesla_autopark) {
    violation = true;
  }

  // Steering control: (0.1 * val) - 1638.35 in deg.
  if (addr == 0x488) {
    // We use 1/10 deg as a unit here
    int raw_angle_can = ((GET_BYTE(to_send, 0) & 0x7FU) << 8) | GET_BYTE(to_send, 1);
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = GET_BYTE(to_send, 2) >> 6;
    bool steer_control_enabled = steer_control_type == 1;  // ANGLE_CONTROL

    if (steer_angle_cmd_checks(desired_angle, steer_control_enabled, TESLA_STEERING_LIMITS)) {
      violation = true;
    }

    bool valid_steer_control_type = (steer_control_type == 0) ||  // NONE
                                    (steer_control_type == 1);    // ANGLE_CONTROL
    if (!valid_steer_control_type) {
      violation = true;
    }

    if (tesla_stock_lkas) {
      // Don't allow any steering commands when stock LKAS is active
      violation = true;
    }
  }

  // DAS_control: longitudinal control message
  if (addr == 0x2b9) {
    // No AEB events may be sent by openpilot
    int aeb_event = GET_BYTE(to_send, 2) & 0x03U;
    if (aeb_event != 0) {
      violation = true;
    }

    // Don't send long/cancel messages when the stock AEB system is active
    if (tesla_stock_aeb) {
      violation = true;
    }

    int raw_accel_max = ((GET_BYTE(to_send, 6) & 0x1FU) << 4) | (GET_BYTE(to_send, 5) >> 4);
    int raw_accel_min = ((GET_BYTE(to_send, 5) & 0x0FU) << 5) | (GET_BYTE(to_send, 4) >> 3);
    int acc_state = GET_BYTE(to_send, 1) >> 4;

    if (tesla_longitudinal) {
      // Prevent both acceleration from being negative, as this could cause the car to reverse after coming to standstill
      if ((raw_accel_max < TESLA_LONG_LIMITS.inactive_accel) && (raw_accel_min < TESLA_LONG_LIMITS.inactive_accel)) {
        violation = true;
      }

      // Don't allow any acceleration limits above the safety limits
      violation |= longitudinal_accel_checks(raw_accel_max, TESLA_LONG_LIMITS);
      violation |= longitudinal_accel_checks(raw_accel_min, TESLA_LONG_LIMITS);
    } else {
      // Can only send cancel longitudinal messages when not controlling longitudinal
      if (acc_state != 13) {  // ACC_CANCEL_GENERIC_SILENT
        violation = true;
      }

      // No actuation is allowed when not controlling longitudinal
      if ((raw_accel_max != TESLA_LONG_LIMITS.inactive_accel) || (raw_accel_min != TESLA_LONG_LIMITS.inactive_accel)) {
        violation = true;
      }
    }
  }

  if (violation) {
    tx = false;
  }

  return tx;
}

static bool tesla_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  if (bus_num == 2) {
    if (!tesla_autopark) {
      // APS_eacMonitor
      if (addr == 0x27d) {
        block_msg = true;
      }

      // DAS_steeringControl
      if ((addr == 0x488) && !tesla_stock_lkas) {
        block_msg = true;
      }

      // DAS_control
      if (tesla_longitudinal && (addr == 0x2b9) && !tesla_stock_aeb) {
        block_msg = true;
      }
    }
  }

  return block_msg;
}

static safety_config tesla_init(uint16_t param) {

  static const CanMsg TESLA_M3_Y_TX_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},   // DAS_steeringControl
    {0x2b9, 0, 8, .check_relay = false},                                   // DAS_control (for cancel)
    {0x27D, 0, 3, .check_relay = true, .disable_static_blocking = true},   // APS_eacMonitor
  };

  static const CanMsg TESLA_M3_Y_LONG_TX_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},  // DAS_steeringControl
    {0x2b9, 0, 8, .check_relay = true, .disable_static_blocking = true},  // DAS_control
    {0x27D, 0, 3, .check_relay = true, .disable_static_blocking = true},  // APS_eacMonitor
  };

  UNUSED(param);
#ifdef ALLOW_DEBUG
  const int TESLA_FLAG_LONGITUDINAL_CONTROL = 1;
  tesla_longitudinal = GET_FLAG(param, TESLA_FLAG_LONGITUDINAL_CONTROL);
#endif

  tesla_stock_aeb = false;
  tesla_stock_lkas = false;
  tesla_stock_lkas_prev = false;
  // we need to assume Autopark/Summon on startup since DI_state is a low freq msg.
  // this is so that we don't fault if starting while these systems are active
  tesla_autopark = true;
  tesla_autopark_prev = false;

  static RxCheck tesla_model3_y_rx_checks[] = {
    {.msg = {{0x2b9, 2, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 25U}, { 0 }, { 0 }}},   // DAS_control
    {.msg = {{0x488, 2, 4, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},   // DAS_steeringControl
    {.msg = {{0x257, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},   // DI_speed (speed in kph)
    {.msg = {{0x370, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}},  // EPAS3S_sysStatus (steering angle)
    {.msg = {{0x118, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}},  // DI_systemStatus (gas pedal)
    {.msg = {{0x39d, 0, 5, .ignore_checksum = true, .ignore_counter = true, .frequency = 25U}, { 0 }, { 0 }}},   // IBST_status (brakes)
    {.msg = {{0x286, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}},   // DI_state (acc state)
    {.msg = {{0x311, 0, 7, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}},   // UI_warning (blinkers, buckle switch & doors)
  };

  safety_config ret;
  if (tesla_longitudinal) {
    ret = BUILD_SAFETY_CFG(tesla_model3_y_rx_checks, TESLA_M3_Y_LONG_TX_MSGS);
  } else {
    ret = BUILD_SAFETY_CFG(tesla_model3_y_rx_checks, TESLA_M3_Y_TX_MSGS);
  }
  return ret;
}

const safety_hooks tesla_hooks = {
  .init = tesla_init,
  .rx = tesla_rx_hook,
  .tx = tesla_tx_hook,
  .fwd = tesla_fwd_hook,
};
