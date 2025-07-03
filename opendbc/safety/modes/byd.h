#pragma once

#include "opendbc/safety/safety_declarations.h"

static void byd_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if (bus == 0) {
    // current steering angle, factor -0.1 and little endian
    if (addr == 287) {
      int angle_meas_new = (GET_BYTES(to_push, 0, 2) & 0xFFFFU);
      // let it be CAN unit degree
      angle_meas_new = to_signed(angle_meas_new, 16);

      update_sample(&angle_meas, angle_meas_new);
    }

    // gas and brakes
    if (addr == 834) {
      gas_pressed = (GET_BYTE(to_push, 0) > 0U);
      brake_pressed = (GET_BYTE(to_push, 1) > 0U);
    }

    // vehicle speed
    if (addr == 290) {
      // average of FL and BL
      float fl_ms = (float)GET_BYTES(to_push, 0, 2) * 0.1f / 3.6f;
      float bl_ms = (float)GET_BYTES(to_push, 4, 2) * 0.1f / 3.6f;
      float speed = (fl_ms + bl_ms) * 0.5f;
      vehicle_moving = ABS(speed) > 0.1;
      UPDATE_VEHICLE_SPEED(speed);
    }

    // engage logic with buttons
    if (addr == 944) {
      // TODO: does it have to be on the rising edge
      int set_pressed = (GET_BYTE(to_push, 0) >> 3U) & 1U;
      int res_pressed = (GET_BYTE(to_push, 0) >> 4U) & 1U;
      int cancel = (GET_BYTE(to_push, 2) >> 3U) & 1U;

      if (set_pressed || res_pressed) {
        controls_allowed = true;
      }

      if (cancel != 0) {
        controls_allowed = false;
      }
    }
  }

  if (bus == 2) {
    // cruise enabled
    if (addr == 814) {
      bool engaged = (GET_BYTE(to_push, 5) >> 4) & 1U;
      pcm_cruise_check(engaged);
    }
  }
}

static bool byd_tx_hook(const CANPacket_t *to_send) {
  const AngleSteeringLimits BYD_STEERING_LIMITS = {
    .angle_deg_to_can = 10,
    .angle_rate_up_lookup = {
      {0., 5., 15.},
      {6., 4., 3.}
    },
    .angle_rate_down_lookup = {
      {0., 5., 15.},
      {8., 6., 4.}
    },
  };

  const LongitudinalLimits BYD_LONG_LIMITS = {
    .max_accel = 130,       // 2.83 m/s^2
    .min_accel = 50,        // -3.2 m/s^2
    .inactive_accel = 100,  // 0. m/s^2
  };

  bool tx = false;
  bool violation = false;
  int addr = GET_ADDR(to_send);

  // steer violation checks
  if (addr == 482) {

    int desired_angle = (GET_BYTES(to_send, 3, 2) & 0xFFFFU);
    bool lka_active = (GET_BYTE(to_send, 2) >> 5) & 1U;

    desired_angle = to_signed(desired_angle, 16);

    if (steer_angle_cmd_checks(desired_angle, lka_active, BYD_STEERING_LIMITS)) {
      violation = true;
    }
  }

  // acc violation checks
  if (addr == 814) {
    int desired_accel = GET_BYTE(to_send, 0);
    violation |= longitudinal_accel_checks(desired_accel, BYD_LONG_LIMITS);
  }

  if (violation) {
    tx = false;
  }
  return tx;
}

static safety_config byd_init(uint16_t param) {
  static const CanMsg BYD_TX_MSGS[] = {
    {482, 0, 8, .check_relay = true}, // STEERING_MODULE_ADAS
    {790, 0, 8, .check_relay = true}, // LKAS_HUD_ADAS
    {814, 0, 8, .check_relay = true}  // ACC_CMD
  };

  static RxCheck byd_rx_checks[] = {
    {.msg = {{287, 0, 5, .frequency = 100U}, { 0 }, { 0 }}}, // STEER_MODULE_2
    {.msg = {{290, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},  // WHEEL_SPEED
    {.msg = {{508, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},  // STEERING_TORQUE
    {.msg = {{834, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},  // PEDAL
    {.msg = {{944, 0, 8, .frequency = 20U}, { 0 }, { 0 }}},  // PCM_BUTTONS
    {.msg = {{814, 2, 8, .frequency = 50U}, { 0 }, { 0 }}},  // ACC_CMD
  };

  UNUSED(param);
  return BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
}


const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
};
