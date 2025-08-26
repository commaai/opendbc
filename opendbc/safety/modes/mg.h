#pragma once

#include "opendbc/safety/safety_declarations.h"

static void mg_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0)  {
    // Vehicle speed
    if (msg->addr == 0x353) {
      float speed = (((msg->data[0] & 0x7FU) << 8) | msg->data[1]) * 0.015625;
      vehicle_moving = speed > 0.0;
      UPDATE_VEHICLE_SPEED(speed * KPH_TO_MS);
    }

    // Gas pressed
    if (msg->addr == 0xaf) {
      gas_pressed = msg->data[0] != 0;
    }

    // Driver torque
    if (msg->addr == 0x1ec) {
      int torque_driver_new = ((msg->data[4] & 0x7U) << 8) | msg->data[5];
      torque_driver_new = torque_driver_new - 1024U;
      update_sample(&torque_driver, torque_driver_new);
    }

    // Brake pressed
    if (msg->addr == 0x1b6) {
      brake_pressed = GET_BIT(msg, 10U);
    }

    // Cruise state
    if (msg->addr == 0x242) {
      int cruise_state = (msg->data[5] & 0x38U) >> 3;
      bool cruise_engaged = (cruise_state == 2) ||  // Active
                            (cruise_state == 3);    // Override
      pcm_cruise_check(cruise_engaged);
    }
  }
}

static bool mg_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits MG_STEERING_LIMITS = {
    .max_torque = 300,
    .max_rate_up = 10,
    .max_rate_down = 15,
    .max_rt_delta = 125,
    .driver_torque_multiplier = 2,
    .driver_torque_allowance = 100,
    .type = TorqueDriverLimited,
  };

  bool tx = true;
  bool violation = false;

  // Steering control
  if (msg->addr == 0x1fd) {
    int desired_torque = ((msg->data[0] & 0x7U) << 8) | msg->data[1];
    desired_torque = desired_torque - 1024U;
    bool steer_req = GET_BIT(msg, 35U);

    violation |= steer_torque_cmd_checks(desired_torque, steer_req, MG_STEERING_LIMITS);
  }

  if (violation) {
    tx = false;
  }

  return tx;
}

static safety_config mg_init(uint16_t param) {
  // 0x1fd = FVCM_HSC2_FrP03
  static const CanMsg MG_TX_MSGS[] = {{0x1fd, 0, 8, .check_relay = true}};

  static RxCheck mg_rx_checks[] = {
    {.msg = {{0x353, 0, 8, .frequency = 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // SCS_HSC2_FrP15 (speed)
    {.msg = {{0xaf, 0, 8, .frequency = 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // GW_HSC2_HCU_FrP00 (gas pedal)
    {.msg = {{0x1b6, 0, 8, .frequency = 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // EHBS_HSC2_FrP00 (brake pedal)
    {.msg = {{0x1ec, 0, 8, .frequency = 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // EPS_HSC2_FrP03 (driver torque)
    {.msg = {{0x242, 0, 8, .frequency = 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // RADAR_HSC2_FrP00 (cruise state)
  };

  UNUSED(param);
  return BUILD_SAFETY_CFG(mg_rx_checks, MG_TX_MSGS);
}

const safety_hooks mg_hooks = {
  .init = mg_init,
  .rx = mg_rx_hook,
  .tx = mg_tx_hook,
};
