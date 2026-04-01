#pragma once

#include "opendbc/safety/modes/chrysler_common.h"

// SRT uses same addresses as CHRYSLER_PACIFICA
#define CHRYSLER_SRT_EPS_2          0x220
#define CHRYSLER_SRT_ESP_1          0x140
#define CHRYSLER_SRT_ECM_5          0x22F
#define CHRYSLER_SRT_DAS_3          0x1F4
#define CHRYSLER_SRT_DAS_6          0x2A6
#define CHRYSLER_SRT_LKAS_COMMAND   0x292
#define CHRYSLER_SRT_CRUISE_BUTTONS 0x23B

static safety_config chrysler_srt_init(uint16_t param) {
  SAFETY_UNUSED(param);

  static RxCheck chrysler_srt_rx_checks[] = {
    {.msg = {{CHRYSLER_SRT_EPS_2, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_SRT_ESP_1, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{514, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_SRT_ECM_5, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_SRT_DAS_3, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  static const CanMsg CHRYSLER_SRT_TX_MSGS[] = {
    {CHRYSLER_SRT_CRUISE_BUTTONS, 0, 3, .check_relay = false},
    {CHRYSLER_SRT_LKAS_COMMAND,   0, 6, .check_relay = true},
    {CHRYSLER_SRT_DAS_6,          0, 8, .check_relay = true},
  };

  return BUILD_SAFETY_CFG(chrysler_srt_rx_checks, CHRYSLER_SRT_TX_MSGS);
}

static void chrysler_srt_rx_hook(const CANPacket_t *msg) {
  // Measured EPS torque
  if ((msg->bus == 0U) && (msg->addr == CHRYSLER_SRT_EPS_2)) {
    int torque_meas_new = ((msg->data[4] & 0x7U) << 8) + msg->data[5] - 1024U;
    update_sample(&torque_meas, torque_meas_new);
  }

  // enter/exit controls on ACC engage/disengage
  if ((msg->bus == 0U) && (msg->addr == CHRYSLER_SRT_DAS_3)) {
    bool cruise_engaged = GET_BIT(msg, 21U);
    pcm_cruise_check(cruise_engaged);
  }

  // update vehicle moving
  if ((msg->bus == 0U) && (msg->addr == 514U)) {
    int speed_l = (msg->data[0] << 4) + (msg->data[1] >> 4);
    int speed_r = (msg->data[2] << 4) + (msg->data[3] >> 4);
    vehicle_moving = (speed_l != 0) || (speed_r != 0);
  }

  // exit controls on gas press
  if ((msg->bus == 0U) && (msg->addr == CHRYSLER_SRT_ECM_5)) {
    gas_pressed = msg->data[0U] != 0U;
  }

  // exit controls on brake press
  if ((msg->bus == 0U) && (msg->addr == CHRYSLER_SRT_ESP_1)) {
    brake_pressed = ((msg->data[0U] & 0xFU) >> 2U) == 1U;
  }
}

static bool chrysler_srt_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits CHRYSLER_SRT_STEERING_LIMITS = {
    .max_torque = 261,
    .max_rt_delta = 112,
    .max_rate_up = 6,
    .max_rate_down = 6,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };

  bool tx = true;

  // STEERING
  if (msg->addr == CHRYSLER_SRT_LKAS_COMMAND) {
    int desired_torque = ((msg->data[0] & 0x7U) << 8) | msg->data[1];
    desired_torque -= 1024;
    bool steer_req = GET_BIT(msg, 4U);
    if (steer_torque_cmd_checks(desired_torque, steer_req, CHRYSLER_SRT_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // CRUISE BUTTONS
  if (msg->addr == CHRYSLER_SRT_CRUISE_BUTTONS) {
    const bool is_cancel = msg->data[0] == 1U;
    const bool is_resume = msg->data[0] == 0x10U;
    const bool allowed = is_cancel || (is_resume && controls_allowed);
    if (!allowed) {
      tx = false;
    }
  }

  return tx;
}

static uint8_t chrysler_srt_get_counter(const CANPacket_t *msg) {
  return (uint8_t)(msg->data[6] >> 4);
}

const safety_hooks chrysler_srt_hooks = {
  .init = chrysler_srt_init,
  .rx = chrysler_srt_rx_hook,
  .tx = chrysler_srt_tx_hook,
  .get_counter = chrysler_srt_get_counter,
  .get_checksum = chrysler_get_checksum,
  .compute_checksum = chrysler_compute_checksum,
};