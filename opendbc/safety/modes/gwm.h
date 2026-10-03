#pragma once

#include "opendbc/safety/declarations.h"

#define GWM_ADAS_ACTIVATION      0xA1U // RX from STEER_AND_AP_STALK
#define GWM_GAS                  0x60U // RX from CAR_OVERALL_SIGNALS
#define GWM_BRAKE               0x120U // RX from BRAKE2
#define GWM_SPEED               0x13BU // RX from WHEEL_SPEEDS
#define GWM_RX_STEER_RELATED    0x147U // RX from EPS to CAMERA
#define GWM_STEER_CMD           0x12BU // TX from OP to EPS
#define GWM_CRUISE              0x2ABU
#define GWM_LONG_CONTROL        0x143U // TX from OP to PCM
#define GWM_BLIND_SPOT          0x16FU
#define GWM_HUD                 0x23DU

// CAN bus
#define GWM_MAIN_BUS 0U
#define GWM_CAMERA_BUS  2U

// 64-byte messages carry an independent CRC + counter per 8-byte block.
// For GWM_RX_STEER_RELATED, block B (bytes 8-15) holds the EPS/driver torque
// signals used below, so that is the block validated here.
static uint8_t gwm_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0;
  if ((msg->addr == GWM_SPEED) || (msg->addr == GWM_ADAS_ACTIVATION)) {
    cnt = msg->data[7] & 0xFU;
  } else if (msg->addr == GWM_RX_STEER_RELATED) {
    cnt = msg->data[15] & 0xFU;
  } else {
  }
  return cnt;
}

static uint32_t gwm_get_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if ((msg->addr == GWM_SPEED) || (msg->addr == GWM_ADAS_ACTIVATION) || (msg->addr == GWM_BRAKE)) {
    chksum = msg->data[0] & 0xFFU;
  } else if ((msg->addr == GWM_RX_STEER_RELATED) || (msg->addr == GWM_GAS)) {
    // block B: CRC at byte 8 covers the safety-relevant signals in bytes 9-15
    chksum = msg->data[8] & 0xFFU;
  } else {
  }
  return chksum;
}

static uint32_t gwm_compute_checksum(const CANPacket_t *msg) {
  uint8_t crc = 0x00;
  const uint8_t poly = 0x1D;
  uint8_t xor_out = 0x00;
  int start = 1;

  // xor_out constants derived from logged frames (route 075b133b6181e058/00000163)
  if (msg->addr == GWM_ADAS_ACTIVATION) {
    xor_out = 0x2DU;
  } else if (msg->addr == GWM_SPEED) {
    xor_out = 0x7FU;
  } else if (msg->addr == GWM_BRAKE) {
    // block A: CRC at byte 0 covers bytes 1-7 (BRAKE pressed bit)
    xor_out = 0xEEU;
  } else if (msg->addr == GWM_RX_STEER_RELATED) {
    // block B: CRC at byte 8 covers bytes 9-15
    xor_out = 0x61U;
    start = 9;
  } else if (msg->addr == GWM_GAS) {
    // block B: CRC at byte 8 covers bytes 9-15 (GAS_POSITION)
    xor_out = 0x95U;
    start = 9;
  } else {
  }

  for (int i = start; i < (start + 7); i++) {
    uint8_t byte = msg->data[i];
    crc ^= byte;
    for (int bit = 0; bit < 8; bit++) {
      if ((crc & 0x80U) != 0U) {
        crc = (crc << 1) ^ poly;
      } else {
        crc <<= 1;
      }
      crc &= 0xFFU;
    }
  }
  uint8_t chksum = crc ^ xor_out;
  return chksum;
}

static void gwm_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == GWM_MAIN_BUS) {
    // GAS_POSITION
    if (msg->addr == GWM_GAS) {
      gas_pressed = msg->data[9] > 0U;
    }

    if (msg->addr == GWM_SPEED) {
      uint32_t fl = ((msg->data[1] << 8) | msg->data[2]) & 0x1FFFU;
      uint32_t fr = ((msg->data[3] << 8) | msg->data[4]) & 0x1FFFU;
      uint32_t rl = ((msg->data[41] << 8) | msg->data[42]) & 0x1FFFU;
      uint32_t rr = ((msg->data[43] << 8) | msg->data[44]) & 0x1FFFU;
      float speed = (float)((fr + rr + rl + fl) / 4.0f * 0.05924739 * KPH_TO_MS);
      vehicle_moving = speed > 0.0f;
      UPDATE_VEHICLE_SPEED(speed);
    }

    if (msg->addr == GWM_BRAKE) {
      brake_pressed = GET_BIT(msg, 11U);
    }

    if (msg->addr == GWM_RX_STEER_RELATED) {
      int torque_meas_new = ((msg->data[13] & 0x7U) << 8) | (msg->data[14]);
      torque_meas_new = to_signed(torque_meas_new, 11) + 548;
      update_sample(&torque_meas, torque_meas_new);

      // increase torque_meas by 1 to be conservative on rounding
      torque_meas.min--;
      torque_meas.max++;
    }

  }

  if (msg->bus == GWM_CAMERA_BUS) {
    // CRUISE_STATE_2 in ACC (0x2AB): 0-2 = deactivated, >2 = active
    if (msg->addr == GWM_CRUISE) {
      int cruise_state = (msg->data[18] >> 3) & 0x7U;
      pcm_cruise_check(cruise_state > 2);
    }
  }
}

static bool gwm_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits GWM_TORQUE_STEERING_LIMITS = {
    .max_torque = 253,
    .max_rate_up = 4,
    .max_rate_down = 6,
    .max_torque_error = 80,
    .max_rt_delta = 100,
    .type = TorqueMotorLimited,
  };

  const LongitudinalLimits GWM_LONG_LIMITS = {
    .max_gas = 4577,
    .min_gas = -10,
    .inactive_gas = 0,
    .max_brake = 107,
  };

  bool tx = true;
  bool violation = false;

  if (msg->bus == GWM_MAIN_BUS) {
    if (msg->addr == GWM_STEER_CMD) {
      int desired_torque = (((msg->data[12] & 0x7FU) << 3) | ((msg->data[13] & 0xE0U) >> 5));
      desired_torque = to_signed(desired_torque, 10) + 1;
      bool steer_req = GET_BIT(msg, 125U);
      violation |= steer_torque_cmd_checks(desired_torque, steer_req, GWM_TORQUE_STEERING_LIMITS);
    }

    if (msg->addr == GWM_LONG_CONTROL) {
      int brake_raw = msg->data[13];
      brake_raw = 181 - brake_raw;
      violation |= longitudinal_brake_checks(brake_raw, GWM_LONG_LIMITS);

      int gas_raw = ((msg->data[27] & 0x1FU) << 8) | (msg->data[28]);
      gas_raw = gas_raw - 192;
      violation |= longitudinal_gas_checks(gas_raw, GWM_LONG_LIMITS);
    }
  }

  if (violation) {
    tx = false;
  }
  return tx;
}

static safety_config gwm_init(uint16_t param) {
  static const CanMsg GWM_TX_MSGS[] = {
    {GWM_ADAS_ACTIVATION, GWM_CAMERA_BUS, 8, .check_relay = false}, // Cancel command
    {GWM_RX_STEER_RELATED, GWM_CAMERA_BUS, 64, .check_relay = true}, // EPS steering feedback to camera
    {GWM_STEER_CMD, GWM_MAIN_BUS, 64, .check_relay = true}, // Steering command
    {GWM_HUD, GWM_MAIN_BUS, 64, .check_relay = true}, // HUD and dashboard
  };

  static const CanMsg GWM_LONG_TX_MSGS[] = {
    {GWM_ADAS_ACTIVATION, GWM_CAMERA_BUS, 8, .check_relay = false}, // Cancel command
    {GWM_RX_STEER_RELATED, GWM_CAMERA_BUS, 64, .check_relay = true}, // EPS steering feedback to camera
    {GWM_STEER_CMD, GWM_MAIN_BUS, 64, .check_relay = true}, // Steering command
    {GWM_LONG_CONTROL, GWM_MAIN_BUS, 64, .check_relay = true}, // Longitudinal control message from camera
    {GWM_HUD, GWM_MAIN_BUS, 64, .check_relay = true}, // HUD and dashboard
  };

  static RxCheck gwm_rx_checks[] = {
    {.msg = {{GWM_ADAS_ACTIVATION, GWM_MAIN_BUS, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // cruise state, steering angle, steer rate
    {.msg = {{GWM_SPEED, GWM_MAIN_BUS, 64, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // speed
    // GAS/BRAKE checksums validated (xor derived from logs); counters left ignored, they
    // show a systematic ~1/15 irregularity in logs that would cause false counter faults.
    {.msg = {{GWM_GAS, GWM_MAIN_BUS, 64, 50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // gas pedal
    {.msg = {{GWM_BRAKE, GWM_MAIN_BUS, 64, 50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // brake2
    {.msg = {{GWM_RX_STEER_RELATED, GWM_MAIN_BUS, 64, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // eps feedback to camera
    {.msg = {{GWM_STEER_CMD, GWM_CAMERA_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // copy stock steering cmd
    {.msg = {{GWM_CRUISE, GWM_CAMERA_BUS, 64, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // CRUISE_STATE, ACC
    {.msg = {{GWM_LONG_CONTROL, GWM_CAMERA_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // Longitudinal control message from camera
    {.msg = {{GWM_BLIND_SPOT, GWM_MAIN_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // Blind spot monitor
    {.msg = {{GWM_HUD, GWM_CAMERA_BUS, 64, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // HUD and dashboard
  };

  bool gwm_longitudinal = false;
#ifdef ALLOW_DEBUG
  const int FLAG_GWM_LONG_CONTROL = 1;
  gwm_longitudinal = GET_FLAG(param, FLAG_GWM_LONG_CONTROL);
#else
  SAFETY_UNUSED(param);
#endif

  // FIXME: cppcheck thinks that gwm_longitudinal is always false. This is not true
  // if ALLOW_DEBUG is defined but cppcheck is run without ALLOW_DEBUG
  // cppcheck-suppress knownConditionTrueFalse
  return gwm_longitudinal ? BUILD_SAFETY_CFG(gwm_rx_checks, GWM_LONG_TX_MSGS) : \
                            BUILD_SAFETY_CFG(gwm_rx_checks, GWM_TX_MSGS);
}

const safety_hooks gwm_hooks = {
  .init = gwm_init,
  .rx = gwm_rx_hook,
  .tx = gwm_tx_hook,
  .get_counter = gwm_get_counter,
  .get_checksum = gwm_get_checksum,
  .compute_checksum = gwm_compute_checksum,
};
