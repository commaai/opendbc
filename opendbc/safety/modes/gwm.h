#pragma once

#include "opendbc/safety/safety_declarations.h"

#define GWM_STEERING_AND_CRUISE 0xA1U  // RX from STEER_AND_AP_STALK
#define GWM_GAS                 0x60U  // RX from CAR_OVERALL_SIGNALS
#define GWM_BRAKE               0x120U
#define GWM_SPEED               0x13BU  // RX from WHEEL_SPEEDS
#define GWM_LANE_KEEP_ASSIST    0xA1U  // TX from OP,  EPS
#define GWM_RX_STEER_RELATED    0x147U // TX from OP to CAMERA
#define STEER_CMD               0x12BU // TX from OP, CAMERA to EPS
#define GWM_CRUISE              0x2ABU

// CAN bus
#define GWM_MAIN_BUS 0U
#define GWM_CAMERA_BUS  2U

static uint8_t gwm_get_counter(const CANPacket_t *msg) {
  // TO-DO: Each message has a different position; after finishing the port,
  // handle them one by one for each address
  uint8_t cnt = 0;
  if ((uint32_t)msg->addr == (uint32_t)GWM_SPEED) {
    cnt = msg->data[47] & 0xFU;
  } else if (msg->addr == GWM_BRAKE) {
    cnt = msg->data[31] & 0xFU;
  } else {
    cnt = msg->data[7] & 0xFU;
  }
  return cnt;
}

static uint32_t gwm_get_checksum(const CANPacket_t *msg) {
  // TO-DO: Each message has a different position; after finishing the port,
  // handle them one by one for each address
  uint8_t chksum = 0;
  if (msg->addr == GWM_SPEED) {
    chksum = msg->data[24] & 0xFFU;
  } else {
    chksum = msg->data[0] & 0xFFU;
  }
  return chksum;
}

static uint32_t gwm_compute_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  uint8_t crc = 0x00;
  const uint8_t poly = 0x1D;
  uint8_t xor_out = 0x00;
  int len = GET_LEN(msg);
  for (int i = 1; i < len; i++) {
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
  if (msg->addr == GWM_STEERING_AND_CRUISE) {
    xor_out = 0x2DU;
  } else if (msg->addr == GWM_RX_STEER_RELATED) {
    xor_out = 0x61U;
  } else if (msg->addr == STEER_CMD) {
    xor_out = 0x9BU;
  } else {
  }
  chksum = crc ^ xor_out;
  return chksum;
}

static void gwm_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
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

    if (msg->addr == 0x147U) {
      int torque_meas_new = ((msg->data[13] & 0x7U) << 8) | (msg->data[14]);
      torque_meas_new = to_signed(torque_meas_new, 11) + 548;
      update_sample(&torque_meas, torque_meas_new);
    }
  }

  if (msg->bus == 2U) {
    if (msg->addr == GWM_CRUISE) {
      int cruise_state = (msg->data[18] >> 3) & 0x7U;
      bool cruise_engaged = cruise_state > 2;
      pcm_cruise_check(cruise_engaged);
    }
  }
}

static bool gwm_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits GWM_TORQUE_STEERING_LIMITS = {
    .max_torque = 254,
    .max_rate_up = 3,
    .max_rate_down = 5,
    .max_torque_error = 70,
    .max_rt_delta = 100,
    .type = TorqueMotorLimited,
  };

  bool tx = true;

  if (msg->bus == 0U) {
    if (msg->addr == STEER_CMD) {
      int desired_torque = (((msg->data[12] & 0x7FU) << 3) | ((msg->data[13] & 0xE0U) >> 5)) + 1U;
      desired_torque = to_signed(desired_torque, 10);
      bool steer_req = GET_BIT(msg, 125U);
      if (steer_torque_cmd_checks(desired_torque, steer_req, GWM_TORQUE_STEERING_LIMITS)) {
        tx = false;
      }
    }
  }

  return tx;
}

static safety_config gwm_init(uint16_t param) {
  UNUSED(param);
  static const CanMsg GWM_TX_MSGS[] = {
    // {GWM_LANE_KEEP_ASSIST, GWM_MAIN_BUS, 8, .check_relay = false}, // EPS steering
    // {GWM_LANE_KEEP_ASSIST, GWM_CAMERA_BUS, 8, .check_relay = true}, // EPS steering
    {GWM_RX_STEER_RELATED, GWM_CAMERA_BUS, 64, .check_relay = true}, // EPS steering feedback to camera
    {STEER_CMD, GWM_MAIN_BUS, 64, .check_relay = true}, // Steering command
  };

  static RxCheck psa_rx_checks[] = {
    // {.msg = {{GWM_STEERING_AND_CRUISE, GWM_MAIN_BUS, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // cruise state, steering angle, driver torque
    {.msg = {{GWM_STEERING_AND_CRUISE, GWM_MAIN_BUS, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // cruise state, steering angle, driver torque
    {.msg = {{GWM_SPEED, GWM_MAIN_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // speed
    {.msg = {{GWM_GAS, GWM_MAIN_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // gas pedal
    {.msg = {{GWM_BRAKE, GWM_MAIN_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // brake2
    {.msg = {{GWM_RX_STEER_RELATED, GWM_MAIN_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // eps feedback to camera
    {.msg = {{STEER_CMD, GWM_CAMERA_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // copy stock steering cmd
    {.msg = {{GWM_CRUISE, 2U, 64, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // CRUISE_STATE, ACC
  };

  return BUILD_SAFETY_CFG(psa_rx_checks, GWM_TX_MSGS);
}

const safety_hooks gwm_hooks = {
  .init = gwm_init,
  .rx = gwm_rx_hook,
  .tx = gwm_tx_hook,
  .get_counter = gwm_get_counter,
  .get_checksum = gwm_get_checksum,
  .compute_checksum = gwm_compute_checksum,
};
