#pragma once

#include "opendbc/safety/safety_declarations.h"

#define GWM_STEERING_AND_CRUISE 0xA1U  // RX from STEER_AND_AP_STALK
#define GWM_GAS                 0x60U  // RX from CAR_OVERALL_SIGNALS
#define GWM_BRAKE               0x137U // RX from BRAKE
#define GWM_SPEED               0x13B  // RX from WHEEL_SPEEDS
#define GWM_LANE_KEEP_ASSIST    0xA1U  // TX from OP,  EPS
#define GWM_RX_STEER_RELATED    0x147U // TX from OP to CAMERA

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
  if ((uint32_t)msg->addr == (uint32_t)GWM_SPEED) {
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
    xor_out = 0x2D;
  } else if (msg->addr == GWM_RX_STEER_RELATED) {
    xor_out = 0x61;
  } else {
  }
  chksum = crc ^ xor_out;
  return chksum;
}

static void gwm_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == GWM_MAIN_BUS) {
    if (msg->addr == GWM_GAS) {
      gas_pressed = msg->data[9] > 0U; // GAS_POSITION
    }
    if ((uint32_t)msg->addr == (uint32_t)GWM_STEERING_AND_CRUISE) {
      int angle_meas_new = (((msg->data[1] & 0x3FU) << 7) | (msg->data[2] & 0xFEU)); // STEERING_ANGLE
      update_sample(&angle_meas, angle_meas_new);
      pcm_cruise_check((msg->data[5] >> 7) & 1U); // AP_ENABLE_COMMAND
    }
    if ((uint32_t)msg->addr == (uint32_t)GWM_SPEED) {
      uint32_t fl = (((uint16_t)msg->data[1] << 8) | msg->data[2]) & 0x1FFF;
      uint32_t fr = (((uint16_t)msg->data[3] << 8) | msg->data[4]) & 0x1FFF;
      uint32_t rl = (((uint16_t)msg->data[41] << 8) | msg->data[42]) & 0x1FFF;
      uint32_t rr = (((uint16_t)msg->data[43] << 8) | msg->data[44]) & 0x1FFF;
      float speed = (float)((fr + rr + rl + fl) / 4.0f * 0.05924739 * KPH_TO_MS);
      vehicle_moving = speed > 0.0f;
      UPDATE_VEHICLE_SPEED(speed);
    }
    if ((uint32_t)msg->addr == (uint32_t)GWM_BRAKE) {
      brake_pressed = ((msg->data[25] << 8) | (msg->data[26] & 0xF8U)) > 0U; // BRAKE_PRESSURE
    }
  }
}

static bool gwm_tx_hook(const CANPacket_t *msg) {
  bool tx = true;
  static const AngleSteeringLimits GWM_STEERING_LIMITS = {
    .max_angle = 3900,
    .angle_deg_to_can = 10,
    .angle_rate_up_lookup = {
      {0., 5., 25.},
      {2.5, 1.5, .2},
    },
    .angle_rate_down_lookup = {
      {0., 5., 25.},
      {5., 2., .3},
    },
  };

  // Safety check for LKA
  if (msg->addr == GWM_STEERING_AND_CRUISE) {
    // SET_ANGLE
    int desired_angle = ((msg->data[1] & 0x3FU) << 7) | (msg->data[2] & 0xFEU); // STEERING_ANGLE
    // TORQUE_FACTOR
    bool lka_active = (msg->data[4] & 0x3U) != 0U; // EPS_ACTUATING

    if (steer_angle_cmd_checks(desired_angle, lka_active, GWM_STEERING_LIMITS)) {
      tx = false;
    }
  }
  tx = true; // TO-DO remove this line after testing
  return tx;
}

static safety_config gwm_init(uint16_t param) {
  UNUSED(param);
  static const CanMsg GWM_TX_MSGS[] = {
    // {GWM_LANE_KEEP_ASSIST, GWM_MAIN_BUS, 8, .check_relay = false}, // EPS steering
    {GWM_LANE_KEEP_ASSIST, GWM_CAMERA_BUS, 8, .check_relay = true}, // EPS steering
    {GWM_RX_STEER_RELATED, GWM_CAMERA_BUS, 64, .check_relay = true}, // EPS steering feedback to camera
  };

  static RxCheck psa_rx_checks[] = {
    // {.msg = {{GWM_STEERING_AND_CRUISE, GWM_MAIN_BUS, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // cruise state, steering angle, driver torque
    {.msg = {{GWM_STEERING_AND_CRUISE, GWM_MAIN_BUS, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // cruise state, steering angle, driver torque
    {.msg = {{GWM_SPEED, GWM_MAIN_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // speed
    {.msg = {{GWM_GAS, GWM_MAIN_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // gas pedal
    {.msg = {{GWM_BRAKE, GWM_MAIN_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // brake
    {.msg = {{GWM_RX_STEER_RELATED, GWM_MAIN_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // eps feedback to camera
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
