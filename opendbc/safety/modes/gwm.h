#pragma once

#include "opendbc/safety/safety_declarations.h"

#define GWM_STEERING_AND_CRUISE 0xA1U // RX from STEER_AND_AP_STALK
#define GWM_GAS                 0x60U // RX from CAR_OVERALL_SIGNALS
#define GWM_BRAKE               0x137U // RX from BRAKE
#define GWM_SPEED               0x13B // RX from WHEEL_SPEEDS
#define GWM_LANE_KEEP_ASSIST    0xA1U // TX from OP,  EPS

// CAN bus
#define GWM_MAIN_BUS 0U
#define GWM_CAM_BUS  2U

static uint8_t gwm_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0;
  if (msg->addr == GWM_SPEED) {
    cnt = msg->data[47] & 0xFU;
  } else if (msg->addr == GWM_BRAKE) {
    cnt = msg->data[31] & 0xFU;
  } else {
    cnt = msg->data[7] & 0xFU;
  }
  return cnt;
}

static uint32_t gwm_get_checksum(const CANPacket_t *msg) {
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
  int len = GET_LEN(msg);
  uint8_t crc = 0x00;
  const uint8_t poly = 0x1D;
  const uint8_t xor_out = 0x2D;
  if (msg->addr == GWM_STEERING_AND_CRUISE) { // CRC8 only work for this message
    for (int i = 1; i < len; i++) {
      uint8_t byte = msg->data[i];
      crc ^= byte;
      for (int bit = 0; bit < 8; bit++) {
        if (crc & 0x80) {
          crc = (crc << 1) ^ poly;
        } else {
          crc <<= 1;
        }
        crc &= 0xFF;
      }
    }
    chksum = crc ^ xor_out;
  } else {
  }
  return chksum;
}

static void gwm_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == GWM_MAIN_BUS) {
    if (msg->addr == GWM_GAS) {
      gas_pressed = msg->data[9] > 0U; // GAS_POSITION
    }
    if (msg->addr == GWM_STEERING_AND_CRUISE) {
      int angle_meas_new = (((msg->data[1] & 0x3FU) << 7) | msg->data[2] & 0xFEU); // STEERING_ANGLE
      update_sample(&angle_meas, angle_meas_new);
    }
    if (msg->addr == GWM_SPEED) {
      int speed = ((msg->data[41] & 0x1FU) << 8) | msg->data[42]; // REAR_LEFT_WHEEL_SPEED
      vehicle_moving = speed > 0;
      UPDATE_VEHICLE_SPEED(speed * 0.01 * KPH_TO_MS);
    }
    if (msg->addr == GWM_STEERING_AND_CRUISE) {
      pcm_cruise_check((msg->data[5] >> 7) & 1U); // AP_ENABLE_COMMAND
    }
    if (msg->addr == GWM_BRAKE) {
      brake_pressed = ((msg->data[25] << 8) | msg->data[26] & 0xF8U) > 0U; // BRAKE_PRESSURE
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
  return tx;
}

static safety_config gwm_init(uint16_t param) {
  UNUSED(param);
  static const CanMsg GWM_TX_MSGS[] = {
    {GWM_STEERING_AND_CRUISE, GWM_MAIN_BUS, 8, .check_relay = true}, // EPS steering
  };

  static RxCheck psa_rx_checks[] = {
    {.msg = {{PSA_HS2_DAT_MDD_CMD_452, PSA_ADAS_BUS, 6, 20U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},                        // cruise state
    {.msg = {{PSA_HS2_DYN_ABR_38D, PSA_MAIN_BUS, 8, 25U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},                            // speed
    {.msg = {{PSA_STEERING_ALT, PSA_MAIN_BUS, 7, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // steering angle
    {.msg = {{PSA_STEERING, PSA_MAIN_BUS, 7, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},     // driver torque
    {.msg = {{PSA_DYN_CMM, PSA_MAIN_BUS, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},      // gas pedal
    {.msg = {{PSA_DAT_BSI, PSA_CAM_BUS, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},        // brake
  };

  return BUILD_SAFETY_CFG(psa_rx_checks, GWM_TX_MSGS);
}

const safety_hooks psa_hooks = {
  .init = psa_init,
  .rx = gwm_rx_hook,
  .tx = gwm_tx_hook,
  .get_counter = gwm_get_checksum,
  .get_checksum = gwm_get_checksum,
  .compute_checksum = gwm_compute_checksum,
};
