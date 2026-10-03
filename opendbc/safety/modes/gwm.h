#pragma once

#include "opendbc/safety/declarations.h"

#define GWM_GAS           0x60U   // CAR_OVERALL_SIGNALS2
#define GWM_STALK         0xA1U   // STEER_AND_AP_STALK
#define GWM_BRAKE         0x120U  // BRAKE2
#define GWM_STEER_CMD     0x12BU  // STEER_CMD
#define GWM_WHEEL_SPEEDS  0x13BU  // WHEEL_SPEEDS
#define GWM_ACC_CMD       0x143U  // ACC_CMD
#define GWM_EPS           0x147U  // RX_STEER_RELATED
#define GWM_HUD           0x23DU  // LATERAL_STATE
#define GWM_ACC           0x2ABU  // ACC

#define GWM_MAIN_BUS 0U
#define GWM_CAM_BUS 2U

static uint8_t gwm_crc_lut[256];

// 64 byte messages are split into 8 byte blocks, each with a CRC8 in the first byte and a
// 0-14 counter in the low nibble of the last byte. Only the blocks with the signals we use are checked.
static int gwm_get_block(const CANPacket_t *msg) {
  int block = 0;
  if ((msg->addr == GWM_GAS) || (msg->addr == GWM_EPS)) {
    block = 1;
  } else if (msg->addr == GWM_ACC) {
    block = 2;
  } else {
  }
  return block;
}

static uint8_t gwm_block_crc(const CANPacket_t *msg, int block) {
  uint8_t xor_out = 0U;
  if (msg->addr == GWM_GAS) {
    xor_out = 0x95U;
  } else if (msg->addr == GWM_BRAKE) {
    xor_out = 0xEEU;
  } else if (msg->addr == GWM_WHEEL_SPEEDS) {
    xor_out = (block == 0) ? 0x7FU : 0x1AU;
  } else if (msg->addr == GWM_EPS) {
    xor_out = 0x61U;
  } else if (msg->addr == GWM_ACC) {
    xor_out = 0x40U;
  } else {
  }

  uint8_t crc = 0U;
  for (int i = (block * 8) + 1; i < ((block * 8) + 8); i++) {
    crc = gwm_crc_lut[crc ^ msg->data[i]];
  }
  return crc ^ xor_out;
}

static uint8_t gwm_get_counter(const CANPacket_t *msg) {
  return msg->data[(gwm_get_block(msg) * 8) + 7] & 0xFU;
}

static uint32_t gwm_get_checksum(const CANPacket_t *msg) {
  uint32_t checksum = msg->data[gwm_get_block(msg) * 8];
  if (msg->addr == GWM_WHEEL_SPEEDS) {
    // rear wheel speeds are in block 5
    checksum = (checksum << 8) | msg->data[40];
  }
  return checksum;
}

static uint32_t gwm_compute_checksum(const CANPacket_t *msg) {
  uint32_t checksum = gwm_block_crc(msg, gwm_get_block(msg));
  if (msg->addr == GWM_WHEEL_SPEEDS) {
    checksum = (checksum << 8) | gwm_block_crc(msg, 5);
  }
  return checksum;
}

static void gwm_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == GWM_MAIN_BUS) {
    if (msg->addr == GWM_GAS) {
      gas_pressed = msg->data[9] != 0U;  // GAS_POSITION
    }

    if (msg->addr == GWM_BRAKE) {
      brake_pressed = GET_BIT(msg, 11U);  // PEDAL_BRAKE_PRESSED
    }

    if (msg->addr == GWM_WHEEL_SPEEDS) {
      uint32_t fl = ((msg->data[1] & 0x1FU) << 8) | msg->data[2];
      uint32_t fr = ((msg->data[3] & 0x1FU) << 8) | msg->data[4];
      uint32_t rl = ((msg->data[41] & 0x1FU) << 8) | msg->data[42];
      uint32_t rr = ((msg->data[43] & 0x1FU) << 8) | msg->data[44];
      float speed = (float)((fl + fr + rl + rr) / 4.0 * 0.05924739 * KPH_TO_MS);
      vehicle_moving = speed > 0.0f;
      UPDATE_VEHICLE_SPEED(speed);
    }

    if (msg->addr == GWM_EPS) {
      int torque_meas_new = (((msg->data[13] & 0x7U) << 8) | msg->data[14]) - 1500U;  // B_RX_EPS_TORQUE
      update_sample(&torque_meas, torque_meas_new);
    }
  }

  if (msg->bus == GWM_CAM_BUS) {
    if (msg->addr == GWM_ACC) {
      // CRUISE_STATE_2: 1-2 standby, 3 engaged, 5 engaged with driver overriding
      uint8_t cruise_state = (msg->data[18] >> 3) & 0x7U;
      pcm_cruise_check((cruise_state == 3U) || (cruise_state == 5U));
    }
  }
}

static bool gwm_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits GWM_STEERING_LIMITS = {
    .max_torque = 253,
    .max_rate_up = 4,
    .max_rate_down = 6,
    .max_torque_error = 80,
    .max_rt_delta = 100,
    .type = TorqueMotorLimited,
  };

  const LongitudinalLimits GWM_LONG_LIMITS = {
    .max_gas = 4577,
    .min_gas = -192,  // negative torque regens, the stock ACC uses down to -192
    .inactive_gas = 0,
    .max_brake = 107,
  };

  bool tx = true;

  if (msg->addr == GWM_STEER_CMD) {
    uint32_t torque_raw = ((msg->data[12] & 0x7FU) << 3) | (msg->data[13] >> 5);  // TORQUE_CMD
    uint32_t torque_reflected = ((msg->data[9] & 0x3U) << 6) | (msg->data[10] >> 2);  // TORQUE_REFLECTED
    int desired_torque = to_signed(torque_raw, 10) + 1;
    bool steer_req = GET_BIT(msg, 125U);  // STEER_REQUEST
    bool invert_direction = GET_BIT(msg, 103U);  // INVERT_DIRECTION

    // the direction bit and reflected torque must agree with the commanded torque
    bool valid_direction = invert_direction == (steer_req && (desired_torque > 0));
    bool valid_reflected = ((torque_raw + torque_reflected) & 0xFFU) == 0U;
    if (!valid_direction || !valid_reflected || steer_torque_cmd_checks(desired_torque, steer_req, GWM_STEERING_LIMITS)) {
      tx = false;
    }
  }

  if (msg->addr == GWM_ACC_CMD) {
    int brake = 181 - (int)msg->data[13];  // BRAKE_CMD
    int gas = (((msg->data[27] & 0x1FU) << 8) | msg->data[28]) - 192U;  // GAS_CMD
    // brake commands below 0 are never sent
    if ((brake < 0) || longitudinal_brake_checks(brake, GWM_LONG_LIMITS) || longitudinal_gas_checks(gas, GWM_LONG_LIMITS)) {
      tx = false;
    }
  }

  // Only allow cancel while stock cruise is engaged, or no buttons
  if (msg->addr == GWM_STALK) {
    bool other_buttons = ((msg->data[5] & 0xB0U) != 0U) ||  // AP_REDUCE_DISTANCE, AP_INCREASE_DISTANCE, AP_ENABLE
                         ((msg->data[6] & 0x0CU) != 0U);    // AP_DECREASE_SPEED, AP_INCREASE_SPEED
    bool cancel = GET_BIT(msg, 46U);  // AP_CANCEL
    tx = !other_buttons && (!cancel || cruise_engaged_prev);
  }

  return tx;
}

static safety_config gwm_init(uint16_t param) {
  gen_crc_lookup_table_8(0x1DU, gwm_crc_lut);

  static const CanMsg GWM_TX_MSGS[] = {
    {GWM_STEER_CMD, GWM_MAIN_BUS, 64, .check_relay = true},
    {GWM_HUD, GWM_MAIN_BUS, 64, .check_relay = true},
    {GWM_EPS, GWM_CAM_BUS, 64, .check_relay = true},     // EPS feedback to the camera
    {GWM_STALK, GWM_CAM_BUS, 8, .check_relay = false},   // cancel button
  };

  static const CanMsg GWM_LONG_TX_MSGS[] = {
    {GWM_STEER_CMD, GWM_MAIN_BUS, 64, .check_relay = true},
    {GWM_HUD, GWM_MAIN_BUS, 64, .check_relay = true},
    {GWM_EPS, GWM_CAM_BUS, 64, .check_relay = true},     // EPS feedback to the camera
    {GWM_STALK, GWM_CAM_BUS, 8, .check_relay = false},   // cancel button
    {GWM_ACC_CMD, GWM_MAIN_BUS, 64, .check_relay = true},
  };

  static RxCheck gwm_rx_checks[] = {
    {.msg = {{GWM_GAS, GWM_MAIN_BUS, 64, 100U, .max_counter = 14U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{GWM_BRAKE, GWM_MAIN_BUS, 64, 50U, .max_counter = 14U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{GWM_WHEEL_SPEEDS, GWM_MAIN_BUS, 64, 50U, .max_counter = 14U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{GWM_EPS, GWM_MAIN_BUS, 64, 50U, .max_counter = 14U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{GWM_ACC, GWM_CAM_BUS, 64, 10U, .max_counter = 14U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  bool gwm_longitudinal = false;

  SAFETY_UNUSED(param);
  #ifdef ALLOW_DEBUG
    const int FLAG_GWM_LONG_CONTROL = 1;
    gwm_longitudinal = GET_FLAG(param, FLAG_GWM_LONG_CONTROL);
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
