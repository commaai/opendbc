#pragma once

#include "opendbc/safety/declarations.h"

static uint8_t rivian_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0;
  // Signal: ESP_Status_Counter, VDM_PropStatus_Counter
  cnt = msg->data[1] & 0xFU;
  return cnt;
}

static uint32_t rivian_get_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  // Signal: ESP_Status_Checksum, VDM_PropStatus_Checksum
  chksum = msg->data[0];
  return chksum;
}

static uint8_t _rivian_compute_checksum(const CANPacket_t *msg, uint8_t poly, uint8_t xor_output) {
  int len = GET_LEN(msg);

  uint8_t crc = 0;
  // Skip the checksum byte
  for (int i = 1; i < len; i++) {
    crc ^= msg->data[i];
    for (int j = 0; j < 8; j++) {
      if ((crc & 0x80U) != 0U) {
        crc = (crc << 1) ^ poly;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc ^ xor_output;
}

typedef struct {
  uint32_t addr;
  uint8_t poly;
  uint8_t xor_output;
} ChecksumConfig;

static uint32_t rivian_compute_checksum(const CANPacket_t *msg) {
  static const ChecksumConfig checksum_configs[] = {
    {0x208U, 0x1D, 0xB1},  // ESP_Vehicle_Speed_Checksum
    {0x150U, 0x1D, 0x9A},  // VDM_VehicleSpeed_Checksum
  };

  uint8_t chksum = 0;
  for (uint16_t i = 0; i < (sizeof(checksum_configs) / sizeof(ChecksumConfig)); i++) {
    if (msg->addr == checksum_configs[i].addr) {
      chksum = _rivian_compute_checksum(msg, checksum_configs[i].poly, checksum_configs[i].xor_output);
    }
  }
  // Handle unknown addresses if necessary
  return chksum;
}

typedef struct {
  uint32_t addr;
  uint8_t data_index;
  uint8_t shift;
  uint8_t mask;
} QualityFlagConfig;

static bool rivian_get_quality_flag_valid(const CANPacket_t *msg) {
  static const QualityFlagConfig quality_flag_configs[] = {
    {0x208U, 3, 3, 0x3U},  // ESP_Vehicle_Speed_Q
    {0x150U, 1, 6, 0x1U},  // VDM_VehicleSpeedQ

    {0x000U, 1, 1, 0x0U},  // Last entry, prevent mutated address from being used
  };

  bool valid = false;
  uint16_t q_count = sizeof(quality_flag_configs) / sizeof(QualityFlagConfig) - 1U;  // Exclude the last entry
  // FIXME: MULL error: < to <= will also work, which in hindsight is bad since OoB access
  // however, if in the Out-of-Bounds case, it will simply mask to 0 -> False
  // mull-ignore-next: cxx_lt_to_le
  for (uint16_t i = 0; i < q_count; i++) {
    if (msg->addr == quality_flag_configs[i].addr) {
      uint8_t value = (msg->data[quality_flag_configs[i].data_index] >> quality_flag_configs[i].shift) & quality_flag_configs[i].mask;
      valid = (value == 0x1U);
    }
  }
  return valid;
}

static void rivian_rx_hook(const CANPacket_t *msg) {

  if (msg->bus == 0U)  {
    // Vehicle speed
    if (msg->addr == 0x208U) {
      float speed = ((msg->data[6] << 8) | msg->data[7]) * 0.01;
      vehicle_moving = speed > 0.0;
      UPDATE_VEHICLE_SPEED(speed * KPH_TO_MS);
    }

    // Gas pressed and second speed source for variable torque limit
    if (msg->addr == 0x150U) {
      gas_pressed = msg->data[3] | (msg->data[4] & 0xC0U);

      // Disable controls if speeds from VDM and ESP ECUs are too far apart.
      float vdm_speed = ((msg->data[5] << 8) | msg->data[6]) * 0.01 * KPH_TO_MS;
      speed_mismatch_check(vdm_speed);
    }

    // Driver torque
    if (msg->addr == 0x380U) {
      int torque_driver_new = (((msg->data[2] << 4) | (msg->data[3] >> 4))) - 2050U;
      update_sample(&torque_driver, torque_driver_new);
    }

    // Brake pressed
    if (msg->addr == 0x38fU) {
      brake_pressed = (msg->data[2] >> 7) & 1U;
    }
  }

  if (msg->addr == 0x100U) {
    // Cruise state
    const int feature_status = msg->data[2] >> 5U;
    pcm_cruise_check(feature_status == 1);
  }
}

static bool rivian_tx_hook(const CANPacket_t *msg) {
  // Rivian utilizes more torque at low speed to maintain the same lateral accel
  const TorqueSteeringLimits RIVIAN_STEERING_LIMITS = {
    .max_torque = 350,
    .dynamic_max_torque = true,
    .max_torque_lookup = {
      {9., 17., 17.},
      {350, 250, 250},
    },
    .max_rate_up = 3,
    .max_rate_down = 5,
    .max_rt_delta = 125,
    .driver_torque_multiplier = 2,
    .driver_torque_allowance = 100,
    .type = TorqueDriverLimited,
  };

  const LongitudinalLimits RIVIAN_LONG_LIMITS = {
    .max_accel = 200,
    .min_accel = -350,
    .inactive_accel = 0,
  };

  bool tx = true;

  if (msg->bus == 0U) {
    // Steering control
    if (msg->addr == 0x120U) {
      int desired_torque = ((msg->data[2] << 3U) | (msg->data[3] >> 5U)) - 1024U;
      bool steer_req = (msg->data[3] >> 4) & 1U;

      if (steer_torque_cmd_checks(desired_torque, steer_req, RIVIAN_STEERING_LIMITS)) {
        tx = false;
      }
    }

    // Longitudinal control
    if (msg->addr == 0x160U) {
      int raw_accel = ((msg->data[2] << 3) | (msg->data[3] >> 5)) - 1024U;
      if (longitudinal_accel_checks(raw_accel, RIVIAN_LONG_LIMITS)) {
        tx = false;
      }
    }
  }

  return tx;
}

static safety_config rivian_init(uint16_t param) {
  // SCCM_WheelTouch: for hiding hold wheel alert
  // VDM_AdasSts: for canceling stock ACC
  // 0x120 = ACM_lkaHbaCmd, 0x321 = SCCM_WheelTouch, 0x162 = VDM_AdasSts
  static const CanMsg RIVIAN_TX_MSGS[] = {{0x120, 0, 8, .check_relay = true}, {0x321, 2, 7, .check_relay = true}, {0x162, 2, 8, .check_relay = true}};
  // 0x160 = ACM_longitudinalRequest
  static const CanMsg RIVIAN_LONG_TX_MSGS[] = {{0x120, 0, 8, .check_relay = true}, {0x321, 2, 7, .check_relay = true}, {0x160, 0, 5, .check_relay = true}};

  static RxCheck rivian_rx_checks[] = {
    {.msg = {{0x208, 0, 8, 50U, .max_counter = 14U}, { 0 }, { 0 }}},                                                             // ESP_Status (speed)
    {.msg = {{0x150, 0, 7, 50U, .max_counter = 14U}, { 0 }, { 0 }}},                                                             // VDM_PropStatus (gas pedal & 2nd speed)
    {.msg = {{0x380, 0, 5, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // EPAS_SystemStatus (driver torque)
    {.msg = {{0x38f, 0, 6, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // iBESP2 (brakes)
    {.msg = {{0x100, 2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // ACM_Status (cruise state)
  };

  bool rivian_longitudinal = false;

  SAFETY_UNUSED(param);
  #ifdef ALLOW_DEBUG
    const int FLAG_RIVIAN_LONG_CONTROL = 1;
    rivian_longitudinal = GET_FLAG(param, FLAG_RIVIAN_LONG_CONTROL);
  #endif

  // FIXME: cppcheck thinks that rivian_longitudinal is always false. This is not true
  // if ALLOW_DEBUG is defined but cppcheck is run without ALLOW_DEBUG
  // cppcheck-suppress knownConditionTrueFalse
  return rivian_longitudinal ? BUILD_SAFETY_CFG(rivian_rx_checks, RIVIAN_LONG_TX_MSGS) : \
                               BUILD_SAFETY_CFG(rivian_rx_checks, RIVIAN_TX_MSGS);
}

const safety_hooks rivian_hooks = {
  .init = rivian_init,
  .rx = rivian_rx_hook,
  .tx = rivian_tx_hook,
  .get_counter = rivian_get_counter,
  .get_checksum = rivian_get_checksum,
  .compute_checksum = rivian_compute_checksum,
  .get_quality_flag_valid = rivian_get_quality_flag_valid,
};
