#ifndef SAFETY_MODES_FCA_GIORGIO_H
#define SAFETY_MODES_FCA_GIORGIO_H

#include "opendbc/safety/declarations.h"

#define FCA_GIORGIO_ABS_1           0xEEU
#define FCA_GIORGIO_ABS_3           0xFAU
#define FCA_GIORGIO_ENGINE_1        0xFCU
#define FCA_GIORGIO_EPS_2           0x106U
#define FCA_GIORGIO_LKA_COMMAND     0x1F6U
#define FCA_GIORGIO_LKA_COMMAND_2   0x117U
// TODO: this isn't actually an LKA message on promester, coming from radar ECU
#define FCA_GIORGIO_LKA_HUD_1       0x4AEU
#define FCA_GIORGIO_LKA_HUD_2       0x547U
#define FCA_GIORGIO_ACC_2           0x22AU

static safety_config fca_giorgio_init(uint16_t param) {
  SAFETY_UNUSED(param);

  // TODO: need to find a button message for cancel spam
  static const CanMsg FCA_GIORGIO_TX_MSGS[] = {
    {FCA_GIORGIO_LKA_COMMAND, 0, 4, .check_relay = true},
    {FCA_GIORGIO_LKA_COMMAND_2, 0, 4, .check_relay = true},
    {FCA_GIORGIO_LKA_HUD_1, 0, 8, .check_relay = true},
    {FCA_GIORGIO_LKA_HUD_2, 0, 8, .check_relay = true},
  };

  // TODO: need to find a message for driver gas
  // TODO: re-check counter/checksum for ABS_3
  // TODO: re-enable checksums/counters once poly and final XORs are fixed below
  static const RxCheck fca_giorgio_rx_checks[] = {
    {.msg = {{FCA_GIORGIO_ACC_2, 2, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FCA_GIORGIO_ABS_1, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FCA_GIORGIO_ABS_3, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FCA_GIORGIO_ENGINE_1, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FCA_GIORGIO_EPS_2, 0, 7, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  return BUILD_SAFETY_CFG(fca_giorgio_rx_checks, FCA_GIORGIO_TX_MSGS);
}

static uint32_t fca_giorgio_get_checksum(const CANPacket_t *msg) {
  uint32_t checksum_byte = GET_LEN(msg) - 1U;
  return (uint8_t)msg->data[checksum_byte];
}

static uint8_t fca_giorgio_get_counter(const CANPacket_t *msg) {
  uint32_t counter_byte = GET_LEN(msg) - 2U;
  return (uint8_t)msg->data[counter_byte] & 0xFU;
}

static uint32_t fca_giorgio_compute_crc(const CANPacket_t *msg) {
  uint32_t len = GET_LEN(msg);

  // CRC is in the last byte, poly is same as SAE J1850 but uses a different init value and output XOR
  uint8_t crc = 0U;
  uint8_t final_xor = 0U;

  for (uint32_t i = 0U; (i + 1U) < len; i++) {
    // TODO: wrong poly — should be 0x1D (SAE J1850), not 0x2F (AUTOSAR)
    crc = crc8_update(crc, msg->data[i], 0x2FU);
  }

  // TODO: fill in final XORs for RX messages (see FCA_GIORGIO_CHECKSUM_XORS in chryslercan.py):
  //   0x0A: ABS_1 (0xEE), ENGINE_1 (0xFC), ACC_2 (0x22A)
  //   0xF6: EPS_2 (0x106)
  if (msg->addr == 0xFFU) {
    final_xor = 0xFFU;
  } else {
    final_xor = 0x0;
  }

  return (uint8_t)(crc ^ final_xor);
}

static void fca_giorgio_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // Update in-motion state by sampling wheel speeds
    if (msg->addr == FCA_GIORGIO_ABS_1) {
      // Thanks, FCA, for these 13 bit signals. Makes perfect sense. Great work.
      // Signals: ABS_3.WHEEL_SPEED_[FL,FR,RL,RR]
      uint32_t wheel_speed_fl = GET_BYTES_BE(msg, 0, 2) >> 3;
      uint32_t wheel_speed_fr = (GET_BYTES_BE(msg, 1, 3) >> 6) & 0x1FFFU;
      uint32_t wheel_speed_rl = (GET_BYTES_BE(msg, 3, 2) >> 1) & 0x1FFFU;
      uint32_t wheel_speed_rr = (GET_BYTES_BE(msg, 4, 3) >> 4) & 0x1FFFU;
      safety.vehicle_moving = (wheel_speed_fl + wheel_speed_fr + wheel_speed_rl + wheel_speed_rr) > 0U;
    }

    // Update driver input torque samples
    // Signal: EPS_2.DRIVER_TORQUE
    if (msg->addr == FCA_GIORGIO_EPS_2) {
      int torque_driver_new = from_offset(GET_BYTES_BE(msg, 2, 2) >> 5, 1024);
      update_sample(&safety.torque_driver, torque_driver_new);
    }

    // Signal: ENGINE_1.ACCEL_PEDAL
    if (msg->addr == FCA_GIORGIO_ENGINE_1) {
      safety.gas_pressed = ((GET_BYTES_BE(msg, 2, 2) >> 5) & 0xFFU) > 0U;
    }

    // Signal: ABS_3.BRAKE_PEDAL_SWITCH
    if (msg->addr == FCA_GIORGIO_ABS_3) {
      safety.brake_pressed = GET_BIT(msg, 3U);
    }
  }

  if (msg->bus == 2U) {
    if (msg->addr == FCA_GIORGIO_ACC_2) {
      // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
      // Signal: ACC_2.CRUISE_STATUS
      bool cruise_engaged = GET_BIT(msg, 23U);
      safety.acc_main_on = true;
      pcm_cruise_check(cruise_engaged);
    }
  }
}

static bool fca_giorgio_tx_hook(const CANPacket_t *msg) {
  // lateral limits
  const TorqueSteeringLimits FCA_GIORGIO_STEERING_LIMITS = {
    .max_torque = 300,
    .max_rt_delta = 150,
    .max_rate_up = 4,
    .max_rate_down = 4,
    .driver_torque_allowance = 80,
    .driver_torque_multiplier = 3,
    .type = TorqueDriverLimited,
  };

  bool tx = true;

  // Safety check for commanded steering torque
  if (msg->addr == FCA_GIORGIO_LKA_COMMAND) {
    // Signal: LKA_COMMAND.LKA_TORQUE
    int desired_torque = from_offset(GET_BYTES_BE(msg, 0, 2) >> 5, 1024);
    // Signal: LKA_COMMAND.LKA_ACTIVE
    bool steer_req = GET_BIT(msg, 12U);

    if (steer_torque_cmd_checks(desired_torque, steer_req, FCA_GIORGIO_STEERING_LIMITS)) {
      tx = false;
    }
  }

  if (msg->addr == FCA_GIORGIO_LKA_COMMAND_2) {
    // Signal: LKA_COMMAND_2.LKA_TORQUE (12-bit, ~4x scale of LKA_COMMAND), normalized to LKA_COMMAND units
    int desired_torque = from_offset(GET_BYTES_BE(msg, 0, 2) >> 4, 2048) / 4;
    // Signal: LKA_COMMAND_2.LKA_ACTIVE
    bool steer_req = GET_BIT(msg, 11U);

    if (steer_torque_cmd_checks(desired_torque, steer_req, FCA_GIORGIO_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // TODO: sanity check cancel spam, once a button message is found

  return tx;
}

const safety_hooks fca_giorgio_hooks = {
  .init = fca_giorgio_init,
  .rx = fca_giorgio_rx_hook,
  .tx = fca_giorgio_tx_hook,
  .get_counter = fca_giorgio_get_counter,
  .get_checksum = fca_giorgio_get_checksum,
  .compute_checksum = fca_giorgio_compute_crc,
};

#endif  // SAFETY_MODES_FCA_GIORGIO_H
