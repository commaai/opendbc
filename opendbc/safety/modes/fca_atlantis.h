#ifndef SAFETY_MODES_FCA_ATLANTIS_H
#define SAFETY_MODES_FCA_ATLANTIS_H

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/chrysler_common.h"

#define FCA_ATLANTIS_ENGINE_2        0xDCU
#define FCA_ATLANTIS_ABS_1           0xEEU
#define FCA_ATLANTIS_ABS_3           0xFAU
#define FCA_ATLANTIS_EPS_2           0x106U
#define FCA_ATLANTIS_LKA_COMMAND     0x1F6U
#define FCA_ATLANTIS_LKA_COMMAND_2   0x117U
// TODO: this isn't actually an LKA message on promester, coming from radar ECU
#define FCA_ATLANTIS_LKA_HUD_2       0x547U
#define FCA_ATLANTIS_LKA_HUD_3       0x5A2U
#define FCA_ATLANTIS_ACC_2           0x22AU
#define FCA_ATLANTIS_ACC_BUTTON      0x2FAU

// Torque winddown after controls are lost (e.g. brake press). Cutting torque to zero in one frame is suspected of
// latching EPS LKA_FAULT, and stock LKA always ramps down. After a disengage, allow torque that never grows in
// magnitude or changes sign, with LKA_ACTIVE set, for a limited time. Like volkswagen_meb's steer power winddown.
#define FCA_ATLANTIS_WINDDOWN_MAX_US 1500000U  // STEER_MAX at 3/frame takes 1 s
#define FCA_ATLANTIS_WINDDOWN_MAX_GAP_US 50000U  // longest gap since the last allowed frame (blocked frames don't count)

static safety_config fca_atlantis_init(uint16_t param) {
  SAFETY_UNUSED(param);

  static const CanMsg FCA_ATLANTIS_TX_MSGS[] = {
    {FCA_ATLANTIS_LKA_COMMAND, 0, 4, .check_relay = true},
    {FCA_ATLANTIS_LKA_COMMAND_2, 0, 4, .check_relay = true},
    {FCA_ATLANTIS_LKA_HUD_2, 0, 8, .check_relay = true},
    {FCA_ATLANTIS_LKA_HUD_3, 0, 8, .check_relay = true},
    {FCA_ATLANTIS_ACC_BUTTON, 2, 4, .check_relay = false},
  };

  // no quality flag signals are known
  static const RxCheck fca_atlantis_rx_checks[] = {
    {.msg = {{FCA_ATLANTIS_ACC_2, 2, 8, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FCA_ATLANTIS_ABS_1, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FCA_ATLANTIS_ABS_3, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FCA_ATLANTIS_ENGINE_2, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FCA_ATLANTIS_EPS_2, 0, 7, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  return BUILD_SAFETY_CFG(fca_atlantis_rx_checks, FCA_ATLANTIS_TX_MSGS);
}

static uint8_t fca_atlantis_get_counter(const CANPacket_t *msg) {
  uint8_t counter;
  if (msg->addr == FCA_ATLANTIS_ABS_3) {
    counter = (uint8_t)((msg->data[4] >> 3) & 0xFU);
  } else {
    uint32_t counter_byte = GET_LEN(msg) - 2U;
    counter = (uint8_t)msg->data[counter_byte] & 0xFU;
  }
  return counter;
}

static void fca_atlantis_rx_hook(const CANPacket_t *msg) {
  // Update in-motion state by sampling wheel speeds
  if (msg_matches(msg, FCA_ATLANTIS_ABS_1, 0U)) {
    // Signals: ABS_1.WHEEL_SPEED_[FL,FR,RL,RR]
    uint32_t wheel_speed_fl = GET_BYTES_BE(msg, 0, 2) >> 3;
    uint32_t wheel_speed_fr = (GET_BYTES_BE(msg, 1, 3) >> 6) & 0x1FFFU;
    uint32_t wheel_speed_rl = (GET_BYTES_BE(msg, 3, 2) >> 1) & 0x1FFFU;
    uint32_t wheel_speed_rr = (GET_BYTES_BE(msg, 4, 3) >> 4) & 0x1FFFU;
    safety.vehicle_moving = (wheel_speed_fl + wheel_speed_fr + wheel_speed_rl + wheel_speed_rr) > 0U;
  }

  // Update driver input torque samples
  // Signal: EPS_2.DRIVER_TORQUE
  if (msg_matches(msg, FCA_ATLANTIS_EPS_2, 0U)) {
    int torque_driver_new = from_offset(GET_BYTES_BE(msg, 2, 2) >> 5, 1024);
    update_sample(&safety.torque_driver, torque_driver_new);
  }

  // Signal: ENGINE_2.ACCEL_PEDAL_USER
  if (msg_matches(msg, FCA_ATLANTIS_ENGINE_2, 0U)) {
    safety.gas_pressed = msg->data[2] > 0U;
  }

  // Signal: ABS_3.BRAKE_PEDAL_SWITCH
  if (msg_matches(msg, FCA_ATLANTIS_ABS_3, 0U)) {
    safety.brake_pressed = GET_BIT(msg, 3U);
  }

  if (msg_matches(msg, FCA_ATLANTIS_ACC_2, 2U)) {
    // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
    // Signal: ACC_2.CRUISE_STATUS
    bool cruise_engaged = GET_BIT(msg, 23U);
    pcm_cruise_check(cruise_engaged);
  }
}

static bool fca_atlantis_winddown_allowed(int desired_torque, bool steer_req, uint32_t ts) {
  bool allowed = !safety.controls_allowed && steer_req && (desired_torque != 0) && (safety.fca_atlantis_winddown_torque != 0);
  // same sign, magnitude not growing
  allowed = allowed && ((desired_torque > 0) == (safety.fca_atlantis_winddown_torque > 0));
  allowed = allowed && (safety_max_int(desired_torque, -desired_torque) <=
                        safety_max_int(safety.fca_atlantis_winddown_torque, -safety.fca_atlantis_winddown_torque));
  allowed = allowed && (safety_get_ts_elapsed(ts, safety.fca_atlantis_ts_lka_last) <= FCA_ATLANTIS_WINDDOWN_MAX_GAP_US);
  allowed = allowed && (safety_get_ts_elapsed(ts, safety.fca_atlantis_ts_controls_last) <= FCA_ATLANTIS_WINDDOWN_MAX_US);
  return allowed;
}

static bool fca_atlantis_tx_hook(const CANPacket_t *msg) {
  // lateral limits
  const TorqueSteeringLimits FCA_ATLANTIS_STEERING_LIMITS = {
    .max_torque = 300,
    .max_rt_delta = 112,
    .max_rate_up = 3,
    .max_rate_down = 3,
    .driver_torque_allowance = 80,
    .driver_torque_multiplier = 2,
    .type = TorqueDriverLimited,
  };

  bool tx = true;

  // Safety check for commanded steering torque
  if (msg->addr == FCA_ATLANTIS_LKA_COMMAND) {
    // Signal: LKA_COMMAND.LKA_TORQUE
    int desired_torque = from_offset(GET_BYTES_BE(msg, 0, 2) >> 5, 1024);
    // Signal: LKA_COMMAND.LKA_ACTIVE
    bool steer_req = GET_BIT(msg, 12U);

    uint32_t ts = safety_ts;
    bool allowed;
    if (fca_atlantis_winddown_allowed(desired_torque, steer_req, ts)) {
      allowed = true;
      // re-engaging mid-winddown continues the rate limits from here
      safety.desired_torque_last = desired_torque;
    } else {
      allowed = !steer_torque_cmd_checks(desired_torque, steer_req, FCA_ATLANTIS_STEERING_LIMITS);
      if (!allowed && safety.controls_allowed) {
        // no winddown after a violation; a frame blocked after controls are lost (e.g. one openpilot sent before
        // it saw the brake) doesn't end the winddown
        safety.fca_atlantis_winddown_torque = 0;
      }
    }

    if (allowed) {
      safety.fca_atlantis_lka_torque_last = desired_torque;
      safety.fca_atlantis_lka_steer_req_last = steer_req;
      safety.fca_atlantis_winddown_torque = desired_torque;
      safety.fca_atlantis_ts_lka_last = ts;
      if (safety.controls_allowed) {
        safety.fca_atlantis_ts_controls_last = ts;
      }
    } else {
      tx = false;
      safety.fca_atlantis_lka_torque_last = 0;
      safety.fca_atlantis_lka_steer_req_last = false;
    }
  }

  // LKA_COMMAND_2 is a 4x scaled copy of LKA_COMMAND. The EPS is suspected to listen only to LKA_COMMAND_2: it
  // engaged and steered with LKA_COMMAND_2 alone, and stock sends LKA_COMMAND one frame later as a rounded copy
  // of LKA_COMMAND_2 / 4, so the two aren't exact 4x matches in stock routes. Rather than running the
  // shared torque rate-limit state twice per frame, only allow an exact copy of the last allowed LKA_COMMAND
  // (which openpilot sends first each frame). After a blocked LKA_COMMAND, only zero torque is allowed.
  if (msg->addr == FCA_ATLANTIS_LKA_COMMAND_2) {
    // Signal: LKA_COMMAND_2.LKA_TORQUE
    int desired_torque = from_offset(GET_BYTES_BE(msg, 0, 2) >> 4, 2048);
    // Signal: LKA_COMMAND_2.LKA_ACTIVE
    bool steer_req = GET_BIT(msg, 11U);

    if ((desired_torque != (safety.fca_atlantis_lka_torque_last * 4)) || (steer_req != safety.fca_atlantis_lka_steer_req_last)) {
      tx = false;
    }
  }

  // FORCE CANCEL: only the cancel button press is allowed
  if (msg->addr == FCA_ATLANTIS_ACC_BUTTON) {
    // Signal: ACC_BUTTON.CANCEL, with no other button and the constant bits as sent by the car
    const bool is_cancel = (msg->data[0] == 0x80U) && ((msg->data[1] & 0xF0U) == 0x30U);
    if (!is_cancel) {
      tx = false;
    }
  }

  return tx;
}

const safety_hooks fca_atlantis_hooks = {
  .init = fca_atlantis_init,
  .rx = fca_atlantis_rx_hook,
  .tx = fca_atlantis_tx_hook,
  .get_counter = fca_atlantis_get_counter,
  .get_checksum = chrysler_get_checksum,
  .compute_checksum = chrysler_compute_checksum,
};

#endif  // SAFETY_MODES_FCA_ATLANTIS_H
