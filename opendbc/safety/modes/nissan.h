#ifndef SAFETY_MODES_NISSAN_H
#define SAFETY_MODES_NISSAN_H

#include "opendbc/safety/declarations.h"

static const AngleSteeringLimits NISSAN_STEERING_LIMITS = {
  .max_angle = 60000,  // 600 deg, reasonable limit
  .angle_deg_to_can = 100,
  .angle_rate_up_lookup = {
    {0.f, 5.f, 15.f},
    {5.f, .8f, .15f},
  },
  .angle_rate_down_lookup = {
    {0.f, 5.f, 15.f},
    {5.f, 3.5f, .4f},
  },
};

// Nissan's angle signal is inverted around 1310 degrees
static int nissan_flip_angle(int angle) {
  const float flipped = (1310.0f * NISSAN_STEERING_LIMITS.angle_deg_to_can) - (float)angle;
  return (int)flipped;
}

static void nissan_rx_hook(const CANPacket_t *msg) {
  // Altima: on camera bus, others: on pt bus
  if (msg_matches(msg, 0x185U, 0U)) {
    // Current steering angle
    int angle_meas_new = (msg->data[2] << 10) | (msg->data[3] << 2) | (msg->data[4] >> 6);
    // Factor is -0.01, offset is 1310. Flip to correct sign, but keep units in CAN scale
    angle_meas_new = nissan_flip_angle(angle_meas_new);

    // update array of samples
    update_sample(&safety.angle_meas, angle_meas_new);
  }

  if (msg_matches(msg, 0x285U, safety_setup.nissan_alt_eps ? 1U : 0U)) {
    // Get current speed and standstill
    uint16_t right_rear = GET_BYTES_BE(msg, 0, 2);
    uint16_t left_rear = GET_BYTES_BE(msg, 2, 2);
    safety.vehicle_moving = (right_rear | left_rear) != 0U;
    UPDATE_VEHICLE_SPEED((right_rear + left_rear) / 2.0f * 0.005f * KPH_TO_MS);
  }

  // X-Trail 0x15c, Leaf 0x239
  if (msg_matches(msg, 0x15cU, safety_setup.nissan_alt_eps ? 1U : 0U) || msg_matches(msg, 0x239U, safety_setup.nissan_alt_eps ? 1U : 0U)) {
    if (msg->addr == 0x15cU) {
      safety.gas_pressed = ((msg->data[5] << 2) | ((msg->data[6] >> 6) & 0x3U)) > 3U;
    } else {
      safety.gas_pressed = msg->data[0] > 3U;
    }
  }

  // X-trail 0x454, Leaf 0x239
  if (msg_matches(msg, 0x454U, safety_setup.nissan_alt_eps ? 1U : 0U) || msg_matches(msg, 0x239U, safety_setup.nissan_alt_eps ? 1U : 0U)) {
    if (msg->addr == 0x454U) {
      safety.brake_pressed = GET_BIT(msg, 23U);
    } else {
      safety.brake_pressed = GET_BIT(msg, 37U);
    }
  }

  // Handle cruise enabled
  if (msg_matches(msg, 0x30fU, safety_setup.nissan_alt_eps ? 1U : 2U)) {
    bool cruise_engaged = GET_BIT(msg, 3U);
    pcm_cruise_check(cruise_engaged);
  }
}

static bool nissan_tx_hook(const CANPacket_t *msg) {
  bool tx = true;
  bool violation = false;

  // steer cmd checks
  if (msg->addr == 0x169U) {
    int desired_angle = ((msg->data[0] << 10) | (msg->data[1] << 2) | ((msg->data[2] >> 6) & 0x3U));
    bool lka_active = GET_BIT(msg, 52U);

    // Factor is -0.01, offset is 1310. Flip to correct sign, but keep units in CAN scale
    desired_angle = nissan_flip_angle(desired_angle);

    if (steer_angle_cmd_checks(desired_angle, lka_active, NISSAN_STEERING_LIMITS)) {
      violation = true;
    }
  }

  // acc button check, only allow cancel button to be sent
  if (msg->addr == 0x20bU) {
    // Violation of any button other than cancel is pressed
    violation |= ((msg->data[1] & 0x3dU) > 0U);
  }

  if (violation) {
    tx = false;
  }

  return tx;
}

static safety_config nissan_init(uint16_t param) {
  static const CanMsg NISSAN_TX_MSGS[] = {
    {0x169, 0, 8, .check_relay = true},   // LKAS
    {0x2b1, 0, 8, .check_relay = true},   // PROPILOT_HUD
    {0x4cc, 0, 8, .check_relay = true},   // PROPILOT_HUD_INFO_MSG
    {0x20b, 2, 6, .check_relay = false},  // CRUISE_THROTTLE (X-Trail)
    {0x20b, 1, 6, .check_relay = false},  // CRUISE_THROTTLE (Altima)
    {0x280, 2, 8, .check_relay = true},   // CANCEL_MSG (Leaf)
  };

  // Signals duplicated below due to the fact that these messages can come in on either CAN bus, depending on car model.
  static const RxCheck nissan_rx_checks[] = {
    {.msg = {{0x185, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // STEER_TORQUE_SENSOR
    {.msg = {{0x285, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
             {0x285, 1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }}},  // WHEEL_SPEEDS_REAR
    {.msg = {{0x30f, 2, 3, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
             {0x30f, 1, 3, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }}},  // CRUISE_STATE
    {.msg = {{0x15c, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
             {0x15c, 1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
             {0x239, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}}},  // GAS_PEDAL
    {.msg = {{0x454, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
             {0x454, 1, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
             {0x1cc, 0, 4, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}}},  // DOORS_LIGHTS / BRAKE
  };

  // EPS Location. false = V-CAN, true = C-CAN
  const uint16_t NISSAN_PARAM_ALT_EPS_BUS = 1;

  safety_setup.nissan_alt_eps = GET_FLAG(param, NISSAN_PARAM_ALT_EPS_BUS);
  return BUILD_SAFETY_CFG(nissan_rx_checks, NISSAN_TX_MSGS);
}

const safety_hooks nissan_hooks = {
  .init = nissan_init,
  .rx = nissan_rx_hook,
  .tx = nissan_tx_hook,
};

#endif  // SAFETY_MODES_NISSAN_H
