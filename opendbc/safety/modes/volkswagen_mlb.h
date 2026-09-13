#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"

// ACC_01.ACC_Status_ACC, the states that tell the drivetrain ACC is regulating
#define VOLKSWAGEN_MLB_ACC_AKTIV_REGELT        3U
#define VOLKSWAGEN_MLB_ACC_OVERRIDE            4U


static safety_config volkswagen_mlb_init(uint16_t param) {
  // Transmit of LS_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  static const CanMsg VOLKSWAGEN_MLB_STOCK_TX_MSGS[] = {{MSG_HCA_01, 0, 8, .check_relay = true}, {MSG_LDW_02, 0, 8, .check_relay = true},
                                                        {MSG_LS_01, 0, 4, .check_relay = false}, {MSG_LS_01, 2, 4, .check_relay = false}};

  static const CanMsg VOLKSWAGEN_MLB_LONG_TX_MSGS[] = {
    {MSG_HCA_01, 0, 8, .check_relay = true},
    {MSG_LS_01, 0, 4, .check_relay = false},
    {MSG_LS_01, 2, 4, .check_relay = false},
    {MSG_LDW_02, 0, 8, .check_relay = true},
    {MSG_ACC_02, 0, 8, .check_relay = true},
    {MSG_ACC_01, 0, 8, .check_relay = true},
  };

  static RxCheck volkswagen_mlb_rx_checks[] = {
    // TODO: implement checksum validation
    {.msg = {{MSG_ESP_03, 0, 8, 50U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_LH_EPS_03, 0, 8, 100U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_ESP_05, 0, 8, 50U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_TSK_04, 1, 8, 50U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_03, 0, 8, 100U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_LS_01, 0, 4, 10U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  volkswagen_common_init();

#ifdef ALLOW_DEBUG
  volkswagen_longitudinal = GET_FLAG(param, FLAG_VOLKSWAGEN_LONG_CONTROL);
#else
  SAFETY_UNUSED(param);
#endif

  return volkswagen_longitudinal ? BUILD_SAFETY_CFG(volkswagen_mlb_rx_checks, VOLKSWAGEN_MLB_LONG_TX_MSGS) : \
                                   BUILD_SAFETY_CFG(volkswagen_mlb_rx_checks, VOLKSWAGEN_MLB_STOCK_TX_MSGS);
}

static void volkswagen_mlb_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // Check all wheel speeds for any movement
    // Signals: ESP_03.ESP_[VL|VR|HL|HR]_Radgeschw
    if (msg->addr == MSG_ESP_03) {
      uint32_t speed = 0;
      speed += ((msg->data[3] & 0xFU) << 8) | msg->data[2];   // FL
      speed += (msg->data[4] << 4) | (msg->data[3] >> 4);     // FR
      speed += ((msg->data[6] & 0xFU) << 8) | msg->data[5];   // RL
      speed += (msg->data[7] << 4) | (msg->data[6] >> 4);     // RR
      vehicle_moving = speed > 0U;
    }

    // Update driver input torque
    if (msg->addr == MSG_LH_EPS_03) {
      update_sample(&torque_driver, volkswagen_mlb_mqb_driver_input_torque(msg));
    }

    if (msg->addr == MSG_LS_01) {
      // If using openpilot longitudinal, enter controls on falling edge of Set or Resume with main switch on
      // Signal: LS_01.LS_Tip_Setzen
      // Signal: LS_01.LS_Tip_Wiederaufnahme
      if (volkswagen_longitudinal) {
        bool set_button = GET_BIT(msg, 16U);
        bool resume_button = GET_BIT(msg, 19U);
        if ((volkswagen_set_button_prev && !set_button) ||
            (volkswagen_resume_button_prev && !resume_button)) {
          controls_allowed = GET_BIT(msg, 12U);  // LS_Hauptschalter
        }
        volkswagen_set_button_prev = set_button;
        volkswagen_resume_button_prev = resume_button;
      }
      // Always exit controls on rising edge of Cancel
      // Signal: LS_01.LS_Abbrechen
      if (GET_BIT(msg, 13U)) {
        controls_allowed = false;
      }
    }

    // Signal: Motor_03.MO_Fahrpedalrohwert_01
    // Signal: Motor_03.MO_BLS (bit 34)
    if (msg->addr == MSG_MOTOR_03) {
      gas_pressed = msg->data[6] != 0U;
      volkswagen_brake_pedal_switch = GET_BIT(msg, 34U);
    }

    if (msg->addr == MSG_ESP_05) {
      volkswagen_brake_pressure_detected = GET_BIT(msg, 26U);
    }

    brake_pressed = volkswagen_brake_pedal_switch || volkswagen_brake_pressure_detected;

  }

  if (msg->bus == 1U) {
    if (msg->addr == MSG_TSK_04) {
      // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
      // Signal: TSK_04.TSK_Status_GRA_ACC_02
      int acc_status = (msg->data[7] & 0xC0U) >> 6;
      bool cruise_engaged = (acc_status == 1) || (acc_status == 2);

      if (!volkswagen_longitudinal) {
        pcm_cruise_check(cruise_engaged);
      }
    }
  }
}

static bool volkswagen_mlb_tx_hook(const CANPacket_t *msg) {
  // lateral limits
  const TorqueSteeringLimits VOLKSWAGEN_MLB_STEERING_LIMITS = {
    .max_torque = 300,             // 3.0 Nm (EPS side max of 3.0Nm with fault if violated)
    .max_rt_delta = 169,           // 10 max rate up * 50Hz send rate * 250000 RT interval / 1000000 = 112.5 ; 112.5 * 1.5 for safety pad = 168.75
    .max_rate_up = 9,              // 5.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
    .max_rate_down = 10,           // 5.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
    .driver_torque_allowance = 60,
    .driver_torque_multiplier = 3,
    .type = TorqueDriverLimited,
  };

  // longitudinal limits
  // acceleration in m/s2 * 1000 to avoid floating point math
  // Braking limited to -2.95m/s^2: -3.0 faults the 2014 Audi Q5 ACC ECU (requires ignition cycle to clear)
  const LongitudinalLimits VOLKSWAGEN_MLB_LONG_LIMITS = {
    .max_accel = 2000,
    .min_accel = -2950,
    .inactive_accel = 0,
  };

  bool tx = true;

  // Safety check for HCA_01 Heading Control Assist torque
  if (msg->addr == MSG_HCA_01) {
    int desired_torque = volkswagen_mlb_mqb_steering_control_torque(msg);

    int steer_status = msg->data[4] & 0xFU;
    bool steer_req = (steer_status == 5) || (steer_status == 7);

    if (steer_torque_cmd_checks(desired_torque, steer_req, VOLKSWAGEN_MLB_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // Safety check for ACC_01 acceleration request
  // To avoid floating point math, scale upward and compare to pre-scaled safety m/s^2 boundaries
  if (msg->addr == MSG_ACC_01) {
    bool violation = false;
    int desired_accel = 0;

    // Signal: ACC_01.ACC_Sollbeschleunigung (acceleration in m/s^2, scale 0.005, offset -7.22)
    desired_accel = ((((msg->data[4] & 0x07U) << 8) | msg->data[3]) * 5U) - 7220U;

    violation |= longitudinal_accel_checks(desired_accel, VOLKSWAGEN_MLB_LONG_LIMITS);

    // Signal: ACC_01.ACC_Status_ACC
    uint8_t acc_status = (msg->data[7] >> 4) & 0x07U;
    bool acc_status_active = (acc_status == VOLKSWAGEN_MLB_ACC_AKTIV_REGELT) ||
                             (acc_status == VOLKSWAGEN_MLB_ACC_OVERRIDE);
    violation |= acc_status_active && !controls_allowed;

    if (violation) {
      tx = false;
    }
  }

  // FORCE CANCEL: ensuring that only the cancel button press is sent when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if ((msg->addr == MSG_LS_01) && !controls_allowed) {
    // disallow resume and set: bits 16 and 19
    if (GET_BIT(msg, 16U) || GET_BIT(msg, 19U)) {
      tx = false;
    }
  }

  return tx;
}

// TODO: rename these functions to MXB or something
const safety_hooks volkswagen_mlb_hooks = {
  .init = volkswagen_mlb_init,
  .rx = volkswagen_mlb_rx_hook,
  .tx = volkswagen_mlb_tx_hook,
  .get_counter = volkswagen_mqb_meb_get_counter,
  .get_checksum = volkswagen_mqb_meb_get_checksum,
  .compute_checksum = volkswagen_mqb_meb_compute_crc,
};
