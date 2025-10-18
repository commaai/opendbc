#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"

// TODO: consolidate all VW brake pedal/pressure bools into volkswagen_common.h
static bool volkswagen_mlb_brake_pedal_switch = false;
static bool volkswagen_mlb_brake_pressure_detected = false;


static safety_config volkswagen_mlb_init(uint16_t param) {
  // Transmit of LS_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  const CanMsg VOLKSWAGEN_MLB_STOCK_TX_MSGS[] = {{MSG_HCA_01, 0, 8, .check_relay = true}, {MSG_LDW_02, 0, 8, .check_relay = true},
                                                 {MSG_LS_01, 0, 4, .check_relay = false}, {MSG_LS_01, 2, 4, .check_relay = false}};

  RxCheck volkswagen_mlb_rx_checks[] = {
    // TODO: implement checksum validation
    {.msg = {{MSG_ESP_03, 0, 8, 50U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_LH_EPS_03, 0, 8, 100U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_ESP_05, 0, 8, 50U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_ACC_05, 2, 8, 50U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_03, 0, 8, 100U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  SAFETY_UNUSED(param);

  volkswagen_set_button_prev = false;
  volkswagen_resume_button_prev = false;
  // TODO: consolidate all VW brake pedal/pressure bools into volkswagen_common.h
  volkswagen_mlb_brake_pedal_switch = false;
  volkswagen_mlb_brake_pressure_detected = false;

  gen_crc_lookup_table_8(0x2F, volkswagen_crc8_lut_8h2f);
  return BUILD_SAFETY_CFG(volkswagen_mlb_rx_checks, VOLKSWAGEN_MLB_STOCK_TX_MSGS);
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
      // Always exit controls on rising edge of Cancel
      // Signal: LS_01.LS_Abbrechen
      if (GET_BIT(msg, 13U) == 1U) {
        controls_allowed = false;
      }
    }

    // Signal: Motor_03.MO_Fahrpedalrohwert_01
    // Signal: Motor_03.MO_Fahrer_bremst
    if (msg->addr == MSG_MOTOR_03) {
      gas_pressed = msg->data[6] != 0U;
      volkswagen_mlb_brake_pedal_switch = GET_BIT(msg, 35U);
    }

    if (msg->addr == MSG_ESP_05) {
      volkswagen_mlb_brake_pressure_detected = volkswagen_mlb_mqb_brake_pressure_threshold(msg);
    }

    // TODO: cleanup/migrate to volkswagen_common.h
    brake_pressed = volkswagen_mlb_brake_pedal_switch || volkswagen_mlb_brake_pressure_detected;

  }

  if (msg->bus == 2U) {
    // TODO: See if there's a bus-agnostic TSK message we can use instead
    if (msg->addr == MSG_ACC_05) {
      // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
      // Always exit controls on main switch off
      // Signal: ACC_05.ACC_Status_ACC
      int acc_status = (msg->data[7] & 0xEU) >> 1;
      bool cruise_engaged = (acc_status == 3) || (acc_status == 4) || (acc_status == 5);
      acc_main_on = cruise_engaged || (acc_status == 2);

      pcm_cruise_check(cruise_engaged);

      if (!acc_main_on) {
        controls_allowed = false;
      }
    }
  }
}

static bool volkswagen_mlb_tx_hook(const CANPacket_t *msg) {
  // lateral limits
  const TorqueSteeringLimits VOLKSWAGEN_MLB_STEERING_LIMITS = {
    .max_torque = 300,             // 3.0 Nm (EPS side max of 3.0Nm with fault if violated)
    .max_rt_delta = 188,           // 10 max rate up * 50Hz send rate * 250000 RT interval / 1000000 = 125 ; 125 * 1.5 for safety pad = 187.5
    .max_rate_up = 10,             // 5.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
    .max_rate_down = 10,           // 5.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
    .driver_torque_allowance = 60,
    .driver_torque_multiplier = 3,
    .type = TorqueDriverLimited,
  };

  bool tx = true;

  // Safety check for HCA_01 Heading Control Assist torque
  // Signal: HCA_01.Assist_Torque (absolute torque)
  // Signal: HCA_01.Assist_VZ (direction)
  // TODO: combine with MQB version in volkswagen_common.h
  if (msg->addr == MSG_HCA_01) {
    int desired_torque = msg->data[2] | (msg->data[3] & 0x3FU) << 8;
    int sign = (msg->data[3] & 0x80U) >> 7;
    if (sign == 1) {
      desired_torque *= -1;
    }

    bool steer_req = GET_BIT(msg, 30U);

    if (steer_torque_cmd_checks(desired_torque, steer_req, VOLKSWAGEN_MLB_STEERING_LIMITS)) {
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

const safety_hooks volkswagen_mlb_hooks = {
  .init = volkswagen_mlb_init,
  .rx = volkswagen_mlb_rx_hook,
  .tx = volkswagen_mlb_tx_hook,
   // TODO: rename these functions to MXB or something
  .get_counter = volkswagen_mqb_meb_get_counter,
  .get_checksum = volkswagen_mqb_meb_get_checksum,
  .compute_checksum = volkswagen_mqb_meb_compute_crc,
};
