
#pragma once

#include "opendbc/safety/safety_declarations.h"

#define PSA_STEERING              757  // RX from XXX, driver torque
#define PSA_STEERING_ALT          773  // RX from EPS, steering angle
#define PSA_DRIVER                1390 // RX from BSI, gas pedal
#define PSA_DAT_BSI               1042 // RX from BSI, doors
#define PSA_HS2_DYN_ABR_38D       909  // RX from UC_FREIN, speed
#define PSA_HS2_DAT_MDD_CMD_452   1106 // RX from BSI, cruise state
#define PSA_LANE_KEEP_ASSIST      1010 // TX from OP,  EPS

// CAN bus
#define PSA_CAM_BUS  0
#define PSA_ADAS_BUS 1
#define PSA_MAIN_BUS 2


static void psa_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if (bus == PSA_CAM_BUS) {
    if (addr == PSA_DAT_BSI) {
      //TODO: check GET_BIT(to_push, 5);
      brake_pressed = (GET_BYTE(to_push, 0U) >> 5U) & 1U; // P013_MainBrake
    }
    if (addr == PSA_DRIVER) {
      gas_pressed = GET_BYTE(to_push, 3) > 0U; // GAS_PEDAL
    }
    if (addr == PSA_STEERING) {
      int torque_driver_new = GET_BYTE(to_push, 1); // TODO: check
      update_sample(&torque_driver, torque_driver_new);
    }
    if (addr == PSA_STEERING_ALT) {
      int angle_meas_new = to_signed((GET_BYTE(to_push, 0) << 8) | GET_BYTE(to_push, 1), 16);
      update_sample(&angle_meas, angle_meas_new);
    }
  }
  if (bus == PSA_ADAS_BUS) {
    if (addr == PSA_HS2_DYN_ABR_38D) {
      int speed = (GET_BYTE(to_push, 0) << 8) | GET_BYTE(to_push, 1);
      vehicle_moving = speed > 0;
      UPDATE_VEHICLE_SPEED(speed * 0.01); // VITESSE_VEHICULE_ROUES
    }
    if (addr == PSA_HS2_DAT_MDD_CMD_452) {
      // TODO: check pcm_cruise_check(GET_BIT(to_push, 3) & 0x40);
      pcm_cruise_check(((GET_BYTE(to_push, 2U) >> 7U) & 1U)); // DDE_ACTIVATION_RVV_ACC
    }
  }
}

static bool psa_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int addr = GET_ADDR(to_send);

  static const AngleSteeringLimits PSA_STEERING_LIMITS = {
      .angle_deg_to_can = 100,
      .angle_rate_up_lookup = {
      {0., 5., 25.},
      {2.5, 1.5, 0.2},
    },
    .angle_rate_down_lookup = {
      {0., 5., 25.},
      {5., 2.0, 0.3},
    },
  };

  // TODO: Safety check for cruise buttons
  // TODO: check resume is not pressed when controls not allowed
  // TODO: check cancel is not pressed when cruise isn't engaged

  // Safety check for LKA
  if (addr == PSA_LANE_KEEP_ASSIST) {
    // SET_ANGLE
    int desired_angle = to_signed((GET_BYTE(to_send, 6) << 6) | ((GET_BYTE(to_send, 7) & 0xFCU) >> 2), 14);
    // TORQUE_FACTOR
    bool lka_active = ((GET_BYTE(to_send, 5) & 0xFEU) >> 1) == 100U;

    if (steer_angle_cmd_checks(desired_angle, lka_active, PSA_STEERING_LIMITS)) {
      tx = false;
    }
  }
  return tx;
}

static bool psa_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  if (bus_num == PSA_MAIN_BUS) {
    block_msg = addr == PSA_LANE_KEEP_ASSIST;
  }

  return block_msg;
}

static safety_config psa_init(uint16_t param) {
  UNUSED(param);
  static const CanMsg PSA_TX_MSGS[] = {
    {PSA_LANE_KEEP_ASSIST, PSA_CAM_BUS, 8, .check_relay = true}, // EPS steering
  };

  static RxCheck psa_rx_checks[] = {
    {.msg = {{PSA_HS2_DAT_MDD_CMD_452, PSA_ADAS_BUS, 6, .max_counter = 15U, .ignore_quality_flag = true, .frequency = 20U}, { 0 }, { 0 }}},                    // cruise state
    {.msg = {{PSA_HS2_DYN_ABR_38D, PSA_ADAS_BUS, 8, .max_counter = 15U, .ignore_quality_flag = true, .frequency = 25U}, { 0 }, { 0 }}},                        // speed
    {.msg = {{PSA_STEERING_ALT, PSA_CAM_BUS, 7, .max_counter = 15U, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}},                           // steering angle
    {.msg = {{PSA_STEERING, PSA_CAM_BUS, 7, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}},  // driver torque
    {.msg = {{PSA_DAT_BSI, PSA_MAIN_BUS, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 20U}, { 0 }, { 0 }}},   // doors
    {.msg = {{PSA_DRIVER, PSA_MAIN_BUS, 6, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 10U}, { 0 }, { 0 }}},    // gas pedal
  };

  return BUILD_SAFETY_CFG(psa_rx_checks, PSA_TX_MSGS);
}

const safety_hooks psa_hooks = {
  .init = psa_init,
  .rx = psa_rx_hook,
  .tx = psa_tx_hook,
  .fwd = psa_fwd_hook,
  .get_counter = psa_get_counter,
  .get_checksum = psa_get_checksum,
  .compute_checksum = psa_compute_checksum,
};
