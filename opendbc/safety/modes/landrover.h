#pragma once

#include "opendbc/safety/declarations.h"

//#define FLEXRAY_MAX_ANGLE   1170  // angle * deg_to_can
//#define FLEXRAY_DEG_TO_CAN 13.009 //  1/factor, 1/0.07687
#define FLEXRAY_MAX_ANGLE   1125  // angle * deg_to_can
#define FLEXRAY_DEG_TO_CAN 12.5   //  1/factor, 1/0.08
#define FLEXRAY_USE_PSCM_OUT 1

static bool landrover_flexray_harness = true;

static void landrover_rx_hook(const CANPacket_t *msg) {
  if (landrover_flexray_harness) {
    if (msg->bus == 0U)  {
      #ifndef FLEXRAY_USE_PSCM_OUT
      // Steering angle: (0.1 * val) - 780 in deg.
      if (msg->addr == 0x56) {
        // Store it 1/10 deg to match steering request
        int angle_raw = (((msg->data[3] & 0x3FU) << 8) | msg->data[4]);

        int angle_meas_new = (angle_raw - 7800U ) * 0.1 * FLEXRAY_DEG_TO_CAN;
        update_sample(&angle_meas, angle_meas_new);
      }
      #else

      // PSCM_Out Steering angleTorque: (0.07687 * val) - 691.83 in deg.
      if (msg->addr == 0x32U) {
        // Store it 1/10 deg to match steering request
        unsigned int raw_val = (((msg->data[2] & 0x3FU) << 8) | msg->data[3]);
        int angle_meas_new = (int)raw_val - 9000;
        update_sample(&angle_meas, angle_meas_new);
      }
      #endif

      // Vehicle speed (info02)
      if (msg->addr == 0x11U) {
        int speed = (msg->data[4] << 8) | msg->data[5];

        vehicle_moving = speed > 0.0;
        UPDATE_VEHICLE_SPEED(speed * 0.01 * KPH_TO_MS);
      }

      // Gas pressed
      if (msg->addr == 0x189U) {
        gas_pressed = (GET_BIT(msg, 58U) == 1);
      }

      // Brake pressed
      if (msg->addr == 0x84U) {
        brake_pressed = (GET_BIT(msg, 22U) == 1);
      }

      // Cruise state
      if (msg->addr == 258U) {
        pcm_cruise_check((GET_BIT(msg, 34U) == 1));
      }

      // lkas btn
      #if 0
      if (msg->addr == 0x24U) {
        mads_button_press = GET_BIT(msg, 61U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
      }
      #endif

    }

  } else {

  }

}


static bool landrover_tx_hook(const CANPacket_t *msg) {
  bool tx = true;

  const AngleSteeringLimits LANDROVER_STEERING_LIMITS = {
    .max_angle = FLEXRAY_MAX_ANGLE,  // angle * deg_to_can
    .angle_deg_to_can = FLEXRAY_DEG_TO_CAN, //  1/factor, 1/0.076
    .angle_rate_up_lookup = {
      {0., 5., 25.},
      {2.5, 1.5, 0.2}
    },
    .angle_rate_down_lookup = {
      {0., 5., 25.},
      {5., 2.0, 0.3}
    },
  };


  // TODO find long params
  const LongitudinalLimits LANDROVER_LONG_LIMITS = {
    .max_accel = 425,       // 2 m/s^2
    .min_accel = 288,       // -3.48 m/s^2
    .inactive_accel = 375,  // 0. m/s^2
  };

  SAFETY_UNUSED(LANDROVER_LONG_LIMITS);

  if (landrover_flexray_harness) {

    if (msg->bus == 1U) {

      // Steering control 
      // (0.076 * val) - 684 in deg.
      // deg_to_can = 1/0.076 , max_angle = angle * deg_to_can
      if (msg->addr == 0x1F0U) {
        unsigned int raw_angle_can = ((msg->data[3] & 0x3FU) << 8) | msg->data[4];
        int desired_angle = (int)raw_angle_can - 9000;

        bool steer_control_enabled = GET_BIT(msg, 31U) == 1;

        if (steer_angle_cmd_checks(desired_angle, steer_control_enabled, LANDROVER_STEERING_LIMITS)) {
          tx = false;
        }
      }
    }
  } else {

  }

  return tx;
}


static safety_config landrover_init(uint16_t param) {
  const int LANDROVER_PARAM_FLEXRAY_HARNESS = 1;

#ifdef _RR_2017_
  // CAN messages for RANGE ROVER 2017 camera
  static const CanMsg LANDROVER_TX_MSGS[] = {
     {0x28F, 0, 8, .check_relay = true},
     {0x3D4, 0, 8, .check_relay = true},
     {0x1D8, 0, 8, .check_relay = true},
  };
#endif

  // CAN messages for OP to Flexray board
  // 0x1F0 = LkasCmd, 0x1F1 = ACC
  static const CanMsg LANDROVER_FLEXRAY_TX_MSGS[] = {
     {0x1F0, 1, 8, .check_relay = false},
     {0x1F9, 1, 8, .check_relay = false},
     {0x1BE, 0, 8, .check_relay = true, .disable_static_blocking = true}, // check for relay
  };

#ifdef _RR_2017_
  static RxCheck landrover_rr_rx_checks[] = {
    {.msg = {{0xf2, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // EPS_01 (STEER_ANGLE01)
   {.msg = {{0x1CB, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},    // SPEED_02  SPEED02
    {.msg = {{0x158, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // ACCELATOR_DRIVER (ACCELATOR_DRIVER)
    {.msg = {{0x156, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},    // CRUISE_CONTROL (DRIVER_BRAKE, CRUISE_ON)
    {.msg = {{0x28F, 2, 8, 25U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // CAM msg LKAS_RUN
  };
#endif

  static RxCheck landrover_flexray_rx_checks[] = {
    {.msg = {{0x24, 0, 8, 15U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // LKAS_btn 
#ifndef FLEXRAY_USE_PSCM_OUT
    {.msg = {{0x56, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
#else
    {.msg = {{0x32, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // PSCM_Out (angleTorque)
#endif
    {.msg = {{0x11, 0, 8, 25U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},    // Speed Info02 
    {.msg = {{0x189, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // GasPedal (gas pedal)
    {.msg = {{0x84, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},    // StopAndGo (brakes)
    {.msg = {{258, 0, 8, 25U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},     // CruiseInfo (cruise state)
  };


  landrover_flexray_harness = GET_FLAG(param, LANDROVER_PARAM_FLEXRAY_HARNESS);

  safety_config ret;

  //if (landrover_flexray_harness) {
    ret = BUILD_SAFETY_CFG(landrover_flexray_rx_checks, LANDROVER_FLEXRAY_TX_MSGS);
  //} else {
  //  ret = BUILD_SAFETY_CFG(landrover_rr_rx_checks, LANDROVER_TX_MSGS);
  //}
  return ret;
}

const safety_hooks landrover_hooks = {
  .init = landrover_init,
  .rx = landrover_rx_hook,
  .tx = landrover_tx_hook,
};
