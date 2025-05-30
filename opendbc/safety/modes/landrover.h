#pragma once

#include "opendbc/safety/safety_declarations.h"

static void landrover_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);

  if (bus == 0)  {
    int addr = GET_ADDR(to_push);

    /**
    // Steering angle: (0.1 * val) - 780 in deg.
    if (addr == 0x56) {
      // Store it 1/10 deg to match steering request
      int angle_meas_new = (((GET_BYTE(to_push, 3) & 0x3FU) << 8) | GET_BYTE(to_push, 4)) - 7800U;
      update_sample(&angle_meas, angle_meas_new);
    }
    **/

    // PSCM_Out Steering angleTorque: (0.07687 * val) - 691.89 in deg.
    if (addr == 0x32) {
      // Store it 1/10 deg to match steering request
      unsigned int raw_val = (((GET_BYTE(to_push, 2) & 0x3FU) << 8) | GET_BYTE(to_push, 3));
      int angle_meas_new = (int)raw_val - 9000;
      update_sample(&angle_meas, angle_meas_new);
    }

    // Vehicle speed (info02)
    if (addr == 0x11) {
      // Vehicle speed: (val * 0.01) / MS_TO_KPH
      uint8_t raw_high = (uint8_t)GET_BYTE(to_push, 4);
      uint8_t high_byte = raw_high & 0x7FU;
      uint8_t low_byte = (uint8_t)GET_BYTE(to_push, 5);

      float high = (float)high_byte;
      float low = (float)low_byte;

      float speed = (((high * 256.0f) + low) * 0.01f) / 3.6f;

      vehicle_moving = speed > 0.0;
      UPDATE_VEHICLE_SPEED(speed);
    }

    // Gas pressed
    if (addr == 0x189) {
      gas_pressed = (GET_BIT(to_push, 58U) == 1);
    }

    // Brake pressed
    if (addr == 0x84) {
      brake_pressed = (GET_BIT(to_push, 22U) == 1);
    }

    // Cruise state
    if (addr == 0x1) {
      pcm_cruise_check((GET_BIT(to_push, 54U) == 1));
    }
  }
}


static bool landrover_tx_hook(const CANPacket_t *to_send) {
  const AngleSteeringLimits LANDROVER_STEERING_LIMITS = {
    .max_angle = 1171,  // 90 deg, but LKAS about 30 deg
    .angle_deg_to_can = 13.009,
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

  UNUSED(LANDROVER_LONG_LIMITS);

  bool tx = true;
  int bus = GET_BUS(to_send);

  if (bus == 1) {

    int addr = GET_ADDR(to_send);

    // Steering control 
    // (0.07783 * val) - 729.63 in deg.
    if (addr == 0x1F0) {
      // We use 1/12.8485 deg as a unit here
      unsigned int raw_angle_can = ((GET_BYTE(to_send, 3) & 0x3FU) << 8) | GET_BYTE(to_send, 4);
      int desired_angle = (int)raw_angle_can - 9000;

      bool steer_control_enabled = GET_BIT(to_send, 31U);

      if (steer_angle_cmd_checks(desired_angle, steer_control_enabled, LANDROVER_STEERING_LIMITS)) {
        tx = false;
      }

    }
  }

  return tx;
}


static bool landrover_fwd_hook(int bus, int addr) {
  bool block_msg = false;

  UNUSED(bus);
  UNUSED(addr);

  //
  // Change data in flexray car harness
  // LKAS cmd 0x1F0, 0x1F1  50hz
  // Lane Info HUD 0x3101   25hz
  //
  return block_msg;
}

static safety_config landrover_init(uint16_t param) {
  // 0x1F0 = LkasCmd, 0x1F1 = ACC
  static const CanMsg LANDROVER_TX_MSGS[] = {
     {0x1F0, 1, 8, .check_relay = false},
     {0x1F9, 1, 8, .check_relay = false},
     {0x1BE, 0, 8, .check_relay = true, .disable_static_blocking = true}, // check for relay
  };
  static const CanMsg LANDROVER_LONG_TX_MSGS[] = {
     {0x1F0, 1, 8, .check_relay = false},
     {0x1F9, 1, 8, .check_relay = false},
     {0x1BE, 0, 8, .check_relay = true, .disable_static_blocking = true},  // check for relay
  };

  static RxCheck landrover_rx_checks[] = {
    {.msg = {{0x56, 0, 8, .frequency = 100U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // SWM_Angle (steer angle)
    {.msg = {{0x32, 0, 8, .frequency = 50U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // PSCM_Out (angleTorque)
    {.msg = {{0x11, 0, 8, .frequency = 25U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},    // Speed Info02 
    {.msg = {{0x2e, 0, 4, .frequency = 50U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},    // SWM_Torque (driver torque)
    {.msg = {{0x189, 0, 8, .frequency = 10U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // GasPedal (gas pedal)
    {.msg = {{0x84, 0, 8, .frequency = 50U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},    // StopAndGo (brakes)
    {.msg = {{0x1, 0, 8, .frequency = 25U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},     // CruiseInfo (cruise state)
    {.msg = {{0x1BE, 2, 8, .frequency = 13U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // CAM msg
  };


  UNUSED(param);
  UNUSED(LANDROVER_LONG_TX_MSGS);

  return BUILD_SAFETY_CFG(landrover_rx_checks, LANDROVER_TX_MSGS);
}

const safety_hooks landrover_hooks = {
  .init = landrover_init,
  .rx = landrover_rx_hook,
  .tx = landrover_tx_hook,
  .fwd = landrover_fwd_hook,
};
