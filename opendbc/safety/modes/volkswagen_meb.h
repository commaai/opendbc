#pragma once

#include "opendbc/safety/safety_declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"


static int volkswagen_steer_power_prev = 0;


static safety_config volkswagen_meb_init(uint16_t param) {
  // Transmit of GRA_ACC_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  static const CanMsg VOLKSWAGEN_MEB_STOCK_TX_MSGS[] = {{MSG_HCA_03, 0, 24, .check_relay=true}, {MSG_EA_01, 0, 8, .check_relay=false}, {MSG_EA_02, 0, 8, .check_relay=false}, {MSG_GRA_ACC_01, 0, 8, .check_relay=false},
                                                       {MSG_GRA_ACC_01, 2, 8, .check_relay=false}, {MSG_LDW_02, 0, 8, .check_relay=false}};

  static RxCheck volkswagen_meb_rx_checks[] = {
    {.msg = {{MSG_LH_EPS_03, 0, 8, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_14, 0, 8, .max_counter = 15U, .frequency = 10U}, { 0 }, { 0 }}},
    {.msg = {{MSG_Motor_51, 0, 32, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{MSG_GRA_ACC_01, 0, 8, .max_counter = 15U, .frequency = 33U}, { 0 }, { 0 }}},
    {.msg = {{MSG_QFK_01, 0, 32, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{MSG_ESC_51, 0, 48, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{MSG_Motor_54, 0, 32, .max_counter = 15U, .frequency = 10U}, { 0 }, { 0 }}},
  };

  volkswagen_set_button_prev = false;
  volkswagen_resume_button_prev = false;

  UNUSED(param);

  gen_crc_lookup_table_8(0x2F, volkswagen_crc8_lut_8h2f);
  return BUILD_SAFETY_CFG(volkswagen_meb_rx_checks, VOLKSWAGEN_MEB_STOCK_TX_MSGS);
}

static void volkswagen_meb_rx_hook(const CANPacket_t *to_push) {
  if (GET_BUS(to_push) == 0U) {
    int addr = GET_ADDR(to_push);

    // Update in-motion state by sampling wheel speeds
    if (addr == MSG_ESC_51) {
      uint32_t fl = GET_BYTES(to_push, 8, 2);
      uint32_t fr = GET_BYTES(to_push, 10, 2);
      uint32_t rl = GET_BYTES(to_push, 12, 2);
      uint32_t rr = GET_BYTES(to_push, 14, 2);

      vehicle_moving = (fl + fr + rl + rr) > 0U;

      UPDATE_VEHICLE_SPEED(((fr + rr + rl + fl) / 4.0) * 0.0075 / 3.6);
    }

    // Update driver input torque samples
    // Signal: LH_EPS_03.EPS_Lenkmoment (absolute torque)
    // Signal: LH_EPS_03.EPS_VZ_Lenkmoment (direction)
    if (addr == MSG_LH_EPS_03) {
      int torque_driver_new = GET_BYTES(to_push, 5, 2) & 0x1FFFU;
      int sign = (GET_BYTE(to_push, 6) & 0x80U) >> 7;
      if (sign == 1) {
        torque_driver_new *= -1;
      }
      update_sample(&torque_driver, torque_driver_new);
    }

    if (addr == MSG_QFK_01) {
      int current_curvature = GET_BYTES(to_push, 4, 2) & 0x7FFFU;

      bool current_curvature_sign = GET_BIT(to_push, 55U);
      if (current_curvature_sign) {
        current_curvature *= -1;
      }

      update_sample(&angle_meas, current_curvature);
    }

    // Update cruise state
    if (addr == MSG_Motor_51) {
      // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
      // Always exit controls on main switch off
      int acc_status = (GET_BYTE(to_push, 11U) & 0x07U);
      bool cruise_engaged = (acc_status == 3) || (acc_status == 4) || (acc_status == 5);
      acc_main_on = cruise_engaged || (acc_status == 2);

      pcm_cruise_check(cruise_engaged);

      if (!acc_main_on) {
        controls_allowed = false;
      }
    }

    // update brake pedal
    if (addr == MSG_MOTOR_14) {
      brake_pressed = GET_BIT(to_push, 28U);
    }

    // update accel pedal
    if (addr == MSG_Motor_54) {
      int accel_pedal_value = GET_BYTE(to_push, 21U) - 37U;
      gas_pressed = accel_pedal_value != 0;
    }
  }
}

// TODO: move STEER constants somewhere sensible
#define STEER_POWER_MAX 50
#define STEER_POWER_MIN 20
#define DRIVER_INPUT_MIN 60
#define DRIVER_INPUT_MAX 300
#define STEER_POWER_STEP 2

static bool volkswagen_curvature_cmd_checks(int steer_power, int steer_curvature, int steer_req) {
  bool violation = false;

  if (steer_req == 0) {
    violation |= (steer_power != 0);
    violation |= (steer_curvature != 0);
  } else {
    if (controls_allowed) {
      violation |= steer_power > STEER_POWER_MAX;
      violation |= steer_power < (volkswagen_steer_power_prev - STEER_POWER_STEP);
      violation |= steer_power > (volkswagen_steer_power_prev + STEER_POWER_STEP);
    } else {
      bool disengaging_power = steer_power == (volkswagen_steer_power_prev - STEER_POWER_STEP);
      violation |= volkswagen_steer_power_prev == 0;
      violation |= (steer_power > 0) && !disengaging_power;
    }
  }

  return violation;
}

static bool volkswagen_meb_tx_hook(const CANPacket_t *msg) {
  int addr = GET_ADDR(msg);
  bool tx = true;

  // Safety check for HCA_03 Heading Control Assist curvature
  if (addr == MSG_HCA_03) {
    int steer_curvature = GET_BYTES(msg, 3, 2) & 0x7FFFU;

    bool sign = GET_BIT(msg, 39U);
    if (!sign) {
      steer_curvature *= -1;
    }

    bool steer_req = ((GET_BYTE(msg, 1) >> 4) & 0x7U) == 4U;
    int steer_power = GET_BYTE(msg, 2U) * 0.4;

    if (volkswagen_curvature_cmd_checks(steer_power, steer_curvature, steer_req)) {
      // tx = false;
      tx = true;
      volkswagen_steer_power_prev = 0;
    } else {
      volkswagen_steer_power_prev = steer_power;
    }
  }

  // FORCE CANCEL: ensuring that only the cancel button press is sent when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if ((addr == MSG_GRA_ACC_01) && !controls_allowed) {
    // disallow resume and set: bits 16 and 19
    if ((GET_BYTE(msg, 2) & 0x9U) != 0U) {
      tx = false;
    }
  }

  return tx;
}

static bool volkswagen_meb_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  switch (bus_num) {
    case 2:
      if ((addr == MSG_HCA_03) || (addr == MSG_LDW_02) || (addr == MSG_EA_01) || (addr == MSG_EA_02)) {
        // openpilot takes over LKAS steering control and related HUD messages from the camera
        block_msg = true;
      }
      break;
    default:
      break;
  }

  return block_msg;
}

const safety_hooks volkswagen_meb_hooks = {
  .init = volkswagen_meb_init,
  .rx = volkswagen_meb_rx_hook,
  .tx = volkswagen_meb_tx_hook,
  .fwd = volkswagen_meb_fwd_hook,
  .get_counter = volkswagen_mqb_meb_get_counter,
  .get_checksum = volkswagen_mqb_meb_get_checksum,
  .compute_checksum = volkswagen_mqb_meb_compute_crc,
};
