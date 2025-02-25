#pragma once

#include "safety_declarations.h"
#include "safety_volkswagen_common.h"


static int volkswagen_steer_power_prev = 0;


static safety_config volkswagen_meb_init(uint16_t param) {
  // Transmit of GRA_ACC_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  static const CanMsg VOLKSWAGEN_MEB_STOCK_TX_MSGS[] = {{MSG_HCA_03, 0, 24}, {MSG_EA_01, 0, 8}, {MSG_EA_02, 0, 8}, {MSG_GRA_ACC_01, 0, 8},
                                                       {MSG_GRA_ACC_01, 2, 8}, {MSG_LDW_02, 0, 8}};

  static RxCheck volkswagen_meb_rx_checks[] = {
    {.msg = {{MSG_LH_EPS_03, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_14, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 10U}, { 0 }, { 0 }}},
    {.msg = {{MSG_Motor_51, 0, 32, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{MSG_GRA_ACC_01, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 33U}, { 0 }, { 0 }}},
    {.msg = {{MSG_QFK_01, 0, 32, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{MSG_ESC_51, 0, 48, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{MSG_Motor_54, 0, 32, .check_checksum = true, .max_counter = 15U, .frequency = 10U}, { 0 }, { 0 }}},
    {.msg = {{MSG_ESC_50, 0, 48, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{MSG_VMM_02, 0, 32, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{MSG_EML_06, 0, 64, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
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
      uint32_t fr = GET_BYTE(to_push, 10U) | GET_BYTE(to_push, 11U) << 8;
      uint32_t rr = GET_BYTE(to_push, 14U) | GET_BYTE(to_push, 15U) << 8;
      uint32_t rl = GET_BYTE(to_push, 12U) | GET_BYTE(to_push, 13U) << 8;
      uint32_t fl = GET_BYTE(to_push, 8U) | GET_BYTE(to_push, 9U) << 8;

      vehicle_moving = (fr > 0U) || (rr > 0U) || (rl > 0U) || (fl > 0U);

      UPDATE_VEHICLE_SPEED(((fr + rr + rl + fl) / 4 ) * 0.0075 / 3.6);
    }

    // Update driver input torque samples
    // Signal: LH_EPS_03.EPS_Lenkmoment (absolute torque)
    // Signal: LH_EPS_03.EPS_VZ_Lenkmoment (direction)
    if (addr == MSG_LH_EPS_03) {
      int torque_driver_new = GET_BYTE(to_push, 5) | ((GET_BYTE(to_push, 6) & 0x1FU) << 8);
      int sign = (GET_BYTE(to_push, 6) & 0x80U) >> 7;
      if (sign == 1) {
        torque_driver_new *= -1;
      }
      update_sample(&torque_driver, torque_driver_new);
    }

    if (addr == MSG_QFK_01) {
      int current_curvature = ((GET_BYTE(to_push, 5U) & 0x7F) << 8 | GET_BYTE(to_push, 4U));

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
      int acc_status = ((GET_BYTE(to_push, 11U) >> 0) & 0x07U);
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
      int accel_pedal_value = GET_BYTE(to_push, 21U) - 37;
      gas_pressed = accel_pedal_value != 0;
    }

    generic_rx_checks((addr == MSG_HCA_03));
  }
}

static bool volkswagen_meb_tx_hook(const CANPacket_t *to_send) {
  int addr = GET_ADDR(to_send);
  bool tx = true;

  // Safety check for HCA_03 Heading Control Assist curvature
  if (addr == MSG_HCA_03) {
    int desired_curvature_raw = (GET_BYTE(to_send, 3U) | (GET_BYTE(to_send, 4U) & 0x7FU << 8));

    bool sign = GET_BIT(to_send, 39U);
    if (!sign) {
      desired_curvature_raw *= -1;
    }

    bool steer_req = GET_BIT(to_send, 14U);
    int steer_power = (GET_BYTE(to_send, 2U) >> 0) & 0x7FU;

    // TODO: implement lateral accel limits based on vehicle speed and QFK curvature
    // TODO: review and implement power backoff on driver input torque
    // if (desired_curvature > conditions-tbd) {
    //  tx = false;
    //
    //  // steer power is still allowed to decrease to zero monotonously
    //  // while controls are not allowed anymore
    //  if (steer_req && steer_power != 0) {
    //    if (steer_power < volkswagen_steer_power_prev) {
    //      tx = true;
    //    }
    //  }
    // }

    if (!steer_req && steer_power != 0) {
      tx = false; // steer power is not 0 when disabled
    }

    volkswagen_steer_power_prev = steer_power;
  }

  // FORCE CANCEL: ensuring that only the cancel button press is sent when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if ((addr == MSG_GRA_ACC_01) && !controls_allowed) {
    // disallow resume and set: bits 16 and 19
    if ((GET_BYTE(to_send, 2) & 0x9U) != 0U) {
      tx = false;
    }
  }

  return tx;
}

static int volkswagen_meb_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  switch (bus_num) {
    case 0:
      bus_fwd = 2;
      break;
    case 2:
      if ((addr == MSG_HCA_03) || (addr == MSG_LDW_02) || (addr == MSG_EA_01) || (addr == MSG_EA_02)) {
        // openpilot takes over LKAS steering control and related HUD messages from the camera
        bus_fwd = -1;
      } else {
        // Forward all remaining traffic from Extended CAN devices to J533 gateway
        bus_fwd = 0;
      }
      break;
    default:
      // No other buses should be in use; fallback to do-not-forward
      bus_fwd = -1;
      break;
  }

  return bus_fwd;
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
