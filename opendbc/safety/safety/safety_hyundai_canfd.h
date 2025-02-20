#pragma once

#include "safety_declarations.h"
#include "safety_hyundai_common.h"

#define MAX_RX_CHECKS 16

static safety_config hyundai_canfd_init_safety_config(void) {
  static RxCheck hyundai_canfd_rx_checks[MAX_RX_CHECKS] = {0};
  safety_config ret = {
    .rx_checks = hyundai_canfd_rx_checks,
    .rx_checks_len = 0,
    .tx_msgs = NULL,
    .tx_msgs_len = 0
  };
  return ret;
}

static void add_rx_check(safety_config *safetyConfig, RxCheck config) {
  const uint8_t index = safetyConfig->rx_checks_len;
  if (index < (uint8_t)MAX_RX_CHECKS) {
    safetyConfig->rx_checks_len++;
    (void)memcpy(&safetyConfig->rx_checks[index], &config, sizeof(RxCheck));
  } else {
    // TODO: Trigger safety_rx_checks_invalid.
  }
}

static bool hyundai_canfd_alt_buttons = false;
static bool hyundai_canfd_hda2_alt_steering = false;

static int hyundai_canfd_hda2_get_lkas_addr(void) {
  return hyundai_canfd_hda2_alt_steering ? 0x110 : 0x50;
}

static uint8_t hyundai_canfd_get_counter(const CANPacket_t *to_push) {
  uint8_t ret = 0;
  if (GET_LEN(to_push) == 8U) {
    ret = GET_BYTE(to_push, 1) >> 4;
  } else {
    ret = GET_BYTE(to_push, 2);
  }
  return ret;
}

static uint32_t hyundai_canfd_get_checksum(const CANPacket_t *to_push) {
  uint32_t chksum = GET_BYTE(to_push, 0) | (GET_BYTE(to_push, 1) << 8);
  return chksum;
}

static void hyundai_canfd_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  const int pt_bus = hyundai_canfd_hda2 ? 1 : 0;
  const int scc_bus = hyundai_camera_scc ? 2 : pt_bus;

  if (bus == pt_bus) {
    // driver torque
    if (addr == 0xea) {
      int torque_driver_new = ((GET_BYTE(to_push, 11) & 0x1fU) << 8U) | GET_BYTE(to_push, 10);
      torque_driver_new -= 4095;
      update_sample(&torque_driver, torque_driver_new);
    }

    // cruise buttons
    const int button_addr = hyundai_canfd_alt_buttons ? 0x1aa : 0x1cf;
    if (addr == button_addr) {
      bool main_button = false;
      int cruise_button = 0;
      if (addr == 0x1cf) {
        cruise_button = GET_BYTE(to_push, 2) & 0x7U;
        main_button = GET_BIT(to_push, 19U);
      } else {
        cruise_button = (GET_BYTE(to_push, 4) >> 4) & 0x7U;
        main_button = GET_BIT(to_push, 34U);
      }
      hyundai_common_cruise_buttons_check(cruise_button, main_button);
    }

    // gas press, different for EV, hybrid, and ICE models
    if ((addr == 0x35) && hyundai_ev_gas_signal) {
      gas_pressed = GET_BYTE(to_push, 5) != 0U;
    } else if ((addr == 0x105) && hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BIT(to_push, 103U) || (GET_BYTE(to_push, 13) != 0U) || GET_BIT(to_push, 112U);
    } else if ((addr == 0x100) && !hyundai_ev_gas_signal && !hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BIT(to_push, 176U);
    } else {
    }

    // brake press
    if (addr == 0x175) {
      brake_pressed = GET_BIT(to_push, 81U);
    }

    // vehicle moving
    if (addr == 0xa0) {
      uint32_t front_left_speed = GET_BYTES(to_push, 8, 2);
      uint32_t rear_right_speed = GET_BYTES(to_push, 14, 2);
      vehicle_moving = (front_left_speed > HYUNDAI_STANDSTILL_THRSLD) || (rear_right_speed > HYUNDAI_STANDSTILL_THRSLD);
    }
  }

  if (bus == scc_bus) {
    // cruise state
    if ((addr == 0x1a0) && !hyundai_longitudinal) {
      // 1=enabled, 2=driver override
      int cruise_status = ((GET_BYTE(to_push, 8) >> 4) & 0x7U);
      bool cruise_engaged = (cruise_status == 1) || (cruise_status == 2);
      hyundai_common_cruise_state_check(cruise_engaged);
    }
  }

  const int steer_addr = hyundai_canfd_hda2 ? hyundai_canfd_hda2_get_lkas_addr() : 0x12a;
  bool stock_ecu_detected = (addr == steer_addr) && (bus == 0);
  if (hyundai_longitudinal) {
    // on HDA2, ensure ADRV ECU is still knocked out
    // on others, ensure accel msg is blocked from camera
    const int stock_scc_bus = hyundai_canfd_hda2 ? 1 : 0;
    stock_ecu_detected = stock_ecu_detected || ((addr == 0x1a0) && (bus == stock_scc_bus));
  }
  generic_rx_checks(stock_ecu_detected);

}

static bool hyundai_canfd_tx_hook(const CANPacket_t *to_send) {
  const SteeringLimits HYUNDAI_CANFD_STEERING_LIMITS = {
    .max_steer = 270,
    .max_rt_delta = 112,
    .max_rt_interval = 250000,
    .max_rate_up = 2,
    .max_rate_down = 3,
    .driver_torque_allowance = 250,
    .driver_torque_factor = 2,
    .type = TorqueDriverLimited,

    // the EPS faults when the steering angle is above a certain threshold for too long. to prevent this,
    // we allow setting torque actuation bit to 0 while maintaining the requested torque value for two consecutive frames
    .min_valid_request_frames = 89,
    .max_invalid_request_frames = 2,
    .min_valid_request_rt_interval = 810000,  // 810ms; a ~10% buffer on cutting every 90 frames
    .has_steer_req_tolerance = true,
  };

  bool tx = true;
  int addr = GET_ADDR(to_send);

  // steering
  const int steer_addr = (hyundai_canfd_hda2 && !hyundai_longitudinal) ? hyundai_canfd_hda2_get_lkas_addr() : 0x12a;
  if (addr == steer_addr) {
    int desired_torque = (((GET_BYTE(to_send, 6) & 0xFU) << 7U) | (GET_BYTE(to_send, 5) >> 1U)) - 1024U;
    bool steer_req = GET_BIT(to_send, 52U);

    if (steer_torque_cmd_checks(desired_torque, steer_req, HYUNDAI_CANFD_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // cruise buttons check
  const int button_addr = hyundai_canfd_alt_buttons ? 0x1aa : 0x1cf;
  if (addr == button_addr) {
    int cruise_button = 0;
    if (addr == 0x1cf) {
      cruise_button = GET_BYTE(to_send, 2) & 0x7U;
    } else {
      cruise_button = (GET_BYTE(to_send, 4) >> 4) & 0x7U;
    }
    bool is_cancel = (cruise_button == HYUNDAI_BTN_CANCEL);
    bool is_resume = (cruise_button == HYUNDAI_BTN_RESUME);

    bool allowed = (is_cancel && cruise_engaged_prev) || (is_resume && controls_allowed);
    if (!allowed) {
      tx = false;
    }
  }

  // UDS: only tester present ("\x02\x3E\x80\x00\x00\x00\x00\x00") allowed on diagnostics address
  if ((addr == 0x730) && hyundai_canfd_hda2) {
    if ((GET_BYTES(to_send, 0, 4) != 0x00803E02U) || (GET_BYTES(to_send, 4, 4) != 0x0U)) {
      tx = false;
    }
  }

  // ACCEL: safety check
  if (addr == 0x1a0) {
    int desired_accel_raw = (((GET_BYTE(to_send, 17) & 0x7U) << 8) | GET_BYTE(to_send, 16)) - 1023U;
    int desired_accel_val = ((GET_BYTE(to_send, 18) << 4) | (GET_BYTE(to_send, 17) >> 4)) - 1023U;

    bool violation = false;

    if (hyundai_longitudinal) {
      violation |= longitudinal_accel_checks(desired_accel_raw, HYUNDAI_LONG_LIMITS);
      violation |= longitudinal_accel_checks(desired_accel_val, HYUNDAI_LONG_LIMITS);
    } else {
      // only used to cancel on here
      if ((desired_accel_raw != 0) || (desired_accel_val != 0)) {
        violation = true;
      }
    }

    if (violation) {
      tx = false;
    }
  }

  return tx;
}

static int hyundai_canfd_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  if (bus_num == 0) {
    bus_fwd = 2;
  }
  if (bus_num == 2) {
    // LKAS for HDA2, LFA for HDA1
    int hda2_lfa_block_addr = hyundai_canfd_hda2_alt_steering ? 0x362 : 0x2a4;
    bool is_lkas_msg = ((addr == hyundai_canfd_hda2_get_lkas_addr()) || (addr == hda2_lfa_block_addr)) && hyundai_canfd_hda2;
    bool is_lfa_msg = ((addr == 0x12a) && !hyundai_canfd_hda2);

    // HUD icons
    bool is_lfahda_msg = ((addr == 0x1e0) && !hyundai_canfd_hda2);
    bool is_ccnc_msg = (((addr == 0x161) || (addr == 0x162)) && !hyundai_canfd_hda2);

    // CRUISE_INFO for non-HDA2, we send our own longitudinal commands
    bool is_scc_msg = ((addr == 0x1a0) && hyundai_longitudinal && !hyundai_canfd_hda2);

    bool block_msg = is_lkas_msg || is_lfa_msg || is_lfahda_msg || is_ccnc_msg || is_scc_msg;
    if (!block_msg) {
      bus_fwd = 0;
    }
  }

  return bus_fwd;
}

static safety_config hyundai_canfd_init(uint16_t param) {
  const int HYUNDAI_PARAM_CANFD_HDA2_ALT_STEERING = 128;
  const int HYUNDAI_PARAM_CANFD_ALT_BUTTONS = 32;

  // TODO: Build these more precisely like RX_CHECKS.
  static const CanMsg HYUNDAI_CANFD_HDA2_TX_MSGS[] = {
    {0x50, 0, 16},  // LKAS
    {0x1CF, 1, 8},  // CRUISE_BUTTON
    // TODO: Include this iff CANFD_ALT_BUTTONS
    {0x1AA, 1, 16}, // CRUISE_BUTTONS_ALT
    {0x2A4, 0, 24}, // CAM_0x2A4
  };

  static const CanMsg HYUNDAI_CANFD_HDA2_ALT_STEERING_TX_MSGS[] = {
    {0x110, 0, 32}, // LKAS_ALT
    {0x1CF, 1, 8},  // CRUISE_BUTTON
    // TODO: Include this iff CANFD_ALT_BUTTONS
    {0x1AA, 1, 16}, // CRUISE_BUTTONS_ALT
    // Needed for cruise control in case of ALT_BUTTONS.
    {0x1A0, 1, 32}, // CRUISE_INFO
    {0x362, 0, 32}, // CAM_0x362
  };

  static const CanMsg HYUNDAI_CANFD_HDA2_LONG_TX_MSGS[] = {
    {0x50, 0, 16},  // LKAS
    {0x1CF, 1, 8},  // CRUISE_BUTTON
    // TODO: Include this iff CANFD_ALT_BUTTONS
    {0x1AA, 1, 16}, // CRUISE_BUTTONS_ALT
    {0x2A4, 0, 24}, // CAM_0x2A4
    // LONG start
    {0x51, 0, 32},  // ADRV_0x51
    {0x730, 1, 8},  // tester present for ADAS ECU disable
    {0x12A, 1, 16}, // LFA
    {0x160, 1, 16}, // ADRV_0x160
    {0x1E0, 1, 16}, // LFAHDA_CLUSTER
    {0x1A0, 1, 32}, // CRUISE_INFO
    {0x1EA, 1, 32}, // ADRV_0x1ea
    {0x200, 1, 8},  // ADRV_0x200
    {0x345, 1, 8},  // ADRV_0x345
    {0x1DA, 1, 32}, // ADRV_0x1da
    {0x38C, 1, 32}, // ADRV_0x38c
    {0x161, 1, 32}, // MSG_161
    {0x162, 1, 32}, // MSG_162
  };

  static const CanMsg HYUNDAI_CANFD_HDA2_ALT_STEERING_LONG_TX_MSGS[] = {
    {0x110, 0, 32}, // LKAS_ALT
    {0x1CF, 1, 8},  // CRUISE_BUTTON
    // TODO: Include this iff CANFD_ALT_BUTTONS
    {0x1AA, 1, 16}, // CRUISE_BUTTONS_ALT
    {0x1A0, 1, 32}, // CRUISE_INFO
    {0x362, 0, 32}, // CAM_0x362
    // LONG start
    {0x51, 0, 32},  // ADRV_0x51
    {0x730, 1, 8},  // tester present for ADAS ECU disable
    {0x12A, 1, 16}, // LFA
    {0x160, 1, 16}, // ADRV_0x160
    {0x1E0, 1, 16}, // LFAHDA_CLUSTER
    {0x1EA, 1, 32}, // ADRV_0x1ea
    {0x200, 1, 8},  // ADRV_0x200
    {0x345, 1, 8},  // ADRV_0x345
    {0x1DA, 1, 32}, // ADRV_0x1da
    {0x38C, 1, 32}, // ADRV_0x38c
    {0x161, 1, 32}, // MSG_161
    {0x162, 1, 32}, // MSG_162
  };

  static const CanMsg HYUNDAI_CANFD_HDA1_TX_MSGS[] = {
    {0x12A, 0, 16}, // LFA
    {0x1A0, 0, 32}, // CRUISE_INFO
    {0x1CF, 2, 8},  // CRUISE_BUTTON
    {0x1AA, 2, 16}, // CRUISE_BUTTONS_ALT
    {0x1E0, 0, 16}, // LFAHDA_CLUSTER
    {0x161, 0, 32}, // MSG_161
    {0x162, 0, 32}, // MSG_162
  };


  hyundai_common_init(param);

  gen_crc_lookup_table_16(0x1021, hyundai_canfd_crc_lut);
  hyundai_canfd_alt_buttons = GET_FLAG(param, HYUNDAI_PARAM_CANFD_ALT_BUTTONS);
  hyundai_canfd_hda2_alt_steering = GET_FLAG(param, HYUNDAI_PARAM_CANFD_HDA2_ALT_STEERING);

  // no long for radar-SCC HDA1 yet
  if (!hyundai_canfd_hda2 && !hyundai_camera_scc) {
    hyundai_longitudinal = false;
  }

  safety_config ret = hyundai_canfd_init_safety_config();

  // RX Common checks.
  const int pt_bus = hyundai_canfd_hda2 ? 1 : 0;
  add_rx_check(&ret, (RxCheck){.msg = {{0x175, (pt_bus), 24, .check_checksum = true, .max_counter = 0xffU, .frequency = 50U}, { 0 }, { 0 }}});
  add_rx_check(&ret, (RxCheck){.msg = {{0xa0, (pt_bus), 24, .check_checksum = true, .max_counter = 0xffU, .frequency = 100U}, { 0 }, { 0 }}});
  add_rx_check(&ret, (RxCheck){.msg = {{0xea, (pt_bus), 24, .check_checksum = true, .max_counter = 0xffU, .frequency = 100U}, { 0 }, { 0 }}});

  if (hyundai_ev_gas_signal) {
    add_rx_check(&ret, (RxCheck){.msg = {{0x35, (pt_bus), 32, .check_checksum = true, .max_counter = 0xffU, .frequency = 100U}, { 0 }, { 0 }}});
  } else if (hyundai_hybrid_gas_signal) {
    add_rx_check(&ret, (RxCheck){.msg = {{0x105, (pt_bus), 32, .check_checksum = true, .max_counter = 0xffU, .frequency = 100U}, { 0 }, { 0 }}});
  } else {
    add_rx_check(&ret, (RxCheck){.msg = {{0x100, (pt_bus), 32, .check_checksum = true, .max_counter = 0xffU, .frequency = 100U}, { 0 }, { 0 }}});
  }

  // RX Cruise buttons.
  if (hyundai_canfd_alt_buttons) {
    add_rx_check(&ret, (RxCheck){.msg = {{0x1aa, (pt_bus), 16, .check_checksum = false, .max_counter = 0xffU, .frequency = 50U}, { 0 }, { 0 }}});
  } else {
    add_rx_check(&ret, (RxCheck){.msg = {{0x1cf, (pt_bus), 8, .check_checksum = false, .max_counter = 0xfU, .frequency = 50U}, { 0 }, { 0 }}});
  }

  if (hyundai_longitudinal) {
    // TX SCC_CONTROL.
  } else {
    // RX SCC_CONTROL.
    const int8_t scc_bus = hyundai_canfd_hda2 ? 1 : hyundai_camera_scc ? 2 : 0;
    add_rx_check(&ret, (RxCheck){.msg = {{0x1a0, (scc_bus), 32, .check_checksum = true, .max_counter = 0xffU, .frequency = 50U}, { 0 }, { 0 }}});
  }

  // TX checks.
  if (hyundai_longitudinal) {
    if (hyundai_canfd_hda2) {
      if (hyundai_canfd_hda2_alt_steering) {
        SET_TX_MSGS(HYUNDAI_CANFD_HDA2_ALT_STEERING_LONG_TX_MSGS, ret);
      } else {
        SET_TX_MSGS(HYUNDAI_CANFD_HDA2_LONG_TX_MSGS, ret);
      }
    } else {
      SET_TX_MSGS(HYUNDAI_CANFD_HDA1_TX_MSGS, ret);
    }
  } else {
    if (hyundai_canfd_hda2) {
      if (hyundai_canfd_hda2_alt_steering) {
        SET_TX_MSGS(HYUNDAI_CANFD_HDA2_ALT_STEERING_TX_MSGS, ret);
      } else {
        SET_TX_MSGS(HYUNDAI_CANFD_HDA2_TX_MSGS, ret);
      }
    } else {
      SET_TX_MSGS(HYUNDAI_CANFD_HDA1_TX_MSGS, ret);
    }
  }

  return ret;
}

const safety_hooks hyundai_canfd_hooks = {
  .init = hyundai_canfd_init,
  .rx = hyundai_canfd_rx_hook,
  .tx = hyundai_canfd_tx_hook,
  .fwd = hyundai_canfd_fwd_hook,
  .get_counter = hyundai_canfd_get_counter,
  .get_checksum = hyundai_canfd_get_checksum,
  .compute_checksum = hyundai_common_canfd_compute_checksum,
};
