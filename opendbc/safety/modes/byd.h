#pragma once

#include "opendbc/safety/declarations.h"

// Hard override thresholds, mirrored in carstate.py: 100.0 Nm of column torque for 100 ms
#define BYD_DRIVER_TORQUE_DISENGAGE 1000
#define BYD_DRIVER_TORQUE_FRAMES 5
#define BYD_WHEELSPEED_TO_KPH 0.072  // 0.02 m/s/LSB, mirrored in values.py
#define BYD_PARAM_LKS_ON 2U
#define BYD_PARAM_LONG_CONTROL 4U
#define BYD_CAM_ACC_CMD_HIST 3U

static int byd_driver_torque_frames = 0;
static bool byd_lks_on = false;
static bool byd_lks_btn_last = false;
static bool byd_acc_on = false;
static bool byd_acc_main = false;
static bool byd_longitudinal = false;
static bool byd_set_res_last = false;
static uint8_t byd_stock_btns = 0;
static uint8_t byd_stock_btns_prev = 0;
static uint8_t byd_cam_acc_cmd[BYD_CAM_ACC_CMD_HIST][6] = {{0}};
static uint8_t byd_cam_acc_cmd_idx = 0;
static uint8_t byd_cam_acc_cmd_count = 0;
static uint32_t byd_op_steer_ts = 0;
static uint32_t byd_op_hud_ts = 0;

#define BYD_OP_STEER_TIMEOUT_US 200000U
#define BYD_OP_HUD_TIMEOUT_US 2000000U

// openpilot sends 0x1E2 for its whole engagement, STEER_REQ=0 included, so stock LKS
// gets the EPS back only once openpilot stops talking to it. Timing the handback off
// the last accepted STEER_REQ=1 instead put the camera on 0x1E2 mid-engagement, since
// rate limited commands leave gaps.
static bool byd_stock_lat_allowed(void) {
  bool allowed = true;
  if (byd_op_steer_ts != 0U) {
    allowed = safety_get_ts_elapsed(microsecond_timer_get(), byd_op_steer_ts) > BYD_OP_STEER_TIMEOUT_US;
  }
  return allowed;
}

static bool byd_stock_hud_allowed(void) {
  // Camera 0x316 stays blocked while either OP 0x1E2 or OP 0x316 is recent, so a
  // HUD-only hold (long-mode release) keeps our cluster bits without idle 0x1E2.
  const uint32_t now = microsecond_timer_get();
  bool allowed = true;
  if (byd_op_steer_ts != 0U) {
    allowed = allowed && (safety_get_ts_elapsed(now, byd_op_steer_ts) > BYD_OP_HUD_TIMEOUT_US);
  }
  if (byd_op_hud_ts != 0U) {
    allowed = allowed && (safety_get_ts_elapsed(now, byd_op_hud_ts) > BYD_OP_HUD_TIMEOUT_US);
  }
  return allowed;
}

// SET RES DEC INC ACC_ON. LKAS_ON_BTN is not in the mask: bus 2 may still pulse it.
static uint8_t byd_button_mask(const CANPacket_t *msg) {
  uint8_t mask = 0;
  if ((msg->data[0] & 0x08U) != 0U) { mask |= 0x01U; }  // SET
  if ((msg->data[0] & 0x10U) != 0U) { mask |= 0x02U; }  // RES
  if ((msg->data[1] & 0x80U) != 0U) { mask |= 0x04U; }  // DEC
  if ((msg->data[2] & 0x01U) != 0U) { mask |= 0x08U; }  // INC
  if ((msg->data[2] & 0x08U) != 0U) { mask |= 0x10U; }  // ACC_ON
  return mask;
}

static void byd_cam_acc_cmd_push(const CANPacket_t *msg) {
  for (int i = 0; i < 6; i++) {
    byd_cam_acc_cmd[byd_cam_acc_cmd_idx][i] = msg->data[i];
  }
  byd_cam_acc_cmd_idx = (byd_cam_acc_cmd_idx + 1U) % BYD_CAM_ACC_CMD_HIST;
  if (byd_cam_acc_cmd_count < BYD_CAM_ACC_CMD_HIST) {
    byd_cam_acc_cmd_count += 1U;
  }
}

static bool byd_cam_acc_cmd_match(const CANPacket_t *msg) {
  bool matched = false;
  for (uint8_t h = 0U; h < byd_cam_acc_cmd_count; h++) {
    bool same = true;
    for (int i = 0; i < 6; i++) {
      same = same && (msg->data[i] == byd_cam_acc_cmd[h][i]);
    }
    matched = matched || same;
  }
  return matched;
}

static uint8_t byd_get_counter(const CANPacket_t *msg) {
  // Speed uses the high nibble; cruise status uses the low nibble.
  return (msg->addr == 0x1F0U) ? (msg->data[6] >> 4) : (msg->data[6] & 0xFU);
}

static uint32_t byd_get_checksum(const CANPacket_t *msg) {
  int len = GET_LEN(msg);
  return msg->data[len - 1];
}

static uint32_t byd_compute_checksum(const CANPacket_t *msg) {
  uint8_t checksum = 0;
  int len = GET_LEN(msg);
  for (int i = 0; i < (len - 1); i++) {
    checksum += msg->data[i];
  }
  return (uint8_t)(~checksum);
}

static void byd_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // Steering angle: 0.1 deg/LSB, signed
    if (msg->addr == 0x11FU) {
      int angle_meas_new = to_signed((msg->data[1] << 8) | msg->data[0], 16);  // STEER_ANGLE_2
      update_sample(&angle_meas, angle_meas_new);
    }

    // Column driver torque: 0.1 Nm/LSB, signed. The EPS measurement in 0x11F is attenuated
    // too far to see an override, so the hard disengage uses this instead.
    if (msg->addr == 0x1FCU) {
      int driver_torque = to_signed((msg->data[1] << 4) | (msg->data[0] >> 4), 12);  // DRIVER_TORQUE

      if ((driver_torque > BYD_DRIVER_TORQUE_DISENGAGE) || (driver_torque < -BYD_DRIVER_TORQUE_DISENGAGE)) {
        if (byd_driver_torque_frames < BYD_DRIVER_TORQUE_FRAMES) {
          byd_driver_torque_frames += 1;
        }
      } else {
        byd_driver_torque_frames = 0;
      }
      steering_disengage = byd_driver_torque_frames >= BYD_DRIVER_TORQUE_FRAMES;
    }

    if (msg->addr == 0x1F0U) {
      int speed = (msg->data[1] << 8) | msg->data[0];  // WHEELSPEED_CLEAN
      vehicle_moving = speed > 0;
      UPDATE_VEHICLE_SPEED(speed * BYD_WHEELSPEED_TO_KPH * KPH_TO_MS);
    }

    // Brake from DRIVE_STATE. Gas is PEDAL.GAS_PEDAL; RAW_THROTTLE stays high under ACC.
    if (msg->addr == 0x242U) {
      brake_pressed = (msg->data[4] >> 5) & 0x1U;   // BRAKE_PRESSED
    }
    if (msg->addr == 0x342U) {
      gas_pressed = msg->data[0] > 0U;              // GAS_PEDAL
    }

    // Driver LKS button. Camera LKAS_STATE 0/4 are not this switch. Bus 2 spoofs
    // must not land here or we would toggle our own latch.
    if (msg->addr == 0x3B0U) {
      byd_stock_btns_prev = byd_stock_btns;
      byd_stock_btns = byd_button_mask(msg);
      const bool btn = GET_BIT(msg, 6U);
      if (btn && (!byd_lks_btn_last)) {
        byd_lks_on = !byd_lks_on;
        if (!byd_longitudinal) {
          pcm_cruise_check(byd_acc_on && byd_lks_on);
        }
      }
      byd_lks_btn_last = btn;

      // Long mode: falling edge of SET or RES engages only while the LKS latch
      // is on and the camera still reports ACC main (state 2/3/5). ACC_ON cancels.
      // Brake and gas are handled by the common checks.
      if (byd_longitudinal) {
        const bool set_res = (byd_stock_btns & 0x03U) != 0U;
        if (!set_res && byd_set_res_last && byd_lks_on && byd_acc_main) {
          controls_allowed = true;
        }
        if (!byd_lks_on || ((byd_stock_btns & 0x10U) != 0U)) {
          controls_allowed = false;
        }
        byd_set_res_last = set_res;
      }
    }
  }

  if (msg->bus == 2U) {
    // Cruise state
    if (msg->addr == 0x32DU) {
      // ACC_STATE: 0=OFF, 2=ACC_ON, 3=ACC_ACTIVE, 5=FORCE_ACCEL, 7=ERROR
      uint8_t acc_state = (msg->data[2] >> 3) & 0x7U;
      byd_acc_on = (acc_state == 3U) || (acc_state == 5U);
      byd_acc_main = (acc_state == 2U) || byd_acc_on;
      if (byd_longitudinal) {
        if (!byd_acc_main) {
          controls_allowed = false;
        }
      } else {
        pcm_cruise_check(byd_acc_on && byd_lks_on);
      }
    }

    // Last camera ACC command. A bus-0 copy of a recent one is AEB passthrough.
    if (byd_longitudinal && (msg->addr == 0x32EU)) {
      byd_cam_acc_cmd_push(msg);
    }
  }
}

static bool byd_tx_hook(const CANPacket_t *msg) {
  const AngleSteeringLimits BYD_STEERING_LIMITS = {
    .max_angle = 3900,  // 390 deg
    .angle_deg_to_can = 10,
    .frequency = 50U,
  };

  // NOTE: based off BYD_ATTO_3 to match openpilot
  const AngleSteeringParams BYD_STEERING_PARAMS = {
    .slip_factor = -0.0006166479109059387,  // calc_slip_factor(VM)
    .steer_ratio = 17.5,
    .wheelbase = 2.72,
  };

  bool tx = true;

  // Steering control: 0.1 deg/LSB, signed
  if (msg->addr == 0x1E2U) {
    int desired_angle = to_signed((msg->data[4] << 8) | msg->data[3], 16);  // STEER_ANGLE
    bool steer_req = ((msg->data[2] >> 5) & 0x1U) != 0U;                    // STEER_REQ

    // Ownership tracks any OP 0x1E2, heartbeat included. A rate limited angle is
    // still openpilot driving.
    byd_op_steer_ts = microsecond_timer_get();

    if (steer_angle_cmd_checks_vm(desired_angle, steer_req, BYD_STEERING_LIMITS, BYD_STEERING_PARAMS)) {
      tx = false;
    }
  }

  // HUD stay: any OP 0x316 holds camera 0x316 for 2 s (see byd_stock_hud_allowed).
  if (msg->addr == 0x316U) {
    byd_op_hud_ts = microsecond_timer_get();
  }

  // Bus 0: only cancel (ACC_ON_BTN) while stock cruise is engaged, or button release.
  // Bus 2: SET/RES/DEC/INC/ACC_ON must be a subset of the last two stock frames.
  // LKAS_ON_BTN stays free. Long mode never relays SET/RES/DEC/INC.
  if (msg->addr == 0x3B0U) {
    uint8_t mask = byd_button_mask(msg);
    if (msg->bus == 2U) {
      bool stock_ok = (mask & ~(byd_stock_btns | byd_stock_btns_prev)) == 0U;
      if (byd_longitudinal) {
        stock_ok = stock_ok && ((mask & 0x0FU) == 0U);
      }
      tx = stock_ok;
    } else {
      bool set_res = (msg->data[0] & 0x18U) != 0U;
      bool lkas_on = (msg->data[0] & 0x40U) != 0U;
      bool distance = ((msg->data[1] & 0x80U) != 0U) || ((msg->data[2] & 0x1U) != 0U);
      bool cancel = (msg->data[2] & 0x8U) != 0U;
      tx = !set_res && !lkas_on && !distance && (!cancel || cruise_engaged_prev);
    }
  }

  // 0x32E: our accel, or a byte-exact copy of a camera frame from the last 60 ms.
  if (msg->addr == 0x32EU) {
    const LongitudinalLimits BYD_LONG_LIMITS = {
      .max_accel = 2000,
      .min_accel = -3500,
      .inactive_accel = 0,
    };
    int accel = ((int)msg->data[0] - 100) * 50;
    if (!byd_cam_acc_cmd_match(msg) && longitudinal_accel_checks(accel, BYD_LONG_LIMITS)) {
      tx = false;
    }
  }

  return tx;
}

static bool byd_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;
  if (bus_num == 2) {
    if (addr == 0x1E2) {
      block_msg = !byd_stock_lat_allowed();
    } else if (addr == 0x316) {
      block_msg = !byd_stock_hud_allowed();
    } else if (byd_longitudinal && ((addr == 0x32E) || (addr == 0x32D))) {
      block_msg = true;
    } else {
    }
  } else if (byd_longitudinal && (bus_num == 0) && (addr == 0x3B0)) {
    block_msg = true;
  }
  return block_msg;
}

static safety_config byd_init(uint16_t param) {
  byd_driver_torque_frames = 0;
  byd_lks_on = GET_FLAG(param, BYD_PARAM_LKS_ON);
  byd_lks_btn_last = false;
  byd_acc_on = false;
  byd_acc_main = false;
  byd_set_res_last = false;
  byd_stock_btns = 0;
  byd_stock_btns_prev = 0;
  byd_cam_acc_cmd_idx = 0;
  byd_cam_acc_cmd_count = 0;
  byd_op_steer_ts = 0;
  byd_op_hud_ts = 0;

#ifdef ALLOW_DEBUG
  byd_longitudinal = GET_FLAG(param, BYD_PARAM_LONG_CONTROL);
#else
  byd_longitudinal = false;
#endif

  static const CanMsg BYD_TX_MSGS[] = {
    {0x1E2, 0, 8, .check_relay = true, .disable_static_blocking = true},   // STEERING_MODULE_ADAS
    {0x316, 0, 8, .check_relay = true, .disable_static_blocking = true},   // LKAS_HUD_ADAS
    {0x3B0, 0, 8, .check_relay = false},  // PCM_BUTTONS (cruise cancel button spoof)
    {0x3B0, 2, 8, .check_relay = false},  // PCM_BUTTONS (camera LKS neutralize / restore)
  };

  static const CanMsg BYD_LONG_TX_MSGS[] = {
    {0x1E2, 0, 8, .check_relay = true, .disable_static_blocking = true},   // STEERING_MODULE_ADAS
    {0x316, 0, 8, .check_relay = true, .disable_static_blocking = true},   // LKAS_HUD_ADAS
    {0x32D, 0, 8, .check_relay = true},                                    // ACC_HUD_ADAS
    {0x32E, 0, 8, .check_relay = true},                                    // ACC_CMD
    {0x3B0, 0, 8, .check_relay = false},  // PCM_BUTTONS (cruise cancel button spoof)
    {0x3B0, 2, 8, .check_relay = false},  // PCM_BUTTONS (camera LKS neutralize / restore)
  };

  static const RxCheck byd_rx_checks[] = {
    {.msg = {{0x11F, 0, 5, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // STEER_MODULE_2 (4-bit checksum)
    {.msg = {{0x1FC, 0, 8,  50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                          // STEERING_TORQUE
    {.msg = {{0x1F0, 0, 8,  50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},                              // WHEELSPEED_CLEAN
    {.msg = {{0x242, 0, 8,  50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                          // DRIVE_STATE
    {.msg = {{0x342, 0, 8,  50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                          // PEDAL
    {.msg = {{0x32D, 2, 8,  50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},                              // ACC_HUD_ADAS
    {.msg = {{0x316, 2, 8,  50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                          // LKAS_HUD_ADAS
    {.msg = {{0x3B0, 0, 8,  20U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                          // PCM_BUTTONS (LKS latch)
  };

  // Camera 0x32E is checked only in long mode, so a dropout there does not
  // disengage lateral-only builds.
  static const RxCheck byd_long_rx_checks[] = {
    {.msg = {{0x11F, 0, 5, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x1FC, 0, 8,  50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x1F0, 0, 8,  50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x242, 0, 8,  50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x342, 0, 8,  50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x32D, 2, 8,  50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x32E, 2, 8,  50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},                              // ACC_CMD (AEB passthrough)
    {.msg = {{0x316, 2, 8,  50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x3B0, 0, 8,  20U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  if (byd_longitudinal) {
    return BUILD_SAFETY_CFG(byd_long_rx_checks, BYD_LONG_TX_MSGS);
  }
  return BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .fwd = byd_fwd_hook,
  .get_counter = byd_get_counter,
  .get_checksum = byd_get_checksum,
  .compute_checksum = byd_compute_checksum,
};
