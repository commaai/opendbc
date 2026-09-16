#pragma once

#include "opendbc/safety/declarations.h"

// Hard override thresholds, mirrored in carstate.py: 100.0 Nm of column torque for 100 ms
#define BYD_DRIVER_TORQUE_DISENGAGE 1000
#define BYD_DRIVER_TORQUE_FRAMES 5
#define BYD_WHEELSPEED_TO_KPH 0.072  // 0.02 m/s/LSB, mirrored in values.py
#define BYD_PARAM_LKS_ON 2U

static int byd_driver_torque_frames = 0;
static bool byd_lks_on = false;
static bool byd_lks_btn_last = false;
static bool byd_acc_on = false;
static uint32_t byd_op_steer_ts = 0;
static uint32_t byd_op_lat_ts = 0;

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
  bool allowed = true;
  // Same ownership as 0x1E2: any OP 0x1E2 (heartbeat included) holds the HUD
  // so TAKE CONTROL bits we paint are not overwritten by the camera.
  if (byd_op_steer_ts != 0U) {
    allowed = safety_get_ts_elapsed(microsecond_timer_get(), byd_op_steer_ts) > BYD_OP_HUD_TIMEOUT_US;
  }
  return allowed;
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
      const bool btn = GET_BIT(msg, 6U);
      if (btn && (!byd_lks_btn_last)) {
        byd_lks_on = !byd_lks_on;
        pcm_cruise_check(byd_acc_on && byd_lks_on);
      }
      byd_lks_btn_last = btn;
    }
  }

  if (msg->bus == 2U) {
    // Cruise state
    if (msg->addr == 0x32DU) {
      // ACC_STATE: 0=OFF, 2=ACC_ON, 3=ACC_ACTIVE, 5=FORCE_ACCEL, 7=ERROR
      uint8_t acc_state = (msg->data[2] >> 3) & 0x7U;
      byd_acc_on = (acc_state == 3U) || (acc_state == 5U);
      pcm_cruise_check(byd_acc_on && byd_lks_on);
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
    .steer_ratio = 14.8,
    .wheelbase = 2.72,
  };

  bool tx = true;

  // Steering control: 0.1 deg/LSB, signed
  if (msg->addr == 0x1E2U) {
    int desired_angle = to_signed((msg->data[4] << 8) | msg->data[3], 16);  // STEER_ANGLE
    bool steer_req = ((msg->data[2] >> 5) & 0x1U) != 0U;                    // STEER_REQ

    // Ownership tracks the request, not the result: a rate limited angle is still
    // openpilot driving. The HUD hold only extends while it asks for torque.
    byd_op_steer_ts = microsecond_timer_get();
    if (steer_req) {
      byd_op_lat_ts = byd_op_steer_ts;
    }

    if (steer_angle_cmd_checks_vm(desired_angle, steer_req, BYD_STEERING_LIMITS, BYD_STEERING_PARAMS)) {
      tx = false;
    }
  }

  // Bus 0: only cancel (ACC_ON_BTN) while stock cruise is engaged, or button release.
  // Bus 2: only LKAS_ON_BTN, the camera LKS spoof. It never reaches bus 0 or our latch.
  if (msg->addr == 0x3B0U) {
    bool set_res = (msg->data[0] & 0x18U) != 0U;                                          // SET, RES
    bool lkas_on = (msg->data[0] & 0x40U) != 0U;                                          // LKAS_ON
    bool distance = ((msg->data[1] & 0x80U) != 0U) || ((msg->data[2] & 0x1U) != 0U);     // DEC, INC_DISTANCE
    bool cancel = (msg->data[2] & 0x8U) != 0U;                                            // ACC_ON_BTN
    if (msg->bus == 2U) {
      tx = !set_res && !distance && !cancel;
    } else {
      tx = !set_res && !lkas_on && !distance && (!cancel || cruise_engaged_prev);
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
    } else {
    }
  }
  return block_msg;
}

static safety_config byd_init(uint16_t param) {
  byd_driver_torque_frames = 0;
  byd_lks_on = GET_FLAG(param, BYD_PARAM_LKS_ON);
  byd_lks_btn_last = false;
  byd_acc_on = false;
  byd_op_steer_ts = 0;
  byd_op_lat_ts = 0;

  static const CanMsg BYD_TX_MSGS[] = {
    {0x1E2, 0, 8, .check_relay = true, .disable_static_blocking = true},   // STEERING_MODULE_ADAS
    {0x316, 0, 8, .check_relay = true, .disable_static_blocking = true},   // LKAS_HUD_ADAS
    {0x3B0, 0, 8, .check_relay = false},  // PCM_BUTTONS (cruise cancel button spoof)
    {0x3B0, 2, 8, .check_relay = false},  // PCM_BUTTONS (camera LKS neutralize)
  };

  static const RxCheck byd_rx_checks[] = {
    {.msg = {{0x11F, 0, 5, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // STEER_MODULE_2 (steering angle)
    {.msg = {{0x1FC, 0, 8,  50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                          // STEERING_TORQUE (column driver torque)
    {.msg = {{0x1F0, 0, 8,  50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},                              // WHEELSPEED_CLEAN (vehicle speed)
    {.msg = {{0x242, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // DRIVE_STATE (gas and brake pressed)
    {.msg = {{0x342, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // PEDAL (accelerator pedal)
    {.msg = {{0x32D, 2, 8,  50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},                              // ACC_HUD_ADAS (cruise state)
    {.msg = {{0x316, 2, 8,  50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                          // LKAS_HUD_ADAS (LKS switch)
    {.msg = {{0x3B0, 0, 8,  20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // PCM_BUTTONS (LKS latch)
  };
  };
  };

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
