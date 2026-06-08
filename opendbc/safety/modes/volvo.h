#pragma once

#include "opendbc/safety/declarations.h"

// Safety-relevant CAN messages for EUCD platform.
#define VOLVO_EUCD_AccPedal      0x020  // RX, gas pedal
#define VOLVO_EUCD_FSM0          0x051  // RX from FSM, cruise state
#define VOLVO_EUCD_VehicleSpeed1 0x148  // RX, vehicle speed
#define VOLVO_EUCD_Brake_Info    0x20a  // RX, driver brake pressed
#define VOLVO_EUCD_CCButtons     0x127  // TX by OP, CC buttons
#define VOLVO_EUCD_PSCM1         0x246  // TX by OP to camera, PSCM state
#define VOLVO_EUCD_FSM1          0x260  // TX by OP, ACC radar/distance message (oplong)
#define VOLVO_EUCD_FSM2          0x262  // TX by OP, LKA command
#define VOLVO_EUCD_FSM3          0x270  // TX by OP, ACC accel request + status

// CAN bus numbers.
#define VOLVO_MAIN_BUS 0U
#define VOLVO_AUX_BUS  1U
#define VOLVO_CAM_BUS  2U

static void volvo_rx_hook(const CANPacket_t *msg) {
  int addr = msg->addr;
  if (msg->bus == VOLVO_MAIN_BUS) {
    if (addr == VOLVO_EUCD_VehicleSpeed1) {
      // Signal: VehicleSpeed
      unsigned int speed_raw = (GET_BYTES(msg, 6, 1) << 8) | GET_BYTES(msg, 7, 1);
      vehicle_moving = speed_raw >= 36U;
      UPDATE_VEHICLE_SPEED(speed_raw * 0.01 / 3.6);
    }

    if (addr == VOLVO_EUCD_AccPedal) {
      // Signal: AccPedal
      unsigned int gas_raw = ((GET_BYTES(msg, 2, 1) & 0x03U) << 8) | GET_BYTES(msg, 3, 1);
      gas_pressed = gas_raw >= 100U;
    }

    if (addr == VOLVO_EUCD_Brake_Info) {
      // Signal: BrakePedal
      brake_pressed = ((GET_BYTES(msg, 2, 1) & 0x0CU) >> 2U) == 2U;
    }

    if (addr == VOLVO_EUCD_PSCM1) {
      // Signal: SteeringAngleServo (16-bit big-endian, bytes 2-3, factor 0.0447, offset -1465 deg).
      // Stored as angle_deg * 100 to match VOLVO_STEERING_LIMITS.angle_deg_to_can in the tx hook.
      unsigned int angle_raw = (GET_BYTES(msg, 2, 1) << 8) | GET_BYTES(msg, 3, 1);
      int angle_meas_new = ROUND(((angle_raw * 0.0447f) - 1465.0f) * 100.0f);
      update_sample(&angle_meas, angle_meas_new);
    }
  } else if (msg->bus == VOLVO_CAM_BUS) {
    if (addr == VOLVO_EUCD_FSM0) {
      // Signal: ACC_Enabled (bit 2 of byte 2, from ACCStatus == 6 || 7)
      bool cruise_engaged = (GET_BYTES(msg, 2, 1) & 0x04U) != 0U;
      pcm_cruise_check(cruise_engaged);
    }
  } else {
    // no other bus is monitored
  }
}

static bool volvo_tx_hook(const CANPacket_t *msg) {
  // FSM3 byte1 = ACC_AccelerationRequest (factor 0.04, offset -5.04); raw_accel = byte1 - 126.
  // +50 raw = +2.0 m/s^2 max accel; -100 raw = -4.0 m/s^2 max decel.
  const LongitudinalLimits VOLVO_LONG_LIMITS = {
    .max_accel = 50,
    .min_accel = -100,
    .inactive_accel = 0,
  };

  // Angle steering limits — mirror CarControllerParams.ANGLE_LIMITS in values.py.
  // angle_deg_to_can = 100 so desired_angle / angle_meas are carried as angle_deg * 100.
  const AngleSteeringLimits VOLVO_STEERING_LIMITS = {
    .max_angle = 9000,  // 90 deg
    .angle_deg_to_can = 100,
    .angle_rate_up_lookup = {
      {0., 5., 15.},
      {5., .8, .15}
    },
    .angle_rate_down_lookup = {
      {0., 5., 15.},
      {5., 3.5, .4}
    },
    // OP sends LKAAngleReq=0 with LKASteerDirection=NONE when not steering, so the
    // commanded angle must be exactly zero while disabled (not near the measurement).
    .inactive_angle_is_zero = true,
  };

  bool tx = true;
  bool violation = false;
  int addr = msg->addr;

  // Safety check for CC button signals.
  if (addr == VOLVO_EUCD_CCButtons) {
    // Violation if resume button is pressed while controls not allowed, or
    // if cancel button is pressed when cruise isn't engaged.
    violation |= !cruise_engaged_prev && (GET_BIT(msg, 59U) || !(GET_BIT(msg, 43U)));  // Signals: ACCOnOffBtn, ACCOnOffBtnInv (cancel)
    violation |= !controls_allowed && (GET_BIT(msg, 61U) || !(GET_BIT(msg, 45U)));  // Signals: ACCResumeBtn, ACCResumeBtnInv (resume)
  }

  // Safety check for Lane Keep Assist action.
  if (addr == VOLVO_EUCD_FSM2) {
    // Signal: LKAAngleReq (14-bit big-endian over bytes 3-4, factor 0.04, offset -327.68 deg).
    // raw*0.04 - 327.68 deg, carried here as angle_deg * 100 => raw*4 - 32768.
    unsigned int angle_raw = ((GET_BYTES(msg, 3, 1) & 0x3FU) << 8) | GET_BYTES(msg, 4, 1);
    int desired_angle = ((int)angle_raw * 4) - 32768;

    // Gate on angle command, not LKASteerDirection: EUCD needs ~8-frame NONE pause on
    // direction change while OP still emits a tracking angle — gating direction would stall lat.
    bool steer_control_enabled = controls_allowed || controls_allowed_lateral;

    if (steer_angle_cmd_checks(desired_angle, steer_control_enabled, VOLVO_STEERING_LIMITS)) {
      violation = true;
    }
  }

  // Longitudinal control: gate on controls_allowed + range check.
  // check_relay=false lets stock FSM3 flow cam->main; OP overlays only when long-active.
  if (addr == VOLVO_EUCD_FSM3) {
    int raw_accel = (int)GET_BYTES(msg, 1, 1) - 126;
    if (!controls_allowed || longitudinal_accel_checks(raw_accel, VOLVO_LONG_LIMITS)) {
      violation = true;
    }
  }

  if (violation) {
    tx = false;
  }

  return tx;
}

static bool volvo_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;
  // Block stock FSM1/FSM3 cam->main when controls_allowed so OP relays at 50Hz
  // and ACC_Check=1 isn't overwritten by stock's ACC_Check=0 during SNG.
  if (((unsigned int)bus_num == VOLVO_CAM_BUS) && controls_allowed && !gas_pressed) {
    if ((addr == VOLVO_EUCD_FSM1) || (addr == VOLVO_EUCD_FSM3)) {
      block_msg = true;  // OP relays via create_radar/create_longitudinal
    }
  }
  return block_msg;
}

static safety_config volvo_init(uint16_t param) {
  (void)param;

  static const CanMsg VOLVO_EUCD_TX_MSGS[] = {
    {VOLVO_EUCD_CCButtons, VOLVO_MAIN_BUS, 8, .check_relay = false},
    {VOLVO_EUCD_PSCM1,     VOLVO_CAM_BUS,  8, .check_relay = true},   // OP replaces stock steering servo state
    {VOLVO_EUCD_FSM2,      VOLVO_MAIN_BUS, 8, .check_relay = true},   // OP replaces stock LKA command
    // FSM1/FSM3: check_relay=false — ECM validates a rolling counter; passthrough delay
    // causes ECM fault (~30s, drive 27). Stock flows untouched; OP's later TX wins via last-message-wins.
    {VOLVO_EUCD_FSM1,      VOLVO_MAIN_BUS, 8, .check_relay = false},
    {VOLVO_EUCD_FSM3,      VOLVO_MAIN_BUS, 8, .check_relay = false},
  };

  // TODO: add counters
  static RxCheck volvo_eucd_rx_checks[] = {
    {.msg = {{VOLVO_EUCD_AccPedal,      VOLVO_MAIN_BUS, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_FSM0,          VOLVO_CAM_BUS,  8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_VehicleSpeed1, VOLVO_MAIN_BUS, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_Brake_Info,    VOLVO_MAIN_BUS, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_PSCM1,         VOLVO_MAIN_BUS, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},  // steering angle meas
  };

  return BUILD_SAFETY_CFG(volvo_eucd_rx_checks, VOLVO_EUCD_TX_MSGS);
}

const safety_hooks volvo_hooks = {
  .init = volvo_init,
  .rx = volvo_rx_hook,
  .tx = volvo_tx_hook,
  .fwd = volvo_fwd_hook,
};
