#pragma once

#include "opendbc/safety/declarations.h"


#define LEAPMOTOR_BRAKE_PEDAL         0x0EAU
#define LEAPMOTOR_ACCEL_PEDAL         0x0EBU
#define LEAPMOTOR_STATE_MIRROR_1      0x0EDU
#define LEAPMOTOR_WHEEL_SPEEDS_FRONT  0x122U
#define LEAPMOTOR_VEHICLE_SPEED       0x125U
#define LEAPMOTOR_STEER_1             0x280U
#define LEAPMOTOR_EPS_LKA_CMD         0x365U
#define LEAPMOTOR_STEERING_BUTTONS    0x389U

#define LEAPMOTOR_MAIN 0U
#define LEAPMOTOR_ICU 2U

#define LEAPMOTOR_LKA_STATE_ACTIVE 19U
#define LEAPMOTOR_STEER_ANGLE_REQ_ZERO 9000
#define LEAPMOTOR_DRIVER_TORQUE_ZERO 10

#define LEAPMOTOR_TAP_GAP_TIME 30000U
#define LEAPMOTOR_TAP_WINDOW_TIME 1500000U

static uint8_t leapmotor_get_counter(const CANPacket_t *msg) {
  uint8_t counter;
  if ((msg->addr == LEAPMOTOR_ACCEL_PEDAL) || (msg->addr == LEAPMOTOR_STATE_MIRROR_1)) {
    counter = msg->data[7] & 0xFU;
  } else {
    counter = msg->data[6] >> 4;
  }
  return counter;
}

static uint32_t leapmotor_get_checksum(const CANPacket_t *msg) {
  return msg->data[7];
}

static uint32_t leapmotor_compute_checksum(const CANPacket_t *msg) {
  // 校验和放在 data[7]（最后一个字节），它让 8 个字节加起来（只取低 8 位）正好等于 0xFF
  uint8_t sum = 0;
  for (int i = 0; i < 7; i++) {
    sum += msg->data[i];
  }
  return 0xFFU - sum;
}

static void leapmotor_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == LEAPMOTOR_MAIN) {
    // 速度原始值每 1 代表 0.05625 km/h
    if (msg->addr == LEAPMOTOR_VEHICLE_SPEED) {
      int speed = msg->data[1] | ((msg->data[2] & 0xFU) << 8);  // VEHICLE_SPEED 车速
      safety.vehicle_moving = speed > 0;
      UPDATE_VEHICLE_SPEED(speed * 0.05625 * KPH_TO_MS);
    }

    if (msg->addr == LEAPMOTOR_WHEEL_SPEEDS_FRONT) {
      int speed_fl = msg->data[0] | ((msg->data[1] & 0xFU) << 8);  // WHEEL_SPEED_FL 左前轮速
      int speed_fr = msg->data[2] | ((msg->data[3] & 0xFU) << 8);  // WHEEL_SPEED_FR 右前轮速
      speed_mismatch_check((speed_fl + speed_fr) * 0.5 * 0.05625 * KPH_TO_MS);
    }

    if (msg->addr == LEAPMOTOR_STEER_1) {
      int torque_raw = msg->data[4] | ((msg->data[5] & 0x7U) << 8);  // STEER_TORQUE_DRIVER 驾驶员扭矩
      update_sample(&safety.torque_driver, 1000 - torque_raw - LEAPMOTOR_DRIVER_TORQUE_ZERO);

      // 上一次按压已经过了窗口，忘掉它。STEER_1 每 10 ms 一帧，所以微秒定时器（约 71.6 分钟一圈）绕回之前一定会忘掉，
      // 很久以前的按压不会因为回绕被当成刚才的
      if (safety.leapmotor_tap_seen && (safety_get_ts_elapsed(safety_ts, safety.leapmotor_tap_start_ts) > LEAPMOTOR_TAP_WINDOW_TIME)) {
        safety.leapmotor_tap_seen = false;
      }
    }

    if (msg->addr == LEAPMOTOR_ACCEL_PEDAL) {
      safety.gas_pressed = msg->data[4] != 0U;  // ACCEL_PEDAL_POS 油门踏板位置
    }

    if (msg->addr == LEAPMOTOR_BRAKE_PEDAL) {
      safety.brake_pressed = ((msg->data[4] >> 6) & 0x1U) != 0U;  // BRAKE_PRESSED 刹车踏板踩下
    }
  }

  if ((msg->bus == LEAPMOTOR_ICU) && (msg->addr == LEAPMOTOR_STEERING_BUTTONS)) {
    uint32_t ts = safety_ts;
    bool roller_pressed = ((msg->data[0] >> 7) & 0x1U) != 0U;  // RIGHT_ROLLER_PRESS 右滚轮按下

    if (roller_pressed) {

      if (!safety.leapmotor_tap_seen || (safety_get_ts_elapsed(ts, safety.leapmotor_tap_ts) > LEAPMOTOR_TAP_GAP_TIME)) {
        if (safety.leapmotor_tap_seen && (safety_get_ts_elapsed(ts, safety.leapmotor_tap_start_ts) <= LEAPMOTOR_TAP_WINDOW_TIME)) {
          safety.controls_allowed = true;
          safety.heartbeat_engaged_mismatches = 0U;
        }
        safety.leapmotor_tap_start_ts = ts;
      }
      safety.leapmotor_tap_ts = ts;
      safety.leapmotor_tap_seen = true;
    }
  }
}

#define LEAPMOTOR_MAX_TORQUE 650
#define LEAPMOTOR_MAX_TORQUE_POINTS 10
static int leapmotor_max_torque(void) {
  // m/s: 35 37.5 40 45 55 60 65 70 75 77.5 km/h
  static const float leapmotor_torque_speeds[LEAPMOTOR_MAX_TORQUE_POINTS] = {
    9.722f, 10.416f, 11.111f, 12.5f, 15.277f, 16.666f, 18.055f, 19.444f, 20.833f, 21.527f};
  static const float leapmotor_torque_caps[LEAPMOTOR_MAX_TORQUE_POINTS] = {
    660.f, 603.f, 565.f, 540.f, 540.f, 513.f, 490.f, 470.f, 453.f, 443.f};
  const float speed = (float)safety.vehicle_speed.min / VEHICLE_SPEED_FACTOR;

  float torque = leapmotor_torque_caps[LEAPMOTOR_MAX_TORQUE_POINTS - 1];
  if (speed <= leapmotor_torque_speeds[0]) {
    torque = leapmotor_torque_caps[0];
  } else {
    for (int i = 1; i < LEAPMOTOR_MAX_TORQUE_POINTS; i++) {
      if (speed < leapmotor_torque_speeds[i]) {
        const float dv = speed - leapmotor_torque_speeds[i - 1];
        const float slope = (leapmotor_torque_caps[i] - leapmotor_torque_caps[i - 1]) / (leapmotor_torque_speeds[i] - leapmotor_torque_speeds[i - 1]);
        torque = leapmotor_torque_caps[i - 1] + (dv * slope);
        break;
      }
    }
  }
  return SAFETY_MIN((int)torque, LEAPMOTOR_MAX_TORQUE);
}

static bool leapmotor_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits LEAPMOTOR_STEERING_LIMITS = {
    .max_torque = leapmotor_max_torque(),
    .max_rate_up = 15,
    .max_rate_down = 30,
    .max_rt_delta = 225,
    .driver_torque_allowance = 80,
    .driver_torque_multiplier = 2,
    .type = TorqueDriverLimited,
  };

  bool tx = true;

  if (msg->addr == LEAPMOTOR_EPS_LKA_CMD) {
    uint8_t lka_state = msg->data[0];
    bool steer_req = lka_state == LEAPMOTOR_LKA_STATE_ACTIVE;
    int angle_req = msg->data[1] | (msg->data[2] << 8);
    int torque_raw = msg->data[4] | ((msg->data[5] & 0xFU) << 8);
    int desired_torque = 1000 - torque_raw;

    if (steer_torque_cmd_checks(desired_torque, steer_req, LEAPMOTOR_STEERING_LIMITS)) {
      tx = false;
    }
    if (steer_req && !safety.controls_allowed) {
      tx = false;
    }

    if (angle_req != LEAPMOTOR_STEER_ANGLE_REQ_ZERO) {
      tx = false;
    }

    // 只放行已知的 LKA 状态：OFF 0、STANDBY 1、READY 2、ACTIVE 19（openpilot 镜像 ICU 时也只发这几个）
    bool known_state = (lka_state == 0U) || (lka_state == 1U) || (lka_state == 2U) || steer_req;
    if (!known_state) {
      tx = false;
    }
  }

  return tx;
}

static safety_config leapmotor_init(uint16_t param) {
  SAFETY_UNUSED(param);

  static const CanMsg LEAPMOTOR_TX_MSGS[] = {
    {LEAPMOTOR_EPS_LKA_CMD, LEAPMOTOR_MAIN, 8, .check_relay = true},
  };

  static const RxCheck leapmotor_rx_checks[] = {
    {.msg = {{LEAPMOTOR_BRAKE_PEDAL, LEAPMOTOR_MAIN, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{LEAPMOTOR_ACCEL_PEDAL, LEAPMOTOR_MAIN, 8, 50U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{LEAPMOTOR_STATE_MIRROR_1, LEAPMOTOR_MAIN, 8, 50U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{LEAPMOTOR_WHEEL_SPEEDS_FRONT, LEAPMOTOR_MAIN, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{LEAPMOTOR_VEHICLE_SPEED, LEAPMOTOR_MAIN, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{LEAPMOTOR_STEER_1, LEAPMOTOR_MAIN, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{LEAPMOTOR_STEERING_BUTTONS, LEAPMOTOR_ICU, 8, 50U, .ignore_checksum = true, .ignore_counter = true,
              .ignore_quality_flag = true, .non_periodic = true}, { 0 }, { 0 }}},
  };

  return BUILD_SAFETY_CFG(leapmotor_rx_checks, LEAPMOTOR_TX_MSGS);
}

const safety_hooks leapmotor_hooks = {
  .init = leapmotor_init,
  .rx = leapmotor_rx_hook,
  .tx = leapmotor_tx_hook,
  .get_counter = leapmotor_get_counter,
  .get_checksum = leapmotor_get_checksum,
  .compute_checksum = leapmotor_compute_checksum,
};
