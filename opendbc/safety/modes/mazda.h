#pragma once

#include "opendbc/safety/declarations.h"

#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_INFO      0x21bU
#define MAZDA_CRZ_CTRL      0x21cU
#define MAZDA_CRZ_BTNS      0x09dU
#define MAZDA_RADAR_STATIC  0x499U
#define MAZDA_RADAR_TRACK_1 0x361U
#define MAZDA_RADAR_TRACK_6 0x366U
#define MAZDA_RADAR_UDS     0x764U
#define MAZDA_STEER_TORQUE  0x240U
#define MAZDA_ENGINE_DATA   0x202U
#define MAZDA_PEDALS        0x165U

#define MAZDA_MAIN 0
#define MAZDA_CAM  2

#define MAZDA_PARAM_LONGITUDINAL 1U
#define MAZDA_ENGAGE_BTN_WINDOW 10U

static bool mazda_longitudinal = false;
static uint32_t mazda_engage_btn_frames = 0U;

static bool mazda_radar_static_msg_valid(const CANPacket_t *msg) {
  return (msg->data[0] == 0x00U) && (msg->data[1] == 0x08U) &&
         (msg->data[2] == 0xc0U) && (msg->data[3] == 0x00U) &&
         (msg->data[4] == 0x00U) && (msg->data[5] == 0x00U) &&
         (msg->data[6] == 0x00U) && (msg->data[7] == 0x00U);
}

static bool mazda_radar_track_msg_valid(const CANPacket_t *msg) {
  static const uint8_t tracks[][8] = {
    {0xffU, 0xf7U, 0xfeU, 0xfeU, 0x1fU, 0xc0U, 0x00U, 0x80U},
    {0xffU, 0xf7U, 0xfeU, 0xfeU, 0x1fU, 0xc7U, 0x8cU, 0x80U},
    {0xffU, 0xf7U, 0xfeU, 0xfeU, 0x1fU, 0xc0U, 0x00U, 0x00U},
    {0xffU, 0xf7U, 0xfeU, 0xfeU, 0x1fU, 0xc0U, 0x00U, 0x00U},
    {0xffU, 0xf7U, 0xfeU, 0x7fU, 0xfbU, 0xffU, 0x3fU, 0xc0U},
    {0xffU, 0xf7U, 0xfeU, 0x7fU, 0xfbU, 0xffU, 0x3fU, 0xc0U},
  };
  uint32_t track = msg->addr - MAZDA_RADAR_TRACK_1;
  bool valid = track < 6U;
  for (uint8_t i = 0U; valid && (i < 7U); i++) {
    valid = msg->data[i] == tracks[track][i];
  }
  return valid && ((msg->data[7] & 0xf0U) == tracks[track][7]);
}

static void mazda_rx_hook(const CANPacket_t *msg) {
  if ((int)msg->bus == MAZDA_MAIN) {
    if (msg->addr == MAZDA_ENGINE_DATA) {
      int speed = (msg->data[2] << 8) | msg->data[3];
      vehicle_moving = speed > 10;  // 0.1 kph
    }

    if (msg->addr == MAZDA_STEER_TORQUE) {
      int torque_driver_new = msg->data[0] - 127U;
      update_sample(&torque_driver, torque_driver_new);
    }

    if ((msg->addr == MAZDA_CRZ_CTRL) && !mazda_longitudinal) {
      bool cruise_engaged = msg->data[0] & 0x8U;
      pcm_cruise_check(cruise_engaged);
    }

    if ((msg->addr == MAZDA_CRZ_BTNS) && mazda_longitudinal) {
      if (GET_BIT(msg, 0U)) {
        controls_allowed = false;
      }
      if (GET_BIT(msg, 2U) || GET_BIT(msg, 4U) || GET_BIT(msg, 5U)) {
        mazda_engage_btn_frames = MAZDA_ENGAGE_BTN_WINDOW;
      } else if (mazda_engage_btn_frames > 0U) {
        mazda_engage_btn_frames -= 1U;
      } else {
      }
    }

    if (msg->addr == MAZDA_ENGINE_DATA) {
      gas_pressed = (msg->data[4] || (msg->data[5] & 0xf0U));
    }

    if (msg->addr == MAZDA_PEDALS) {
      bool brake = (msg->data[0] & 0x10U);
      if (mazda_longitudinal) {
        bool cruise_engaged = GET_BIT(msg, 3U);
        bool acc_armed = GET_BIT(msg, 2U) || cruise_engaged;
        bool brake_free = !brake && !brake_pressed_prev;
        acc_main_on = acc_armed;

        if (acc_armed || cruise_engaged_prev || brake_free) {
          if (cruise_engaged && !cruise_engaged_prev && (mazda_engage_btn_frames > 0U)) {
            controls_allowed = true;
          }
          if (!cruise_engaged) {
            controls_allowed = false;
          }
          cruise_engaged_prev = cruise_engaged;
        }
      }
      brake_pressed = brake;
    }
  }
}

static bool mazda_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits MAZDA_STEERING_LIMITS = {
    .max_torque = 800,
    .max_rate_up = 10,
    .max_rate_down = 25,
    .max_rt_delta = 300,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  const LongitudinalLimits MAZDA_LONG_LIMITS = {
    .max_accel = 2000,
    .min_accel = -3500,
    .inactive_accel = 0,
  };

  bool tx = true;
  bool main_bus = msg->bus == (unsigned char)MAZDA_MAIN;
  bool long_bus = main_bus || (msg->bus == (unsigned char)MAZDA_CAM);

  if (main_bus && (msg->addr == MAZDA_LKAS)) {
    int desired_torque = (((msg->data[0] & 0x0fU) << 8) | msg->data[1]) - 2048U;
    if (steer_torque_cmd_checks(desired_torque, -1, MAZDA_STEERING_LIMITS)) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_bus && (msg->addr == MAZDA_CRZ_INFO)) {
    bool stock_standby = (msg->data[0] == 0x01U) && (msg->data[1] == 0xffU) &&
                         (msg->data[2] == 0xe3U) && (msg->data[3] == 0xffU) &&
                         ((msg->data[4] & 0xfbU) == 0xc0U) && ((msg->data[5] & 0x7fU) == 0x00U) &&
                         ((msg->data[6] & 0xf0U) == 0x00U) &&
                         (msg->data[7] == ((0xffU - ((msg->data[0] + msg->data[1] + msg->data[2] +
                                                     msg->data[3] + msg->data[4] + msg->data[5] +
                                                     msg->data[6]) & 0xffU)) & 0xffU));

    uint32_t accel_raw = (((uint32_t)msg->data[2] & 0x3U) << 11) |
                         ((uint32_t)msg->data[3] << 3) | ((uint32_t)msg->data[4] >> 5);
    int desired_accel = (int)accel_raw - 4096;
    if (!stock_standby && longitudinal_accel_checks(desired_accel, MAZDA_LONG_LIMITS)) {
      tx = false;
    }

    if (!controls_allowed && GET_BIT(msg, 33U)) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_bus && (msg->addr == MAZDA_CRZ_CTRL)) {
    if (!controls_allowed && GET_BIT(msg, 3U)) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_bus && (msg->addr == MAZDA_RADAR_STATIC) && !mazda_radar_static_msg_valid(msg)) {
    tx = false;
  }

  if (mazda_longitudinal && long_bus && (msg->addr >= MAZDA_RADAR_TRACK_1) &&
      (msg->addr <= MAZDA_RADAR_TRACK_6) && !mazda_radar_track_msg_valid(msg)) {
    tx = false;
  }

  if (mazda_longitudinal && main_bus && (msg->addr == MAZDA_RADAR_UDS)) {
    bool tester_present = (msg->data[0] == 0x02U) && (msg->data[1] == 0x3eU) && (msg->data[2] == 0x80U);
    bool session_control = (msg->data[0] == 0x02U) && (msg->data[1] == 0x10U) &&
                           ((msg->data[2] == 0x01U) || (msg->data[2] == 0x02U));
    if (!tester_present && !session_control) {
      tx = false;
    }
  }

  if (main_bus && (msg->addr == MAZDA_CRZ_BTNS)) {
    bool cancel_cmd = msg->data[0] == 0x1U;
    if (!controls_allowed && !cancel_cmd) {
      tx = false;
    }
  }

  return tx;
}

static safety_config mazda_init(uint16_t param) {
  static const CanMsg MAZDA_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
  };
  static const CanMsg MAZDA_LONG_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
    {MAZDA_CRZ_INFO, 0, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, 0, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1 + 1U, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1 + 2U, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1 + 3U, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1 + 4U, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, 0, 8, .check_relay = false},
    {MAZDA_RADAR_UDS, 0, 8, .check_relay = false},
    {MAZDA_CRZ_INFO, 2, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, 2, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, 2, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, 2, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1 + 1U, 2, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1 + 2U, 2, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1 + 3U, 2, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1 + 4U, 2, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, 2, 8, .check_relay = false},
  };

  static RxCheck mazda_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };
  static RxCheck mazda_long_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  mazda_longitudinal = GET_FLAG(param, MAZDA_PARAM_LONGITUDINAL);
  mazda_engage_btn_frames = 0U;
  acc_main_on = false;

  return mazda_longitudinal ? BUILD_SAFETY_CFG(mazda_long_rx_checks, MAZDA_LONG_TX_MSGS) :
                              BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
};
