#pragma once

#include "opendbc/safety/declarations.h"

// CAN msgs we care about
#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_INFO      0x21bU
#define MAZDA_CRZ_CTRL      0x21cU
#define MAZDA_CRZ_BTNS      0x09dU
#define MAZDA_RADAR_UDS     0x764U
#define MAZDA_STEER_TORQUE  0x240U
#define MAZDA_ENGINE_DATA   0x202U
#define MAZDA_PEDALS        0x165U

// CAN bus numbers
#define MAZDA_MAIN 0
#define MAZDA_CAM  2

enum {
  MAZDA_PARAM_LONGITUDINAL = 1,
};

static bool mazda_longitudinal = false;

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void mazda_rx_hook(const CANPacket_t *msg) {
  if ((int)msg->bus == MAZDA_MAIN) {
    if (msg->addr == MAZDA_ENGINE_DATA) {
      // sample speed: scale by 0.01 to get kph
      int speed = (msg->data[2] << 8) | msg->data[3];
      vehicle_moving = speed > 10; // moving when speed > 0.1 kph
    }

    if (msg->addr == MAZDA_STEER_TORQUE) {
      int torque_driver_new = msg->data[0] - 127U;
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    if (msg->addr == MAZDA_CRZ_CTRL) {
      if (!mazda_longitudinal) {
        // enter controls on rising edge of ACC, exit controls on ACC off
        bool cruise_engaged = msg->data[0] & 0x8U;
        pcm_cruise_check(cruise_engaged);
        acc_main_on = GET_BIT(msg, 17U);
      }
    }

    if (msg->addr == MAZDA_CRZ_BTNS && mazda_longitudinal) {
      bool cancel = GET_BIT(msg, 0U);
      if (cancel) {
        controls_allowed = false;
      }
    }

    if (msg->addr == MAZDA_ENGINE_DATA) {
      gas_pressed = (msg->data[4] || (msg->data[5] & 0xF0U));
    }

    if (msg->addr == MAZDA_PEDALS) {
      bool brake = (msg->data[0] & 0x10U);
      if (mazda_longitudinal) {
        // Radar suppression removes the stock CRZ_CTRL frame, so derive Mazda's
        // "main on" state from PEDALS instead. ACC_OFF means MRCC is armed but
        // not actively controlling, and ACC_ACTIVE means stock ACC is engaged.
        bool cruise_engaged = GET_BIT(msg, 3U);
        bool acc_armed = GET_BIT(msg, 2U) || cruise_engaged;
        acc_main_on = acc_armed;

        // Only feed PEDALS into pcm_cruise_check when the ACC state is actually
        // meaningful. Brake-only samples can arrive with both ACC bits low while
        // the driver is holding the pedal; treating those as a stock ACC-off edge
        // drops controls before the normal brake-edge logic runs.
        if (acc_armed || cruise_engaged_prev || (!brake && !brake_pressed_prev)) {
          pcm_cruise_check(cruise_engaged);
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

  bool tx = true;
  // Check if msg is sent on the main BUS
  if (msg->bus == (unsigned char)MAZDA_MAIN) {
    // steer cmd checks
    if (msg->addr == MAZDA_LKAS) {
      int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

      if (steer_torque_cmd_checks(desired_torque, -1, MAZDA_STEERING_LIMITS)) {
        tx = false;
      }
    }

    if (mazda_longitudinal && (msg->addr == MAZDA_CRZ_INFO)) {
      // Keep Panda's Mazda-long safety window aligned with the software clip in
      // opendbc/car/mazda/longitudinal.py. If this is tighter than the sender,
      // Panda will silently drop 0x21b frames once ACCEL_CMD crosses the
      // safety threshold, which looks like an unexplained set-speed unlatch.
      const LongitudinalLimits MAZDA_LONG_LIMITS = {
        .max_accel = 2000,
        .min_accel = -2000,
        .inactive_accel = 0,
      };

      // Mazda's CRZ_INFO.ACCEL_CMD packing in this stack follows the DBC
      // bit ordering used by set_value() in opendbc, which places the 13-bit
      // raw command across data[2] low bits, all of data[3], and data[4] high bits.
      int desired_accel = ((((int)msg->data[2] & 0x3U) << 11) | (((int)msg->data[3]) << 3) | (((int)msg->data[4]) >> 5)) - 4096;
      if (longitudinal_accel_checks(desired_accel, MAZDA_LONG_LIMITS)) {
        tx = false;
      }
    }

    if (mazda_longitudinal && (msg->addr == MAZDA_CRZ_CTRL)) {
      bool cruise_active = GET_BIT(msg, 3U);
      if (!controls_allowed && cruise_active) {
        tx = false;
      }
    }

    if (mazda_longitudinal && (msg->addr == MAZDA_RADAR_UDS)) {
      bool tester_present = (msg->data[0] == 0x02U) && (msg->data[1] == 0x3EU) && (msg->data[2] == 0x80U);
      bool session_control = (msg->data[0] == 0x02U) && (msg->data[1] == 0x10U) &&
                             ((msg->data[2] == 0x01U) || (msg->data[2] == 0x02U));
      if (!tester_present && !session_control) {
        tx = false;
      }
    }

    // cruise buttons check
    if (msg->addr == MAZDA_CRZ_BTNS) {
      // allow resume spamming while controls allowed, but
      // only allow cancel while controls not allowed
      bool cancel_cmd = (msg->data[0] == 0x1U);
      if (!controls_allowed && !cancel_cmd) {
        tx = false;
      }
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
    {MAZDA_RADAR_UDS, 0, 8, .check_relay = false},
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
  acc_main_on = false;

  return mazda_longitudinal ? BUILD_SAFETY_CFG(mazda_long_rx_checks, MAZDA_LONG_TX_MSGS) :
                              BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
};
