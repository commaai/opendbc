#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"

#define MSG_MEB_ESP_01           0xFC    // RX, for wheel speeds
#define MSG_MEB_ESP_03           0x14C   // RX, for accel pedal
#define MSG_MEB_ESP_04           0x102   // RX, for yaw rate
#define MSG_MEB_ESP_05           0x139   // RX, for ESP hold management
#define MSG_MEB_ABS_01           0x20A   // RX, for yaw rate
#define MSG_HCA_03               0x303   // TX by OP, Heading Control Assist steering torque
#define MSG_MEB_EPS_01           0x13D   // RX, for steering angle
#define MSG_MEB_ACC_01           0x300   // RX from ECU, for ACC status
#define MSG_MEB_ACC_02           0x14D   // RX from ECU, for ACC status
#define MSG_MEB_MOTOR_01         0x10B   // RX for TSK state
#define MSG_MEB_TRAVEL_ASSIST_01 0x26B   // TX for Travel Assist status

static int volkswagen_steer_power_prev = 0;

static bool vw_meb_get_longitudinal_allowed_override(void) {
  return controls_allowed && gas_pressed_prev;
}

// Safety checks for longitudinal actuation
static bool vw_meb_longitudinal_accel_checks(int desired_accel, const LongitudinalLimits limits, const int override_accel) {
  bool accel_valid = get_longitudinal_allowed() && !safety_max_limit_check(desired_accel, limits.max_accel, limits.min_accel);
  bool accel_valid_override = vw_meb_get_longitudinal_allowed_override() && desired_accel == override_accel;
  bool accel_inactive = desired_accel == limits.inactive_accel;
  return !(accel_valid || accel_inactive || accel_valid_override);
}

static uint32_t volkswagen_meb_compute_crc(const CANPacket_t *msg) {
  int len = GET_LEN(msg);

  // This is CRC-8H2F/AUTOSAR with a twist. See the opendbc/car/volkswagen/ implementation
  // of this algorithm for a version with explanatory comments.

  uint8_t crc = 0xFFU;
  for (int i = 1; i < len; i++) {
    crc ^= (uint8_t)msg->data[i];
    crc = volkswagen_crc8_lut_8h2f[crc];
  }

  uint8_t counter = volkswagen_mqb_meb_get_counter(msg);
  if (msg->addr == MSG_LH_EPS_03) {
    crc ^= (uint8_t[]){0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5}[counter];
  } else if (msg->addr == MSG_GRA_ACC_01) {
    crc ^= (uint8_t[]){0x6A, 0x38, 0xB4, 0x27, 0x22, 0xEF, 0xE1, 0xBB, 0xF8, 0x80, 0x84, 0x49, 0xC7, 0x9E, 0x1E, 0x2B}[counter];
  } else if (msg->addr == MSG_MEB_EPS_01) {
    crc ^= (uint8_t[]){0x20, 0xCA, 0x68, 0xD5, 0x1B, 0x31, 0xE2, 0xDA, 0x08, 0x0A, 0xD4, 0xDE, 0x9C, 0xE4, 0x35, 0x5B}[counter];
  } else if (msg->addr == MSG_MEB_ESP_01) {
    crc ^= (uint8_t[]){0x77, 0x5C, 0xA0, 0x89, 0x4B, 0x7C, 0xBB, 0xD6, 0x1F, 0x6C, 0x4F, 0xF6, 0x20, 0x2B, 0x43, 0xDD}[counter];
  } else if (msg->addr == MSG_MEB_ESP_03) {
    crc ^= (uint8_t[]){0x16, 0x35, 0x59, 0x15, 0x9A, 0x2A, 0x97, 0xB8, 0x0E, 0x4E, 0x30, 0xCC, 0xB3, 0x07, 0x01, 0xAD}[counter];
  } else if (msg->addr == MSG_MEB_ESP_04) {
    crc ^= (uint8_t[]){0xD7, 0x12, 0x85, 0x7E, 0x0B, 0x34, 0xFA, 0x16, 0x7A, 0x25, 0x2D, 0x8F, 0x04, 0x8E, 0x5D, 0x35}[counter];
  } else if (msg->addr == MSG_MEB_ESP_05) {
    crc ^= (uint8_t[]){0xED, 0x03, 0x1C, 0x13, 0xC6, 0x23, 0x78, 0x7A, 0x8B, 0x40, 0x14, 0x51, 0xBF, 0x68, 0x32, 0xBA}[counter];
  } else if (msg->addr == MSG_MEB_MOTOR_01) {
    crc ^= (uint8_t[]){0x77, 0x5C, 0xA0, 0x89, 0x4B, 0x7C, 0xBB, 0xD6, 0x1F, 0x6C, 0x4F, 0xF6, 0x20, 0x2B, 0x43, 0xDD}[counter];
  } else if (msg->addr == MSG_MOTOR_14) {
    crc ^= (uint8_t[]){0x1F, 0x28, 0xC6, 0x85, 0xE6, 0xF8, 0xB0, 0x19, 0x5B, 0x64, 0x35, 0x21, 0xE4, 0xF7, 0x9C, 0x24}[counter];
  } else if (msg->addr == MSG_MEB_ABS_01) {
    crc ^= (uint8_t[]){0x9D, 0xE8, 0x36, 0xA1, 0xCA, 0x3B, 0x1D, 0x33, 0xE0, 0xD5, 0xBB, 0x5F, 0xAE, 0x3C, 0x31, 0x9F}[counter];
  } else {
    // Undefined CAN message, CRC check expected to fail
  }
  crc = volkswagen_crc8_lut_8h2f[crc];

  return (uint8_t)(crc ^ 0xFFU);
}

// lateral limits for curvature
static const AngleSteeringLimits VOLKSWAGEN_MEB_STEERING_LIMITS = {
  .max_angle = 29105, // ~ 0.195 rad/m
  .angle_deg_to_can = 149253.7313, // ~ 1 / 0.00036 rad/m to can
  .angle_rate_up_lookup = {
    {0., 5., 15.},
    {0.005, 0.0015, 0.00015} // in rad
  },
  .angle_rate_down_lookup = {
    {0., 5., 15.},
    {0.005, 0.0034, 0.00034}
  },
  .inactive_angle_is_zero = true,
};

static safety_config volkswagen_meb_init(uint16_t param) {
  // Transmit of GRA_ACC_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  static const CanMsg VOLKSWAGEN_MEB_STOCK_TX_MSGS[] = {{MSG_HCA_03, 0, 24, .check_relay = true}, {MSG_GRA_ACC_01, 0, 8, .check_relay = false},
                                                        {MSG_GRA_ACC_01, 2, 8, .check_relay = false}, {MSG_LDW_02, 0, 8, .check_relay = true},
                                                        {MSG_LH_EPS_03, 2, 8, .check_relay = true}};

  static const CanMsg VOLKSWAGEN_MEB_LONG_TX_MSGS[] = {{MSG_MEB_ACC_01, 0, 48, .check_relay = true}, {MSG_MEB_ACC_02, 0, 32, .check_relay = true},
                                                       {MSG_HCA_03, 0, 24, .check_relay = true}, {MSG_LDW_02, 0, 8, .check_relay = true},
                                                       {MSG_LH_EPS_03, 2, 8, .check_relay = true}, {MSG_MEB_TRAVEL_ASSIST_01, 0, 8, .check_relay = true}};

  static RxCheck volkswagen_meb_rx_checks[] = {
    {.msg = {{MSG_LH_EPS_03, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_14, 0, 8, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MEB_MOTOR_01, 0, 32, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_GRA_ACC_01, 0, 8, 33U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MEB_EPS_01, 0, 32, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MEB_ESP_01, 0, 48, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MEB_ESP_03, 0, 32, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MEB_ESP_04, 0, 48, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MEB_ESP_05, 0, 32, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MEB_ABS_01, 0, 64, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  volkswagen_common_init();
  volkswagen_steer_power_prev = 0;

#ifdef ALLOW_DEBUG
  volkswagen_longitudinal = GET_FLAG(param, FLAG_VOLKSWAGEN_LONG_CONTROL);
#else
  SAFETY_UNUSED(param);
#endif

  return volkswagen_longitudinal ? BUILD_SAFETY_CFG(volkswagen_meb_rx_checks, VOLKSWAGEN_MEB_LONG_TX_MSGS) : \
                                   BUILD_SAFETY_CFG(volkswagen_meb_rx_checks, VOLKSWAGEN_MEB_STOCK_TX_MSGS);
}

static void volkswagen_meb_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // Update in-motion state by sampling wheel speeds
    if (msg->addr == MSG_MEB_ESP_01) {
      uint32_t fr = msg->data[10] | (msg->data[11] << 8);
      uint32_t rr = msg->data[14] | (msg->data[15] << 8);
      uint32_t rl = msg->data[12] | (msg->data[13] << 8);
      uint32_t fl = msg->data[8] | (msg->data[9] << 8);

      vehicle_moving = (fr > 0U) || (rr > 0U) || (rl > 0U) || (fl > 0U);

      UPDATE_VEHICLE_SPEED(((fr + rr + rl + fl) / 4) * 0.0075 / 3.6);
    }

    // Update vehicle yaw rate for curvature checks
    if (msg->addr == MSG_MEB_ESP_04) {
      float volkswagen_yaw_rate = (msg->data[5] | ((msg->data[6] & 0x3FU) << 8)) * 0.01;

      bool volkswagen_yaw_rate_sign = GET_BIT(msg, 54U);
      if (volkswagen_yaw_rate_sign) {
        volkswagen_yaw_rate *= -1;
      }

      float current_curvature = volkswagen_yaw_rate / SAFETY_MAX(vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR, 0.1);
      // convert current curvature into units on CAN for comparison with desired curvature
      update_sample(&angle_meas, ROUND(current_curvature * VOLKSWAGEN_MEB_STEERING_LIMITS.angle_deg_to_can));
    }

    // Update cruise state
    if (msg->addr == MSG_MEB_MOTOR_01) {
      // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
      // Always exit controls on main switch off
      // Signal: TSK_06.TSK_Status
      int acc_status = (msg->data[11] & 0x07U);
      bool cruise_engaged = (acc_status == 3) || (acc_status == 4) || (acc_status == 5);
      acc_main_on = cruise_engaged || (acc_status == 2);

      if (!volkswagen_longitudinal) {
        pcm_cruise_check(cruise_engaged);
      }

      if (!acc_main_on) {
        controls_allowed = false;
      }
    }

    // update cruise buttons
    if (msg->addr == MSG_GRA_ACC_01) {
      // If using openpilot longitudinal, enter controls on falling edge of Set or Resume with main switch on
      // Signal: GRA_ACC_01.GRA_Tip_Setzen
      // Signal: GRA_ACC_01.GRA_Tip_Wiederaufnahme
      if (volkswagen_longitudinal) {
        bool set_button = GET_BIT(msg, 16U);
        bool resume_button = GET_BIT(msg, 19U);
        if ((volkswagen_set_button_prev && !set_button) || (volkswagen_resume_button_prev && !resume_button)) {
          controls_allowed = acc_main_on;
        }
        volkswagen_set_button_prev = set_button;
        volkswagen_resume_button_prev = resume_button;
      }
      // Always exit controls on rising edge of Cancel
      // Signal: GRA_ACC_01.GRA_Abbrechen
      if (GET_BIT(msg, 13U)) {
        controls_allowed = false;
      }
    }

    // update brake pedal
    if (msg->addr == MSG_MOTOR_14) {
      brake_pressed = GET_BIT(msg, 28U);
    }

    // update accel pedal
    if (msg->addr == MSG_MEB_ESP_03) {
      int accel_pedal_value = msg->data[21] - 37;
      gas_pressed = accel_pedal_value != 0;
    }
  }
}

static bool volkswagen_meb_tx_hook(const CANPacket_t *msg) {
  // longitudinal limits
  // acceleration in m/s2 * 1000 to avoid floating point math
  const LongitudinalLimits VOLKSWAGEN_MEB_LONG_LIMITS = {
    .max_accel = 2000,
    .min_accel = -3500,
    .inactive_accel = 3010,  // VW sends one increment above the max range when inactive
  };

  const int volkswagen_accel_override = 0;

  bool tx = true;

  // Safety check for HCA_03 Heading Control Assist curvature
  if (msg->addr == MSG_HCA_03) {
    int desired_curvature_raw = (msg->data[3] | ((msg->data[4] & 0x7FU) << 8));

    bool sign = GET_BIT(msg, 39U);
    if (!sign) {
      desired_curvature_raw *= -1;
    }

    bool steer_req = GET_BIT(msg, 14U);
    int steer_power = msg->data[2] & 0x7FU;

    if (steer_angle_cmd_checks(desired_curvature_raw, steer_req, VOLKSWAGEN_MEB_STEERING_LIMITS)) {
      tx = false;

      // steer power is still allowed to decrease to zero monotonously
      // while controls are not allowed anymore
      if (steer_req && steer_power != 0) {
        if (steer_power < volkswagen_steer_power_prev) {
          tx = true;
        }
      }
    }

    if (!steer_req && steer_power != 0) {
      tx = false; // steer power is not 0 when disabled
    }

    volkswagen_steer_power_prev = steer_power;
  }

  // Safety check for MSG_MEB_ACC_02 acceleration requests
  // To avoid floating point math, scale upward and compare to pre-scaled safety m/s2 boundaries
  if (msg->addr == MSG_MEB_ACC_02) {
    // WARNING: IF WE TAKE THE SIGNAL FROM THE CAR WHILE ACC ACTIVE AND BELOW ABOUT 3km/h, THE CAR ERRORS AND PUTS ITSELF IN PARKING MODE WITH EPB!
    int desired_accel = ((((msg->data[4] & 0x7U) << 8) | msg->data[3]) * 5U) - 7220U;

    if (vw_meb_longitudinal_accel_checks(desired_accel, VOLKSWAGEN_MEB_LONG_LIMITS, volkswagen_accel_override)) {
      tx = false;
    }
  }

  // FORCE CANCEL: ensuring that only the cancel button press is sent when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if ((msg->addr == MSG_GRA_ACC_01) && !controls_allowed) {
    // disallow resume and set: bits 16 and 19
    if ((msg->data[2] & 0x9U) != 0U) {
      tx = false;
    }
  }

  return tx;
}

const safety_hooks volkswagen_meb_hooks = {
  .init = volkswagen_meb_init,
  .rx = volkswagen_meb_rx_hook,
  .tx = volkswagen_meb_tx_hook,
  .get_counter = volkswagen_mqb_meb_get_counter,
  .get_checksum = volkswagen_mqb_meb_get_checksum,
  .compute_checksum = volkswagen_meb_compute_crc,
};
