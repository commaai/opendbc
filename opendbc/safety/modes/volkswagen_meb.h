#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"

#define MSG_ESC_51           0xFCU    // RX, for wheel speeds
#define MSG_HCA_03           0x303U   // TX by OP, Heading Control Assist steering torque
#define MSG_QFK_01           0x13DU   // RX, for steering angle
#define MSG_ACC_19           0x300U   // RX from ECU, for ACC status
#define MSG_ACC_18           0x14DU   // RX from ECU, for ACC status
#define MSG_GRA_ACC_01       0x12BU   // TX by OP, ACC control buttons for cancel/resume
#define MSG_MOTOR_14         0x3BEU   // RX from ECU, for brake switch status
#define MSG_LDW_02           0x397U   // TX by OP, Lane line recognition and text alerts
#define MSG_Motor_51         0x10BU   // RX for TSK state and accel pedal
#define MSG_TA_01            0x26BU   // TX for Travel Assist status
#define MSG_EA_02            0x1F0U   // TX, for EA mitigation
#define MSG_KLR_01           0x25DU   // TX, for capacitive steering wheel



// PANDA SAFETY SHOULD INTRODUCE A .ignore_length flag (ALLOWED ONLY IF CHECKSUM CHECK IS REQUIRED TO BE SAFE)
#define VW_MEB_COMMON_RX_CHECKS                                                                     \
  {.msg = {{MSG_LH_EPS_03, 0, 8, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_MOTOR_14, 0, 8, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   \
  {.msg = {{MSG_GRA_ACC_01, 0, 8, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
  {.msg = {{MSG_QFK_01, 0, 32, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},    \
  
#define VW_MEB_RX_CHECKS                                                                            \
  {.msg = {{MSG_Motor_51, 0, 32, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_ESC_51, 0, 48, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},    \
  
#define VW_MEB_GEN2_RX_CHECKS                                                                       \
  {.msg = {{MSG_Motor_51, 0, 48, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_ESC_51, 0, 64, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},    \

#define VW_MEB_LONG_TX_MSGS                                                            \
  {MSG_HCA_03, 0, 24, .check_relay = true},                                            \
  {MSG_ACC_19, 0, 48, .check_relay = true}, {MSG_ACC_18, 0, 32, .check_relay = true},  \
  {MSG_EA_02, 0, 8, .check_relay = true},                                              \
  {MSG_KLR_01, 0, 8, .check_relay = false}, {MSG_KLR_01, 2, 8, .check_relay = true},   \
  {MSG_LDW_02, 0, 8, .check_relay = true}, {MSG_TA_01, 0, 8, .check_relay = true},     \


static uint8_t volkswagen_crc8_lut_8h2f[256]; // Static lookup table for CRC8 poly 0x2F, aka 8H2F/AUTOSAR

static uint32_t volkswagen_meb_get_checksum(const CANPacket_t *msg) {
  return (uint8_t)msg->data[0];
}

static uint8_t volkswagen_meb_get_counter(const CANPacket_t *msg) {
  // MQB message counters are consistently found at LSB 8.
  return (uint8_t)msg->data[1] & 0xFU;
}

static uint32_t volkswagen_meb_compute_crc(const CANPacket_t *msg) {
  int len = GET_LEN(msg);

  // This is CRC-8H2F/AUTOSAR with a twist. See the OpenDBC implementation
  // of this algorithm for a version with explanatory comments.

  uint8_t crc = 0xFFU;
  for (int i = 1; i < len; i++) {
    crc ^= (uint8_t)msg->data[i];
    crc = volkswagen_crc8_lut_8h2f[crc];
  }
  
  uint8_t counter = volkswagen_meb_get_counter(msg);
  if (msg->addr == MSG_LH_EPS_03) {
    crc ^= (uint8_t[]){0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5}[counter];
  } else if (msg->addr == MSG_GRA_ACC_01) {
    crc ^= (uint8_t[]){0x6A,0x38,0xB4,0x27,0x22,0xEF,0xE1,0xBB,0xF8,0x80,0x84,0x49,0xC7,0x9E,0x1E,0x2B}[counter];
  } else if (msg->addr == MSG_QFK_01) {
    crc ^= (uint8_t[]){0x20,0xCA,0x68,0xD5,0x1B,0x31,0xE2,0xDA,0x08,0x0A,0xD4,0xDE,0x9C,0xE4,0x35,0x5B}[counter];
  } else if (msg->addr == MSG_ESC_51) {
    crc ^= (uint8_t[]){0x77,0x5C,0xA0,0x89,0x4B,0x7C,0xBB,0xD6,0x1F,0x6C,0x4F,0xF6,0x20,0x2B,0x43,0xDD}[counter];
  } else if (msg->addr == MSG_Motor_51) {
    crc ^= (uint8_t[]){0x77,0x5C,0xA0,0x89,0x4B,0x7C,0xBB,0xD6,0x1F,0x6C,0x4F,0xF6,0x20,0x2B,0x43,0xDD}[counter];
  } else if (msg->addr == MSG_MOTOR_14) {
    crc ^= (uint8_t[]){0x1F,0x28,0xC6,0x85,0xE6,0xF8,0xB0,0x19,0x5B,0x64,0x35,0x21,0xE4,0xF7,0x9C,0x24}[counter];
  } else if (msg->addr == MSG_KLR_01) {
    crc ^= (uint8_t[]){0xDA,0x6B,0x0E,0xB2,0x78,0xBD,0x5A,0x81,0x7B,0xD6,0x41,0x39,0x76,0xB6,0xD7,0x35}[counter];
  } else if (msg->addr == MSG_EA_02) {
    crc ^= (uint8_t[]){0x2F,0x3C,0x22,0x60,0x18,0xEB,0x63,0x76,0xC5,0x91,0x0F,0x27,0x34,0x04,0x7F,0x02}[counter];
  }
  else {
    // Undefined CAN message, CRC check expected to fail
  }
  crc = volkswagen_crc8_lut_8h2f[crc];

  return (uint8_t)(crc ^ 0xFFU);
}

static uint32_t volkswagen_meb_gen2_compute_crc(const CANPacket_t *msg) {
  // For newer variants the checksum is calculated over a specific signal length.
  if (!volkswagen_alt_crc_variant_1) {
	return volkswagen_meb_compute_crc(msg); // fallback
  }
  
  int len = GET_LEN(msg);
  
  if (msg->addr == MSG_QFK_01) {
    len = 28;
  } else if (msg->addr == MSG_ESC_51) {
    len = 60;
  } else if (msg->addr == MSG_Motor_51) {
    len = 44;
  } else {
	return volkswagen_meb_compute_crc(msg); // fallback
  }
  
  uint8_t crc = 0xFFU;
  for (int i = 1; i < len; i++) {
    crc ^= (uint8_t)msg->data[i];
    crc = volkswagen_crc8_lut_8h2f[crc];
  }
  
  uint8_t counter = volkswagen_meb_get_counter(msg);
  if (msg->addr == MSG_QFK_01) {
	crc ^= (uint8_t[]){0x18,0x71,0x10,0x8D,0xD7,0xAA,0xB0,0x78,0xAC,0x12,0xAE,0x0C,0xDD,0xF1,0x85,0x68}[counter];
  } else if (msg->addr == MSG_ESC_51) {
	crc ^= (uint8_t[]){0x69,0xDC,0xF9,0x64,0x6A,0xCE,0x55,0x2C,0xC4,0x38,0x8F,0xD1,0xC6,0x43,0xB4,0xB1}[counter];
  } else if (msg->addr == MSG_Motor_51) {
	crc ^= (uint8_t[]){0x2C,0xB1,0x1A,0x75,0xBB,0x65,0x79,0x47,0x81,0x2B,0xCC,0x96,0x17,0xDB,0xC0,0x94}[counter];
  } else {
	return volkswagen_meb_compute_crc(msg); // fallback
  }

  crc = (uint8_t)(volkswagen_crc8_lut_8h2f[crc] ^ 0xFFU);
  
  if (crc != msg->data[0]) {
	return volkswagen_meb_compute_crc(msg); // fallback
  }
  
  return (uint8_t)(crc);
}

static safety_config volkswagen_meb_init(uint16_t param) {
  // Transmit of GRA_ACC_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  static const CanMsg VOLKSWAGEN_MEB_STOCK_TX_MSGS[] = {{MSG_HCA_03, 0, 24, .check_relay = true}, {MSG_GRA_ACC_01, 0, 8, .check_relay = false},
                                                        {MSG_EA_02, 0, 8, .check_relay = true},
                                                        {MSG_KLR_01, 0, 8, .check_relay = false}, {MSG_KLR_01, 2, 8, .check_relay = true},
                                                        {MSG_GRA_ACC_01, 2, 8, .check_relay = false}, {MSG_LDW_02, 0, 8, .check_relay = true}};
  
  static const CanMsg VOLKSWAGEN_MEB_LONG_TX_MSGS[] = {
	VW_MEB_LONG_TX_MSGS
  };

  static RxCheck volkswagen_meb_rx_checks[] = {
    VW_MEB_COMMON_RX_CHECKS
	VW_MEB_RX_CHECKS
  };

  static RxCheck volkswagen_meb_gen2_rx_checks[] = {
    VW_MEB_COMMON_RX_CHECKS
	VW_MEB_GEN2_RX_CHECKS
  };

  volkswagen_set_button_prev = false;
  volkswagen_resume_button_prev = false;

  volkswagen_alt_crc_variant_1 = GET_FLAG(param, FLAG_VOLKSWAGEN_ALT_CRC_VARIANT_1);

#ifdef ALLOW_DEBUG
  volkswagen_longitudinal = GET_FLAG(param, FLAG_VOLKSWAGEN_LONG_CONTROL);
#endif
  
  gen_crc_lookup_table_8(0x2F, volkswagen_crc8_lut_8h2f);
  
  safety_config ret;
  
  if (volkswagen_longitudinal) {
	SET_TX_MSGS(VOLKSWAGEN_MEB_LONG_TX_MSGS, ret);
  } else {
	SET_TX_MSGS(VOLKSWAGEN_MEB_STOCK_TX_MSGS, ret);
  }
  
  if (volkswagen_alt_crc_variant_1) {
	SET_RX_CHECKS(volkswagen_meb_gen2_rx_checks, ret);
  } else {
	SET_RX_CHECKS(volkswagen_meb_rx_checks, ret);
  }
  
  return ret;
}

// lateral limits for curvature
static const CurvatureSteeringLimits VOLKSWAGEN_MEB_STEERING_LIMITS = {
  .max_curvature = 29105, // 0.195 rad/m
  .curvature_to_can = 149253.7313, // 1 / 6.7e-6 rad/m to can
  .send_rate = 0.02,
  .inactive_curvature_is_zero = true,
  .max_power = 125 // 50%
};

static void volkswagen_meb_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {

    // Update in-motion state by sampling wheel speeds
    if (msg->addr == MSG_ESC_51) {
      uint32_t fr = msg->data[10] | msg->data[11] << 8;
      uint32_t rr = msg->data[14] | msg->data[15] << 8;
      uint32_t rl = msg->data[12] | msg->data[13] << 8;
      uint32_t fl = msg->data[8] | msg->data[9] << 8;

      vehicle_moving = (fr > 0U) || (rr > 0U) || (rl > 0U) || (fl > 0U);

      UPDATE_VEHICLE_SPEED(((fr + rr + rl + fl) / 4 ) * 0.0075 / 3.6);
    }

    if (msg->addr == MSG_QFK_01) { // we do not need conversion deg to can, same scaling as HCA_03 curvature
      int current_curvature = ((msg->data[6] & 0x7F) << 8) | msg->data[5];
      
      bool current_curvature_sign = GET_BIT(msg, 55U);
      if (!current_curvature_sign) {
        current_curvature *= -1;
      }

      update_sample(&curvature_meas, current_curvature);
    }

    // Update driver input torque samples
    // Signal: LH_EPS_03.EPS_Lenkmoment (absolute torque)
    // Signal: LH_EPS_03.EPS_VZ_Lenkmoment (direction)
    if (msg->addr == MSG_LH_EPS_03) {
      update_sample(&torque_driver, volkswagen_mlb_mqb_driver_input_torque(msg));
    }

    // Update cruise state
    if (msg->addr == MSG_Motor_51) {
      // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
      // Always exit controls on main switch off
      // Signal: TSK_06.TSK_Status
      int acc_status = ((msg->data[11] >> 0) & 0x07U);
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
    if (msg->addr == MSG_Motor_51) {
      int accel_pedal_value = ((msg->data[1] >> 4) & 0x0FU) | ((msg->data[2] & 0x1FU) << 4);
      gas_pressed = accel_pedal_value > 0;
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
	.override_accel = 0,
	.allow_override = true,
  };
  
  bool tx = true;

  // Safety check for HCA_03 Heading Control Assist curvature
  if (msg->addr == MSG_HCA_03) {
    int desired_curvature_raw = GET_BYTES(msg, 3, 2) & 0x7FFFU;

    bool desired_curvature_sign = GET_BIT(msg, 39U);
    if (!desired_curvature_sign) {
      desired_curvature_raw *= -1;
    }

    bool steer_req = (((msg->data[1] >> 4) & 0x0FU) == 4U);
    int steer_power = msg->data[2];

    if (steer_power_cmd_checks(steer_power, steer_req, VOLKSWAGEN_MEB_STEERING_LIMITS)) {
      tx = false;
    }

	if (steer_curvature_cmd_checks_average(desired_curvature_raw, steer_req, VOLKSWAGEN_MEB_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // Safety check for MSG_ACC_18 acceleration requests
  // To avoid floating point math, scale upward and compare to pre-scaled safety m/s2 boundaries
  if (msg->addr == MSG_ACC_18) {
    // WARNING: IF WE TAKE THE SIGNAL FROM THE CAR WHILE ACC ACTIVE AND BELOW ABOUT 3km/h, THE CAR ERRORS AND PUTS ITSELF IN PARKING MODE WITH EPB!
    int desired_accel = ((((msg->data[4] & 0x7U) << 8) | msg->data[3]) * 5U) - 7220U;

    if (longitudinal_accel_checks(desired_accel, VOLKSWAGEN_MEB_LONG_LIMITS)) {
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
  .get_counter = volkswagen_meb_get_counter,
  .get_checksum = volkswagen_meb_get_checksum,
  .compute_checksum = volkswagen_meb_gen2_compute_crc,
};