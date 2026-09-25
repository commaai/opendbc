#pragma once

#include "opendbc/safety/declarations.h"

// Stock longitudinal
#define TOYOTA_BASE_TX_MSGS \
  {0x191, 0, 8, .check_relay = true}, {0x412, 0, 8, .check_relay = true}, {0x1D2, 0, 8, .check_relay = false},  /* LKAS + LTA + PCM cancel cmd */  \

#define TOYOTA_COMMON_TX_MSGS \
  TOYOTA_BASE_TX_MSGS \
  {0x2E4, 0, 5, .check_relay = true}, \
  {0x343, 0, 8, .check_relay = false},  /* ACC cancel cmd */  \

#define TOYOTA_COMMON_SECOC_TX_MSGS \
  TOYOTA_BASE_TX_MSGS \
  {0x2E4, 0, 8, .check_relay = true}, {0x131, 0, 8, .check_relay = true}, \
  {0x343, 0, 8, .check_relay = false},  /* ACC cancel cmd */ \

#define TOYOTA_COMMON_LONG_TX_MSGS \
  TOYOTA_COMMON_TX_MSGS \
  /* DSU bus 0 */ \
  {0x283, 0, 7, .check_relay = false}, {0x2E6, 0, 8, .check_relay = false}, {0x2E7, 0, 8, .check_relay = false}, {0x33E, 0, 7, .check_relay = false}, \
  {0x344, 0, 8, .check_relay = false}, {0x365, 0, 7, .check_relay = false}, {0x366, 0, 7, .check_relay = false}, {0x4CB, 0, 8, .check_relay = false}, \
  /* DSU bus 1 */ \
  {0x128, 1, 6, .check_relay = false}, {0x141, 1, 4, .check_relay = false}, {0x160, 1, 8, .check_relay = false}, {0x161, 1, 7, .check_relay = false}, \
  {0x470, 1, 4, .check_relay = false}, \
  /* PCS_HUD */                        \
  {0x411, 0, 8, .check_relay = false}, \
  /* radar diagnostic address */       \
  {0x750, 0, 8, .check_relay = false}, \
  /* ACC */                            \
  {0x343, 0, 8, .check_relay = true},  \

#define TOYOTA_COMMON_SECOC_LONG_TX_MSGS \
  TOYOTA_COMMON_SECOC_TX_MSGS \
  {0x343, 0, 8, .check_relay = true}, \
  {0x183, 0, 8, .check_relay = true},  /* ACC_CONTROL_2 */ \

#define TOYOTA_TSS3_TX_MSGS \
  {0x777, 1, 8, .check_relay = false},  /* signer admin */ \
  {0x777, 0, 8, .check_relay = false},  /* signer requests */ \
  {0x08A, 0, 32, .check_relay = true, .disable_static_blocking = true},  /* CONTROL_REQUEST */ \
  {0x412, 0, 8, .check_relay = true},  /* LKAS_HUD */ \
  {0x101, 2, 8, .check_relay = false},  /* BRAKE_MODULE cancel */ \

#define TOYOTA_COMMON_RX_CHECKS(lta)                                                                                                       \
  {.msg = {{ 0xaa, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},      \
  {.msg = {{0x260, 0, 8, 50U, .ignore_counter = true, .ignore_quality_flag=!(lta)}, { 0 }, { 0 }}},  \

#define TOYOTA_RX_CHECKS(lta)                                                                                                               \
  TOYOTA_COMMON_RX_CHECKS(lta)                                                                                                              \
  {.msg = {{0x1D2, 0, 8, 33U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                            \
  {.msg = {{0x226, 0, 8, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  { 0 }, { 0 }}},  \

#define TOYOTA_ALT_BRAKE_RX_CHECKS(lta)                                                                                                    \
  TOYOTA_COMMON_RX_CHECKS(lta)                                                                                                             \
  {.msg = {{0x1D2, 0, 8, 33U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                           \
  {.msg = {{0x224, 0, 8, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define TOYOTA_TSS3_RX_CHECKS                                                                                                                         \
  {.msg = {{0x025, 0, 32, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0x0AA, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},                               \
  {.msg = {{0x116, 0, 8, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   \
  {.msg = {{0x101, 0, 8, 50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                            \
  {.msg = {{0x08A, 2, 32, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define TOYOTA_SECOC_RX_CHECKS                                                                                                             \
  TOYOTA_COMMON_RX_CHECKS(false)                                                                                                           \
  {.msg = {{0x176, 0, 8, 32U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                           \
  {.msg = {{0x116, 0, 8, 42U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0x101, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

static bool toyota_secoc = false;
static bool toyota_alt_brake = false;
static bool toyota_stock_longitudinal = false;
static bool toyota_lta = false;
static bool toyota_tss3 = false;

// TSS3: openpilot replaces the FRC's CONTROL_REQUEST (0x08A) while the EPS signer is armed.
// Requests are checked when they are sent to the signer, and only approved requests can be published once signed.
// The VMC in the brake ECU needs a continuous CONTROL_REQUEST: it sets CONTROL_RESULT.REQUEST_LOSS ~90 ms after the
// last valid one and latches a cruise fault until restart if that persists for ~1 s.
#define TOYOTA_TSS3_08A_TIMEOUT_US 100000U
#define TOYOTA_TSS3_08A_LEN 28U  // without the SecOC trailer
#define TOYOTA_TSS3_FRAGMENT_LEN 7U
#define TOYOTA_TSS3_APPROVED_LEN 8U
#define TOYOTA_TSS3_STOCK_08A_HISTORY 2U
static bool toyota_tss3_08a_active = false;
static uint32_t toyota_tss3_08a_last_tx_ts = 0U;
static uint8_t toyota_tss3_request_next_fragment = 0U;
static uint32_t toyota_tss3_last_request_ts = 0U;
static uint8_t toyota_tss3_approved[TOYOTA_TSS3_APPROVED_LEN][TOYOTA_TSS3_08A_LEN];
static uint32_t toyota_tss3_approved_ts[TOYOTA_TSS3_APPROVED_LEN];
static bool toyota_tss3_approved_valid[TOYOTA_TSS3_APPROVED_LEN];
static uint8_t toyota_tss3_approved_idx = 0U;
static uint8_t toyota_tss3_stock_08a[TOYOTA_TSS3_STOCK_08A_HISTORY][TOYOTA_TSS3_08A_LEN];
static uint32_t toyota_tss3_stock_08a_ts[TOYOTA_TSS3_STOCK_08A_HISTORY];
static uint32_t toyota_tss3_stock_08a_last_ts = 0U;
static uint8_t toyota_tss3_stock_08a_idx = 0U;
static uint8_t toyota_tss3_stock_08a_cnt = 0U;
static int toyota_dbc_eps_torque_factor = 100;   // conversion factor for STEER_TORQUE_EPS in %: see dbc file

static uint32_t toyota_compute_checksum(const CANPacket_t *msg) {
  int len = GET_LEN(msg);
  uint8_t checksum = (uint8_t)(msg->addr) + (uint8_t)((unsigned int)(msg->addr) >> 8U) + (uint8_t)(len);
  for (int i = 0; i < (len - 1); i++) {
    checksum += (uint8_t)msg->data[i];
  }
  return checksum;
}

static uint32_t toyota_get_checksum(const CANPacket_t *msg) {
  int checksum_byte = GET_LEN(msg) - 1U;
  return (uint8_t)(msg->data[checksum_byte]);
}

static bool toyota_get_quality_flag_valid(const CANPacket_t *msg) {
  bool valid = false;
  if (msg->addr == 0x260U) {
    valid = !GET_BIT(msg, 3U);  // STEER_TORQUE_SENSOR.STEER_ANGLE_INITIALIZING
  }
  if (msg->addr == 0xaaU) {  // WHEEL_SPEEDS
    // each wheel speed is 1-bit fault + 15-bit speed
    valid = true;
    for (uint8_t i = 0U; i < 4U; i += 1U) {
      if (GET_BIT(msg, (i * 16U) + 7U)) {  // WHEEL_SPEED_xx_FAULT
        valid = false;
        break;
      }
    }
  }
  return valid;
}

static void toyota_rx_hook(const CANPacket_t *msg) {
  if (toyota_tss3) {
    // CONTROL_REQUEST from the FRC
    if (msg_matches(msg, 0x8AU, 2U, 32U)) {
      for (uint8_t i = 0U; i < TOYOTA_TSS3_08A_LEN; i++) {
        toyota_tss3_stock_08a[toyota_tss3_stock_08a_idx][i] = msg->data[i];
      }
      toyota_tss3_stock_08a_last_ts = microsecond_timer_get();
      toyota_tss3_stock_08a_ts[toyota_tss3_stock_08a_idx] = toyota_tss3_stock_08a_last_ts;
      toyota_tss3_stock_08a_idx = (toyota_tss3_stock_08a_idx + 1U) % TOYOTA_TSS3_STOCK_08A_HISTORY;
      if (toyota_tss3_stock_08a_cnt < TOYOTA_TSS3_STOCK_08A_HISTORY) {
        toyota_tss3_stock_08a_cnt++;
      }

      pcm_cruise_check(GET_BIT(msg, 27U));  // CONTROL_REQUEST.CRUISE_OPERATING_LATCH
    }

    // STEER_ANGLE_SENSOR, in LATERAL_REQUEST_PINION_ANGLE units
    if (msg_matches(msg, 0x25U, 0U)) {
      int angle_coarse = to_signed(((msg->data[0] & 0xFU) << 8U) | msg->data[1], 12);
      int angle_fraction = to_signed((msg->data[4] >> 4U) & 0xFU, 4);
      int angle_tenths = (angle_coarse * 15) + angle_fraction;
      update_sample(&angle_meas, ROUND(((float)angle_tenths * 1787.0F) / 1024.0F));
    }
  }

  // get eps motor torque (0.66 factor in dbc)
  if (!toyota_tss3 && msg_matches(msg, 0x260U, 0U)) {
    int torque_meas_new = (msg->data[5] << 8) | msg->data[6];
    torque_meas_new = to_signed(torque_meas_new, 16);

    // scale by dbc_factor
    torque_meas_new = (torque_meas_new * toyota_dbc_eps_torque_factor) / 100;

    // update array of sample
    update_sample(&torque_meas, torque_meas_new);

    // increase torque_meas by 1 to be conservative on rounding
    torque_meas.min--;
    torque_meas.max++;

    // driver torque for angle limiting
    int torque_driver_new = (msg->data[1] << 8) | msg->data[2];
    torque_driver_new = to_signed(torque_driver_new, 16);
    update_sample(&torque_driver, torque_driver_new);

    // LTA request angle should match current angle while inactive, clipped to max accepted angle.
    // note that angle can be relative to init angle on some TSS2 platforms, LTA has the same offset
    bool steer_angle_initializing = GET_BIT(msg, 3U);
    if (!steer_angle_initializing) {
      int angle_meas_new = (msg->data[3] << 8U) | msg->data[4];
      angle_meas_new = to_signed(angle_meas_new, 16);
      update_sample(&angle_meas, angle_meas_new);
    }
  }

  // enter controls on rising edge of ACC, exit controls on ACC off
  // exit controls on rising edge of gas press, if not alternative experience
  // exit controls on rising edge of brake press
  if (toyota_secoc || toyota_tss3) {
    if (toyota_secoc && msg_matches(msg, 0x176U, 0U)) {
      bool cruise_engaged = GET_BIT(msg, 5U);  // PCM_CRUISE.CRUISE_ACTIVE
      pcm_cruise_check(cruise_engaged);
    }
    if (msg_matches(msg, 0x116U, 0U)) {
      gas_pressed = msg->data[1] != 0U;  // GAS_PEDAL.GAS_PEDAL_USER
    }
    if (msg_matches(msg, 0x101U, 0U)) {
      brake_pressed = GET_BIT(msg, 3U);  // BRAKE_MODULE.BRAKE_PRESSED (toyota_rav4_prime_generated.dbc)
    }
  } else {
    if (msg_matches(msg, 0x1D2U, 0U)) {
      bool cruise_engaged = GET_BIT(msg, 5U);  // PCM_CRUISE.CRUISE_ACTIVE
      pcm_cruise_check(cruise_engaged);
      gas_pressed = !GET_BIT(msg, 4U);  // PCM_CRUISE.GAS_RELEASED
    }
    if (!toyota_alt_brake && msg_matches(msg, 0x226U, 0U)) {
      brake_pressed = GET_BIT(msg, 37U);  // BRAKE_MODULE.BRAKE_PRESSED (toyota_nodsu_pt_generated.dbc)
    }
    if (toyota_alt_brake && msg_matches(msg, 0x224U, 0U)) {
      brake_pressed = GET_BIT(msg, 5U);  // BRAKE_MODULE.BRAKE_PRESSED (toyota_new_mc_pt_generated.dbc)
    }
  }

  // sample speed
  if (msg_matches(msg, 0xaaU, 0U)) {
    int speed = 0;
    // sum 4 wheel speeds. conversion: raw * 0.01 - 67.67
    for (uint8_t i = 0U; i < 8U; i += 2U) {
      int wheel_speed = ((msg->data[i] & 0x7FU) << 8U) | msg->data[(i + 1U)];
      speed += wheel_speed - 6767;
    }
    // check that all wheel speeds are at zero value
    vehicle_moving = speed != 0;

    UPDATE_VEHICLE_SPEED(speed / 4.0 * 0.01 * KPH_TO_MS);
  }
}

static bool toyota_tss3_stock_08a_match(const uint8_t request[], uint32_t now) {
  // with stock longitudinal, only the lateral request and sequence of the latest FRC requests may change
  bool match = false;
  for (uint8_t j = 0U; j < toyota_tss3_stock_08a_cnt; j++) {
    bool candidate = safety_get_ts_elapsed(now, toyota_tss3_stock_08a_ts[j]) <= TOYOTA_TSS3_08A_TIMEOUT_US;
    for (uint8_t i = 0U; i < TOYOTA_TSS3_08A_LEN; i++) {
      uint8_t mask = 0xFFU;
      if ((i == 18U) || (i == 19U) || (i == 24U) || (i == 25U)) {
        mask = 0U;
      } else if ((i == 21U) || (i == 26U)) {
        mask = 0xC0U;
      } else {
        // compare the whole byte
      }
      candidate = candidate && (((request[i] ^ toyota_tss3_stock_08a[j][i]) & mask) == 0U);
    }
    match = match || candidate;
  }
  return match;
}

static bool toyota_tss3_08a_match(const uint8_t request[]) {
  // only the set speed, lateral and longitudinal requests, and sequence may change
  return (request[0] == 0U) && (request[1] == 0U) && (request[2] == 0U) && (request[3] == 0x08U) &&
         (request[4] == 0x80U) && (request[5] == 0U) && (request[13] == 0x7FU) && (request[14] == 0xFFU) &&
         (request[15] == 0U) && (request[16] == 0x7FU) && (request[17] == 0xFFU) && (request[20] == 0xC0U) &&
         ((request[21] & 0xC0U) == 0U) && (request[22] == 0x10U) && (request[23] == 0U) &&
         (request[25] == 0U) && (request[27] == 0U);
}

static bool toyota_tss3_request_valid(const uint8_t request[], const LongitudinalLimits long_limits, uint32_t now) {
  static const AngleSteeringLimits TOYOTA_TSS3_ANGLE_STEERING_LIMITS = {
    .max_angle = 1745,
    .angle_deg_to_can = 17.451171875F,  // 17870 / 1024
    .frequency = 100U,
  };

  static const AngleSteeringParams TOYOTA_TSS3_STEERING_PARAMS = {
    .slip_factor = -0.0007484283457339188F,  // calc_slip_factor(VM)
    .steer_ratio = 15.3F,
    .wheelbase = 2.825F,
  };

  bool valid = toyota_stock_longitudinal ? toyota_tss3_stock_08a_match(request, now) : toyota_tss3_08a_match(request);

  // LATERAL_REQUEST_ID and LATERAL_ASSIST_GAIN
  const uint8_t lat_id = request[21] & 0x3FU;
  const bool lat_active = (lat_id == 11U) && (request[24] == 100U);
  const bool lat_inactive = (lat_id == 0U) && (request[24] == 50U);
  valid = valid && (lat_active || lat_inactive) && (request[25] == 0U);

  if (valid) {
    // requests are only sent while openpilot is engaged, start from the measured angle after a gap
    if (safety_get_ts_elapsed(now, toyota_tss3_last_request_ts) > TOYOTA_TSS3_08A_TIMEOUT_US) {
      desired_angle_last = SAFETY_CLAMP(angle_meas.values[0], -TOYOTA_TSS3_ANGLE_STEERING_LIMITS.max_angle,
                                        TOYOTA_TSS3_ANGLE_STEERING_LIMITS.max_angle);
    }
    toyota_tss3_last_request_ts = now;

    // LATERAL_REQUEST_PINION_ANGLE
    int angle = to_signed((request[18] << 8U) | request[19], 16);
    bool violation = steer_angle_cmd_checks_vm(angle, lat_active, TOYOTA_TSS3_ANGLE_STEERING_LIMITS, TOYOTA_TSS3_STEERING_PARAMS);
    if (safety_max_limit_check(angle, TOYOTA_TSS3_ANGLE_STEERING_LIMITS.max_angle, -TOYOTA_TSS3_ANGLE_STEERING_LIMITS.max_angle)) {
      violation = true;
      desired_angle_last = SAFETY_CLAMP(angle_meas.values[0], -TOYOTA_TSS3_ANGLE_STEERING_LIMITS.max_angle,
                                        TOYOTA_TSS3_ANGLE_STEERING_LIMITS.max_angle);
    }

    if (!toyota_stock_longitudinal) {
      // LONGITUDINAL_REQUEST_ID/ALLOCATION_METHOD and LONGITUDINAL_REQUEST_ACCEL_UPPER/LOWER
      int accel_upper = to_signed((request[8] << 8U) | request[9], 16);
      int accel_lower = to_signed((request[11] << 8U) | request[12], 16);
      violation = violation || (request[6] != 0x2DU) || (request[7] != 0x47U) || (accel_upper != accel_lower);

      // No gas pressed check: the motion controller in the brake ECU arbitrates driver gas against this request
      bool accel_valid = controls_allowed && !safety_max_limit_check(accel_upper, long_limits.max_accel, long_limits.min_accel);
      violation = violation || !(accel_valid || (accel_upper == long_limits.inactive_accel));
    }
    valid = !violation;
  }
  return valid;
}

static bool toyota_tss3_request_fragment(const CANPacket_t *msg, const LongitudinalLimits long_limits, uint32_t now) {
  static uint8_t toyota_tss3_request[TOYOTA_TSS3_08A_LEN];
  static uint8_t toyota_tss3_request_seq[2];  // low and high sequence nibbles, repeated in fragments 2 and 3

  // the header's high nibble is the fragment index (8-B), the low nibble alternates the low and high sequence nibbles
  const uint8_t fragment = (msg->data[0] >> 4U) & 0x3U;
  const uint8_t seq_nibble = msg->data[0] & 0xFU;
  bool valid = !msg->fd && ((msg->data[0] & 0xC0U) == 0x80U) && ((fragment == 0U) || (fragment == toyota_tss3_request_next_fragment));
  valid = valid && ((fragment < 2U) || (seq_nibble == toyota_tss3_request_seq[fragment - 2U]));

  if (valid) {
    if (fragment < 2U) {
      toyota_tss3_request_seq[fragment] = seq_nibble;
    }
    for (uint8_t i = 0U; i < TOYOTA_TSS3_FRAGMENT_LEN; i++) {
      toyota_tss3_request[(fragment * TOYOTA_TSS3_FRAGMENT_LEN) + i] = msg->data[i + 1U];
    }
    toyota_tss3_request_next_fragment = fragment + 1U;
  } else {
    toyota_tss3_request_next_fragment = 0U;
  }

  // the signer only signs complete requests, so the last fragment carries the check
  if (valid && (fragment == 3U)) {
    toyota_tss3_request_next_fragment = 0U;
    valid = toyota_tss3_request_valid(toyota_tss3_request, long_limits, now);
    if (valid) {
      for (uint8_t i = 0U; i < TOYOTA_TSS3_08A_LEN; i++) {
        toyota_tss3_approved[toyota_tss3_approved_idx][i] = toyota_tss3_request[i];
      }
      toyota_tss3_approved_ts[toyota_tss3_approved_idx] = now;
      toyota_tss3_approved_valid[toyota_tss3_approved_idx] = true;
      toyota_tss3_approved_idx = (toyota_tss3_approved_idx + 1U) % TOYOTA_TSS3_APPROVED_LEN;
    }
  }
  return valid;
}

static bool toyota_tss3_publish_approved(const CANPacket_t *msg, uint32_t now) {
  // each approved request can be published once
  bool approved = false;
  for (uint8_t j = 0U; j < TOYOTA_TSS3_APPROVED_LEN; j++) {
    bool match = !approved && toyota_tss3_approved_valid[j] &&
                 (safety_get_ts_elapsed(now, toyota_tss3_approved_ts[j]) <= TOYOTA_TSS3_08A_TIMEOUT_US);
    for (uint8_t i = 0U; i < TOYOTA_TSS3_08A_LEN; i++) {
      match = match && (msg->data[i] == toyota_tss3_approved[j][i]);
    }
    if (match) {
      approved = true;
      toyota_tss3_approved_valid[j] = false;
    }
  }
  return approved;
}

static bool toyota_tss3_tx_hook(const CANPacket_t *msg, const LongitudinalLimits long_limits) {
  bool tx = true;
  const uint32_t now = microsecond_timer_get();

  // signer admin: arm or release openpilot's CONTROL_REQUEST
  if (msg_matches(msg, 0x777U, 1U)) {
    bool admin = !msg->fd && (msg->data[0] == 7U) && (msg->data[1] == 0xC9U) && (msg->data[2] == 0xA8U) && (msg->data[3] <= 1U) &&
                 (msg->data[4] == 0U) && (msg->data[5] == 0U) && (msg->data[6] == 0U) && (msg->data[7] == 0U);
    if (!admin) {
      tx = false;
    } else if (msg->data[3] == 0U) {
      toyota_tss3_08a_active = false;
    } else {
      // with stock longitudinal, openpilot needs a recent FRC request to copy
      bool stock_08a_recent = (toyota_tss3_stock_08a_cnt > 0U) &&
                              (safety_get_ts_elapsed(now, toyota_tss3_stock_08a_last_ts) <= TOYOTA_TSS3_08A_TIMEOUT_US);
      tx = !toyota_stock_longitudinal || stock_08a_recent;
      if (tx) {
        toyota_tss3_08a_active = true;
        toyota_tss3_08a_last_tx_ts = now;
      }
    }
  }

  // signer requests: four fragments of the unsigned CONTROL_REQUEST
  if (msg_matches(msg, 0x777U, 0U)) {
    tx = toyota_tss3_request_fragment(msg, long_limits, now);
  }

  // CONTROL_REQUEST with the SecOC trailer from the signer
  if (msg_matches(msg, 0x8AU, 0U)) {
    tx = toyota_tss3_08a_active && msg->fd && toyota_tss3_publish_approved(msg, now);
    if (tx) {
      toyota_tss3_08a_last_tx_ts = now;
    } else {
      // the car always needs a CONTROL_REQUEST, forward the FRC's again
      toyota_tss3_08a_active = false;
    }
  }

  // BRAKE_MODULE with only BRAKE_PRESSED set cancels stock cruise
  if (msg_matches(msg, 0x101U, 2U)) {
    tx = (toyota_compute_checksum(msg) == toyota_get_checksum(msg)) && (msg->data[0] == 0x88U) &&
         (msg->data[2] == 0U) && (msg->data[4] == 0U) && (msg->data[5] == 0U) && (msg->data[6] == 0U);
  }

  // LKAS_HUD
  if (msg_matches(msg, 0x412U, 0U)) {
    tx = !msg->fd;
  }

  return tx;
}

static bool toyota_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits TOYOTA_TORQUE_STEERING_LIMITS = {
    .max_torque = 1500,
    .max_rate_up = 15,          // ramp up slow
    .max_rate_down = 25,        // ramp down fast
    .max_torque_error = 350,    // max torque cmd in excess of motor torque
    .max_rt_delta = 450,        // the real time limit is 1800/sec, a 20% buffer
    .type = TorqueMotorLimited,

    // the EPS faults when the steering angle rate is above a certain threshold for too long. to prevent this,
    // we allow setting STEER_REQUEST bit to 0 while maintaining the requested torque value for a single frame
    .min_valid_request_frames = 17,
    .max_invalid_request_frames = 1,
    .min_valid_request_rt_interval = 162000,  // 162ms; a ~10% buffer on cutting every 18 frames
    .has_steer_req_tolerance = true,
  };

  static const AngleSteeringLimits TOYOTA_ANGLE_STEERING_LIMITS = {
    // LTA angle limits
    // factor for STEER_TORQUE_SENSOR->STEER_ANGLE and STEERING_LTA->STEER_ANGLE_CMD (1 / 0.0573)
    .max_angle = 1657,  // EPS only accepts up to 94.9461
    .angle_deg_to_can = 17.452007,
    .angle_rate_up_lookup = {
      {5., 25., 25.},
      {0.3, 0.15, 0.15}
    },
    .angle_rate_down_lookup = {
      {5., 25., 25.},
      {0.36, 0.26, 0.26}
    },
  };

  const int TOYOTA_LTA_MAX_MEAS_TORQUE = 1500;
  const int TOYOTA_LTA_MAX_DRIVER_TORQUE = 150;

  // longitudinal limits
  const LongitudinalLimits TOYOTA_LONG_LIMITS = {
    .max_accel = 2000,   // 2.0 m/s2
    .min_accel = -3500,  // -3.5 m/s2
  };

  bool tx = true;

  if (toyota_tss3) {
    tx = toyota_tss3_tx_hook(msg, TOYOTA_LONG_LIMITS);
  }

  // Check if msg is sent on BUS 0
  // ACCEL: safety check on byte 1-2
  if (msg_matches(msg, 0x343U, 0U)) {
    int desired_accel = (msg->data[0] << 8) | msg->data[1];
    desired_accel = to_signed(desired_accel, 16);

    bool violation = false;
    if (toyota_secoc) {
      // SecOC cars move accel to 0x183. Only allow inactive accel on 0x343 to match stock behavior
      violation = desired_accel != TOYOTA_LONG_LIMITS.inactive_accel;
    }
    violation |= longitudinal_accel_checks(desired_accel, TOYOTA_LONG_LIMITS);

    // only ACC messages that cancel are allowed when openpilot is not controlling longitudinal
    if (toyota_stock_longitudinal) {
      bool cancel_req = GET_BIT(msg, 24U);
      if (!cancel_req) {
        violation = true;
      }
      if (desired_accel != TOYOTA_LONG_LIMITS.inactive_accel) {
        violation = true;
      }
    }

    if (violation) {
      tx = false;
    }
  }

  if (msg_matches(msg, 0x183U, 0U)) {
    int desired_accel = (msg->data[0] << 8) | msg->data[1];
    desired_accel = to_signed(desired_accel, 16);

    tx = !longitudinal_accel_checks(desired_accel, TOYOTA_LONG_LIMITS);
  }

  // AEB: block all actuation. only used when DSU is unplugged
  if (msg_matches(msg, 0x283U, 0U)) {
    // only allow the checksum, which is the last byte
    bool block = GET_BYTES_64_LE(msg, 0, 6) != 0U;
    if (block) {
      tx = false;
    }
  }

  // STEERING_LTA angle steering check
  if (msg_matches(msg, 0x191U, 0U)) {
    // check the STEER_REQUEST, STEER_REQUEST_2, TORQUE_WIND_DOWN, STEER_ANGLE_CMD signals
    bool lta_request = GET_BIT(msg, 0U);
    bool lta_request2 = GET_BIT(msg, 25U);
    int torque_wind_down = msg->data[5];
    int lta_angle = (msg->data[1] << 8) | msg->data[2];
    lta_angle = to_signed(lta_angle, 16);

    bool steer_control_enabled = lta_request || lta_request2;
    if (!toyota_lta) {
      // using torque (LKA), block LTA msgs with actuation requests
      if (steer_control_enabled || (lta_angle != 0) || (torque_wind_down != 0)) {
        tx = false;
      }
    } else {
      // check angle rate limits and inactive angle
      if (steer_angle_cmd_checks(lta_angle, steer_control_enabled, TOYOTA_ANGLE_STEERING_LIMITS)) {
        tx = false;
      }

      if (lta_request != lta_request2) {
        tx = false;
      }

      // TORQUE_WIND_DOWN is gated on steer request
      if (!steer_control_enabled && (torque_wind_down != 0)) {
        tx = false;
      }

      // TORQUE_WIND_DOWN can only be no or full torque
      if ((torque_wind_down != 0) && (torque_wind_down != 100)) {
        tx = false;
      }

      // check if we should wind down torque
      int driver_torque = SAFETY_MIN(SAFETY_ABS(torque_driver.min), SAFETY_ABS(torque_driver.max));
      if ((driver_torque > TOYOTA_LTA_MAX_DRIVER_TORQUE) && (torque_wind_down != 0)) {
        tx = false;
      }

      int eps_torque = SAFETY_MIN(SAFETY_ABS(torque_meas.min), SAFETY_ABS(torque_meas.max));
      if ((eps_torque > TOYOTA_LTA_MAX_MEAS_TORQUE) && (torque_wind_down != 0)) {
        tx = false;
      }
    }
  }

  // STEERING_LTA_2 angle steering check (SecOC)
  if (toyota_secoc && msg_matches(msg, 0x131U, 0U)) {
    // SecOC cars block any form of LTA actuation for now
    bool lta_request = GET_BIT(msg, 3U);  // STEERING_LTA_2.STEER_REQUEST
    bool lta_request2 = GET_BIT(msg, 0U);  // STEERING_LTA_2.STEER_REQUEST_2
    int lta_angle_msb = msg->data[2];  // STEERING_LTA_2.STEER_ANGLE_CMD (MSB)
    int lta_angle_lsb = msg->data[3];  // STEERING_LTA_2.STEER_ANGLE_CMD (LSB)

    bool actuation = lta_request || lta_request2 || (lta_angle_msb != 0) || (lta_angle_lsb != 0);
    if (actuation) {
      tx = false;
    }
  }

  // STEER: safety check on bytes 2-3
  if (msg_matches(msg, 0x2E4U, 0U)) {
    int desired_torque = (msg->data[1] << 8) | msg->data[2];
    desired_torque = to_signed(desired_torque, 16);
    bool steer_req = GET_BIT(msg, 0U);
    // When using LTA (angle control), assert no actuation on LKA message
    if (!toyota_lta) {
      if (steer_torque_cmd_checks(desired_torque, steer_req, TOYOTA_TORQUE_STEERING_LIMITS)) {
        tx = false;
      }
    } else {
      if ((desired_torque != 0) || steer_req) {
        tx = false;
      }
    }
  }

  // UDS: Only tester present ("\x0F\x02\x3E\x00\x00\x00\x00\x00") allowed on diagnostics address
  if (msg->addr == 0x750U) {
    // this address is sub-addressed. only allow tester present to radar (0xF)
    bool invalid_uds_msg = GET_BYTES_64_LE(msg, 0, 8) != 0x00000000003E020FULL;
    if (invalid_uds_msg) {
      tx = false;
    }
  }

  return tx;
}

static bool toyota_fwd_hook(int bus_num, int addr) {
  // block the FRC's CONTROL_REQUEST while openpilot sends it, forward it again if openpilot stops
  bool block = false;
  if (toyota_tss3_08a_active && (bus_num == 2) && (addr == 0x8A)) {
    if (safety_get_ts_elapsed(microsecond_timer_get(), toyota_tss3_08a_last_tx_ts) > TOYOTA_TSS3_08A_TIMEOUT_US) {
      toyota_tss3_08a_active = false;
    } else {
      block = true;
    }
  }
  return block;
}

static safety_config toyota_init(uint16_t param) {
  static const CanMsg TOYOTA_TSS3_TX_MSGS_ARR[] = {
    TOYOTA_TSS3_TX_MSGS
  };

  static const CanMsg TOYOTA_TX_MSGS[] = {
    TOYOTA_COMMON_TX_MSGS
  };

  static const CanMsg TOYOTA_SECOC_TX_MSGS[] = {
    TOYOTA_COMMON_SECOC_TX_MSGS
  };

  static const CanMsg TOYOTA_LONG_TX_MSGS[] = {
    TOYOTA_COMMON_LONG_TX_MSGS
  };

  static const CanMsg TOYOTA_SECOC_LONG_TX_MSGS[] = {
    TOYOTA_COMMON_SECOC_LONG_TX_MSGS
  };

  // safety param flags
  // first byte is for EPS factor, second is for flags
  const uint32_t TOYOTA_PARAM_OFFSET = 8U;
  const uint32_t TOYOTA_EPS_FACTOR = (1UL << TOYOTA_PARAM_OFFSET) - 1U;
  const uint32_t TOYOTA_PARAM_ALT_BRAKE = 1UL << TOYOTA_PARAM_OFFSET;
  const uint32_t TOYOTA_PARAM_STOCK_LONGITUDINAL = 2UL << TOYOTA_PARAM_OFFSET;
  const uint32_t TOYOTA_PARAM_LTA = 4UL << TOYOTA_PARAM_OFFSET;

#ifdef ALLOW_DEBUG
  const uint32_t TOYOTA_PARAM_SECOC = 8UL << TOYOTA_PARAM_OFFSET;
  toyota_secoc = GET_FLAG(param, TOYOTA_PARAM_SECOC);

  // TSS3 relies on an EPS-resident SecOC signer, development builds only for now
  const uint32_t TOYOTA_PARAM_TSS3 = 16UL << TOYOTA_PARAM_OFFSET;
  toyota_tss3 = GET_FLAG(param, TOYOTA_PARAM_TSS3);
#endif

  toyota_alt_brake = GET_FLAG(param, TOYOTA_PARAM_ALT_BRAKE);
  toyota_stock_longitudinal = GET_FLAG(param, TOYOTA_PARAM_STOCK_LONGITUDINAL);
  toyota_lta = GET_FLAG(param, TOYOTA_PARAM_LTA);
  toyota_tss3_08a_active = false;
  toyota_tss3_08a_last_tx_ts = 0U;
  toyota_tss3_request_next_fragment = 0U;
  toyota_tss3_last_request_ts = 0U;
  toyota_tss3_approved_idx = 0U;
  for (uint8_t i = 0U; i < TOYOTA_TSS3_APPROVED_LEN; i++) {
    toyota_tss3_approved_valid[i] = false;
  }
  toyota_tss3_stock_08a_last_ts = 0U;
  toyota_tss3_stock_08a_idx = 0U;
  toyota_tss3_stock_08a_cnt = 0U;
  toyota_dbc_eps_torque_factor = param & TOYOTA_EPS_FACTOR;

  safety_config ret;
  if (toyota_tss3) {
    SET_TX_MSGS(TOYOTA_TSS3_TX_MSGS_ARR, ret);
  } else if (toyota_secoc) {
    if (toyota_stock_longitudinal) {
      SET_TX_MSGS(TOYOTA_SECOC_TX_MSGS, ret);
    } else {
      SET_TX_MSGS(TOYOTA_SECOC_LONG_TX_MSGS, ret);
    }
  } else {
    if (toyota_stock_longitudinal) {
      SET_TX_MSGS(TOYOTA_TX_MSGS, ret);
    } else {
      SET_TX_MSGS(TOYOTA_LONG_TX_MSGS, ret);
    }
  }

  if (toyota_tss3) {
    static RxCheck toyota_tss3_rx_checks[] = {
      TOYOTA_TSS3_RX_CHECKS
    };

    SET_RX_CHECKS(toyota_tss3_rx_checks, ret);
  } else if (toyota_secoc) {
    static RxCheck toyota_secoc_rx_checks[] = {
      TOYOTA_SECOC_RX_CHECKS
    };

    SET_RX_CHECKS(toyota_secoc_rx_checks, ret);
  } else if (toyota_lta) {
    // Check the quality flag for angle measurement when using LTA, since it's not set on TSS-P cars
    static RxCheck toyota_lta_rx_checks[] = {
      TOYOTA_RX_CHECKS(true)
    };

    SET_RX_CHECKS(toyota_lta_rx_checks, ret);
  } else {
    static RxCheck toyota_lka_rx_checks[] = {
      TOYOTA_RX_CHECKS(false)
    };
    static RxCheck toyota_lka_alt_brake_rx_checks[] = {
      TOYOTA_ALT_BRAKE_RX_CHECKS(false)
    };

    if (!toyota_alt_brake) {
      SET_RX_CHECKS(toyota_lka_rx_checks, ret);
    } else {
      SET_RX_CHECKS(toyota_lka_alt_brake_rx_checks, ret);
    }
  }

  return ret;
}

const safety_hooks toyota_hooks = {
  .init = toyota_init,
  .rx = toyota_rx_hook,
  .tx = toyota_tx_hook,
  .fwd = toyota_fwd_hook,
  .get_checksum = toyota_get_checksum,
  .compute_checksum = toyota_compute_checksum,
  .get_quality_flag_valid = toyota_get_quality_flag_valid,
};
