#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"

// Leapmotor B10 (2025, global market). The front camera's ADAS_LKA (0x83) carries the LCC steering angle request;
// openpilot replaces it through a camera harness. Stock ACC stays in control of longitudinal, unless the longitudinal
// flag is set: then openpilot may also replace ADAS_LKA.ACC_ACCEL_REQ while stock ACC stays engaged.

#define LEAPMOTOR_ADAS_LKA          0x83U   // TX: steering angle request (stock: front camera)
#define LEAPMOTOR_VEHICLE_STATE     0xB6U   // RX from VCU: accelerator pedal, brake pressed
#define LEAPMOTOR_POWERTRAIN_STATE  0xB7U   // RX from VCU: stock ACC engaged; TX to the camera: stalk-up cancel only
#define LEAPMOTOR_WHEEL_SPEEDS      0x184U  // RX from ESC: wheel speeds
#define LEAPMOTOR_EPS_STEERING      0x18DU  // RX from EPS: steering angle

#define LEAPMOTOR_MAIN_BUS 0U
#define LEAPMOTOR_CAM_BUS  2U

// Steering angle is 0.02 deg per bit with the center at raw 32490. Raw rises to the right, so the angle is
// negated to match openpilot's convention (positive = left).
#define LEAPMOTOR_ANGLE_CENTER 32490

// ADAS_LKA.ACC_ACCEL_REQ: byte 4, 0.05 m/s^2 per bit, raw 100 = 0 m/s^2. Must match CarControllerParams.
#define LEAPMOTOR_ACCEL_INACTIVE 100
// The camera's request at or below this (-2.0 m/s^2) is passed on: openpilot may ask for more braking, not less
#define LEAPMOTOR_ACCEL_STOCK_BRAKE 60
#define LEAPMOTOR_FLAG_LONGITUDINAL_CONTROL 1U
// openpilot builds its frame from a camera frame that can be older than the latest one the panda has seen: on 6 log
// segments by 0 (88%), 1 (10%), 2 to 6 frames (0.4%). So the camera's last LEAPMOTOR_CAMERA_ACCEL_HISTORY requests
// (160 ms at 50 Hz) count.

static void leapmotor_reset_camera_accel(void) {
  for (uint8_t i = 0U; i < LEAPMOTOR_CAMERA_ACCEL_HISTORY; i++) {
    safety.leapmotor_camera_accel[i] = LEAPMOTOR_ACCEL_INACTIVE;
  }
  safety.leapmotor_camera_accel_idx = 0U;
}

// was this request among the camera's recent ones?
static bool leapmotor_is_camera_accel(int accel) {
  bool found = false;
  for (uint8_t i = 0U; i < LEAPMOTOR_CAMERA_ACCEL_HISTORY; i++) {
    found = found || (safety.leapmotor_camera_accel[i] == accel);
  }
  return found;
}

// the least braking (highest raw) among the camera's recent requests
static int leapmotor_camera_accel_max(void) {
  int max_accel = safety.leapmotor_camera_accel[0];
  for (uint8_t i = 1U; i < LEAPMOTOR_CAMERA_ACCEL_HISTORY; i++) {
    if (safety.leapmotor_camera_accel[i] > max_accel) {
      max_accel = safety.leapmotor_camera_accel[i];
    }
  }
  return max_accel;
}

#define LEAPMOTOR_E2E_COUNT 5U

typedef struct {
  uint32_t addr;
  uint8_t constants[16];
} LeapmotorE2EConstants;

static uint32_t leapmotor_get_checksum(const CANPacket_t *msg) {
  return (uint8_t)msg->data[0];
}

static uint8_t leapmotor_get_counter(const CANPacket_t *msg) {
  return (uint8_t)msg->data[1] & 0xFU;
}

// AUTOSAR E2E Profile 2, the same CRC as Volkswagen MQB/MEB, with a per-message constant chosen by the counter
static uint32_t leapmotor_compute_checksum(const CANPacket_t *msg) {
  static const LeapmotorE2EConstants E2E[LEAPMOTOR_E2E_COUNT] = {
    {LEAPMOTOR_WHEEL_SPEEDS, {0xD9U, 0x90U, 0x33U, 0xE1U, 0x3AU, 0x1AU, 0xDFU, 0xC6U, 0xB5U, 0xDCU, 0xEEU, 0x04U, 0xFAU, 0xD7U, 0xD4U, 0x99U}},
    {LEAPMOTOR_EPS_STEERING, {0x61U, 0x4BU, 0x17U, 0xE4U, 0x78U, 0x0BU, 0xA1U, 0xB4U, 0xBCU, 0xD8U, 0xEAU, 0xF5U, 0xD9U, 0x21U, 0xCAU, 0x86U}},
    {LEAPMOTOR_VEHICLE_STATE, {0x71U, 0x0DU, 0x0BU, 0xC2U, 0x08U, 0xC6U, 0x63U, 0x4BU, 0x3FU, 0x2BU, 0x06U, 0x10U, 0xF0U, 0xD2U, 0x87U, 0x0DU}},
    {LEAPMOTOR_POWERTRAIN_STATE, {0xEBU, 0x4EU, 0xAEU, 0xCEU, 0x79U, 0x7CU, 0x52U, 0x03U, 0x6AU, 0x84U, 0xC2U, 0x3AU, 0x0AU, 0x64U, 0x2FU, 0x17U}},
    {LEAPMOTOR_ADAS_LKA, {0xCDU, 0x81U, 0x0AU, 0xC1U, 0x99U, 0x7FU, 0x8DU, 0x05U, 0x3BU, 0xA2U, 0x43U, 0xD2U, 0xB6U, 0x72U, 0x2AU, 0xD9U}},
  };

  uint8_t constant = 0U;
  for (uint8_t i = 0U; i < LEAPMOTOR_E2E_COUNT; i++) {
    constant = (E2E[i].addr == msg->addr) ? E2E[i].constants[leapmotor_get_counter(msg)] : constant;
  }
  return volkswagen_compute_crc(msg, GET_LEN(msg), constant);
}

static void leapmotor_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == LEAPMOTOR_MAIN_BUS) {
    if (msg->addr == LEAPMOTOR_WHEEL_SPEEDS) {
      // four 16-bit little-endian wheel speeds at 0.01 km/h: FL, FR, RL, RR
      uint32_t sum = GET_BYTES_LE(msg, 2, 2) + GET_BYTES_LE(msg, 4, 2) + GET_BYTES_LE(msg, 6, 2) + GET_BYTES_LE(msg, 8, 2);
      safety.vehicle_moving = sum > 0U;
      UPDATE_VEHICLE_SPEED((sum / 4.0) * 0.01 * KPH_TO_MS);
    }

    if (msg->addr == LEAPMOTOR_EPS_STEERING) {
      int angle_meas_new = LEAPMOTOR_ANGLE_CENTER - (int)GET_BYTES_LE(msg, 3, 2);  // STEER_ANGLE
      update_sample(&safety.angle_meas, angle_meas_new);
    }

    if (msg->addr == LEAPMOTOR_VEHICLE_STATE) {
      safety.gas_pressed = msg->data[11] > 0U;   // ACCEL_PEDAL
      safety.brake_pressed = GET_BIT(msg, 99U);  // BRAKE_PRESSED
    }

    if (msg->addr == LEAPMOTOR_POWERTRAIN_STATE) {
      pcm_cruise_check(GET_BIT(msg, 30U));       // ASSIST_ENGAGED
    }
  }

  // only rx-checked frames reach this hook, and on the camera bus that is only ADAS_LKA (with the longitudinal flag)
  if (msg->bus == LEAPMOTOR_CAM_BUS) {
    safety.leapmotor_camera_accel[safety.leapmotor_camera_accel_idx] = msg->data[4];  // ACC_ACCEL_REQ
    safety.leapmotor_camera_accel_idx++;
    if (safety.leapmotor_camera_accel_idx >= LEAPMOTOR_CAMERA_ACCEL_HISTORY) {
      safety.leapmotor_camera_accel_idx = 0U;
    }
  }
}

static bool leapmotor_tx_hook(const CANPacket_t *msg) {
  // Must match LEAPMOTOR_B10 CarControllerParams.ANGLE_LIMITS
  const AngleSteeringLimits LEAPMOTOR_STEERING_LIMITS = {
    .max_angle = 25000,  // 500 deg; mechanical lock is about 522 deg
    .angle_deg_to_can = 50,
    .frequency = 50U,
  };

  const AngleSteeringParams LEAPMOTOR_STEERING_PARAMS = {
    .slip_factor = -0.0006132659333604452,  // calc_slip_factor(VM)
    .steer_ratio = 15.2,
    .wheelbase = 2.735,
  };

  bool tx = true;

  if (msg->addr == LEAPMOTOR_ADAS_LKA) {
    int desired_angle = LEAPMOTOR_ANGLE_CENTER - (int)GET_BYTES_LE(msg, 13, 2);  // STEER_ANGLE_REQ
    bool steer_req = GET_BIT(msg, 99U);                                          // LCC_STEERING_ACTIVE
    if (steer_angle_cmd_checks_vm(desired_angle, steer_req, LEAPMOTOR_STEERING_LIMITS, LEAPMOTOR_STEERING_PARAMS)) {
      tx = false;
    }

    if (safety_setup.leapmotor_longitudinal) {
      int accel = msg->data[4];  // ACC_ACCEL_REQ
      // The camera's own request may always be passed on: that is what the stock ACC sends when openpilot is not in
      // control. openpilot's own request must be within -3.5..2.0 m/s^2 (raw 30..140) with controls allowed. Not also
      // with the accelerator released, as longitudinal_accel_checks() wants: the panda sees a press one frame before
      // openpilot can hand back to the stock ACC, and a blocked ADAS_LKA cancels the stock ACC. The car follows the
      // pedal over any ACC request anyway, as with the camera's own requests, which it keeps sending for a few frames.
      bool own_accel_ok = safety.controls_allowed && !safety_max_limit_check(accel, 140, 30);
      bool inactive = accel == LEAPMOTOR_ACCEL_INACTIVE;
      if (!leapmotor_is_camera_accel(accel) && !own_accel_ok && !inactive) {
        tx = false;
      }
      // never less braking than the camera asks for while all its recent requests are hard braking
      int camera_floor = leapmotor_camera_accel_max();
      if ((camera_floor <= LEAPMOTOR_ACCEL_STOCK_BRAKE) && (accel > camera_floor)) {
        tx = false;
      }
    }
  }

  // The camera's copy of POWERTRAIN_STATE may only carry the stalk pushed up (STALK_PUSH = 3, cancel), while engaged
  if (msg->addr == LEAPMOTOR_POWERTRAIN_STATE) {
    bool cancel = (msg->data[5] & 0x3U) == 3U;
    tx = cancel && safety.cruise_engaged_prev;
  }

  return tx;
}

static safety_config leapmotor_init(uint16_t param) {
  safety_setup.leapmotor_longitudinal = GET_FLAG(param, LEAPMOTOR_FLAG_LONGITUDINAL_CONTROL);
  leapmotor_reset_camera_accel();

  static const CanMsg LEAPMOTOR_TX_MSGS[] = {
    {LEAPMOTOR_ADAS_LKA, LEAPMOTOR_MAIN_BUS, 32, .check_relay = true},           // ADAS_LKA (steering)
    {LEAPMOTOR_POWERTRAIN_STATE, LEAPMOTOR_CAM_BUS, 32, .check_relay = false},  // POWERTRAIN_STATE (cancel to camera)
  };

  static const RxCheck leapmotor_rx_checks[] = {
    {.msg = {{LEAPMOTOR_WHEEL_SPEEDS, LEAPMOTOR_MAIN_BUS, 32, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{LEAPMOTOR_EPS_STEERING, LEAPMOTOR_MAIN_BUS, 32, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{LEAPMOTOR_VEHICLE_STATE, LEAPMOTOR_MAIN_BUS, 16, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{LEAPMOTOR_POWERTRAIN_STATE, LEAPMOTOR_MAIN_BUS, 32, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  // with openpilot longitudinal, the camera's own ADAS_LKA is also checked: its acceleration request is passed on
  static const RxCheck leapmotor_long_rx_checks[] = {
    {.msg = {{LEAPMOTOR_WHEEL_SPEEDS, LEAPMOTOR_MAIN_BUS, 32, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{LEAPMOTOR_EPS_STEERING, LEAPMOTOR_MAIN_BUS, 32, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{LEAPMOTOR_VEHICLE_STATE, LEAPMOTOR_MAIN_BUS, 16, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{LEAPMOTOR_POWERTRAIN_STATE, LEAPMOTOR_MAIN_BUS, 32, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{LEAPMOTOR_ADAS_LKA, LEAPMOTOR_CAM_BUS, 32, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  safety_config ret;
  if (safety_setup.leapmotor_longitudinal) {
    ret = BUILD_SAFETY_CFG(leapmotor_long_rx_checks, LEAPMOTOR_TX_MSGS);
  } else {
    ret = BUILD_SAFETY_CFG(leapmotor_rx_checks, LEAPMOTOR_TX_MSGS);
  }
  return ret;
}

const safety_hooks leapmotor_hooks = {
  .init = leapmotor_init,
  .rx = leapmotor_rx_hook,
  .tx = leapmotor_tx_hook,
  .get_counter = leapmotor_get_counter,
  .get_checksum = leapmotor_get_checksum,
  .compute_checksum = leapmotor_compute_checksum,
};
