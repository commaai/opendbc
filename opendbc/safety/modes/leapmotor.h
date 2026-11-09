#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"

// Leapmotor B10 (2025, global market). The front camera's ADAS_LKA (0x83) carries the LCC steering angle request;
// openpilot replaces it through a camera harness. Stock ACC stays in control of longitudinal.

#define LEAPMOTOR_ADAS_LKA          0x83U   // TX: steering angle request (stock: front camera)
#define LEAPMOTOR_VEHICLE_STATE     0xB6U   // RX from VCU: accelerator pedal, brake pressed
#define LEAPMOTOR_POWERTRAIN_STATE  0xB7U   // RX from VCU: stock ACC engaged; TX to the camera: stalk-up cancel only
#define LEAPMOTOR_WHEEL_SPEEDS      0x184U  // RX from ESC: wheel speeds
#define LEAPMOTOR_EPS_STEERING      0x18DU  // RX from EPS: steering angle

#define LEAPMOTOR_MAIN_BUS 0U
#define LEAPMOTOR_CAM_BUS  2U

// Steering angle is 0.02 deg per bit with the centre at raw 32490. Raw rises to the right, so the angle is
// negated to match openpilot's convention (positive = left).
#define LEAPMOTOR_ANGLE_CENTER 32490

#define LEAPMOTOR_E2E_COUNT 4U

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
  }

  // The camera's copy of POWERTRAIN_STATE may only carry the stalk pushed up (STALK_PUSH = 3, cancel), while engaged
  if (msg->addr == LEAPMOTOR_POWERTRAIN_STATE) {
    bool cancel = (msg->data[5] & 0x3U) == 3U;
    tx = cancel && safety.cruise_engaged_prev;
  }

  return tx;
}

static safety_config leapmotor_init(uint16_t param) {
  SAFETY_UNUSED(param);

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
