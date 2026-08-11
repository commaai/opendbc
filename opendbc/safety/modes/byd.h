// BYD ATTO3 Safety Implementation
//
// Bus layout: 0 = car chassis CAN, 1 = camera/radar private CAN, 2 = camera ADAS CAN.
// openpilot transmits STEERING_MODULE_ADAS and LKAS_HUD_ADAS on bus 0 only while it is
// actively steering. The camera's copies are forwarded 2 -> 0 normally, so the stock LKAS
// keeps the wheel whenever openpilot is quiet; each is blocked only while openpilot is
// transmitting it, so the EPS never sees two sources at once and there is never a gap with
// neither. The HUD is cosmetic — it exists because the camera drops its own LKAS within
// seconds of us blocking its steering command, which would otherwise leave the cluster
// showing LKAS off while openpilot is steering.
//
// STEERING_MODULE_ADAS.STEER_ANGLE is an absolute steering wheel angle target, so this
// mode uses the angle checks (speed-dependent rate limits + inactive-angle tracking),
// not the torque checks.

#pragma once

#include "opendbc/safety/declarations.h"

// CAN message IDs (verified against byd_general.dbc)
#define BYD_STEERING_TORQUE      0x1FCU  // 508  - MAIN_TORQUE from EPS, bus 0
#define BYD_STEER_MODULE_2       0x11FU  // 287  - STEER_ANGLE_2 + DRIVER_EPS_TORQUE, bus 0
#define BYD_WHEEL_SPEED          0x122U  // 290  - per-wheel speeds, bus 0
#define BYD_PEDAL                0x342U  // 834  - GAS_PEDAL/BRAKE_PEDAL, bus 0
#define BYD_DRIVE_STATE          0x242U  // 578  - GEAR, bus 0
#define BYD_STEERING_MODULE_ADAS 0x1E2U  // 482  - LKAS steering command
#define BYD_LKAS_HUD_ADAS        0x316U  // 790  - LKAS HUD
#define BYD_ACC_HUD_ADAS         0x32DU  // 813  - ACC main/standby state, camera on bus 2
#define BYD_ACC_CMD              0x32EU  // 814  - ACC command; CMD_REQ_ACTIVE_LOW is the real engaged flag

// STEER_ANGLE has DBC factor 0.1 deg, so 1 physical degree is 10 raw units.
#define BYD_DEG_TO_CAN 10.0f

// BRAKE_PEDAL raw threshold. This is the only signal that tracks the driver's foot:
// DRIVE_STATE.BRAKE_PRESSED is dead on this platform (constant 0) and
// PEDAL_PRESSED_ACTIVE_LOW is the brake-light switch, which ACC also asserts.
#define BYD_BRAKE_THRESHOLD 3U

// How long the camera's copy stays blocked after openpilot sends one. Longer than a few
// missed frames at 50Hz, short enough to hand back promptly on disengage.
#define BYD_TAKEOVER_TIMEOUT 150000U  // us

static bool byd_steering = false;      // openpilot currently owns the EPS
static uint32_t byd_last_steer_tx = 0U;

// Tracked separately from byd_steering on purpose: the HUD carries no safety checks, so a
// HUD frame must never be able to keep the camera's *steering* command blocked.
static bool byd_hud = false;           // openpilot currently owns the LKAS HUD
static uint32_t byd_last_hud_tx = 0U;

// BYD's checksum: nibble sums of every byte but the last, folded with a fixed key.
static uint32_t byd_compute_checksum(const CANPacket_t *msg) {
  const uint8_t key = 0xAFU;
  int len = GET_LEN(msg);

  uint8_t high_sum = 0U;
  uint8_t low_sum = 0U;
  for (int i = 0; i < (len - 1); i++) {
    high_sum += (uint8_t)(msg->data[i] >> 4);
    low_sum += (uint8_t)(msg->data[i] & 0xFU);
  }

  // the remainder is taken before the key nibbles are folded in
  const uint8_t remainder = (uint8_t)(low_sum >> 4);
  high_sum += (uint8_t)(key & 0xFU);
  low_sum += (uint8_t)(key >> 4);

  const uint8_t high_part = (uint8_t)((0x9U - high_sum) & 0xFU);
  const uint8_t low_part = (uint8_t)((0x9U - low_sum) & 0xFU);
  const uint8_t folded = (uint8_t)((high_part + 5U) - remainder);

  return (uint32_t)((uint8_t)(((uint32_t)folded * 16U) + (uint32_t)low_part));
}

static uint32_t byd_get_checksum(const CANPacket_t *msg) {
  return (uint32_t)msg->data[GET_LEN(msg) - 1U];
}

static uint8_t byd_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0U;
  if ((msg->addr == BYD_ACC_HUD_ADAS) || (msg->addr == BYD_ACC_CMD)) {
    cnt = msg->data[6] & 0xFU;          // COUNTER : 48|4@1+
  } else {
    cnt = (uint8_t)(msg->data[6] >> 4); // COUNTER : 55|4@0+
  }
  return cnt;
}

static safety_config byd_init(uint16_t param) {
  // openpilot transmits both of these on bus 0. check_relay keeps stuck-relay detection,
  // but static blocking is disabled so byd_fwd_hook can decide per-frame whether the
  // camera still owns the EPS and the HUD.
  static const CanMsg BYD_TX_MSGS[] = {
    {(int)BYD_STEERING_MODULE_ADAS, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {(int)BYD_LKAS_HUD_ADAS,        0, 8, .check_relay = true, .disable_static_blocking = true},
  };

  // STEER_MODULE_2, WHEEL_SPEED and DRIVE_STATE carry no checksum or counter in the DBC.
  static RxCheck byd_rx_checks[] = {
    {.msg = {{(int)BYD_STEERING_TORQUE, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{(int)BYD_STEER_MODULE_2,  0, 5, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{(int)BYD_WHEEL_SPEED,     0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{(int)BYD_PEDAL,           0, 8,  50U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{(int)BYD_DRIVE_STATE,     0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{(int)BYD_ACC_CMD,         2, 8,  50U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},
  };

  // Longitudinal is stock-only: ACC_CMD is deliberately absent from the TX allowlist, so
  // panda blocks it outright. Enabling it needs the ACCEL_CMD scaling calibrated on the car.
  SAFETY_UNUSED(param);
  byd_steering = false;
  byd_last_steer_tx = 0U;
  byd_hud = false;
  byd_last_hud_tx = 0U;

  safety_config ret;
  SET_TX_MSGS(BYD_TX_MSGS, ret);
  SET_RX_CHECKS(byd_rx_checks, ret);
  return ret;
}

static void byd_rx_hook(const CANPacket_t *msg) {
  const uint32_t addr = msg->addr;

  if (msg->bus == 0U) {
    // Driver column torque and the measured wheel angle both live in STEER_MODULE_2.
    // DRIVER_EPS_TORQUE is the column sensor (unsigned magnitude); MAIN_TORQUE in
    // STEERING_TORQUE is EPS motor output and must not be used for driver override.
    if (addr == BYD_STEER_MODULE_2) {
      int angle_meas_new = (int)GET_BYTES(msg, 0, 2);      // STEER_ANGLE_2 : 0|16@1-
      angle_meas_new = to_signed(angle_meas_new, 16);
      update_sample(&angle_meas, angle_meas_new);

      int torque_driver_new = (int)msg->data[2];           // DRIVER_EPS_TORQUE : 16|8@1+
      update_sample(&torque_driver, torque_driver_new);
    }

    // Sum the three trustworthy wheels; WHEELSPEED_BR's high byte is a status byte.
    if (addr == BYD_WHEEL_SPEED) {
      int speed = 0;
      for (uint8_t i = 0U; i < 6U; i += 2U) {
        speed += (int)GET_BYTES(msg, i, 2);
      }
      vehicle_moving = speed != 0;
      // DBC factor 0.1 gives km/h; 40/53 corrects this platform's raw scaling.
      UPDATE_VEHICLE_SPEED(((float)speed / 3.0f) * 0.1f * (40.0f / 53.0f) * KPH_TO_MS);
    }

    if (addr == BYD_PEDAL) {
      gas_pressed = msg->data[0] > 10U;                    // GAS_PEDAL : 0|8@1+
      brake_pressed = msg->data[1] > BYD_BRAKE_THRESHOLD;  // BRAKE_PEDAL : 8|8@1+
    }
  }

  // ACC engagement — sets controls_allowed on the rising edge of the stock ACC actively
  // commanding. ACC_HUD_ADAS.ACC_ON1/ON2 are only the main-switch/standby state (they stay
  // set while the driver brakes and drives manually); the real engaged flag is in ACC_CMD:
  // ACC_ON_1 : 9|1@0+, ACC_ON_2 : 17|1@0+, CMD_REQ_ACTIVE_LOW : 36|1@0+ (low = commanding).
  if ((msg->bus == 2U) && (addr == BYD_ACC_CMD)) {
    bool cruise_engaged = GET_BIT(msg, 9U) && GET_BIT(msg, 17U) && !GET_BIT(msg, 36U);
    pcm_cruise_check(cruise_engaged);
  }
}

static bool byd_tx_hook(const CANPacket_t *msg) {
  // Rate limits are per CAN message at 50Hz and mirror CarControllerParams.ANGLE_LIMITS,
  // with the +1 raw tolerance steer_angle_cmd_checks adds internally.
  const AngleSteeringLimits BYD_ANGLE_STEERING_LIMITS = {
    .max_angle = 900,  // 90 deg; the EPS faults past this
    .angle_deg_to_can = BYD_DEG_TO_CAN,
    .angle_rate_up_lookup = {
      {0., 5., 25.},
      {2.5, 1.5, 0.4}
    },
    .angle_rate_down_lookup = {
      {0., 5., 25.},
      {2.5, 1.5, 0.6}
    },
    .frequency = 50U,
  };

  bool tx = true;

  if (msg->bus == 0U) {
    const uint32_t addr = msg->addr;
    // STEERING_MODULE_ADAS: STEER_ANGLE 24|16@1- (bytes 3-4), STEER_REQ bit 21
    if (addr == BYD_STEERING_MODULE_ADAS) {
      int desired_angle = (int)GET_BYTES(msg, 3, 2);
      desired_angle = to_signed(desired_angle, 16);
      bool steer_control_enabled = GET_BIT(msg, 21U);

      if (steer_angle_cmd_checks(desired_angle, steer_control_enabled, BYD_ANGLE_STEERING_LIMITS)) {
        tx = false;
      }
    }

  }

  if (tx && (msg->bus == 0U)) {
    if (msg->addr == BYD_STEERING_MODULE_ADAS) {
      byd_steering = true;
      byd_last_steer_tx = microsecond_timer_get();
    }
    if (msg->addr == BYD_LKAS_HUD_ADAS) {
      byd_hud = true;
      byd_last_hud_tx = microsecond_timer_get();
    }
  }

  return tx;
}

// Hand each message back to the camera whenever openpilot stops transmitting it.
static bool byd_expired(bool owned, uint32_t last_tx) {
  return owned && (safety_get_ts_elapsed(microsecond_timer_get(), last_tx) >= BYD_TAKEOVER_TIMEOUT);
}

static bool byd_fwd_hook(int bus_num, int addr) {
  bool blocked = false;
  if (bus_num == 2) {
    if ((uint32_t)addr == BYD_STEERING_MODULE_ADAS) {
      if (byd_expired(byd_steering, byd_last_steer_tx)) {
        byd_steering = false;
      }
      blocked = byd_steering;
    } else if ((uint32_t)addr == BYD_LKAS_HUD_ADAS) {
      if (byd_expired(byd_hud, byd_last_hud_tx)) {
        byd_hud = false;
      }
      blocked = byd_hud;
    } else {
      // everything else the camera sends forwards untouched
    }
  }
  return blocked;
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .fwd = byd_fwd_hook,
  .get_checksum = byd_get_checksum,
  .compute_checksum = byd_compute_checksum,
  .get_counter = byd_get_counter,
};
