#pragma once

#include "safety_declarations.h"

#define BYD_CANADDR_IPB               0x1F0
#define BYD_CANADDR_ACC_MPC_STATE     0x316
#define BYD_CANADDR_ACC_EPS_STATE     0x318
#define BYD_CANADDR_ACC_HUD_ADAS      0x32D
#define BYD_CANADDR_ACC_CMD           0x32E
#define BYD_CANADDR_PCM_BUTTONS       0x3B0
#define BYD_CANADDR_DRIVE_STATE       0x242
#define BYD_CANADDR_PEDAL             0x342
#define BYD_CANADDR_CARSPEED          0x121

#define BYD_CANBUS_ESC  0
#define BYD_CANBUS_MRR  1
#define BYD_CANBUS_MPC  2

static bool byd_eps_cruiseactivated = false;

typedef enum {
  HAN_TANG_DMEV,
  TANG_DMI,
  SONG_PLUS_DMI,
  QIN_PLUS_DMI,
  YUAN_PLUS_DMI_ATTO3
} BydPlatform;
static BydPlatform byd_platform;

static void byd_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);
  if (bus == BYD_CANBUS_ESC) {
    if (addr == BYD_CANADDR_PEDAL) {
      gas_pressed = (GET_BYTE(to_push, 0) != 0U);
      brake_pressed = (GET_BYTE(to_push, 1) != 0U);
    } else if (addr == BYD_CANADDR_CARSPEED) {
      int speed_raw = (((GET_BYTE(to_push, 1) & 0x0FU) << 8) | GET_BYTE(to_push, 0));
      vehicle_moving = (speed_raw != 0);
    } else if (addr == BYD_CANADDR_ACC_EPS_STATE) {
      byd_eps_cruiseactivated = GET_BIT(to_push, 1U) != 0U; // CruiseActivated
      int torque_motor = (((GET_BYTE(to_push, 2) & 0x0FU) << 8) | GET_BYTE(to_push, 1)); // MainTorque
      if ( torque_motor >= 2048 )
        torque_motor -= 4096;
      update_sample(&torque_meas, torque_motor);
    }
    else {
      //empty
    }
  } else if (bus == BYD_CANBUS_MPC) {
    if (addr == BYD_CANADDR_ACC_HUD_ADAS) {
      unsigned int accstate = ((GET_BYTE(to_push, 2) >> 3) & 0x07U);
      bool cruise_engaged = (accstate == 3U) || (accstate == 5U); // 3=acc_active, 5=user force accel
      pcm_cruise_check(cruise_engaged);
    }
  }
  else {
    //empty
  }
}


static bool byd_tx_hook(const CANPacket_t *to_send) {
  const TorqueSteeringLimits HAN_DMEV_STEERING_LIMITS = {
    .max_torque = 300,
    .max_rate_up = 17,
    .max_rate_down = 17,
    .max_torque_error = 80,
    .max_rt_delta = 243,
    .type = TorqueMotorLimited,
  };
  const TorqueSteeringLimits TANG_DMI_STEERING_LIMITS = { //values to be check
    .max_torque = 300,
    .max_rate_up = 17,
    .max_rate_down = 17,
    .max_rt_delta = 243,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };
  const TorqueSteeringLimits SONG_STEERING_LIMITS = { //values to be check
    .max_torque = 300,
    .max_rate_up = 17,
    .max_rate_down = 17,
    .max_rt_delta = 243,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };
  const TorqueSteeringLimits QIN_STEERING_LIMITS = { //values to be check
    .max_torque = 300,
    .max_rate_up = 17,
    .max_rate_down = 17,
    .max_rt_delta = 243,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };
  const TorqueSteeringLimits YUAN_ATTO3_STEERING_LIMITS = { //values to be check
    .max_torque = 300,
    .max_rate_up = 17,
    .max_rate_down = 17,
    .max_rt_delta = 243,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };

  bool tx = true;
  int bus = GET_BUS(to_send);

  if (bus == BYD_CANBUS_ESC) {
    int addr = GET_ADDR(to_send);
    if (addr == BYD_CANADDR_ACC_MPC_STATE) {
      int desired_torque = ((GET_BYTE(to_send, 3) & 0x07U) << 8U) | GET_BYTE(to_send, 2);
      bool steer_req = GET_BIT(to_send, 28U) && byd_eps_cruiseactivated; //LKAS_Active
      if ( desired_torque >= 1024 )
        desired_torque -= 2048;
      const TorqueSteeringLimits limits = (byd_platform == HAN_TANG_DMEV) ? HAN_DMEV_STEERING_LIMITS :
                                          (byd_platform == TANG_DMI) ? TANG_DMI_STEERING_LIMITS :
                                          (byd_platform == SONG_PLUS_DMI) ? SONG_STEERING_LIMITS :
                                          (byd_platform == QIN_PLUS_DMI) ? QIN_STEERING_LIMITS : YUAN_ATTO3_STEERING_LIMITS;

      if (steer_torque_cmd_checks(desired_torque, steer_req, limits)) {
        tx = false;
      }
    }

  }

  return tx;
}

static bool byd_fwd_hook(int bus, int addr) {
  bool block_msg = false;

  const bool is_lkas = ((addr == BYD_CANADDR_ACC_MPC_STATE) || (addr == BYD_CANADDR_ACC_CMD));
  const bool is_eps = (addr == BYD_CANADDR_ACC_EPS_STATE);

  if ( ((bus == BYD_CANBUS_ESC) && is_eps) || ((bus == BYD_CANBUS_MPC) && is_lkas) ) {
    block_msg = true;
  }

  return block_msg;
}

static safety_config byd_init(uint16_t param) {

  const uint32_t FLAG_HAN_TANG_DMEV = 0x1U;
  const uint32_t FLAG_TANG_DMI = 0x2U;
  const uint32_t FLAG_SONG_PLUS_DMI = 0x4U;
  const uint32_t FLAG_QIN_PLUS_DMI = 0x8U;
  const uint32_t FLAG_YUAN_PLUS_DMI_ATTO3 = 0x10U;

  static const CanMsg BYD_HAN_DMEV_TX_MSGS[] = {
    {BYD_CANADDR_ACC_CMD,         BYD_CANBUS_ESC, 8, false},
    {BYD_CANADDR_ACC_MPC_STATE,   BYD_CANBUS_ESC, 8, true},
    {BYD_CANADDR_ACC_EPS_STATE,   BYD_CANBUS_MPC, 8, false},
  };

  static const CanMsg BYD_YUANPLUS_ATTO3_TX_MSGS[] = {
    {BYD_CANADDR_ACC_CMD,         BYD_CANBUS_ESC, 8, false},
    {BYD_CANADDR_ACC_MPC_STATE,   BYD_CANBUS_ESC, 8, true},
    {BYD_CANADDR_ACC_EPS_STATE,   BYD_CANBUS_MPC, 8, false},
  };

  static RxCheck byd_han_dmev_rx_checks[] = {
    {.msg = {{BYD_CANADDR_ACC_EPS_STATE,    BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_CARSPEED,         BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_IPB,              BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_DRIVE_STATE,      BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_ACC_HUD_ADAS,     BYD_CANBUS_MPC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
  };

  // static RxCheck byd_yuanplus_atto3_rx_checks[] = {
  //   {.msg = {{BYD_CANADDR_ACC_EPS_STATE,    BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
  //   {.msg = {{BYD_CANADDR_IPB,              BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
  //   {.msg = {{BYD_CANADDR_DRIVE_STATE,      BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
  // };

  safety_config ret;

  bool use_han_dm = GET_FLAG(param, FLAG_HAN_TANG_DMEV);
  bool use_tang_dmi = GET_FLAG(param, FLAG_TANG_DMI);
  bool use_song = GET_FLAG(param, FLAG_SONG_PLUS_DMI);
  bool use_qin = GET_FLAG(param, FLAG_QIN_PLUS_DMI);
  bool use_yuan = GET_FLAG(param, FLAG_YUAN_PLUS_DMI_ATTO3);

  if(use_han_dm) {
    byd_platform = HAN_TANG_DMEV;
    ret = BUILD_SAFETY_CFG(byd_han_dmev_rx_checks, BYD_HAN_DMEV_TX_MSGS);
  } else if (use_tang_dmi || use_song || use_qin) {
    byd_platform = TANG_DMI;
    ret = BUILD_SAFETY_CFG(byd_han_dmev_rx_checks, BYD_HAN_DMEV_TX_MSGS);
  } else if (use_yuan) {
    byd_platform = YUAN_PLUS_DMI_ATTO3;
    static RxCheck byd_yuanplus_atto3_rx_checks[] = {
      {.msg = {{BYD_CANADDR_ACC_EPS_STATE,    BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
      {.msg = {{BYD_CANADDR_IPB,              BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
      {.msg = {{BYD_CANADDR_DRIVE_STATE,      BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    }; //why should I write such an ugly code just to comply with misra?
    ret = BUILD_SAFETY_CFG(byd_yuanplus_atto3_rx_checks, BYD_YUANPLUS_ATTO3_TX_MSGS);
  } else {
    //should not reach here
    byd_platform = HAN_TANG_DMEV;
    ret = BUILD_SAFETY_CFG(byd_han_dmev_rx_checks, BYD_HAN_DMEV_TX_MSGS);
  }

  return ret;
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .fwd = byd_fwd_hook,
};
