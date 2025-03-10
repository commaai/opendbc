#pragma once

enum {
  HYUNDAI_PARAM_EV_GAS = 1,
  HYUNDAI_PARAM_HYBRID_GAS = 2,
#ifdef ALLOW_DEBUG
  HYUNDAI_PARAM_LONGITUDINAL = 4,
#endif
  HYUNDAI_PARAM_CAMERA_SCC = 8,
  HYUNDAI_PARAM_CANFD_LKA_STEERING = 16,  // TODO: rename for support with CAN/CAN FD Blended platforms
  HYUNDAI_PARAM_CANFD_ALT_BUTTONS = 32,
  HYUNDAI_PARAM_ALT_LIMITS = 64,
  HYUNDAI_PARAM_CANFD_LKA_STEERING_ALT = 128,
  HYUNDAI_PARAM_FCEV_GAS = 256,
  HYUNDAI_PARAM_ALT_LIMITS_2 = 512,
};

// common flags
extern bool hyundai_ev_gas_signal;
bool hyundai_ev_gas_signal = false;

extern bool hyundai_hybrid_gas_signal;
bool hyundai_hybrid_gas_signal = false;

extern bool hyundai_longitudinal;
bool hyundai_longitudinal = false;

extern bool hyundai_camera_scc;
bool hyundai_camera_scc = false;

extern bool hyundai_canfd_lka_steering;
bool hyundai_canfd_lka_steering = false;

extern bool hyundai_fcev_gas_signal;
bool hyundai_fcev_gas_signal = false;

// shared flags for non CAN FD cars
extern bool hyundai_alt_limits;
bool hyundai_alt_limits = false;

extern bool hyundai_alt_limits_2;
bool hyundai_alt_limits_2 = false;

// shared flags for CAN FD cars
extern bool hyundai_canfd_alt_buttons;
bool hyundai_canfd_alt_buttons = false;

extern bool hyundai_canfd_lka_steering_alt;
bool hyundai_canfd_lka_steering_alt = false;

void hyundai_common_flags(uint16_t param) {
  hyundai_ev_gas_signal = GET_FLAG(param, HYUNDAI_PARAM_EV_GAS);
  hyundai_hybrid_gas_signal = GET_FLAG(param, HYUNDAI_PARAM_HYBRID_GAS);
  hyundai_camera_scc = GET_FLAG(param, HYUNDAI_PARAM_CAMERA_SCC);
  hyundai_canfd_lka_steering = GET_FLAG(param, HYUNDAI_PARAM_CANFD_LKA_STEERING);
  hyundai_fcev_gas_signal = GET_FLAG(param, HYUNDAI_PARAM_FCEV_GAS);

#ifdef ALLOW_DEBUG
  hyundai_longitudinal = GET_FLAG(param, HYUNDAI_PARAM_LONGITUDINAL);
#else
  hyundai_longitudinal = false;
#endif
}

void hyundai_flags(uint16_t param) {
  hyundai_common_flags(param);

  hyundai_alt_limits = GET_FLAG(param, HYUNDAI_PARAM_ALT_LIMITS);
  hyundai_alt_limits_2 = GET_FLAG(param, HYUNDAI_PARAM_ALT_LIMITS_2);
}

void hyundai_canfd_flags(uint16_t param) {
  hyundai_common_flags(param);

  hyundai_canfd_lka_steering_alt = GET_FLAG(param, HYUNDAI_PARAM_CANFD_LKA_STEERING_ALT);
  hyundai_canfd_alt_buttons = GET_FLAG(param, HYUNDAI_PARAM_CANFD_ALT_BUTTONS);
}
