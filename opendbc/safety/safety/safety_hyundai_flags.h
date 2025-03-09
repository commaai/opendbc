#pragma once

// common state
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

void hyundai_common_flags(uint16_t param) {
  const int HYUNDAI_PARAM_EV_GAS = 1;
  const int HYUNDAI_PARAM_HYBRID_GAS = 2;
  const int HYUNDAI_PARAM_CAMERA_SCC = 8;
  const int HYUNDAI_PARAM_CANFD_LKA_STEERING = 16;  // TODO: rename for support with CAN/CAN FD Blended platforms
  const int HYUNDAI_PARAM_FCEV_GAS = 256;

  hyundai_ev_gas_signal = GET_FLAG(param, HYUNDAI_PARAM_EV_GAS);
  hyundai_hybrid_gas_signal = !hyundai_ev_gas_signal && GET_FLAG(param, HYUNDAI_PARAM_HYBRID_GAS);
  hyundai_camera_scc = GET_FLAG(param, HYUNDAI_PARAM_CAMERA_SCC);
  hyundai_canfd_lka_steering = GET_FLAG(param, HYUNDAI_PARAM_CANFD_LKA_STEERING);
  hyundai_fcev_gas_signal = GET_FLAG(param, HYUNDAI_PARAM_FCEV_GAS);

#ifdef ALLOW_DEBUG
  const int HYUNDAI_PARAM_LONGITUDINAL = 4;
  hyundai_longitudinal = GET_FLAG(param, HYUNDAI_PARAM_LONGITUDINAL);
#else
  hyundai_longitudinal = false;
#endif
}
