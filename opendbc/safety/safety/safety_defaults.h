#pragma once

#include "safety_declarations.h"

void default_rx_hook(const CANPacket_t *to_push) {
  UNUSED(to_push);
}

// *** no output safety mode ***

static safety_config nooutput_init(uint16_t param) {
  UNUSED(param);
  // TODO: just use a bool for this
  static const FwdBus fwd_buses[] = {0};
  return (safety_config){NULL, 0, NULL, 0, fwd_buses, 0};
}

// GCOV_EXCL_START
// Unreachable by design (doesn't define any tx msgs)
static bool nooutput_tx_hook(const CANPacket_t *to_send) {
  UNUSED(to_send);
  return false;
}
// GCOV_EXCL_STOP

const safety_hooks nooutput_hooks = {
  .init = nooutput_init,
  .rx = default_rx_hook,
  .tx = nooutput_tx_hook,
};

// *** all output safety mode ***
static safety_config alloutput_init(uint16_t param) {
  // Enables passthrough mode where relay is open and bus 0 gets forwarded to bus 2 and vice versa
  const uint16_t ALLOUTPUT_PARAM_PASSTHROUGH = 1;
  controls_allowed = true;
  bool alloutput_passthrough = GET_FLAG(param, ALLOUTPUT_PARAM_PASSTHROUGH);

  safety_config ret = {NULL, 0, NULL, 0};
  if (!alloutput_passthrough) {
    // TODO: just use a bool for this
    static const FwdBus fwd_buses[] = {0};
    ret.fwd_buses = fwd_buses;
    ret.fwd_buses_len = 0;
  }
  return ret;
}

static bool alloutput_tx_hook(const CANPacket_t *to_send) {
  UNUSED(to_send);
  return true;
}

const safety_hooks alloutput_hooks = {
  .init = alloutput_init,
  .rx = default_rx_hook,
  .tx = alloutput_tx_hook,
};
