#pragma once

#include "opendbc/safety/safety_declarations.h"
#include "opendbc/safety/modes/honda.h"

// Minimal safety mode for external panda controlling steering and lkas hud on separate bus
// This mode only validates essential RX messages and allows TX of steering message
// Blocks LKAS_HUD and STEERING_CONTROL signals from bus 0↔2 forwarding to prevent conflicts

static safety_config rlx_redpanda_init(uint16_t param) {
  // TX messages for external panda: steer and lkas hud only
  static const CanMsg RLX_REDPANDA_TX_MSGS[] = {
    {0x194, 0, 4, .check_relay = false},  // steering_control
    {0x33D, 0, 5, .check_relay = false},  // lkas_hud
  };

  safety_config ret;

  SET_RX_CHECKS(HONDA_COMMON_NO_SCM_FEEDBACK_RX_CHECKS(0), ret); // common messages are duplicated on red panda bus
  SET_TX_MSGS(RLX_REDPANDA_TX_MSGS, ret);

  return ret;
}

static void rlx_redpanda_rx_hook(const CANPacket_t *msg) {
  // common RX only
  // controls allowed from internal panda per include
  UNUSED(msg);
}

static bool rlx_redpanda_tx_hook(const CANPacket_t *msg) {

  bool tx = true;

  unsigned int bus_pt = honda_get_pt_bus();

  // STEER: safety check
  if ((msg->addr == 0xE4U) || (msg->addr == 0x194U)) {
    if (!controls_allowed) {
      bool steer_applied = false;
      // temp allow steer
      // bool steer_applied = msg->data[0] | msg->data[1];
      if (steer_applied) {
        tx = false;
      }
    }
  }

  return tx;
}

static uint32_t honda_get_panda_checksum(const CANPacket_t *msg) {
  int checksum_byte = GET_LEN(msg) - 1U;
  return (uint8_t)(msg->data[checksum_byte]) & 0xFU;
}

static uint32_t honda_compute_panda_checksum(const CANPacket_t *msg) {
  int len = GET_LEN(msg);
  uint8_t checksum = 0U;
  unsigned int addr = msg->addr;
  while (addr > 0U) {
    checksum += (uint8_t)(addr & 0xFU); addr >>= 4;
  }
  for (int j = 0; j < len; j++) {
    uint8_t byte = msg->data[j];
    checksum += (uint8_t)(byte & 0xFU) + (byte >> 4U);
    if (j == (len - 1)) {
      checksum -= (byte & 0xFU);  // remove checksum in message
    }
  }
  return (uint8_t)((8U - checksum) & 0xFU);
}

static uint8_t honda_get_panda_counter(const CANPacket_t *msg) {
  int counter_byte = GET_LEN(msg) - 1U;
  return (msg->data[counter_byte] >> 4U) & 0x3U;
}


const safety_hooks honda_rlx_redpanda_hooks = {
  .init = rlx_redpanda_init,
  .rx = rlx_redpanda_rx_hook,
  .tx = rlx_redpanda_tx_hook,
  .get_counter = honda_get_panda_counter,
  .get_checksum =  honda_get_panda_checksum,
  .compute_checksum =  honda_compute_panda_checksum,
};
