#pragma once

#include "opendbc/safety/declarations.h"
// #include "opendbc/safety/modes/honda.h"

// Minimal safety mode for external panda controlling steering and lkas hud on separate bus
// This mode only validates essential RX messages and allows TX of steering message
// Blocks LKAS_HUD and STEERING_CONTROL signals from bus 0↔2 forwarding to prevent conflicts

static safety_config rlx_redpanda_init(uint16_t param) {
  // TX messages for external panda: steer and lkas hud only
  static const CanMsg RLX_REDPANDA_TX_MSGS[] = {
    {0x194, 0, 4, .check_relay = false},  // steering_control
    {0x33D, 0, 5, .check_relay = false},  // lkas_hud
  };

  (void) param; // ignore param
  
  safety_config ret;
  
  static RxCheck honda_nidec_alt_rx_checks[] = {
    // HONDA_COMMON_NO_SCM_FEEDBACK_RX_CHECKS(0)
    {.msg = {{0x1FA, 2, 8, 50U, .max_counter = 3U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // BRAKE_COMMAND
  };

  SET_RX_CHECKS(honda_nidec_alt_rx_checks, ret);
  SET_TX_MSGS(RLX_REDPANDA_TX_MSGS, ret);

  return ret;
}

static bool rlx_redpanda_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  // Block LKAS_HUD and STEERING_CONTROL signals from bus 0↔2 forwarding on external panda
  // This prevents stock messages from camera (bus 6/physical 2) reaching powertrain (bus 4/physical 0)
  if (((bus_num == 0) || (bus_num == 2)) &&
      ((addr == 0x33D) || (addr == 0x194) )) {
    block_msg = true;
  }

  return block_msg;
}

static void rlx_redpanda_rx_hook(const CANPacket_t *msg) {
  // common RX only
  // controls allowed from internal panda per include
  (void) msg; // ignore msg
}

static bool rlx_redpanda_tx_hook(const CANPacket_t *msg) {

  bool tx = true;

  return tx;
}

static uint32_t honda_get_rlxpanda_checksum(const CANPacket_t *msg) {
  int checksum_byte = GET_LEN(msg) - 1U;
  return (uint8_t)(msg->data[checksum_byte]) & 0xFU;
}

static uint32_t honda_compute_rlxpanda_checksum(const CANPacket_t *msg) {
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

static uint8_t honda_get_rlxpanda_counter(const CANPacket_t *msg) {
  int counter_byte = GET_LEN(msg) - 1U;
  return (msg->data[counter_byte] >> 4U) & 0x3U;
}


const safety_hooks honda_rlx_redpanda_hooks = {
  .init = rlx_redpanda_init,
  .rx = rlx_redpanda_rx_hook,
  .tx = rlx_redpanda_tx_hook,
  .fwd = rlx_redpanda_fwd_hook,
  .get_counter = honda_get_rlxpanda_counter,
  .get_checksum =  honda_get_rlxpanda_checksum,
  .compute_checksum =  honda_compute_rlxpanda_checksum,
};
