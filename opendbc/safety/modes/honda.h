#pragma once

#include "opendbc/safety/declarations.h"
// #include "opendbc/safety/modes/honda.h"

// Minimal safety mode for internal panda controlling gas and brake, separately from other steering bus
// This mode only validates essential RX messages and allows TX of gas/brake message, more validations to return after successful
// Blocks BRAKE_COMMAND and ACC_HUD signals from bus 0↔2 forwarding to prevent conflicts

static bool honda_alt_brake_msg = false;
static bool honda_bosch_long = false;
static bool honda_fwd_brake = false;
typedef enum {HONDA_NIDEC, HONDA_BOSCH} HondaHw;
static HondaHw honda_hw = HONDA_NIDEC;

static unsigned int honda_get_pt_bus(void) {
  return 0U;
}

static safety_config rlx_internal_init(uint16_t param) {
  // TX messages for internal panda: gas and brake only
  static const CanMsg RLX_INTERNAL_TX_MSGS[] = {
    {0x30C, 0, 8, .check_relay = false},  // ACC_HUD
    {0x1FA, 0, 8, .check_relay = false},  // BRAKE_COMMAND
  };

  (void) param; // ignore param
  (void) honda_hw; // ignore param
  
  safety_config ret;
  
  static RxCheck honda_nidec_alt_rx_checks[] = {
    // HONDA_COMMON_NO_SCM_FEEDBACK_RX_CHECKS(0)
    {.msg = {{0x1FA, 2, 8, 50U, .max_counter = 3U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // BRAKE_COMMAND
  };

  SET_RX_CHECKS(honda_nidec_alt_rx_checks, ret);
  SET_TX_MSGS(RLX_INTERNAL_TX_MSGS, ret);

  return ret;
}

static bool rlx_internal_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  // Block BRAKE_COMMAND and ACC_HUD signals from bus 0↔2 forwarding on internal panda
  // This prevents stock messages from camera (bus 6/physical 2) reaching powertrain (bus 4/physical 0)
  if (((bus_num == 0) || (bus_num == 2)) &&
      ((addr == 0x30C) || (addr == 0x1FA) )) {
    block_msg = true;
  }

  return block_msg;
}

static void rlx_internal_rx_hook(const CANPacket_t *msg) {
  // common RX only
  // controls allowed from internal panda per include
  (void) msg; // ignore msg
  (void) alternative_experience;
}

static bool rlx_internal_tx_hook(const CANPacket_t *msg) {

    const LongitudinalLimits HONDA_NIDEC_LONG_LIMITS = {
    .max_gas = 198,  // 0xc6
    .max_brake = 255,

    .inactive_speed = 0,
  };

  bool tx = true;

  unsigned int bus_pt = honda_get_pt_bus();

  // ACC_HUD: safety check (nidec w/o pedal)
  if ((msg->addr == 0x30CU) && (msg->bus == bus_pt)) {
    int pcm_speed = (msg->data[0] << 8) | msg->data[1];
    int pcm_gas = msg->data[2];

    bool violation = true;
    violation |= longitudinal_speed_checks(pcm_speed, HONDA_NIDEC_LONG_LIMITS);
    violation |= longitudinal_gas_checks(pcm_gas, HONDA_NIDEC_LONG_LIMITS);
    if (violation) {
      tx = true;
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



const safety_hooks honda_nidec_hooks = {
  .init = rlx_internal_init,
  .rx = rlx_internal_rx_hook,
  .tx = rlx_internal_tx_hook,
  .fwd = rlx_internal_fwd_hook,
  .get_counter = honda_get_panda_counter,
  .get_checksum =  honda_get_panda_checksum,
  .compute_checksum =  honda_compute_panda_checksum,
};


const safety_hooks honda_bosch_hooks = {
  .init = rlx_internal_init,
  .rx = rlx_internal_rx_hook,
  .tx = rlx_internal_tx_hook,
  .fwd = rlx_internal_fwd_hook,
  .get_counter = honda_get_panda_counter,
  .get_checksum =  honda_get_panda_checksum,
  .compute_checksum =  honda_compute_panda_checksum,
};
