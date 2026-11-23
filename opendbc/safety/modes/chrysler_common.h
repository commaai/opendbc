#ifndef SAFETY_MODES_CHRYSLER_COMMON_H
#define SAFETY_MODES_CHRYSLER_COMMON_H

static uint32_t chrysler_get_checksum(const CANPacket_t *msg) {
  uint32_t checksum_byte = GET_LEN(msg) - 1U;
  return (uint8_t)(msg->data[checksum_byte]);
}

static uint32_t chrysler_compute_checksum(const CANPacket_t *msg) {
  uint8_t checksum = 0xFFU;
  uint32_t len = GET_LEN(msg);
  for (uint32_t j = 0U; (j + 1U) < len; j++) {
    checksum = crc8_update(checksum, msg->data[j], 0x1DU);
  }
  return (uint8_t)(~checksum);
}

// Only one button at a time: cancel, or resume while controls are allowed
static bool chrysler_cruise_buttons_allowed(const CANPacket_t *msg) {
  // Signal: CRUISE_BUTTONS.ACC_Cancel ... ACC_Distance_Inc: the buttons are bits 0-4 and 6-8
  const uint32_t buttons = GET_BYTES_LE(msg, 0, 2) & 0x1DFU;
  const bool is_cancel = buttons == 0x1U;
  const bool is_resume = buttons == 0x10U;
  return is_cancel || (is_resume && safety.controls_allowed);
}

#endif  // SAFETY_MODES_CHRYSLER_COMMON_H
