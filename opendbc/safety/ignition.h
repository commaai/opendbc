#pragma once

typedef struct {
  uint32_t addr;
  bool ignition;
} IgnitionMsg;

const IgnitionMsg IGNITION_MSGS[] = {
  {0x1F9, false},  // GM DI_torque2 (ignition off)
  {0x120, true},   // GM DI_torque2 (ignition on)
  {0x4A1, false},  // Mazda CRZ_DASHBOARD (ignition off)
  {0x3A0, true},   // Mazda CRZ_DASHBOARD (ignition on)
  {0x3E0, false},  // Rivian UI_driverInterface (ignition off)
  {0x292, true},   // Rivian UI_driverInterface (ignition on)
  {0x348, true},   // Tesla DI_torque2 (ignition on)
};

const size_t IGNITION_MSGS_LEN = sizeof(IGNITION_MSGS) / sizeof(IGNITION_MSGS[0]);

bool ignition_can_hook(const CANPacket_t *to_push) {
  bool ignition = false;
  
  for (size_t i = 0; i < IGNITION_MSGS_LEN; i++) {
    if ((GET_ADDR(to_push) == IGNITION_MSGS[i].addr)) {
      ignition = IGNITION_MSGS[i].ignition;
      break;
    }
  }
  
  return ignition;
}
