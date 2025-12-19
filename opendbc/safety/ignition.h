// CAN ignition detection ported from panda/board/drivers/can_common.h
#pragma once

#include "opendbc/safety/declarations.h"

bool ignition_can = false;
uint32_t ignition_can_cnt = 0U;
static int prev_rivian_cnt = -1;
static int prev_tesla_cnt = -1;

static void ignition_can_hook(const CANPacket_t *to_push) {
  int len = GET_LEN(to_push);

  if ((to_push->bus == 0U) && (len == 8)) {
    int addr = to_push->addr;

    // GM: SystemPowerMode 2=Run, 3=Crank
    if (addr == 0x1F1) {
      ignition_can = (to_push->data[0] & 0x2U) != 0U;
      ignition_can_cnt = 0U;
    }

    // Rivian: 0x152 overlaps Subaru pre-global high beam, use counter to distinguish
    if (addr == 0x152) {
      int cnt = to_push->data[1] & 0xFU;
      if ((cnt == ((prev_rivian_cnt + 1) % 15)) && (prev_rivian_cnt != -1)) {
        ignition_can = ((to_push->data[7] >> 4U) & 0x3U) == 1U;
        ignition_can_cnt = 0U;
      }
      prev_rivian_cnt = cnt;
    }

    // Tesla: 0x221 may overlap with other OEMs, use counter to distinguish
    if (addr == 0x221) {
      int cnt = to_push->data[6] >> 4;
      if ((cnt == ((prev_tesla_cnt + 1) % 16)) && (prev_tesla_cnt != -1)) {
        ignition_can = ((to_push->data[0] >> 5U) & 0x3U) == 0x3U;
        ignition_can_cnt = 0U;
      }
      prev_tesla_cnt = cnt;
    }

    // Mazda
    if (addr == 0x9E) {
      ignition_can = (to_push->data[0] >> 5) == 0x6U;
      ignition_can_cnt = 0U;
    }
  }
}

static void ignition_can_init(void) {
  ignition_can = false;
  ignition_can_cnt = 0U;
  prev_rivian_cnt = -1;
  prev_tesla_cnt = -1;
}
