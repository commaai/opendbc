#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "opendbc/safety/can.h"
#include "opendbc/safety/helpers.h"

bool ignition_can = false;
uint32_t ignition_can_cnt = 0U;

void ignition_can_hook(const CANPacket_t *msg) {
  // GM exception
  if (msg_matches(msg, 0x1F1U, 0U, 8U)) {
    // SystemPowerMode (2=Run, 3=Crank Request)
    ignition_can = (msg->data[0] & 0x2U) != 0U;
    ignition_can_cnt = 0U;
  }

  // Rivian R1S/T GEN1 exception
  if (msg_matches(msg, 0x152U, 0U, 8U)) {
    // 0x152 overlaps with Subaru pre-global which has this bit as the high beam
    int counter = msg->data[1] & 0xFU;  // max is only 14

    static int prev_counter_rivian = -1;
    if ((counter == ((prev_counter_rivian + 1) % 15)) && (prev_counter_rivian != -1)) {
      // VDM_OutputSignals->VDM_EpasPowerMode
      ignition_can = ((msg->data[7] >> 4U) & 0x3U) == 1U;  // VDM_EpasPowerMode_Drive_On=1
      ignition_can_cnt = 0U;
    }
    prev_counter_rivian = counter;
  }

  // Tesla Model 3/Y exception
  if (msg_matches(msg, 0x221U, 0U, 8U)) {
    // 0x221 overlaps with Rivian which has random data on byte 0
    int counter = msg->data[6] >> 4;

    static int prev_counter_tesla = -1;
    if ((counter == ((prev_counter_tesla + 1) % 16)) && (prev_counter_tesla != -1)) {
      // VCFRONT_LVPowerState->VCFRONT_vehiclePowerState
      int power_state = (msg->data[0] >> 5U) & 0x3U;
      ignition_can = power_state == 0x3;  // VEHICLE_POWER_STATE_DRIVE=3
      ignition_can_cnt = 0U;
    }
    prev_counter_tesla = counter;
  }

  // Mazda exception
  if (msg_matches(msg, 0x9EU, 0U, 8U)) {
    ignition_can = (msg->data[0] >> 5) == 0x6U;
    ignition_can_cnt = 0U;
  }

  // PSA exception
  // 0x348 has no counter on most cars and overlaps with VW cameras and Tesla Model S,
  // only trust it after consecutive valid frames of Dyn_EasyMove 0x3AD
  static int prev_counter_psa = -1;
  static uint8_t valid_cnt_psa = 0U;
  if (msg_matches(msg, 0x3ADU, 0U, 8U)) {
    int counter = msg->data[7] >> 4;
    uint8_t sum = 3U;
    for (int i = 0; i < 8; i++) {
      sum += (msg->data[i] >> 4) + (msg->data[i] & 0xFU);
    }

    if ((counter == ((prev_counter_psa + 1) % 16)) && (prev_counter_psa != -1) && ((sum & 0xFU) == 0U)) {
      if (valid_cnt_psa < 3U) {
        valid_cnt_psa++;
      }
    } else {
      valid_cnt_psa = 0U;
    }
    prev_counter_psa = counter;
  }

  if (msg_matches(msg, 0x348U, 0U, 8U) && (valid_cnt_psa >= 3U)) {
    // Dyn2_CMM, bit 41: EV running, bit 42: ICE running
    ignition_can = (msg->data[5] & 0x6U) != 0U;
    ignition_can_cnt = 0U;
  }

  // Volkswagen MEB exception
  if (msg_matches(msg, 0x3C0U, 0U, 4U)) {
    int counter = msg->data[1] & 0xFU;

    static int prev_counter_vw_meb = -1;
    if ((counter == ((prev_counter_vw_meb + 1) % 16)) && (prev_counter_vw_meb != -1)) {
      // Klemmen_Status_01->ZAS_Kl_15
      ignition_can = ((msg->data[2] >> 1) & 1U) != 0U;
      ignition_can_cnt = 0U;
    }
    prev_counter_vw_meb = counter;
  }
}
