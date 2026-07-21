#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "opendbc/safety/can.h"

// This header is the supported interface for consumers of opendbc safety.
// Everything not declared here is internal and may change without notice.

// from cereal.car.CarParams.SafetyModel
#define SAFETY_SILENT 0U
#define SAFETY_HONDA_NIDEC 1U
#define SAFETY_TOYOTA 2U
#define SAFETY_ELM327 3U
#define SAFETY_GM 4U
#define SAFETY_HONDA_BOSCH_GIRAFFE 5U
#define SAFETY_FORD 6U
#define SAFETY_HYUNDAI 8U
#define SAFETY_CHRYSLER 9U
#define SAFETY_TESLA 10U
#define SAFETY_SUBARU 11U
#define SAFETY_MAZDA 13U
#define SAFETY_NISSAN 14U
#define SAFETY_VOLKSWAGEN_MQB 15U
#define SAFETY_ALLOUTPUT 17U
#define SAFETY_GM_ASCM 18U
#define SAFETY_NOOUTPUT 19U
#define SAFETY_HONDA_BOSCH 20U
#define SAFETY_VOLKSWAGEN_PQ 21U
#define SAFETY_SUBARU_PREGLOBAL 22U
#define SAFETY_HYUNDAI_LEGACY 23U
#define SAFETY_HYUNDAI_COMMUNITY 24U
#define SAFETY_VOLKSWAGEN_MLB 25U
#define SAFETY_FAW 26U
#define SAFETY_BODY 27U
#define SAFETY_HYUNDAI_CANFD 28U
#define SAFETY_CHRYSLER_CUSW 30U
#define SAFETY_PSA 31U
#define SAFETY_RIVIAN 33U
#define SAFETY_VOLKSWAGEN_MEB 34U

typedef struct {
  bool controls_allowed;
  bool relay_malfunction;
  bool heartbeat_engaged;
  bool rx_checks_invalid;
  uint16_t mode;
  uint16_t param;
  int alternative_experience;
} safety_state;

// Configure and run the safety hooks.
int set_safety_hooks(uint16_t mode, uint16_t param);
bool safety_rx_hook(const CANPacket_t *msg);
bool safety_tx_hook(CANPacket_t *msg);
int safety_fwd_hook(int bus_num, int addr);
void safety_tick(void);

// Read and update state shared with the safety host.
safety_state safety_get_state(void);
void safety_disengage(void);
void safety_set_alternative_experience(int mode);
void safety_set_heartbeat_engaged(bool engaged);
