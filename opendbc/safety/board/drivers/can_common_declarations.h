#pragma once
#include <stdint.h>
#include "board/can_declarations.h"


uint8_t calculate_checksum(const uint8_t *dat, uint32_t len);
void can_set_checksum(CANPacket_t *packet);
