#pragma once
#include <stdint.h>
#include <stdbool.h>


uint8_t calculate_checksum(const uint8_t *dat, uint32_t len);
void can_set_checksum(CANPacket_t *packet);
bool can_check_checksum(CANPacket_t *packet);
