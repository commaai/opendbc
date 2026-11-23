#ifndef SAFETY_CAN_H
#define SAFETY_CAN_H

static const unsigned char dlc_to_len[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};

#define CANPACKET_DATA_SIZE_MAX 64U

typedef struct {
  unsigned char fd : 1;
  unsigned char bus : 3;
  unsigned char data_len_code : 4;  // lookup length with dlc_to_len
  unsigned char rejected : 1;
  unsigned char returned : 1;
  unsigned char extended : 1;
  unsigned int addr : 29;
  unsigned char checksum;
  unsigned char data[CANPACKET_DATA_SIZE_MAX];
  // MISRA deviation from Rule 1.2: this is panda's USB/SPI packet layout, where addr fills header bits 11-39 across a
  // 32-bit boundary, sizeof is 72 and the alignment 4. Standard C can't express it: gcc and clang keep a bit-field in an
  // aligned unit of its type, so addr would need a 64-bit type and an alignment of 8, which _Alignas can't weaken.
  // Both support the attribute, and no other compiler builds this code.
  // cppcheck-suppress misra-c2012-1.2
} __attribute__((packed, aligned(4))) CANPacket_t;

#define GET_LEN(msg) ((unsigned int)dlc_to_len[(msg)->data_len_code])

#endif
