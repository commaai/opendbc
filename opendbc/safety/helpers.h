#pragma once

#include "opendbc/safety/declarations.h"

// The locals get a unique suffix so that nested uses don't hide each other's locals (Rule 5.3)
#define SAFETY_UNIQUE(name, n) name##n

#define SAFETY_MIN(a, b) SAFETY_MIN_IMPL(a, b, __COUNTER__)
// cppcheck-suppress-macro misra-c2012-1.2; allow __typeof__ extension
// cppcheck-suppress-macro misra-c2012-17.3; suppress false implicit declaration alert on typeof extension
#define SAFETY_MIN_IMPL(a, b, n) ({ \
  __typeof__(a) SAFETY_UNIQUE(_a, n) = (a); \
  __typeof__(b) SAFETY_UNIQUE(_b, n) = (b); \
  (SAFETY_UNIQUE(_a, n) < SAFETY_UNIQUE(_b, n)) ? SAFETY_UNIQUE(_a, n) : SAFETY_UNIQUE(_b, n); \
})

#define SAFETY_MAX(a, b) SAFETY_MAX_IMPL(a, b, __COUNTER__)
// cppcheck-suppress-macro misra-c2012-1.2; allow __typeof__ extension
// cppcheck-suppress-macro misra-c2012-17.3; suppress false implicit declaration alert on typeof extension
#define SAFETY_MAX_IMPL(a, b, n) ({ \
  __typeof__(a) SAFETY_UNIQUE(_a, n) = (a); \
  __typeof__(b) SAFETY_UNIQUE(_b, n) = (b); \
  (SAFETY_UNIQUE(_a, n) > SAFETY_UNIQUE(_b, n)) ? SAFETY_UNIQUE(_a, n) : SAFETY_UNIQUE(_b, n); \
})

#define SAFETY_CLAMP(x, low, high) SAFETY_CLAMP_IMPL(x, low, high, __COUNTER__)
// cppcheck-suppress-macro misra-c2012-1.2; allow __typeof__ extension
// cppcheck-suppress-macro misra-c2012-17.3; suppress false implicit declaration alert on typeof extension
#define SAFETY_CLAMP_IMPL(x, low, high, n) ({ \
  __typeof__(x) SAFETY_UNIQUE(_x, n) = (x); \
  __typeof__(low) SAFETY_UNIQUE(_low, n) = (low); \
  __typeof__(high) SAFETY_UNIQUE(_high, n) = (high); \
  (SAFETY_UNIQUE(_x, n) > SAFETY_UNIQUE(_high, n)) ? SAFETY_UNIQUE(_high, n) : \
    ((SAFETY_UNIQUE(_x, n) < SAFETY_UNIQUE(_low, n)) ? SAFETY_UNIQUE(_low, n) : SAFETY_UNIQUE(_x, n)); \
})

#define SAFETY_ABS(a) SAFETY_ABS_IMPL(a, __COUNTER__)
// cppcheck-suppress-macro misra-c2012-1.2; allow __typeof__ extension
// cppcheck-suppress-macro misra-c2012-17.3; suppress false implicit declaration alert on typeof extension
#define SAFETY_ABS_IMPL(a, n) ({ \
  __typeof__(a) SAFETY_UNIQUE(_a, n) = (a); \
  (SAFETY_UNIQUE(_a, n) > 0) ? SAFETY_UNIQUE(_a, n) : (-SAFETY_UNIQUE(_a, n)); \
})

#define SAFETY_UNUSED(x) ((void)(x))

// Update an MSB-first CRC8 with one byte; callers choose the initial value and final XOR.
static uint8_t crc8_update(uint8_t initial_crc, uint8_t data, uint8_t poly) {
  uint8_t crc = initial_crc ^ data;
  for (int i = 0; i < 8; i++) {
    if ((crc & 0x80U) != 0U) {
      crc = (crc << 1) ^ poly;
    } else {
      crc <<= 1;
    }
  }
  return crc;
}

// Compare address and bus as one key: CANPacket_t has a 29-bit address and 3-bit bus.
static bool msg_matches_addr_bus(const CANPacket_t *msg, uint32_t addr, uint32_t bus) {
  uint32_t actual = ((uint32_t)msg->addr << 3) | (uint32_t)msg->bus;
  uint32_t expected = (addr << 3) | bus;
  return actual == expected;
}

// Compare address, bus, and decoded byte length as one key.
static bool msg_matches_addr_bus_len(const CANPacket_t *msg, uint32_t addr, uint32_t bus, uint32_t len) {
  uint64_t actual = ((uint64_t)msg->addr << 10) | ((uint64_t)msg->bus << 7) | (uint64_t)dlc_to_len[msg->data_len_code];
  uint64_t expected = ((uint64_t)addr << 10) | ((uint64_t)bus << 7) | (uint64_t)len;
  return actual == expected;
}

#define MSG_MATCHES_SELECT(_msg, _addr, _bus, _len, NAME, ...) NAME
#define msg_matches(...) MSG_MATCHES_SELECT(__VA_ARGS__, msg_matches_addr_bus_len, msg_matches_addr_bus)(__VA_ARGS__)

// compute the time elapsed (in microseconds) from 2 counter samples
// case where ts < ts_last is ok: overflow is properly re-casted into uint32_t
static inline uint32_t safety_get_ts_elapsed(uint32_t ts, uint32_t ts_last) {
  return ts - ts_last;
}

static bool safety_max_limit_check(int val, const int MAX_VAL, const int MIN_VAL) {
  return (val > MAX_VAL) || (val < MIN_VAL);
}

// interp function that holds extreme values
static float safety_interpolate(struct lookup_t xy, float x) {
  int size = sizeof(xy.x) / sizeof(xy.x[0]);
  float ret = xy.y[size - 1];  // default output is last point

  // x is lower than the first point in the x array. Return the first point
  if (x <= xy.x[0]) {
    ret = xy.y[0];

  } else {
    // find the index such that (xy.x[i] <= x < xy.x[i+1]) and linearly interp
    for (int i=0; i < (size - 1); i++) {
      if (x < xy.x[i+1]) {
        float x0 = xy.x[i];
        float y0 = xy.y[i];
        float dx = xy.x[i+1] - x0;
        float dy = xy.y[i+1] - y0;
        // dx should not be zero as xy.x is supposed to be monotonic
        dx = SAFETY_MAX(dx, 0.0001);
        ret = (dy * (x - x0) / dx) + y0;
        break;
      }
    }
  }
  return ret;
}
