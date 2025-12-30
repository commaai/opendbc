#include "opendbc/safety/declarations.h"

// Type-safe inline functions (no double-evaluation)
static inline int safety_max_i(int a, int b) { return (a > b) ? a : b; }
static inline int safety_min_i(int a, int b) { return (a < b) ? a : b; }
static inline int safety_abs_i(int a) { return (a > 0) ? a : -a; }
static inline int safety_clamp_i(int x, int low, int high) {
  return (x > high) ? high : ((x < low) ? low : x);
}

static inline float safety_max_f(float a, float b) { return (a > b) ? a : b; }
static inline float safety_min_f(float a, float b) { return (a < b) ? a : b; }
static inline float safety_abs_f(float a) { return (a > 0.0f) ? a : -a; }
static inline float safety_clamp_f(float x, float low, float high) {
  return (x > high) ? high : ((x < low) ? low : x);
}

static inline double safety_max_d(double a, double b) { return (a > b) ? a : b; }
static inline double safety_min_d(double a, double b) { return (a < b) ? a : b; }

// C11 _Generic type-safe macros
#define SAFETY_MAX(a, b) _Generic((a) + (b), \
    float: safety_max_f, \
    double: safety_max_d, \
    default: safety_max_i \
)((a), (b))

#define SAFETY_MIN(a, b) _Generic((a) + (b), \
    float: safety_min_f, \
    double: safety_min_d, \
    default: safety_min_i \
)((a), (b))

#define SAFETY_ABS(a) _Generic((a), \
    float: safety_abs_f, \
    default: safety_abs_i \
)((a))

#define SAFETY_CLAMP(x, low, high) _Generic((x) + (low) + (high), \
    float: safety_clamp_f, \
    default: safety_clamp_i \
)((x), (low), (high))

#define SAFETY_UNUSED(x) ((void)(x))

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
