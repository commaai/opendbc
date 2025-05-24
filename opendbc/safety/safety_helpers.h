// interp function that holds extreme values
static float interpolate(struct lookup_t xy, float x) {

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
        dx = MAX(dx, 0.0001);
        ret = (dy * (x - x0) / dx) + y0;
        break;
      }
    }
  }
  return ret;
}

bool max_limit_check(int val, const int MAX_VAL, const int MIN_VAL) {
  return (val > MAX_VAL) || (val < MIN_VAL);
}

int ROUND(float val) {
  return val + ((val > 0.0) ? 0.5 : -0.5);
}


static void relay_malfunction_set(void) {
  relay_malfunction = true;
  fault_occurred(FAULT_RELAY_MALFUNCTION);
}

static void generic_rx_checks(void) {
  gas_pressed_prev = gas_pressed;

  // exit controls on rising edge of brake press
  if (brake_pressed && (!brake_pressed_prev || vehicle_moving)) {
    controls_allowed = false;
  }
  brake_pressed_prev = brake_pressed;

  // exit controls on rising edge of regen paddle
  if (regen_braking && (!regen_braking_prev || vehicle_moving)) {
    controls_allowed = false;
  }
  regen_braking_prev = regen_braking;

  // exit controls on rising edge of steering override/disengage
  if (steering_disengage && !steering_disengage_prev) {
    controls_allowed = false;
  }
  steering_disengage_prev = steering_disengage;
}

static void stock_ecu_check(bool stock_ecu_detected) {
  // allow 1s of transition timeout after relay changes state before assessing malfunctioning
  const uint32_t RELAY_TRNS_TIMEOUT = 1U;

  // check if stock ECU is on bus broken by car harness
  if ((safety_mode_cnt > RELAY_TRNS_TIMEOUT) && stock_ecu_detected) {
    relay_malfunction_set();
  }
}

static void relay_malfunction_reset(void) {
  relay_malfunction = false;
  fault_recovered(FAULT_RELAY_MALFUNCTION);
}

// resets values and min/max for sample_t struct
static void reset_sample(struct sample_t *sample) {
  for (int i = 0; i < MAX_SAMPLE_VALS; i++) {
    sample->values[i] = 0;
  }
  update_sample(sample, 0);
}

// given a new sample, update the sample_t struct
void update_sample(struct sample_t *sample, int sample_new) {
  for (int i = MAX_SAMPLE_VALS - 1; i > 0; i--) {
    sample->values[i] = sample->values[i-1];
  }
  sample->values[0] = sample_new;

  // get the minimum and maximum measured samples
  sample->min = sample->values[0];
  sample->max = sample->values[0];
  for (int i = 1; i < MAX_SAMPLE_VALS; i++) {
    if (sample->values[i] < sample->min) {
      sample->min = sample->values[i];
    }
    if (sample->values[i] > sample->max) {
      sample->max = sample->values[i];
    }
  }
}
