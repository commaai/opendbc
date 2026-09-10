#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// TODO: time should just be passed into the hooks we expose
uint32_t timer_cnt = 0;
uint32_t microsecond_timer_get(void);
uint32_t microsecond_timer_get(void) {
  return timer_cnt;
}

#include "opendbc/safety/can.h"
#include "opendbc/safety/safety.h"
#include "opendbc/safety/ignition.h"

static RxCheck *test_rx_checks;
static safety_hooks test_hooks;
static unsigned int test_rx_count;

static void test_rx_hook(const CANPacket_t *msg) {
  steering_disengage = (msg->data[4] & 1U) != 0U;
  test_rx_count++;
}

static uint32_t test_get_checksum(const CANPacket_t *msg) {
  return msg->data[0];
}

static uint32_t test_compute_checksum(const CANPacket_t *msg) {
  return msg->data[1];
}

static uint8_t test_get_counter(const CANPacket_t *msg) {
  return msg->data[2];
}

static bool test_get_quality_flag_valid(const CANPacket_t *msg) {
  return msg->data[3] == 1U;
}

// Build configurations that production modes intentionally avoid, so the common
// safety checks' fail-closed behavior can be tested independently of any car.
void safety_test_configure_rx(uint32_t frequency, bool ignore_checksum, bool ignore_counter,
                              bool ignore_quality_flag, uint8_t max_counter, uint8_t callbacks) {
  set_safety_hooks(SAFETY_NOOUTPUT, 0);
  const RxCheck checks[] = {
    {.msg = {{0x123, 0, 8, frequency, .ignore_checksum = ignore_checksum, .ignore_counter = ignore_counter,
              .max_counter = max_counter, .ignore_quality_flag = ignore_quality_flag}, {0}, {0}}},
  };
  free(test_rx_checks);
  // Fresh storage initializes the const message descriptors without modifying
  // the const subobjects of a previously declared RxCheck.
  test_rx_checks = malloc(sizeof(checks));
  if (test_rx_checks == NULL) {
    abort();
  }
  memcpy(test_rx_checks, checks, sizeof(checks));
  current_safety_config.rx_checks = test_rx_checks;
  current_safety_config.rx_checks_len = 1;
  test_hooks = (safety_hooks){
    .rx = test_rx_hook,
    .get_checksum = (callbacks & 1U) ? test_get_checksum : NULL,
    .compute_checksum = (callbacks & 2U) ? test_compute_checksum : NULL,
    .get_counter = (callbacks & 4U) ? test_get_counter : NULL,
    .get_quality_flag_valid = (callbacks & 8U) ? test_get_quality_flag_valid : NULL,
  };
  current_hooks = &test_hooks;
  test_rx_count = 0;
}

unsigned int safety_test_get_rx_count(void) {
  return test_rx_count;
}

bool get_safety_rx_checks_invalid(void) {
  return safety_rx_checks_invalid;
}

void safety_test_tick_null(void) {
  safety_tick(NULL);
}

float safety_test_interpolate(float x, float midpoint) {
  const struct lookup_t table = {{0., midpoint, 1.}, {0., 1., 2.}};
  return safety_interpolate(table, x);
}

bool safety_test_dynamic_torque_limit(float torque) {
  const TorqueSteeringLimits limits = {
    .max_torque = 300, .max_rate_up = 10, .max_rate_down = 10,
    .dynamic_max_torque = true, .max_torque_lookup = {{0., 1., 2.}, {torque, torque, torque}},
    .type = TorqueDriverLimited,
  };
  return steer_torque_cmd_checks(0, 0, limits);
}

void safety_tick_current_safety_config() {
  safety_tick(&current_safety_config);
}

bool safety_config_valid() {
  if (current_safety_config.rx_checks_len <= 0) {
    printf("missing RX checks\n");
    return false;
  }

  for (int i = 0; i < current_safety_config.rx_checks_len; i++) {
    const RxCheck addr = current_safety_config.rx_checks[i];
    bool valid = addr.status.msg_seen && !addr.status.lagging && addr.status.valid_checksum && (addr.status.wrong_counters < MAX_WRONG_COUNTERS) && addr.status.valid_quality_flag;
    if (!valid) {
      // printf("i %d seen %d lagging %d valid checksum %d wrong counters %d valid quality flag %d\n", i, addr.status.msg_seen, addr.status.lagging, addr.status.valid_checksum, addr.status.wrong_counters, addr.status.valid_quality_flag);
      return false;
    }
  }
  return true;
}

void set_controls_allowed(bool c){
  controls_allowed = c;
}

void set_alternative_experience(int mode){
  alternative_experience = mode;
}

void set_relay_malfunction(bool c){
  relay_malfunction = c;
}

void set_ignition_can(bool c){
  ignition_can = c;
}

bool get_controls_allowed(void){
  return controls_allowed;
}

bool get_ignition_can(void){
  return ignition_can;
}

int get_alternative_experience(void){
  return alternative_experience;
}

bool get_relay_malfunction(void){
  return relay_malfunction;
}

bool get_gas_pressed_prev(void){
  return gas_pressed_prev;
}

void set_gas_pressed_prev(bool c){
  gas_pressed_prev = c;
}

bool get_brake_pressed_prev(void){
  return brake_pressed_prev;
}

bool get_regen_braking_prev(void){
  return regen_braking_prev;
}

bool get_steering_disengage_prev(void){
  return steering_disengage_prev;
}

bool get_cruise_engaged_prev(void){
  return cruise_engaged_prev;
}

void set_cruise_engaged_prev(bool engaged){
  cruise_engaged_prev = engaged;
}

bool get_vehicle_moving(void){
  return vehicle_moving;
}

bool get_acc_main_on(void){
  return acc_main_on;
}

float get_vehicle_speed_min(void){
  return vehicle_speed.min / VEHICLE_SPEED_FACTOR;
}

float get_vehicle_speed_max(void){
  return vehicle_speed.max / VEHICLE_SPEED_FACTOR;
}

int get_current_safety_mode(void){
  return current_safety_mode;
}

int get_current_safety_param(void){
  return current_safety_param;
}

void set_timer(uint32_t t){
  timer_cnt = t;
}

void set_torque_meas(int min, int max){
  torque_meas.min = min;
  torque_meas.max = max;
}

int get_torque_meas_min(void){
  return torque_meas.min;
}

int get_torque_meas_max(void){
  return torque_meas.max;
}

void set_torque_driver(int min, int max){
  torque_driver.min = min;
  torque_driver.max = max;
}

int get_torque_driver_min(void){
  return torque_driver.min;
}

int get_torque_driver_max(void){
  return torque_driver.max;
}

void set_rt_torque_last(int t){
  rt_torque_last = t;
}

void set_desired_torque_last(int t){
  desired_torque_last = t;
}

void set_desired_angle_last(int t){
  desired_angle_last = t;
}

int get_desired_angle_last(void){
  return desired_angle_last;
}

void set_angle_meas(int min, int max){
  angle_meas.min = min;
  angle_meas.max = max;
}

int get_angle_meas_min(void){
  return angle_meas.min;
}

int get_angle_meas_max(void){
  return angle_meas.max;
}

void set_desired_curvature_last(int t){
  curvature_state.desired_last = t;
}

int get_desired_curvature_last(void){
  return curvature_state.desired_last;
}

void set_curvature_meas(int min, int max){
  curvature_state.meas.min = min;
  curvature_state.meas.max = max;
}

int get_curvature_meas_min(void){
  return curvature_state.meas.min;
}

int get_curvature_meas_max(void){
  return curvature_state.meas.max;
}


// ***** car specific helpers *****

void set_honda_alt_brake_msg(bool c){
  honda_alt_brake_msg = c;
}

void set_honda_bosch_long(bool c){
  honda_bosch_long = c;
}

int get_honda_hw(void) {
  return honda_hw;
}

void set_honda_fwd_brake(bool c){
  honda_fwd_brake = c;
}

bool get_honda_fwd_brake(void){
  return honda_fwd_brake;
}

void init_tests(void){
  safety_mode_cnt = 2U;  // avoid ignoring relay_malfunction logic
  alternative_experience = 0;
  set_timer(0);
  ts_steer_req_mismatch_last = 0;
  valid_steer_req_count = 0;
  invalid_steer_req_count = 0;

  // assumes autopark on safety mode init to avoid a fault. get rid of that for testing
  tesla_autopark = false;

  ignition_can = false;
  ignition_can_cnt = 0U;
}
