#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

// TODO: time should just be passed into the hooks we expose
uint32_t timer_cnt = 0;
uint32_t microsecond_timer_get(void);
uint32_t microsecond_timer_get(void) {
  return timer_cnt;
}

#include "opendbc/safety/can.h"
#include "opendbc/safety/safety.h"
#include "opendbc/safety/ignition.h"

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

void set_steering_disengage_prev(bool b) {
  steering_disengage_prev = b;
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

void set_acc_main_on(bool on){
  acc_main_on = on;
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

bool get_honda_bosch_long(void) {
  return honda_bosch_long;
}

int get_honda_hw(void) {
  return honda_hw;
}

void set_honda_hw(int h) {
  honda_hw = h;
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

uint8_t _test_get_counter(const CANPacket_t *msg) {
  return current_hooks->get_counter(msg);
}

uint32_t _test_get_checksum(const CANPacket_t *msg) {
  return current_hooks->get_checksum(msg);
}

uint32_t _test_compute_checksum(const CANPacket_t *msg) {
  return current_hooks->compute_checksum(msg);
}

bool _test_get_quality_flag_valid(const CANPacket_t *msg) {
  return current_hooks->get_quality_flag_valid(msg);
}

void _test_rx_hook(const CANPacket_t *msg) {
  current_hooks->rx(msg);
}

bool _test_tx_hook(const CANPacket_t *msg) {
  return current_hooks->tx(msg);
}

// Configure one RX check for unit-testing the safety_tick condition paths.
void _test_setup_safety_tick_rx_check(bool frequency_invalid, bool msg_invalid) {
  static RxCheck valid_frequency_rx_check[] = {
    {.msg = {{.addr = 1, .bus = 0, .len = 8, .frequency = 10U}, { 0 }, { 0 }}},
  };
  static RxCheck invalid_frequency_rx_check[] = {
    {.msg = {{.addr = 1, .bus = 0, .len = 8, .frequency = 9U}, { 0 }, { 0 }}},
  };

  RxCheck *rx_check = frequency_invalid ? invalid_frequency_rx_check : valid_frequency_rx_check;
  rx_check[0].status = (RxStatus){
    .msg_seen = true,
    .index = 0,
    .valid_checksum = !msg_invalid,
    .wrong_counters = 0,
    .valid_quality_flag = true,
    .last_counter = 0U,
    .last_timestamp = 0U,
    .lagging = false,
  };
  current_safety_config.rx_checks = rx_check;
  current_safety_config.rx_checks_len = 1;
}

void _test_setup_safety_config_valid_checks(int inverted) {
  if (inverted == -1) {
    current_safety_config.rx_checks_len = 0;
  } else {
    static RxCheck valid_rx_check[] = {{}};

    valid_rx_check[0].status.msg_seen = true;
    valid_rx_check[0].status.lagging = false;
    valid_rx_check[0].status.valid_checksum = true;
    valid_rx_check[0].status.wrong_counters = 0;
    valid_rx_check[0].status.valid_quality_flag = true;

    switch (inverted) {
    case 0:
      valid_rx_check[0].status.msg_seen = false;
      break;
    case 1:
      valid_rx_check[0].status.lagging = true;
      break;
    case 2:
      valid_rx_check[0].status.valid_checksum = false;
      break;
    case 3:
      valid_rx_check[0].status.wrong_counters = MAX_WRONG_COUNTERS;
      break;
    case 4:
      valid_rx_check[0].status.valid_quality_flag = false;
      break;
    default:
      break;
    }
    current_safety_config.rx_checks = valid_rx_check;
    current_safety_config.rx_checks_len = 1;
  }
}

void _test_nullify_compute_checksum(void) {
  _test_hooks = *current_hooks;
  _test_hooks.compute_checksum = NULL;
  current_hooks = &_test_hooks;
}

void _test_nullify_init(void) {
  _test_hooks = *current_hooks;
  _test_hooks.init = NULL;
  current_hooks = &_test_hooks;
}

int _test_get_rx_checks_len(void) {
  return current_safety_config.rx_checks_len;
}
