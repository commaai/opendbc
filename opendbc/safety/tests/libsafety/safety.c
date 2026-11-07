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

bool safety_config_valid() {
  if (current_safety_config.rx_checks_len <= 0) {
    printf("missing RX checks\n");
    return false;
  }

  for (int i = 0; i < current_safety_config.rx_checks_len; i++) {
    const RxStatus status = safety.rx_status[i];
    bool valid = status.msg_seen && !status.lagging && status.valid_checksum && (status.wrong_counters < MAX_WRONG_COUNTERS) && status.valid_quality_flag;
    if (!valid) {
      // printf("i %d seen %d lagging %d valid checksum %d wrong counters %d valid quality flag %d\n", i, status.msg_seen, status.lagging, status.valid_checksum, status.wrong_counters, status.valid_quality_flag);
      return false;
    }
  }
  return true;
}

int get_rx_checks_len(void) {
  return current_safety_config.rx_checks_len;
}

// all RX checks valid, check stale last received age us ago
void set_rx_checks_received(int stale, uint32_t age) {
  for (int i = 0; i < current_safety_config.rx_checks_len; i++) {
    safety.rx_status[i].msg_seen = true;
    safety.rx_status[i].valid_checksum = true;
    safety.rx_status[i].valid_quality_flag = true;
    safety.rx_status[i].wrong_counters = 0;
    safety.rx_status[i].last_timestamp = (i == stale) ? (timer_cnt - age) : timer_cnt;
  }
}

void set_controls_allowed(bool c){
  safety.controls_allowed = c;
}

void set_alternative_experience(int mode){
  alternative_experience = mode;
}

void set_relay_malfunction(bool c){
  safety.relay_malfunction = c;
}

bool get_controls_allowed(void){
  return safety.controls_allowed;
}

bool get_ignition_can(void){
  return ignition_can;
}

bool get_relay_malfunction(void){
  return safety.relay_malfunction;
}

bool get_gas_pressed_prev(void){
  return safety.gas_pressed_prev;
}

void set_gas_pressed_prev(bool c){
  safety.gas_pressed_prev = c;
}

bool get_brake_pressed_prev(void){
  return safety.brake_pressed_prev;
}

bool get_regen_braking_prev(void){
  return safety.regen_braking_prev;
}

bool get_steering_disengage_prev(void){
  return safety.steering_disengage_prev;
}

bool get_cruise_engaged_prev(void){
  return safety.cruise_engaged_prev;
}

void set_cruise_engaged_prev(bool engaged){
  safety.cruise_engaged_prev = engaged;
}

bool get_vehicle_moving(void){
  return safety.vehicle_moving;
}

bool get_acc_main_on(void){
  return safety.acc_main_on;
}

float get_vehicle_speed_min(void){
  return safety.vehicle_speed.min / VEHICLE_SPEED_FACTOR;
}

float get_vehicle_speed_max(void){
  return safety.vehicle_speed.max / VEHICLE_SPEED_FACTOR;
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
  safety.torque_meas.min = min;
  safety.torque_meas.max = max;
}

int get_torque_meas_min(void){
  return safety.torque_meas.min;
}

int get_torque_meas_max(void){
  return safety.torque_meas.max;
}

void set_torque_driver(int min, int max){
  safety.torque_driver.min = min;
  safety.torque_driver.max = max;
}

int get_torque_driver_min(void){
  return safety.torque_driver.min;
}

int get_torque_driver_max(void){
  return safety.torque_driver.max;
}

void set_rt_torque_last(int t){
  safety.rt_torque_last = t;
}

void set_desired_torque_last(int t){
  safety.desired_torque_last = t;
}

void set_desired_angle_last(int t){
  safety.desired_angle_last = t;
}

void set_angle_meas(int min, int max){
  safety.angle_meas.min = min;
  safety.angle_meas.max = max;
}

int get_angle_meas_min(void){
  return safety.angle_meas.min;
}

int get_angle_meas_max(void){
  return safety.angle_meas.max;
}

void set_desired_curvature_last(int t){
  safety.curvature_state.desired_last = t;
}

int get_desired_curvature_last(void){
  return safety.curvature_state.desired_last;
}

void set_curvature_meas(int min, int max){
  safety.curvature_state.meas.min = min;
  safety.curvature_state.meas.max = max;
}

int get_curvature_meas_min(void){
  return safety.curvature_state.meas.min;
}

int get_curvature_meas_max(void){
  return safety.curvature_state.meas.max;
}


// ***** car specific helpers *****

void set_honda_alt_brake_msg(bool c){
  honda_alt_brake_msg = c;
}

void set_honda_fwd_brake(bool c){
  safety.honda_fwd_brake = c;
}

bool get_honda_fwd_brake(void){
  return safety.honda_fwd_brake;
}

void init_tests(void){
  safety.safety_mode_cnt = 2U;  // avoid ignoring relay_malfunction logic
  alternative_experience = 0;
  set_timer(0);
  safety.ts_steer_req_mismatch_last = 0;
  safety.valid_steer_req_count = 0;
  safety.invalid_steer_req_count = 0;

  // assumes autopark on safety mode init to avoid a fault. get rid of that for testing
  safety.tesla_autopark = false;

  ignition_can = false;
  ignition_can_cnt = 0U;
}
