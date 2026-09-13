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
  safety_state.controls_allowed = c;
}

void set_alternative_experience(int mode){
  alternative_experience = mode;
}

void set_relay_malfunction(bool c){
  safety_state.relay_malfunction = c;
}

void set_ignition_can(bool c){
  ignition_can = c;
}

bool get_controls_allowed(void){
  return safety_state.controls_allowed;
}

bool get_ignition_can(void){
  return ignition_can;
}

int get_alternative_experience(void){
  return alternative_experience;
}

bool get_relay_malfunction(void){
  return safety_state.relay_malfunction;
}

bool get_gas_pressed_prev(void){
  return safety_state.gas_pressed_prev;
}

void set_gas_pressed_prev(bool c){
  safety_state.gas_pressed_prev = c;
}

bool get_brake_pressed_prev(void){
  return safety_state.brake_pressed_prev;
}

bool get_regen_braking_prev(void){
  return safety_state.regen_braking_prev;
}

bool get_steering_disengage_prev(void){
  return safety_state.steering_disengage_prev;
}

bool get_cruise_engaged_prev(void){
  return safety_state.cruise_engaged_prev;
}

void set_cruise_engaged_prev(bool engaged){
  safety_state.cruise_engaged_prev = engaged;
}

bool get_vehicle_moving(void){
  return safety_state.vehicle_moving;
}

bool get_acc_main_on(void){
  return safety_state.acc_main_on;
}

float get_vehicle_speed_min(void){
  return safety_state.vehicle_speed.min / VEHICLE_SPEED_FACTOR;
}

float get_vehicle_speed_max(void){
  return safety_state.vehicle_speed.max / VEHICLE_SPEED_FACTOR;
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
  safety_state.torque_meas.min = min;
  safety_state.torque_meas.max = max;
}

int get_torque_meas_min(void){
  return safety_state.torque_meas.min;
}

int get_torque_meas_max(void){
  return safety_state.torque_meas.max;
}

void set_torque_driver(int min, int max){
  safety_state.torque_driver.min = min;
  safety_state.torque_driver.max = max;
}

int get_torque_driver_min(void){
  return safety_state.torque_driver.min;
}

int get_torque_driver_max(void){
  return safety_state.torque_driver.max;
}

void set_rt_torque_last(int t){
  safety_state.rt_torque_last = t;
}

void set_desired_torque_last(int t){
  safety_state.desired_torque_last = t;
}

void set_desired_angle_last(int t){
  safety_state.desired_angle_last = t;
}

int get_desired_angle_last(void){
  return safety_state.desired_angle_last;
}

void set_angle_meas(int min, int max){
  safety_state.angle_meas.min = min;
  safety_state.angle_meas.max = max;
}

int get_angle_meas_min(void){
  return safety_state.angle_meas.min;
}

int get_angle_meas_max(void){
  return safety_state.angle_meas.max;
}

void set_desired_curvature_last(int t){
  safety_state.curvature_state.desired_last = t;
}

int get_desired_curvature_last(void){
  return safety_state.curvature_state.desired_last;
}

void set_curvature_meas(int min, int max){
  safety_state.curvature_state.meas.min = min;
  safety_state.curvature_state.meas.max = max;
}

int get_curvature_meas_min(void){
  return safety_state.curvature_state.meas.min;
}

int get_curvature_meas_max(void){
  return safety_state.curvature_state.meas.max;
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
  safety_state.safety_mode_cnt = 2U;  // avoid ignoring relay_malfunction logic
  alternative_experience = 0;
  set_timer(0);
  safety_state.ts_steer_req_mismatch_last = 0;
  safety_state.valid_steer_req_count = 0;
  safety_state.invalid_steer_req_count = 0;

  // assumes autopark on safety mode init to avoid a fault. get rid of that for testing
  tesla_autopark = false;

  ignition_can = false;
  ignition_can_cnt = 0U;
}
