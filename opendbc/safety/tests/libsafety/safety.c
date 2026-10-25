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
    const RxStatus status = rx_status[i];
    bool valid = status.msg_seen && !status.lagging && status.valid_checksum && (status.wrong_counters < MAX_WRONG_COUNTERS) && status.valid_quality_flag;
    if (!valid) {
      // printf("i %d seen %d lagging %d valid checksum %d wrong counters %d valid quality flag %d\n", i, status.msg_seen, status.lagging, status.valid_checksum, status.wrong_counters, status.valid_quality_flag);
      return false;
    }
  }
  return true;
}

// Retained for opendbc/car/tests/test_models.py; safety tests drive state through CAN.
void set_controls_allowed(bool c){
  controls_allowed = c;
}

void set_alternative_experience(int mode){
  alternative_experience = mode;
}

// Retained for opendbc/car/tests/test_models.py.
void set_relay_malfunction(bool c){
  relay_malfunction = c;
}

bool get_controls_allowed(void){
  return controls_allowed;
}

bool get_ignition_can(void){
  return ignition_can;
}

bool get_relay_malfunction(void){
  return relay_malfunction;
}

bool get_gas_pressed_prev(void){
  return gas_pressed_prev;
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

// Retained for opendbc/car/tests/test_models.py.
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

uint32_t get_timer(void){
  return timer_cnt;
}

int get_torque_meas_min(void){
  return torque_meas.min;
}

int get_torque_meas_max(void){
  return torque_meas.max;
}

int get_torque_driver_min(void){
  return torque_driver.min;
}

int get_torque_driver_max(void){
  return torque_driver.max;
}

int get_angle_meas_min(void){
  return angle_meas.min;
}

int get_angle_meas_max(void){
  return angle_meas.max;
}

int get_desired_curvature_last(void){
  return curvature_state.desired_last;
}

int get_curvature_meas_min(void){
  return curvature_state.meas.min;
}

int get_curvature_meas_max(void){
  return curvature_state.meas.max;
}


// ***** car specific helpers *****

bool get_honda_fwd_brake(void){
  return honda_fwd_brake;
}

void init_tests(void){
  // Firmware lifecycle input: the relay transition grace period has elapsed.
  safety_mode_cnt = 2U;
  alternative_experience = 0;
  set_timer(0);

  ignition_can = false;
  ignition_can_cnt = 0U;
}
