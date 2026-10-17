#pragma once

#include <stdbool.h>
#include <stdint.h>

// Interface of libsafety (safety.c) for the Python tests, loaded through cffi.
extern uint32_t timer_cnt;

uint32_t microsecond_timer_get(void);
bool safety_config_valid(void);
void set_controls_allowed(bool c);
void set_alternative_experience(int mode);
void set_relay_malfunction(bool c);
bool get_controls_allowed(void);
bool get_ignition_can(void);
bool get_relay_malfunction(void);
bool get_gas_pressed_prev(void);
void set_gas_pressed_prev(bool c);
bool get_brake_pressed_prev(void);
bool get_regen_braking_prev(void);
bool get_steering_disengage_prev(void);
bool get_cruise_engaged_prev(void);
void set_cruise_engaged_prev(bool engaged);
bool get_vehicle_moving(void);
bool get_acc_main_on(void);
float get_vehicle_speed_min(void);
float get_vehicle_speed_max(void);
int get_current_safety_mode(void);
int get_current_safety_param(void);
void set_timer(uint32_t t);
void set_torque_meas(int min, int max);
int get_torque_meas_min(void);
int get_torque_meas_max(void);
void set_torque_driver(int min, int max);
int get_torque_driver_min(void);
int get_torque_driver_max(void);
void set_rt_torque_last(int t);
void set_desired_torque_last(int t);
void set_desired_angle_last(int t);
void set_angle_meas(int min, int max);
int get_angle_meas_min(void);
int get_angle_meas_max(void);
void set_desired_curvature_last(int t);
int get_desired_curvature_last(void);
void set_curvature_meas(int min, int max);
int get_curvature_meas_min(void);
int get_curvature_meas_max(void);
void set_honda_alt_brake_msg(bool c);
void set_honda_fwd_brake(bool c);
bool get_honda_fwd_brake(void);
void init_tests(void);
