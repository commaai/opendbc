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

static bool rx_field_valid(RxMsgField field, int len) {
  if (field.shift >= 8U) {
    return false;
  }
  uint32_t bits = (uint32_t)field.mask << field.shift;
  return (bits <= 0xFFFFU) &&
         ((field.mask & (field.mask + 1U)) == 0U) &&
         ((field.byte + ((bits > 0xFFU) ? 2 : 1)) <= len);
}

bool safety_rx_checks_metadata_valid(void) {
  static const RxMsgChecks no_checks = {0};
  for (int i = 0; i < current_safety_config.rx_checks_len; i++) {
    for (uint8_t j = 0U; j < MAX_ADDR_CHECK_MSGS; j++) {
      const CanMsgCheck *msg = &current_safety_config.rx_checks[i].msg[j];
      if (msg->addr == 0) {
        continue;
      }
      const RxMsgChecks *checks = (msg->checks != NULL) ? msg->checks : &no_checks;
      if (((checks->counter.mask != 0U) && ((checks->get_counter != NULL) || (checks->counter.mask > 0xFFU) || !rx_field_valid(checks->counter, msg->len))) ||
          ((checks->checksum.mask != 0U) && ((checks->get_checksum != NULL) || !rx_field_valid(checks->checksum, msg->len)))) {
        return false;
      }
      if ((!msg->ignore_checksum && ((checks->compute_checksum == NULL) || ((checks->checksum.mask == 0U) && (checks->get_checksum == NULL)))) ||
          (((msg->max_counter > 0U) || !msg->ignore_counter) && ((msg->max_counter == 0U) || ((checks->counter.mask == 0U) && (checks->get_counter == NULL)))) ||
          (!msg->ignore_quality_flag && (checks->get_quality_flag_valid == NULL))) {
        return false;
      }
    }
  }
  return true;
}

uint32_t get_rx_msg_field(const CANPacket_t *msg, uint8_t byte, uint8_t shift, uint16_t mask) {
  return rx_get_field(msg, (RxMsgField){.byte = byte, .shift = shift, .mask = mask});
}

bool rx_check_missing_metadata(const CANPacket_t *msg, bool descriptor_present, bool ignore_checksum,
                               bool ignore_counter, uint8_t max_counter, bool ignore_quality_flag) {
  const RxMsgChecks no_checks = {0};
  RxCheck rx_checks[] = {
    {.msg = {{0x123, 0, 8, 50U, .ignore_checksum = ignore_checksum, .ignore_counter = ignore_counter,
              .max_counter = max_counter, .ignore_quality_flag = ignore_quality_flag, .checks = descriptor_present ? &no_checks : NULL}, {0}, {0}}},
  };
  const safety_config cfg = {.rx_checks = rx_checks, .rx_checks_len = 1};
  return rx_msg_safety_check(msg, &cfg);
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
