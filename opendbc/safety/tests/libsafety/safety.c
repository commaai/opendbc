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

void set_timer(uint32_t t){
  timer_cnt = t;
}

// speed in m/s
float get_vehicle_speed_min(void){
  return safety.vehicle_speed.min / VEHICLE_SPEED_FACTOR;
}

float get_vehicle_speed_max(void){
  return safety.vehicle_speed.max / VEHICLE_SPEED_FACTOR;
}

// ***** car specific helpers *****

void set_honda_alt_brake_msg(bool c){
  honda_alt_brake_msg = c;
}

void init_tests(void){
  safety.mode_cnt = 2U;  // avoid ignoring relay_malfunction logic
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
