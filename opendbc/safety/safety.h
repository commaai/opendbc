#pragma once
#include "opendbc/safety/helpers.h"
#include "opendbc/safety/lateral.h"
#include "opendbc/safety/longitudinal.h"
#include "opendbc/safety/declarations.h"
#include "opendbc/safety/can.h"
#include "opendbc/safety/ignition.h"
// all the safety modes
#include "opendbc/safety/modes/defaults.h"
#include "opendbc/safety/modes/honda.h"
#include "opendbc/safety/modes/toyota.h"
#include "opendbc/safety/modes/tesla.h"
#include "opendbc/safety/modes/gm.h"
#include "opendbc/safety/modes/ford.h"
#include "opendbc/safety/modes/hyundai.h"
#include "opendbc/safety/modes/chrysler.h"
#include "opendbc/safety/modes/rivian.h"
#include "opendbc/safety/modes/subaru.h"
#include "opendbc/safety/modes/subaru_preglobal.h"
#include "opendbc/safety/modes/mazda.h"
#include "opendbc/safety/modes/nissan.h"
#include "opendbc/safety/modes/volkswagen_mqb.h"
#include "opendbc/safety/modes/volkswagen_pq.h"
#include "opendbc/safety/modes/elm327.h"
#include "opendbc/safety/modes/body.h"
#include "opendbc/safety/modes/psa.h"
#include "opendbc/safety/modes/hyundai_canfd.h"

uint32_t GET_BYTES(const CANPacket_t *msg, int start, int len) {
  uint32_t ret = 0U;
  for (int i = 0; i < len; i++) {
    const uint32_t shift = i * 8;
    ret |= (((uint32_t)msg->data[start + i]) << shift);
  }
  return ret;
}

const int MAX_WRONG_COUNTERS = 5;
// This can be set by the safety hooks
bool controls_allowed = false;
bool relay_malfunction = false;
bool gas_pressed = false;
bool gas_pressed_prev = false;
bool brake_pressed = false;
bool brake_pressed_prev = false;
bool regen_braking = false;
bool regen_braking_prev = false;
bool steering_disengage;
bool steering_disengage_prev;
bool cruise_engaged_prev = false;
struct sample_t vehicle_speed;
bool vehicle_moving = false;
bool acc_main_on = false;  // referred to as "ACC off" in ISO 15622:2018
int cruise_button_prev = 0;
bool safety_rx_checks_invalid = false;
// for safety modes with torque steering control
int desired_torque_last = 0;       // last desired steer torque
int rt_torque_last = 0;            // last desired torque for real time check
int valid_steer_req_count = 0;     // counter for steer request bit matching non-zero torque
int invalid_steer_req_count = 0;   // counter to allow multiple frames of mismatching torque request bit
struct sample_t torque_meas;       // last 6 motor torques produced by the eps
struct sample_t torque_driver;     // last 6 driver torques measured
uint32_t ts_torque_check_last = 0;
uint32_t ts_steer_req_mismatch_last = 0;  // last timestamp steer req was mismatched with torque
// state for controls_allowed timeout logic
bool heartbeat_engaged = false;             // openpilot enabled, passed in heartbeat USB command
uint32_t heartbeat_engaged_mismatches = 0;  // count of mismatches between heartbeat_engaged and controls_allowed
// for safety modes with angle steering control
uint32_t rt_angle_msgs = 0;
uint32_t ts_angle_check_last = 0;
int desired_angle_last = 0;
struct sample_t angle_meas;         // last 6 steer angles/curvatures
int alternative_experience = 0;

// time since safety mode has been changed
uint32_t safety_mode_cnt = 0U;
uint16_t current_safety_mode = SAFETY_SILENT;
uint16_t current_safety_param = 0;
static const safety_hooks *current_hooks = &nooutput_hooks;
safety_config current_safety_config;
static void generic_rx_checks(void);
static void stock_ecu_check(bool stock_ecu_detected);

static bool is_msg_valid(RxCheck addr_list[], int index) {
  bool valid = true;
  if (index != -1) {
    if (!addr_list[index].status.valid_checksum || !addr_list[index].status.valid_quality_flag || (addr_list[index].status.wrong_counters >= MAX_WRONG_COUNTERS)) {
      valid = false;
      controls_allowed = false;
    }
  }
  return valid;
}

static int get_addr_check_index(const CANPacket_t *msg, RxCheck addr_list[], const int len) {
  int addr = msg->addr;
  int length = GET_LEN(msg);
  int index = -1;
  for (int i = 0; i < len; i++) {
    if (!addr_list[i].status.msg_seen) {
      for (uint8_t j = 0U; (j < MAX_ADDR_CHECK_MSGS) && (addr_list[i].msg[j].addr != 0); j++) {
        if ((addr == addr_list[i].msg[j].addr) && (msg->bus == addr_list[i].msg[j].bus) && (length == addr_list[i].msg[j].len)) {
          addr_list[i].status.index = j;
          addr_list[i].status.msg_seen = true;
          break;
        }
      }
    }
    if (addr_list[i].status.msg_seen) {
      int idx = addr_list[i].status.index;
      if ((addr == addr_list[i].msg[idx].addr) && (msg->bus == addr_list[i].msg[idx].bus) && (length == addr_list[i].msg[idx].len)) {
        index = i;
        break;
      }
    }
  }
  return index;
}

static void update_addr_timestamp(RxCheck addr_list[], int index) {
  if (index != -1) {
    uint32_t ts = microsecond_timer_get();
    addr_list[index].status.last_timestamp = ts;
  }
}

static void update_counter(RxCheck addr_list[], int index, uint8_t counter) {
  if (index != -1) {
    uint8_t expected_counter = (addr_list[index].status.last_counter + 1U) % (addr_list[index].msg[addr_list[index].status.index].max_counter + 1U);
    addr_list[index].status.wrong_counters += (expected_counter == counter) ? -1 : 1;
    addr_list[index].status.wrong_counters = SAFETY_CLAMP(addr_list[index].status.wrong_counters, 0, MAX_WRONG_COUNTERS);
    addr_list[index].status.last_counter = counter;
  }
}

static bool rx_msg_safety_check(const CANPacket_t *msg,
                                const safety_config *cfg,
                                const safety_hooks *safety_hooks) {
  int index = get_addr_check_index(msg, cfg->rx_checks, cfg->rx_checks_len);
  update_addr_timestamp(cfg->rx_checks, index);
  if (index != -1) {
    if ((safety_hooks->get_checksum != NULL) && (safety_hooks->compute_checksum != NULL) && !cfg->rx_checks[index].msg[cfg->rx_checks[index].status.index].ignore_checksum) {
      uint32_t checksum = safety_hooks->get_checksum(msg);
      uint32_t checksum_comp = safety_hooks->compute_checksum(msg);
      cfg->rx_checks[index].status.valid_checksum = checksum_comp == checksum;
    } else {
      cfg->rx_checks[index].status.valid_checksum = cfg->rx_checks[index].msg[cfg->rx_checks[index].status.index].ignore_checksum;
    }
    if ((safety_hooks->get_counter != NULL) && (cfg->rx_checks[index].msg[cfg->rx_checks[index].status.index].max_counter > 0U)) {
      uint8_t counter = safety_hooks->get_counter(msg);
      update_counter(cfg->rx_checks, index, counter);
    } else {
      cfg->rx_checks[index].status.wrong_counters = cfg->rx_checks[index].msg[cfg->rx_checks[index].status.index].ignore_counter ? 0 : MAX_WRONG_COUNTERS;
    }
    if ((safety_hooks->get_quality_flag_valid != NULL) && !cfg->rx_checks[index].msg[cfg->rx_checks[index].status.index].ignore_quality_flag) {
      cfg->rx_checks[index].status.valid_quality_flag = safety_hooks->get_quality_flag_valid(msg);
    } else {
      cfg->rx_checks[index].status.valid_quality_flag = cfg->rx_checks[index].msg[cfg->rx_checks[index].status.index].ignore_quality_flag;
    }
  }
  return is_msg_valid(cfg->rx_checks, index);
}

bool safety_rx_hook(const CANPacket_t *msg) {
  bool controls_allowed_prev = controls_allowed;
  bool valid = rx_msg_safety_check(msg, &current_safety_config, current_hooks);
  bool whitelisted = get_addr_check_index(msg, current_safety_config.rx_checks, current_safety_config.rx_checks_len) != -1;
  if (valid && whitelisted) {
    current_hooks->rx(msg);
    switch (current_safety_mode) {
      case SAFETY_GM:
      case SAFETY_MAZDA:
      case SAFETY_RIVIAN:
      case SAFETY_TESLA: {
        ignition_rx_hook_can(msg);
        break;
      }
      default:
        break;
    }
  }
  generic_rx_checks();
  const int addr = msg->addr;
  for (int i = 0; i < current_safety_config.tx_msgs_len; i++) {
    const CanMsg *m = &current_safety_config.tx_msgs[i];
    if (m->check_relay) {
      stock_ecu_check((m->addr == addr) && (m->bus == msg->bus));
    }
  }
  if (controls_allowed && !controls_allowed_prev) {
    heartbeat_engaged_mismatches = 0;
  }
  return valid;
}

static bool tx_msg_safety_check(const CANPacket_t *msg, const CanMsg msg_list[], int len) {
  int addr = msg->addr;
  int length = GET_LEN(msg);
  bool whitelisted = false;
  for (int i = 0; i < len; i++) {
    if ((addr == msg_list[i].addr) && (msg->bus == msg_list[i].bus) && (length == msg_list[i].len)) {
      whitelisted = true;
      break;
    }
  }
  return whitelisted;
}

bool safety_tx_hook(CANPacket_t *msg) {
  bool whitelisted = tx_msg_safety_check(msg, current_safety_config.tx_msgs, current_safety_config.tx_msgs_len);
  if ((current_safety_mode == SAFETY_ALLOUTPUT) || (current_safety_mode == SAFETY_ELM327)) {
    whitelisted = true;
  }
  bool safety_allowed = false;
  if (whitelisted) {
    safety_allowed = current_hooks->tx(msg);
  }
  return !relay_malfunction && whitelisted && safety_allowed;
}

static int get_fwd_bus(int bus_num) {
  int destination_bus;
  if (bus_num == 0) {
    destination_bus = 2;
  } else if (bus_num == 2) {
    destination_bus = 0;
  } else {
    destination_bus = -1;
  }
  return destination_bus;
}

int safety_fwd_hook(int bus_num, int addr) {
  bool blocked = relay_malfunction || current_safety_config.disable_forwarding;
  const int destination_bus = get_fwd_bus(bus_num);
  if (!blocked) {
    for (int i = 0; i < current_safety_config.tx_msgs_len; i++) {
      const CanMsg *m = &current_safety_config.tx_msgs[i];
      if (m->check_relay && !m->disable_static_blocking && (m->addr == addr) && (m->bus == (unsigned int)destination_bus)) {
        blocked = true;
        break;
      }
    }
  }
  if (!blocked && (current_hooks->fwd != NULL)) {
    blocked = current_hooks->fwd(bus_num, addr);
  }
  return blocked ? -1 : destination_bus;
}

void gen_crc_lookup_table_8(uint8_t poly, uint8_t crc_lut[]) {
  for (uint16_t i = 0U; i <= 0xFFU; i++) {
    uint8_t crc = (uint8_t)i;
    for (int j = 0; j < 8; j++) {
      if ((crc & 0x80U) != 0U) {
        crc = (uint8_t)((crc << 1) ^ poly);
      } else {
        crc <<= 1;
      }
    }
    crc_lut[i] = crc;
  }
}

void gen_crc_lookup_table_16(uint16_t poly, uint16_t crc_lut[]) {
  for (uint16_t i = 0; i < 256U; i++) {
    uint16_t crc = i << 8U;
    for (uint16_t j = 0; j < 8U; j++) {
      if ((crc & 0x8000U) != 0U) {
        crc = (uint16_t)((crc << 1) ^ poly);
      } else {
        crc <<= 1;
      }
    }
    crc_lut[i] = crc;
  }
}

void safety_tick(const safety_config *cfg) {
  const uint8_t MAX_MISSED_MSGS = 10U;
  bool rx_checks_invalid = false;
  uint32_t ts = microsecond_timer_get();
  if (cfg != NULL) {
    for (int i=0; i < cfg->rx_checks_len; i++) {
      uint32_t elapsed_time = safety_get_ts_elapsed(ts, cfg->rx_checks[i].status.last_timestamp);
      uint32_t timestep = 1e6 / cfg->rx_checks[i].msg[cfg->rx_checks[i].status.index].frequency;
      bool lagging = elapsed_time > SAFETY_MAX(timestep * MAX_MISSED_MSGS, 1e6);
      cfg->rx_checks[i].status.lagging = lagging;
      if (lagging) {
        controls_allowed = false;
      }
      if (lagging || !is_msg_valid(cfg->rx_checks, i)) {
        rx_checks_invalid = true;
      }
    }
  }
  safety_rx_checks_invalid = rx_checks_invalid;
}

static void relay_malfunction_set(void) {
  relay_malfunction = true;
}

static void generic_rx_checks(void) {
  gas_pressed_prev = gas_pressed;
  if (brake_pressed && (!brake_pressed_prev || vehicle_moving)) {
    controls_allowed = false;
  }
  brake_pressed_prev = brake_pressed;
  if (regen_braking && (!regen_braking_prev || vehicle_moving)) {
    controls_allowed = false;
  }
  regen_braking_prev = regen_braking;
  if (steering_disengage && !steering_disengage_prev) {
    controls_allowed = false;
  }
  steering_disengage_prev = steering_disengage;
}

static void stock_ecu_check(bool stock_ecu_detected) {
  const uint32_t RELAY_TRNS_TIMEOUT = 1U;
  if ((safety_mode_cnt > RELAY_TRNS_TIMEOUT) && stock_ecu_detected) {
    relay_malfunction_set();
  }
}

static void relay_malfunction_reset(void) {
  relay_malfunction = false;
}

static void reset_sample(struct sample_t *sample) {
  for (int i = 0; i < MAX_SAMPLE_VALS; i++) {
    sample->values[i] = 0;
  }
  update_sample(sample, 0);
}

int set_safety_hooks(uint16_t mode, uint16_t param) {
  const safety_hook_config safety_hook_registry[] = {
    {SAFETY_SILENT, &nooutput_hooks},
    {SAFETY_HONDA_NIDEC, &honda_nidec_hooks},
    {SAFETY_TOYOTA, &toyota_hooks},
    {SAFETY_ELM327, &elm327_hooks},
    {SAFETY_GM, &gm_hooks},
    {SAFETY_HONDA_BOSCH, &honda_bosch_hooks},
    {SAFETY_HYUNDAI, &hyundai_hooks},
    {SAF
