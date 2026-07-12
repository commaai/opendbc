#pragma once

// Test-only RPC transported in a rejected CAN FD packet. This is only present
// in debug panda builds and lets the Python safety tests exercise the exact
// safety code running on the MCU.
#define SAFETY_TEST_ADDR 0x1FFFFF00U

enum safety_test_op {
  TEST_SET_HOOKS = 1,
  TEST_RX_HOOK,
  TEST_TX_HOOK,
  TEST_FWD_HOOK,
  TEST_LOAD_PACKET,
  TEST_TICK,
  TEST_CONFIG_VALID,
  TEST_INIT,
  TEST_IGNITION_HOOK,
  TEST_GET,
  TEST_SET,
  TEST_RX_PACKET,
  TEST_TX_PACKET,
  TEST_FWD_HOOK_STATE,
  TEST_TX_PACKET_STATE,
  TEST_RX_PACKET_STATE,
};

enum safety_test_value {
  TEST_CONTROLS_ALLOWED = 1,
  TEST_LONGITUDINAL_ALLOWED,
  TEST_ALTERNATIVE_EXPERIENCE,
  TEST_RELAY_MALFUNCTION,
  TEST_GAS_PRESSED_PREV,
  TEST_BRAKE_PRESSED_PREV,
  TEST_REGEN_BRAKING_PREV,
  TEST_STEERING_DISENGAGE_PREV,
  TEST_ACC_MAIN_ON,
  TEST_VEHICLE_SPEED_MIN,
  TEST_VEHICLE_SPEED_MAX,
  TEST_CURRENT_SAFETY_MODE,
  TEST_CURRENT_SAFETY_PARAM,
  TEST_TORQUE_MEAS_MIN,
  TEST_TORQUE_MEAS_MAX,
  TEST_TORQUE_DRIVER_MIN,
  TEST_TORQUE_DRIVER_MAX,
  TEST_DESIRED_ANGLE_LAST,
  TEST_ANGLE_MEAS_MIN,
  TEST_ANGLE_MEAS_MAX,
  TEST_DESIRED_CURVATURE_LAST,
  TEST_CURVATURE_MEAS_MIN,
  TEST_CURVATURE_MEAS_MAX,
  TEST_CRUISE_ENGAGED_PREV,
  TEST_VEHICLE_MOVING,
  TEST_HONDA_FWD_BRAKE,
  TEST_HONDA_HW,
  TEST_IGNITION_CAN,
  TEST_TIMER,
  TEST_TORQUE_MEAS,
  TEST_TORQUE_DRIVER,
  TEST_DESIRED_TORQUE_LAST,
  TEST_RT_TORQUE_LAST,
  TEST_ANGLE_MEAS,
  TEST_DESIRED_CURVATURE,
  TEST_CURVATURE_MEAS,
  TEST_HONDA_ALT_BRAKE,
  TEST_HONDA_BOSCH_LONG,
  TEST_TIMER_ELAPSED,
};

static CANPacket_t test_packet;
static uint32_t test_timer_now;
static uint32_t test_ts_torque;
static uint32_t test_ts_angle;
static uint32_t test_ts_curvature;

static int32_t test_get_i32(const CANPacket_t *msg, int offset) {
  uint32_t ret = 0U;
  for (int i = 0; i < 4; i++) ret |= ((uint32_t)msg->data[offset + i]) << (8 * i);
  return (int32_t)ret;
}

static void test_put_i32(CANPacket_t *msg, int32_t value) {
  for (int i = 0; i < 4; i++) msg->data[1 + i] = ((uint32_t)value >> (8 * i)) & 0xFFU;
}

static int32_t safety_test_get(uint8_t value) {
  int32_t ret = 0;
  switch (value) {
    case TEST_CONTROLS_ALLOWED: ret = controls_allowed; break;
    case TEST_LONGITUDINAL_ALLOWED: ret = get_longitudinal_allowed(); break;
    case TEST_ALTERNATIVE_EXPERIENCE: ret = alternative_experience; break;
    case TEST_RELAY_MALFUNCTION: ret = relay_malfunction; break;
    case TEST_GAS_PRESSED_PREV: ret = gas_pressed_prev; break;
    case TEST_BRAKE_PRESSED_PREV: ret = brake_pressed_prev; break;
    case TEST_REGEN_BRAKING_PREV: ret = regen_braking_prev; break;
    case TEST_STEERING_DISENGAGE_PREV: ret = steering_disengage_prev; break;
    case TEST_ACC_MAIN_ON: ret = acc_main_on; break;
    case TEST_VEHICLE_SPEED_MIN: ret = vehicle_speed.min; break;
    case TEST_VEHICLE_SPEED_MAX: ret = vehicle_speed.max; break;
    case TEST_CURRENT_SAFETY_MODE: ret = current_safety_mode; break;
    case TEST_CURRENT_SAFETY_PARAM: ret = current_safety_param; break;
    case TEST_TORQUE_MEAS_MIN: ret = torque_meas.min; break;
    case TEST_TORQUE_MEAS_MAX: ret = torque_meas.max; break;
    case TEST_TORQUE_DRIVER_MIN: ret = torque_driver.min; break;
    case TEST_TORQUE_DRIVER_MAX: ret = torque_driver.max; break;
    case TEST_DESIRED_ANGLE_LAST: ret = desired_angle_last; break;
    case TEST_ANGLE_MEAS_MIN: ret = angle_meas.min; break;
    case TEST_ANGLE_MEAS_MAX: ret = angle_meas.max; break;
    case TEST_DESIRED_CURVATURE_LAST: ret = curvature_state.desired_last; break;
    case TEST_CURVATURE_MEAS_MIN: ret = curvature_state.meas.min; break;
    case TEST_CURVATURE_MEAS_MAX: ret = curvature_state.meas.max; break;
    case TEST_CRUISE_ENGAGED_PREV: ret = cruise_engaged_prev; break;
    case TEST_VEHICLE_MOVING: ret = vehicle_moving; break;
    case TEST_HONDA_FWD_BRAKE: ret = honda_fwd_brake; break;
    case TEST_HONDA_HW: ret = honda_hw; break;
    case TEST_IGNITION_CAN: ret = ignition_can; break;
    case TEST_TIMER: ret = microsecond_timer_get(); break;
    default: break;
  }
  return ret;
}

static void safety_test_set(uint8_t value, int32_t a, int32_t b) {
  switch (value) {
    case TEST_CONTROLS_ALLOWED: controls_allowed = a != 0; break;
    case TEST_ALTERNATIVE_EXPERIENCE: alternative_experience = a; break;
    case TEST_RELAY_MALFUNCTION: relay_malfunction = a != 0; break;
    case TEST_GAS_PRESSED_PREV: gas_pressed_prev = a != 0; break;
    case TEST_CRUISE_ENGAGED_PREV: cruise_engaged_prev = a != 0; break;
    case TEST_TORQUE_MEAS: torque_meas.min = a; torque_meas.max = b; break;
    case TEST_TORQUE_DRIVER: torque_driver.min = a; torque_driver.max = b; break;
    case TEST_DESIRED_TORQUE_LAST: desired_torque_last = a; break;
    case TEST_RT_TORQUE_LAST: rt_torque_last = a; break;
    case TEST_DESIRED_ANGLE_LAST: desired_angle_last = a; break;
    case TEST_ANGLE_MEAS: angle_meas.min = a; angle_meas.max = b; break;
    case TEST_DESIRED_CURVATURE: curvature_state.desired_last = a; break;
    case TEST_CURVATURE_MEAS: curvature_state.meas.min = a; curvature_state.meas.max = b; break;
    case TEST_HONDA_FWD_BRAKE: honda_fwd_brake = a != 0; break;
    case TEST_HONDA_ALT_BRAKE: honda_alt_brake_msg = a != 0; break;
    case TEST_HONDA_BOSCH_LONG: honda_bosch_long = a != 0; break;
    case TEST_TIMER_ELAPSED: test_timer_now = a; break;
    case TEST_IGNITION_CAN: ignition_can = a != 0; break;
    default: break;
  }
}

bool safety_test_harness(CANPacket_t *msg) {
  if ((msg->addr != SAFETY_TEST_ADDR) || !msg->fd || (GET_LEN(msg) != 64)) return false;

  int32_t ret = 0;
  const uint8_t op = msg->data[0];
  if (op == TEST_LOAD_PACKET) {
    test_packet.fd = msg->data[1] & 1U;
    test_packet.bus = msg->data[2];
    test_packet.data_len_code = msg->data[3];
    test_packet.addr = (uint32_t)test_get_i32(msg, 4);
    int offset = msg->data[8];
    for (int i = 9; i < 64; i++) {
      if ((offset + i - 9) < 64) test_packet.data[offset + i - 9] = msg->data[i];
    }
  } else if ((op == TEST_RX_PACKET) || (op == TEST_TX_PACKET)) {
    test_packet.fd = msg->data[1] & 1U;
    test_packet.bus = msg->data[2];
    test_packet.data_len_code = msg->data[3];
    test_packet.addr = (uint32_t)test_get_i32(msg, 4);
    for (int i = 8; i < 64; i++) test_packet.data[i - 8] = msg->data[i];
    ret = (op == TEST_RX_PACKET) ? safety_rx_hook(&test_packet) : safety_tx_hook(&test_packet);
  } else if ((op == TEST_TX_PACKET_STATE) || (op == TEST_RX_PACKET_STATE)) {
    test_packet.fd = msg->data[1] & 1U;
    test_packet.bus = msg->data[2];
    test_packet.data_len_code = msg->data[3];
    test_packet.addr = (uint32_t)test_get_i32(msg, 4);
    for (int i = 8; i < 62; i++) test_packet.data[i - 8] = msg->data[i];
    controls_allowed = msg->data[62] != 0U;
    relay_malfunction = msg->data[63] != 0U;
    // Map persistent logical timestamp ages onto the hardware clock around
    // each hook, excluding debug transport latency between hook calls.
    if ((op == TEST_TX_PACKET_STATE) && ((msg->data[1] & 6U) != 0U)) {
      const uint32_t now = microsecond_timer_get();
      uint32_t torque_elapsed = test_timer_now - test_ts_torque;
      uint32_t angle_elapsed = test_timer_now - test_ts_angle;
      uint32_t curvature_elapsed = test_timer_now - test_ts_curvature;
      // Leave room for the few microseconds between this anchor and the timer
      // read inside the hook, without altering exact/over-boundary advances.
      if ((torque_elapsed != MAX_RT_INTERVAL) && (torque_elapsed > 100U)) torque_elapsed -= 100U;
      if ((angle_elapsed != MAX_RT_INTERVAL) && (angle_elapsed > 100U)) angle_elapsed -= 100U;
      if ((curvature_elapsed != (MAX_RT_INTERVAL / 2U)) && (curvature_elapsed > 100U)) curvature_elapsed -= 100U;
      const uint32_t torque_anchor = now - torque_elapsed;
      const uint32_t angle_anchor = now - angle_elapsed;
      const uint32_t curvature_anchor = now - curvature_elapsed;
      ts_torque_check_last = torque_anchor;
      ts_angle_check_last = angle_anchor;
      curvature_state.ts_check_last = curvature_anchor;

      ret = safety_tx_hook(&test_packet);
      if (ts_torque_check_last != torque_anchor) test_ts_torque = test_timer_now;
      if (ts_angle_check_last != angle_anchor) test_ts_angle = test_timer_now;
      if (curvature_state.ts_check_last != curvature_anchor) test_ts_curvature = test_timer_now;
    } else {
      ret = (op == TEST_RX_PACKET_STATE) ? safety_rx_hook(&test_packet) : safety_tx_hook(&test_packet);
    }
  } else if (op == TEST_SET_HOOKS) {
    ret = set_safety_hooks(test_get_i32(msg, 1), test_get_i32(msg, 5));
  } else if (op == TEST_RX_HOOK) {
    ret = safety_rx_hook(&test_packet);
  } else if (op == TEST_TX_HOOK) {
    ret = safety_tx_hook(&test_packet);
  } else if (op == TEST_FWD_HOOK) {
    ret = safety_fwd_hook(test_get_i32(msg, 1), test_get_i32(msg, 5));
  } else if (op == TEST_FWD_HOOK_STATE) {
    relay_malfunction = msg->data[9] != 0U;
    ret = safety_fwd_hook(test_get_i32(msg, 1), test_get_i32(msg, 5));
  } else if (op == TEST_TICK) {
    safety_tick(&current_safety_config);
  } else if (op == TEST_CONFIG_VALID) {
    ret = current_safety_config.rx_checks_len > 0;
    for (int i = 0; i < current_safety_config.rx_checks_len; i++) {
      const RxStatus *s = &current_safety_config.rx_checks[i].status;
      ret = ret && s->msg_seen && !s->lagging && s->valid_checksum && (s->wrong_counters < MAX_WRONG_COUNTERS) && s->valid_quality_flag;
    }
  } else if (op == TEST_INIT) {
    safety_mode_cnt = 2U;
    alternative_experience = 0;
    ts_steer_req_mismatch_last = 0U;
    valid_steer_req_count = 0;
    invalid_steer_req_count = 0;
    tesla_autopark = false;
    ignition_can = false;
    ignition_can_cnt = 0U;
    test_timer_now = 0U;
    test_ts_torque = 0U;
    test_ts_angle = 0U;
    test_ts_curvature = 0U;
  } else if (op == TEST_IGNITION_HOOK) {
    ignition_can_hook(&test_packet);
  } else if (op == TEST_GET) {
    ret = safety_test_get(msg->data[1]);
  } else if (op == TEST_SET) {
    safety_test_set(msg->data[1], test_get_i32(msg, 2), test_get_i32(msg, 6));
  }
  test_put_i32(msg, ret);
  msg->data[5] = relay_malfunction;
  msg->data[6] = current_safety_mode & 0xFFU;
  msg->data[7] = current_safety_mode >> 8U;
  msg->data[8] = controls_allowed;
  return true;
}
