/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

#include "opendbc/safety/sunnypilot/safety_mads_declarations.h"

// ===============================
// Global Variables
// ===============================

ButtonState mads_button_press = MADS_BUTTON_UNAVAILABLE;
MADSState m_mads_state;

// state for mads controls_allowed_lat timeout logic
bool heartbeat_engaged_mads = false;  // MADS enabled, passed in heartbeat USB command
uint32_t heartbeat_engaged_mads_mismatches = 0U;  // count of mismatches between heartbeat_engaged_mads and controls_allowed_lat

// ===============================
// State Update Helpers
// ===============================

inline EdgeTransition m_get_edge_transition(const bool current, const bool last) {
  EdgeTransition state;

  if (current && !last) {
    state = MADS_EDGE_RISING;
  } else if (!current && last) {
    state = MADS_EDGE_FALLING;
  } else {
    state = MADS_EDGE_NO_CHANGE;
  }

  return state;
}

inline void m_mads_state_init(void) {
  m_mads_state.is_vehicle_moving = NULL;
  m_mads_state.acc_main.current = NULL;
  m_mads_state.mads_button.current = MADS_BUTTON_UNAVAILABLE;

  m_mads_state.system_enabled = false;
  m_mads_state.disengage_lateral_on_brake = false;
  m_mads_state.pause_lateral_on_brake = false;

  m_mads_state.acc_main.previous = false;
  m_mads_state.acc_main.transition = MADS_EDGE_NO_CHANGE;

  m_mads_state.mads_button.last = MADS_BUTTON_UNAVAILABLE;
  m_mads_state.mads_button.transition = MADS_EDGE_NO_CHANGE;


  m_mads_state.current_disengage.active_reason = MADS_DISENGAGE_REASON_NONE;
  m_mads_state.current_disengage.pending_reasons = MADS_DISENGAGE_REASON_NONE;

  m_mads_state.controls_requested_lat = false;
  m_mads_state.controls_allowed_lat = false;
}

inline void m_update_button_state(ButtonStateTracking *button_state) {
  if (button_state->current != MADS_BUTTON_UNAVAILABLE) {
    button_state->transition = m_get_edge_transition(button_state->current == MADS_BUTTON_PRESSED, button_state->last == MADS_BUTTON_PRESSED);
    button_state->last = button_state->current;
  }
}

inline void m_update_binary_state(BinaryStateTracking *state) {
  state->transition = m_get_edge_transition(state->current, state->previous);
  state->previous = state->current;
}

/**
 * @brief Updates the MADS control state based on current system conditions
 * 
 * @return void
 */
inline void m_update_control_state(void) {
  bool allowed = true;

  // Initial control requests from button or ACC transitions
  if ((m_mads_state.acc_main.transition == MADS_EDGE_RISING) ||
      (m_mads_state.mads_button.transition == MADS_EDGE_RISING) ||
      (m_mads_state.op_controls_allowed.transition == MADS_EDGE_RISING)) {
    m_mads_state.controls_requested_lat = true;
  }

  // Primary control blockers - these prevent any further control processing
  if (m_mads_state.acc_main.transition == MADS_EDGE_FALLING) {
    mads_exit_controls(MADS_DISENGAGE_REASON_ACC_MAIN_OFF);
    allowed = false;  // No matter what, no further control processing on this cycle
  }

  if (m_mads_state.mads_steering_disengage.transition == MADS_EDGE_RISING) {
    mads_exit_controls(MADS_DISENGAGE_REASON_STEERING_DISENGAGE);
    allowed = false;  // No matter what, no further control processing on this cycle
  }

  if (m_mads_state.disengage_lateral_on_brake && (m_mads_state.braking.transition == MADS_EDGE_RISING)) {
    mads_exit_controls(MADS_DISENGAGE_REASON_BRAKE);
    allowed = false;
  }

  // Secondary control conditions - only checked if primary conditions don't block further control processing
  if (allowed && m_mads_state.pause_lateral_on_brake) {
    // Brake rising edge immediately blocks controls
    // Brake release might request controls if brake was the ONLY reason for disengagement
    if (m_mads_state.braking.transition == MADS_EDGE_RISING) {
      mads_exit_controls(MADS_DISENGAGE_REASON_BRAKE);
      allowed = false;
    } else if ((m_mads_state.braking.transition == MADS_EDGE_FALLING) &&
               (m_mads_state.current_disengage.active_reason == MADS_DISENGAGE_REASON_BRAKE) &&
               (m_mads_state.current_disengage.pending_reasons == MADS_DISENGAGE_REASON_BRAKE)) {
      m_mads_state.controls_requested_lat = true;
    } else if (m_mads_state.braking.current) {
      allowed = false;
    } else {
    }
  }

  // Process control request if conditions allow
  if (allowed && m_mads_state.controls_requested_lat && !m_mads_state.controls_allowed_lat) {
    m_mads_state.controls_requested_lat = false;
    m_mads_state.controls_allowed_lat = true;
    m_mads_state.current_disengage.active_reason = MADS_DISENGAGE_REASON_NONE;
    m_mads_state.current_disengage.pending_reasons = MADS_DISENGAGE_REASON_NONE;
  }
}

inline void mads_heartbeat_engaged_check(void) {
  if (m_mads_state.controls_allowed_lat && !heartbeat_engaged_mads) {
    heartbeat_engaged_mads_mismatches += 1U;
    if (heartbeat_engaged_mads_mismatches >= 3U) {
      mads_exit_controls(MADS_DISENGAGE_REASON_HEARTBEAT_ENGAGED_MISMATCH);
    }
  } else {
    heartbeat_engaged_mads_mismatches = 0U;
  }
}

// ===============================
// Function Implementations
// ===============================

inline void mads_set_alternative_experience(const int *mode) {
  const bool mads_enabled = (*mode & ALT_EXP_ENABLE_MADS) != 0;
  const bool disengage_lateral_on_brake = (*mode & ALT_EXP_MADS_DISENGAGE_LATERAL_ON_BRAKE) != 0;
  const bool pause_lateral_on_brake = (*mode & ALT_EXP_MADS_PAUSE_LATERAL_ON_BRAKE) != 0;

  mads_set_system_state(mads_enabled, disengage_lateral_on_brake, pause_lateral_on_brake);
}

extern inline void mads_set_system_state(const bool enabled, const bool disengage_lateral_on_brake, const bool pause_lateral_on_brake) {
  m_mads_state_init();
  m_mads_state.system_enabled = enabled;
  m_mads_state.disengage_lateral_on_brake = disengage_lateral_on_brake;
  m_mads_state.pause_lateral_on_brake = pause_lateral_on_brake;
}

inline void mads_exit_controls(const DisengageReason reason) {
  // Always track this as a pending reason
  m_mads_state.current_disengage.pending_reasons |= reason;

  if (m_mads_state.controls_allowed_lat) {
    m_mads_state.current_disengage.active_reason = reason;
    m_mads_state.controls_requested_lat = false;
    m_mads_state.controls_allowed_lat = false;
  }
}

inline bool mads_is_lateral_control_allowed_by_mads(void) {
  return m_mads_state.system_enabled && m_mads_state.controls_allowed_lat;
}

inline void mads_state_update(const bool op_vehicle_moving, const bool op_acc_main, const bool op_allowed, const bool is_braking, const bool _steering_disengage) {
  m_mads_state.is_vehicle_moving = op_vehicle_moving;
  m_mads_state.acc_main.current = op_acc_main;
  m_mads_state.op_controls_allowed.current = op_allowed;
  m_mads_state.mads_button.current = mads_button_press;
  m_mads_state.braking.current = is_braking;
  m_mads_state.mads_steering_disengage.current = _steering_disengage;

  m_update_binary_state(&m_mads_state.acc_main);
  m_update_binary_state(&m_mads_state.op_controls_allowed);
  m_update_binary_state(&m_mads_state.braking);
  m_update_binary_state(&m_mads_state.mads_steering_disengage);
  m_update_button_state(&m_mads_state.mads_button);

  m_update_control_state();
}
