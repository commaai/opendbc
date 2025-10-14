/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

extern bool subaru_stop_and_go;
bool subaru_stop_and_go = false;

void subaru_common_init(void) {
  const int SUBARU_PARAM_SP_STOP_AND_GO = 1;

  subaru_stop_and_go = GET_FLAG(current_safety_param_sp, SUBARU_PARAM_SP_STOP_AND_GO);
}

/*
bool subaru_common_stop_and_go_throttle_check(const int throttle_pedal) {
  bool violation = throttle_pedal != 5U || !controls_allowed || vehicle_moving;
  return violation;
}

bool subaru_common_stop_and_go_brake_pedal_check(const int speed, const bool is_preglobal) {
  int val = is_preglobal ? 1U : 3U;
  bool violation = speed != val || !controls_allowed || vehicle_moving;
  return violation;
}
*/
