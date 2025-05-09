"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from opendbc.car import structs, DT_CTRL, rate_limit
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.longitudinal.config import CarTuningConfig, TUNING_CONFIGS, CAR_SPECIFIC_CONFIGS

JERK_THRESHOLD = 0.1
JERK_STEP = 0.1


class LongitudinalTuningType:
  OFF = 0
  DYNAMIC = 1
  PREDICTIVE = 2


def get_car_config(CP: structs.CarParams) -> CarTuningConfig:
  # Get car type flags from specific configs or determine from car flags
  car_config = CAR_SPECIFIC_CONFIGS.get(CP.carFingerprint)
  # If car is not in specific configs, determine from flags
  if car_config is None:
    if CP.flags & HyundaiFlags.CANFD:
      car_config = TUNING_CONFIGS["CANFD"]
    elif CP.flags & HyundaiFlags.EV:
      car_config = TUNING_CONFIGS["EV"]
    elif CP.flags & HyundaiFlags.HYBRID:
      car_config = TUNING_CONFIGS["HYBRID"]
    else:
      car_config = TUNING_CONFIGS["DEFAULT"]

  return car_config


def get_longitudinal_tune(CP: structs.CarParams) -> None:
  config = get_car_config(CP)
  CP.vEgoStopping = config.v_ego_stopping
  CP.vEgoStarting = config.v_ego_starting
  CP.stoppingDecelRate = config.stopping_decel_rate
  CP.startingState = False
  CP.longitudinalActuatorDelay = config.longitudinal_actuator_delay


def jerk_limited_integrator(desired_accel, last_accel, jerk_upper, jerk_lower) -> float:
  if desired_accel >= last_accel:
    val = jerk_upper * DT_CTRL * 2
  else:
    val = jerk_lower * DT_CTRL * 2

  return rate_limit(desired_accel, last_accel, -val, val)


def ramp_update(current, target):
  error = target - current
  if abs(error) > JERK_THRESHOLD:
    return current + float(np.clip(error, -JERK_STEP, JERK_STEP))
  return target
