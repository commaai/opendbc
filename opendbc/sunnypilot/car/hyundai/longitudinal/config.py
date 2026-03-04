"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from dataclasses import dataclass, field

from opendbc.car.hyundai.values import CAR


@dataclass
class CarTuningConfig:
  v_ego_stopping: float = 0.25
  v_ego_starting: float = 0.10
  stopping_decel_rate: float = 0.40
  lookahead_jerk_bp: list[float] = field(default_factory=lambda: [2., 5., 20.])
  lookahead_jerk_upper_v: list[float] = field(default_factory=lambda: [0.3, 0.45, 0.6])
  lookahead_jerk_lower_v: list[float] = field(default_factory=lambda: [0.3, 0.45, 0.6])
  longitudinal_actuator_delay: float = 0.50
  jerk_limits: float = 4.0


# Default configurations for different car types
TUNING_CONFIGS = {
  "CANFD": CarTuningConfig(
    v_ego_stopping=0.30,
  ),
  "EV": CarTuningConfig(
    stopping_decel_rate=0.45,
    v_ego_stopping=0.35,
  ),
  "HYBRID": CarTuningConfig(
    v_ego_starting=0.15,
    stopping_decel_rate=0.45,
    v_ego_stopping=0.4,
  ),
  "DEFAULT": CarTuningConfig(
    v_ego_stopping=0.3,
  )
}

# Car-specific configs
CAR_SPECIFIC_CONFIGS = {
  CAR.KIA_NIRO_EV: CarTuningConfig(
    v_ego_stopping=0.1,
    stopping_decel_rate=0.3,
    jerk_limits=3.3,
  ),
  CAR.KIA_NIRO_PHEV_2022: CarTuningConfig(
    stopping_decel_rate=0.8,
    jerk_limits=5.0,
  ),
}
