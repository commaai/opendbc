"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from opendbc.car import structs
from opendbc.car.lateral import apply_center_deadzone


def get_friction(lateral_accel_error: float, lateral_accel_deadzone: float, friction_threshold: float,
                 torque_params: structs.CarParams.LateralTorqueTuning) -> float:
  # TODO torque params' friction should be in lataxel space, not torque space
  friction_interp = np.interp(
    apply_center_deadzone(lateral_accel_error, lateral_accel_deadzone),
    [-friction_threshold, friction_threshold],
    [-torque_params.friction, torque_params.friction]
  )
  return float(friction_interp)
