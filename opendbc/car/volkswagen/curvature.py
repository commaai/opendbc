import numpy as np
from dataclasses import dataclass
from typing import Tuple
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, DT_CTRL
from opendbc.car.lateral import ISO_LATERAL_ACCEL, ISO_LATERAL_JERK

AVERAGE_ROAD_ROLL = 0.06
MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL)


@dataclass
class CurvatureSteeringLimits:
  CURVATURE_MAX: float


def get_max_curvature_jerk(v_ego: float, steer_step: int) -> float:
  ts_elapsed = steer_step * DT_CTRL
  curvature_rate_limit = ISO_LATERAL_JERK / (max(v_ego, 1.0) ** 2)
  return curvature_rate_limit * ts_elapsed


def get_max_curvature_average(v_ego: float) -> Tuple[float, float]:
  max_curvature = MAX_LATERAL_ACCEL / (max(v_ego, 1.0) ** 2)
  return -max_curvature, max_curvature


def apply_std_curvature_limits(apply_curvature: float, apply_curvature_last: float, v_ego: float, curvature: float, override: bool,
                               steer_step: int, lat_active: bool, limits: CurvatureSteeringLimits) -> float:

  new_apply_curvature = apply_curvature

  max_jerk = get_max_curvature_jerk(v_ego, steer_step)
  curvature_up = apply_curvature_last + max_jerk
  curvature_down = apply_curvature_last - max_jerk

  new_apply_curvature = float(np.clip(new_apply_curvature, curvature_down, curvature_up))

  min_curvature, max_curvature = get_max_curvature_average(v_ego)
  new_apply_curvature = float(np.clip(new_apply_curvature, min_curvature, max_curvature))

  if not lat_active:
    new_apply_curvature = curvature

  return float(np.clip(new_apply_curvature, -limits.CURVATURE_MAX, limits.CURVATURE_MAX))
