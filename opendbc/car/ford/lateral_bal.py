from __future__ import annotations

import math


# Ford CAN-FD path assist using c0/c1 only. c2/c3 remain inactive because c2 is
# too slow and limited for high-angle maneuvers on this platform.
FORD_PATH_C0_CAN_CLIP = (-0.35, 0.35)
FORD_PATH_C1_CAN_CLIP = (-0.42, 0.42)

FORD_PATH_MIN_ASSIST_CURVATURE = 0.0012
FORD_PATH_CURVATURE_ERROR = 0.008
FORD_PATH_CURVATURE_ERROR_DEADBAND = 0.0003

FORD_PATH_C1_LOOKAHEAD_SPEED_BP = (0.0, 5.0, 15.0, 25.0, 35.0)
FORD_PATH_C1_LOOKAHEAD = (5.0, 6.0, 8.0, 7.0, 5.5)
FORD_PATH_C1_ERROR_GAIN = (0.0, 1.5, 5.0, 4.0, 3.0)
FORD_PATH_C1_RATE_SPEED_BP = (0.0, 15.0, 35.0)
FORD_PATH_C1_RATE = (0.12, 0.18, 0.14)

FORD_PATH_C0_ERROR_GAIN = (0.0, 8.0, 20.0, 18.0, 12.0)
FORD_PATH_C0_CURVATURE_BP = (0.0, FORD_PATH_MIN_ASSIST_CURVATURE, 0.004, 0.012, 0.02)
FORD_PATH_C0_CURVATURE_GAIN = (0.0, 0.0, 0.6, 1.0, 0.9)
FORD_PATH_C0_TRACKING_RATIO_BP = (0.0, 0.45, 0.8, 1.0)
FORD_PATH_C0_TRACKING_RATIO_GAIN = (1.0, 0.8, 0.25, 0.0)
FORD_PATH_C0_LIMIT_SPEED_BP = (0.0, 10.0, 22.35, 35.0)
FORD_PATH_C0_LIMIT = (0.08, 0.16, 0.24, 0.18)
FORD_PATH_C0_RATE_UP = 0.12
FORD_PATH_C0_RATE_DOWN = 0.20


def _clip(value: float, lo: float, hi: float) -> float:
  return lo if value < lo else (hi if value > hi else value)


def _interp(x: float, xs: tuple[float, ...], ys: tuple[float, ...]) -> float:
  if x <= xs[0]:
    return ys[0]
  if x >= xs[-1]:
    return ys[-1]

  for i in range(1, len(xs)):
    if x < xs[i]:
      t = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
      return ys[i - 1] + t * (ys[i] - ys[i - 1])
  return ys[-1]


def _finite(value: float, fallback: float = 0.0) -> float:
  return float(value) if math.isfinite(value) else fallback


def _apply_deadband(value: float, deadband: float) -> float:
  if abs(value) <= deadband:
    return 0.0
  return math.copysign(abs(value) - deadband, value)


def _rate_limit(value: float, last_value: float, up_step: float, down_step: float) -> float:
  same_direction = value * last_value >= 0.0
  winding_up = same_direction and abs(value) > abs(last_value)
  step = up_step if winding_up else down_step
  return _clip(value, last_value - step, last_value + step)


def _undertracking_error(desired_curvature: float, current_curvature: float) -> float:
  desired_abs = abs(desired_curvature)
  if desired_abs < FORD_PATH_MIN_ASSIST_CURVATURE:
    return 0.0

  curvature_error = _clip(desired_curvature - current_curvature, -FORD_PATH_CURVATURE_ERROR, FORD_PATH_CURVATURE_ERROR)
  if curvature_error * desired_curvature <= 0.0:
    return 0.0
  return _apply_deadband(curvature_error, FORD_PATH_CURVATURE_ERROR_DEADBAND)


def _tracking_gain(desired_curvature: float, current_curvature: float) -> float:
  if desired_curvature * current_curvature <= 0.0:
    return 1.0

  tracking_ratio = abs(current_curvature) / max(abs(desired_curvature), FORD_PATH_MIN_ASSIST_CURVATURE)
  return _interp(tracking_ratio, FORD_PATH_C0_TRACKING_RATIO_BP, FORD_PATH_C0_TRACKING_RATIO_GAIN)


def lightweight_path_from_curvature(desired_curvature: float, current_curvature: float, v_ego: float,
                                    path_offset_last: float, path_angle_last: float,
                                    lat_active: bool) -> tuple[float, float, float]:
  """Return Ford c0/c1 path assist with c2 held inactive.

  c1 carries the main requested curve shape. c0 is only an under-tracking assist:
  it is not derived from model lateral position, and it unwinds as measured
  curvature catches up so it does not become a persistent lane-hugging bias.
  """
  if not lat_active:
    return 0.0, 0.0, 0.0

  desired_curvature = _finite(desired_curvature)
  current_curvature = _finite(current_curvature)
  v_ego = _finite(v_ego)
  path_offset_last = _finite(path_offset_last)
  path_angle_last = _finite(path_angle_last)

  curvature_error = _undertracking_error(desired_curvature, current_curvature)

  path_angle = desired_curvature * _interp(v_ego, FORD_PATH_C1_LOOKAHEAD_SPEED_BP, FORD_PATH_C1_LOOKAHEAD)
  if curvature_error != 0.0:
    path_angle += curvature_error * _interp(v_ego, FORD_PATH_C1_LOOKAHEAD_SPEED_BP, FORD_PATH_C1_ERROR_GAIN)
  c1_rate = _interp(v_ego, FORD_PATH_C1_RATE_SPEED_BP, FORD_PATH_C1_RATE)
  path_angle = _rate_limit(path_angle, path_angle_last, c1_rate, c1_rate)

  path_offset = 0.0
  if curvature_error != 0.0:
    curvature_gain = _interp(abs(desired_curvature), FORD_PATH_C0_CURVATURE_BP, FORD_PATH_C0_CURVATURE_GAIN)
    path_offset = curvature_error * _interp(v_ego, FORD_PATH_C1_LOOKAHEAD_SPEED_BP, FORD_PATH_C0_ERROR_GAIN)
    path_offset *= curvature_gain * _tracking_gain(desired_curvature, current_curvature)
    c0_limit = _interp(v_ego, FORD_PATH_C0_LIMIT_SPEED_BP, FORD_PATH_C0_LIMIT)
    path_offset = _clip(path_offset, -c0_limit, c0_limit)
  path_offset = _rate_limit(path_offset, path_offset_last, FORD_PATH_C0_RATE_UP, FORD_PATH_C0_RATE_DOWN)

  return (
    _clip(path_offset, *FORD_PATH_C0_CAN_CLIP),
    _clip(path_angle, *FORD_PATH_C1_CAN_CLIP),
    0.0,
  )
