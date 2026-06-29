from __future__ import annotations

import math


# Ford CAN-FD path assist using model-derived c0/c1 only. c2/c3 stay inactive.
FORD_PATH_C0_CAN_CLIP = (-0.35, 0.35)
FORD_PATH_C1_CAN_CLIP = (-0.42, 0.42)

FORD_PATH_LOOKAHEAD_TIME = 0.65
FORD_PATH_LOOKAHEAD_MIN = 6.0
FORD_PATH_LOOKAHEAD_MAX = 14.0
FORD_PATH_FULL_CURVATURE = 0.010
FORD_PATH_FULL_LAT_ACCEL = 1.5
FORD_PATH_FULL_STEER_ANGLE_DEG = 12.0
FORD_PATH_C0_RATE = 0.08
FORD_PATH_C1_RATE = 0.16


def _clip(value: float, lo: float, hi: float) -> float:
  return lo if value < lo else (hi if value > hi else value)


def _interp(x: float, xs, ys) -> float:
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


def _rate_limit(value: float, last_value: float, step: float) -> float:
  return _clip(value, last_value - step, last_value + step)


def _path_lookahead(v_ego: float) -> float:
  return _clip(v_ego * FORD_PATH_LOOKAHEAD_TIME, FORD_PATH_LOOKAHEAD_MIN, FORD_PATH_LOOKAHEAD_MAX)


def _path_gain(desired_curvature: float, desired_steering_angle_deg: float, v_ego: float) -> float:
  curvature_gain = abs(desired_curvature) / FORD_PATH_FULL_CURVATURE
  lat_accel_gain = abs(desired_curvature) * v_ego * v_ego / FORD_PATH_FULL_LAT_ACCEL
  steer_angle_gain = abs(desired_steering_angle_deg) / FORD_PATH_FULL_STEER_ANGLE_DEG
  return _clip(max(curvature_gain, lat_accel_gain, steer_angle_gain), 0.0, 1.0)


def _valid_model_path(model) -> bool:
  if model is None:
    return False
  try:
    return len(model.position.x) > 1 and len(model.position.x) == len(model.position.y) == len(model.orientation.z)
  except (AttributeError, TypeError):
    return False


def _model_path_offset_at_car(model) -> float:
  x_pts = model.position.x
  y_pts = model.position.y
  if x_pts[0] <= 0.0 <= x_pts[-1]:
    return _interp(0.0, x_pts, y_pts)

  dx = x_pts[1] - x_pts[0]
  if abs(dx) < 1e-3:
    return y_pts[0]

  slope = (y_pts[1] - y_pts[0]) / dx
  return y_pts[0] - (slope * x_pts[0])


def lightweight_path_from_curvature(desired_curvature: float, desired_steering_angle_deg: float, v_ego: float,
                                    path_offset_last: float, path_angle_last: float,
                                    lat_active: bool) -> tuple[float, float, float]:
  """Fallback c0/c1 path when model samples are unavailable."""
  if not lat_active:
    return 0.0, 0.0, 0.0

  desired_curvature = _finite(desired_curvature)
  desired_steering_angle_deg = _finite(desired_steering_angle_deg)
  v_ego = _finite(v_ego)
  path_angle_last = _finite(path_angle_last)

  gain = _path_gain(desired_curvature, desired_steering_angle_deg, v_ego)
  path_angle = desired_curvature * _path_lookahead(v_ego) * gain
  path_angle = _rate_limit(path_angle, path_angle_last, FORD_PATH_C1_RATE)

  return (
    0.0,
    _clip(path_angle, *FORD_PATH_C1_CAN_CLIP),
    0.0,
  )


def lightweight_path_from_model(model, desired_curvature: float, desired_steering_angle_deg: float, v_ego: float,
                                path_offset_last: float, path_angle_last: float,
                                lat_active: bool) -> tuple[float, float, float]:
  """Return Ford c0/c1 from the model path with c2 held inactive.

  The model path defines the target geometry. Desired curvature and steering
  angle only scale how strongly we assert model heading; c0 stays tied to the
  model's near-car offset so future road curvature does not become lane bias.
  """
  if not _valid_model_path(model):
    return lightweight_path_from_curvature(
      desired_curvature, desired_steering_angle_deg, v_ego, path_offset_last, path_angle_last, lat_active
    )
  if not lat_active:
    return 0.0, 0.0, 0.0

  desired_curvature = _finite(desired_curvature)
  desired_steering_angle_deg = _finite(desired_steering_angle_deg)
  v_ego = _finite(v_ego)
  path_offset_last = _finite(path_offset_last)
  path_angle_last = _finite(path_angle_last)

  gain = _path_gain(desired_curvature, desired_steering_angle_deg, v_ego)
  x_pts = model.position.x
  path_angle = _interp(_path_lookahead(v_ego), x_pts, model.orientation.z) * gain
  path_angle = _rate_limit(path_angle, path_angle_last, FORD_PATH_C1_RATE)

  path_offset = _model_path_offset_at_car(model)
  path_offset = _rate_limit(path_offset, path_offset_last, FORD_PATH_C0_RATE)

  return (
    _clip(path_offset, *FORD_PATH_C0_CAN_CLIP),
    _clip(path_angle, *FORD_PATH_C1_CAN_CLIP),
    0.0,
  )
