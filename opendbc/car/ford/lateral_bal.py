from __future__ import annotations

import math


# Ford CAN-FD path assist using model-derived c0/c1 only. c2/c3 stay inactive.
FORD_PATH_C0_CAN_CLIP = (-0.35, 0.35)
FORD_PATH_C1_CAN_CLIP = (-0.42, 0.42)

FORD_MODEL_DLOOK_MIN = 7.0
FORD_MODEL_C1_DLOOK_TIME = 0.70
FORD_MODEL_C1_DLOOK_MAX = 16.0
FORD_MODEL_C0_LOOKAHEAD = 6.0

FORD_PATH_CURVATURE_ERROR = 0.004
FORD_PATH_C1_ERROR_GAIN = 4.0
FORD_PATH_CURVATURE_DEADBAND = 0.00025
FORD_PATH_C0_RATE = 0.16
FORD_PATH_C1_RATE = 0.30


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


def _path_angle_lookahead(v_ego: float) -> float:
  return _clip(v_ego * FORD_MODEL_C1_DLOOK_TIME, FORD_MODEL_DLOOK_MIN, FORD_MODEL_C1_DLOOK_MAX)


def _path_offset_lookahead(v_ego: float, path_angle_lookahead: float) -> float:
  return min(path_angle_lookahead, FORD_MODEL_C0_LOOKAHEAD)


def _valid_model_path(model) -> bool:
  if model is None:
    return False
  try:
    return len(model.position.x) > 1 and len(model.position.x) == len(model.position.y) == len(model.orientation.z)
  except (AttributeError, TypeError):
    return False


def _curvature_c1_feedback(desired_curvature: float, current_curvature: float) -> float:
  curvature_error = _clip(desired_curvature - current_curvature, -FORD_PATH_CURVATURE_ERROR, FORD_PATH_CURVATURE_ERROR)
  return curvature_error * FORD_PATH_C1_ERROR_GAIN


def _zero_demand(desired_curvature: float, current_curvature: float) -> bool:
  return abs(desired_curvature) < FORD_PATH_CURVATURE_DEADBAND and abs(current_curvature) < FORD_PATH_CURVATURE_DEADBAND


def lightweight_path_from_curvature(desired_curvature: float, current_curvature: float, v_ego: float,
                                    path_offset_last: float, path_angle_last: float,
                                    lat_active: bool) -> tuple[float, float, float]:
  """Fallback c0/c1 path when model samples are unavailable."""
  if not lat_active:
    return 0.0, 0.0, 0.0

  desired_curvature = _finite(desired_curvature)
  current_curvature = _finite(current_curvature)
  v_ego = _finite(v_ego)
  path_offset_last = _finite(path_offset_last)
  path_angle_last = _finite(path_angle_last)
  if _zero_demand(desired_curvature, current_curvature):
    return (
      _rate_limit(0.0, path_offset_last, FORD_PATH_C0_RATE),
      _rate_limit(0.0, path_angle_last, FORD_PATH_C1_RATE),
      0.0,
    )

  d_look = _path_angle_lookahead(v_ego)
  d_c0 = _path_offset_lookahead(v_ego, d_look)

  path_angle = (desired_curvature * d_look) + _curvature_c1_feedback(desired_curvature, current_curvature)
  path_angle = _rate_limit(path_angle, path_angle_last, FORD_PATH_C1_RATE)

  path_offset = 0.5 * desired_curvature * d_c0 * d_c0
  path_offset = _rate_limit(path_offset, path_offset_last, FORD_PATH_C0_RATE)

  return (
    _clip(path_offset, *FORD_PATH_C0_CAN_CLIP),
    _clip(path_angle, *FORD_PATH_C1_CAN_CLIP),
    0.0,
  )


def lightweight_path_from_model(model, desired_curvature: float, current_curvature: float, v_ego: float,
                                path_offset_last: float, path_angle_last: float,
                                lat_active: bool) -> tuple[float, float, float]:
  """Return Ford c0/c1 from the model path with c2 held inactive.

  The model path carries preview geometry, which gives the PSCM the unwind
  information that desired curvature alone lacks. Desired/current curvature
  only trims c1 tracking error; c0 stays model-derived so it does not become a
  persistent feedback bias.
  """
  if not _valid_model_path(model):
    return lightweight_path_from_curvature(
      desired_curvature, current_curvature, v_ego, path_offset_last, path_angle_last, lat_active
    )
  if not lat_active:
    return 0.0, 0.0, 0.0

  desired_curvature = _finite(desired_curvature)
  current_curvature = _finite(current_curvature)
  v_ego = _finite(v_ego)
  path_offset_last = _finite(path_offset_last)
  path_angle_last = _finite(path_angle_last)
  if _zero_demand(desired_curvature, current_curvature):
    return (
      _rate_limit(0.0, path_offset_last, FORD_PATH_C0_RATE),
      _rate_limit(0.0, path_angle_last, FORD_PATH_C1_RATE),
      0.0,
    )

  d_look = _path_angle_lookahead(v_ego)
  d_c0 = _path_offset_lookahead(v_ego, d_look)

  x_pts = model.position.x
  path_angle = _interp(d_look, x_pts, model.orientation.z)
  path_angle += _curvature_c1_feedback(desired_curvature, current_curvature)
  path_angle = _rate_limit(path_angle, path_angle_last, FORD_PATH_C1_RATE)

  path_offset = _interp(d_c0, x_pts, model.position.y)
  path_offset = _rate_limit(path_offset, path_offset_last, FORD_PATH_C0_RATE)

  return (
    _clip(path_offset, *FORD_PATH_C0_CAN_CLIP),
    _clip(path_angle, *FORD_PATH_C1_CAN_CLIP),
    0.0,
  )
