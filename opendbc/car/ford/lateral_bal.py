from __future__ import annotations

import math


# Lightweight Ford CAN-FD path controller, matching the recent sp-dev-c3 setup.
# It emits c0/c1 plus a small dynamic c2 assist and leaves c3 inactive. The
# controller sends these with inverted CAN sign convention.
FORD_PATH_C0_CAN_CLIP = (-4.61, 4.60)
FORD_PATH_C1_CAN_CLIP = (-0.475, 0.497)
FORD_PATH_C2_CAN_CLIP = (-0.004, 0.004)

FORD_MODEL_DLOOK_MIN = 7.0
FORD_MODEL_C1_DLOOK_TIME_BP = (0.0, 10.0, 20.0, 30.0)
FORD_MODEL_C1_DLOOK_TIME = (1.0, 0.9, 0.62, 0.50)
FORD_MODEL_C1_DLOOK_MAX = 16.0
FORD_MODEL_C0_HIGH_SPEED_LOOKAHEAD = 6.0
FORD_MODEL_C0_SPEED_BP = (11.0, 14.0)
FORD_MODEL_C0_GAIN_SPEED_BP = (0.0, 5.0, 15.0, 25.0, 35.0)
FORD_MODEL_C0_GAIN = (0.80, 0.75, 0.50, 0.30, 0.25)

FORD_PATH_FEEDBACK_SPEED_BP = (0.0, 5.0, 15.0, 22.35, 30.0)
FORD_PATH_C1_FEEDBACK = (0.0, 2.0, 4.5, 4.0, 3.5)
FORD_PATH_CURVATURE_ERROR = 0.006
FORD_PATH_C2_ERROR_DEADBAND = 0.0004
FORD_PATH_C2_ERROR_GAIN_SPEED_BP = (0.0, 5.0, 10.0, 20.0, 30.0)
FORD_PATH_C2_ERROR_GAIN = (0.0, 0.25, 0.45, 0.60, 0.55)
FORD_PATH_C2_RATE_UP = 0.04
FORD_PATH_C2_RATE_DOWN = 0.08
FORD_PATH_DT = 0.05

FORD_PATH_C1_RATE_BP = (5.0, 25.0)
FORD_PATH_C1_RATE = (0.50, 0.50)


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


def _path_angle_lookahead(v_ego: float) -> float:
  lookahead_time = _interp(v_ego, FORD_MODEL_C1_DLOOK_TIME_BP, FORD_MODEL_C1_DLOOK_TIME)
  return min(max(v_ego * lookahead_time, FORD_MODEL_DLOOK_MIN), FORD_MODEL_C1_DLOOK_MAX)


def _path_offset_gain(v_ego: float) -> float:
  return _interp(v_ego, FORD_MODEL_C0_GAIN_SPEED_BP, FORD_MODEL_C0_GAIN)


def _path_curvature_assist(desired_curvature: float, current_curvature: float, v_ego: float,
                           path_curvature_last: float) -> float:
  curvature_error = _clip(desired_curvature - current_curvature, -FORD_PATH_CURVATURE_ERROR, FORD_PATH_CURVATURE_ERROR)
  c2_target = _apply_deadband(curvature_error, FORD_PATH_C2_ERROR_DEADBAND)
  c2_target *= _interp(v_ego, FORD_PATH_C2_ERROR_GAIN_SPEED_BP, FORD_PATH_C2_ERROR_GAIN)
  c2_target = _clip(c2_target, *FORD_PATH_C2_CAN_CLIP)

  winding_up = c2_target * path_curvature_last >= 0.0 and abs(c2_target) > abs(path_curvature_last)
  c2_rate = FORD_PATH_C2_RATE_UP if winding_up else FORD_PATH_C2_RATE_DOWN
  return _clip(c2_target, path_curvature_last - (c2_rate * FORD_PATH_DT),
               path_curvature_last + (c2_rate * FORD_PATH_DT))


def _valid_model_path(model) -> bool:
  if model is None:
    return False
  try:
    return len(model.position.x) > 1 and len(model.position.x) == len(model.position.y) == len(model.orientation.z)
  except (AttributeError, TypeError):
    return False


def lightweight_path_from_curvature(desired_curvature: float, current_curvature: float, v_ego: float,
                                    path_angle_last: float, path_curvature_last: float,
                                    lat_active: bool) -> tuple[float, float, float]:
  """Return Ford path offset, path angle, and dynamic path curvature assist.

  This fallback synthesizes a constant-curvature model path when live model
  samples are unavailable: c1 = k*d_look and c0 = 0.5*k*d_c0^2. c2 is only a
  small curvature-error assist so it can unwind when the car has caught up.
  """
  if not lat_active:
    return 0.0, 0.0, 0.0

  desired_curvature = _finite(desired_curvature)
  current_curvature = _finite(current_curvature)
  v_ego = _finite(v_ego)

  d_look = _path_angle_lookahead(v_ego)
  d_c0 = _interp(v_ego, FORD_MODEL_C0_SPEED_BP, (d_look, FORD_MODEL_C0_HIGH_SPEED_LOOKAHEAD))

  path_angle = desired_curvature * d_look
  path_offset = 0.5 * desired_curvature * d_c0 * d_c0 * _path_offset_gain(v_ego)

  c1_rate = _interp(v_ego, FORD_PATH_C1_RATE_BP, FORD_PATH_C1_RATE)
  path_angle = _clip(path_angle, path_angle_last - c1_rate, path_angle_last + c1_rate)
  path_curvature = _path_curvature_assist(desired_curvature, current_curvature, v_ego, path_curvature_last)

  return (
    _clip(path_offset, *FORD_PATH_C0_CAN_CLIP),
    _clip(path_angle, *FORD_PATH_C1_CAN_CLIP),
    path_curvature,
  )


def lightweight_path_from_model(model, desired_curvature: float, current_curvature: float, v_ego: float,
                                path_angle_last: float, path_curvature_last: float,
                                lat_active: bool) -> tuple[float, float, float]:
  """Return Ford c0/c1/c2 from the model path plus bounded curvature feedback.

  The model supplies the path shape Ford's PSCM wants: c1 from orientation.z at
  a far lookahead and c0 from position.y at a shorter speed-dependent lookahead.
  The curvature error term adjusts heading authority without adding a steady
  lateral offset. c2 is a rate-limited transient assist for responsiveness.
  """
  if not _valid_model_path(model):
    return lightweight_path_from_curvature(
      desired_curvature, current_curvature, v_ego, path_angle_last, path_curvature_last, lat_active
    )
  if not lat_active:
    return 0.0, 0.0, 0.0

  desired_curvature = _finite(desired_curvature)
  current_curvature = _finite(current_curvature)
  v_ego = _finite(v_ego)

  d_look = _path_angle_lookahead(v_ego)
  d_c0 = _interp(v_ego, FORD_MODEL_C0_SPEED_BP, (d_look, FORD_MODEL_C0_HIGH_SPEED_LOOKAHEAD))

  x_pts = model.position.x
  path_angle = _interp(d_look, x_pts, model.orientation.z)
  path_offset = _interp(d_c0, x_pts, model.position.y) * _path_offset_gain(v_ego)

  curvature_error = _clip(desired_curvature - current_curvature, -FORD_PATH_CURVATURE_ERROR, FORD_PATH_CURVATURE_ERROR)
  path_angle += curvature_error * _interp(v_ego, FORD_PATH_FEEDBACK_SPEED_BP, FORD_PATH_C1_FEEDBACK)

  c1_rate = _interp(v_ego, FORD_PATH_C1_RATE_BP, FORD_PATH_C1_RATE)
  path_angle = _clip(path_angle, path_angle_last - c1_rate, path_angle_last + c1_rate)
  path_curvature = _path_curvature_assist(desired_curvature, current_curvature, v_ego, path_curvature_last)

  return (
    _clip(path_offset, *FORD_PATH_C0_CAN_CLIP),
    _clip(path_angle, *FORD_PATH_C1_CAN_CLIP),
    path_curvature,
  )
