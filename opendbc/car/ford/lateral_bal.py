from __future__ import annotations

import math


# Lightweight Ford CAN-FD path controller, matching the recent sp-dev-c3 setup.
# It emits c0/c1 from a model-style constant-curvature path and leaves c2/c3
# inactive. The controller sends these with inverted CAN sign convention.
FORD_PATH_C0_CAN_CLIP = (-4.61, 4.60)
FORD_PATH_C1_CAN_CLIP = (-0.475, 0.497)

FORD_MODEL_DLOOK_TIME = 1.0
FORD_MODEL_DLOOK_MIN = 7.0
FORD_MODEL_C0_HIGH_SPEED_LOOKAHEAD = 6.0
FORD_MODEL_C0_SPEED_BP = (11.0, 14.0)

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


def lightweight_path_from_curvature(desired_curvature: float, v_ego: float,
                                    path_angle_last: float, lat_active: bool) -> tuple[float, float]:
  """Return Ford path offset and path angle for the lightweight path controller.

  In Sunnypilot this controller reads model.position.y and model.orientation.z
  directly. Plain opendbc only receives desired curvature, so this uses the
  same geometry for a constant-curvature model path: c1 = k*d_look and
  c0 = 0.5*k*d_c0^2. This preserves the lightweight controller behavior without
  reintroducing the older PSCM inverse/bal allocator.
  """
  if not lat_active:
    return 0.0, 0.0

  desired_curvature = _finite(desired_curvature)
  v_ego = _finite(v_ego)

  d_look = max(v_ego * FORD_MODEL_DLOOK_TIME, FORD_MODEL_DLOOK_MIN)
  d_c0 = _interp(v_ego, FORD_MODEL_C0_SPEED_BP, (d_look, FORD_MODEL_C0_HIGH_SPEED_LOOKAHEAD))

  path_angle = desired_curvature * d_look
  path_offset = 0.5 * desired_curvature * d_c0 * d_c0

  c1_rate = _interp(v_ego, FORD_PATH_C1_RATE_BP, FORD_PATH_C1_RATE)
  path_angle = _clip(path_angle, path_angle_last - c1_rate, path_angle_last + c1_rate)

  return (
    _clip(path_offset, *FORD_PATH_C0_CAN_CLIP),
    _clip(path_angle, *FORD_PATH_C1_CAN_CLIP),
  )
