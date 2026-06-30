import math
from types import SimpleNamespace

from opendbc.car.ford.lateral_bal import lightweight_path_from_curvature, lightweight_path_from_model


def test_lightweight_path_uses_short_c0_lookahead_at_speed():
  curvature = 0.002
  path_offset, path_angle = lightweight_path_from_curvature(curvature, 20.0, 0.0, True)

  assert math.isclose(path_angle, curvature * 20.0)
  assert math.isclose(path_offset, 0.5 * curvature * 6.0 * 6.0)


def test_lightweight_path_uses_same_low_speed_lookahead_for_c0_and_c1():
  curvature = 0.01
  d_look = 7.0
  path_offset, path_angle = lightweight_path_from_curvature(curvature, 5.0, 0.0, True)

  assert math.isclose(path_angle, curvature * d_look)
  assert math.isclose(path_offset, 0.5 * curvature * d_look * d_look)


def test_lightweight_path_rate_limits_large_c1_reversal():
  _, path_angle = lightweight_path_from_curvature(-0.02, 20.0, 0.497, True)

  assert math.isclose(path_angle, -0.003)


def test_lightweight_path_uses_model_c0_and_c1():
  model = SimpleNamespace(
    position=SimpleNamespace(x=[0.0, 10.0, 20.0], y=[0.0, 1.0, 2.0]),
    orientation=SimpleNamespace(z=[0.0, 0.1, 0.2]),
  )

  path_offset, path_angle = lightweight_path_from_model(model, 0.002, 0.002, 20.0, 0.0, True)

  assert math.isclose(path_angle, 0.2)
  model_angle_offset = 0.5 * 0.2 * 6.0 * 6.0 / 20.0
  residual_gain = 0.35 + (20.0 - 15.0) * (0.20 - 0.35) / (30.0 - 15.0)
  assert math.isclose(path_offset, model_angle_offset + (0.6 - model_angle_offset) * residual_gain)


def test_lightweight_path_adds_curvature_error_feedback_to_c1_only():
  model = SimpleNamespace(
    position=SimpleNamespace(x=[0.0, 10.0, 20.0], y=[0.0, 0.0, 0.0]),
    orientation=SimpleNamespace(z=[0.0, 0.0, 0.0]),
  )

  path_offset, path_angle = lightweight_path_from_model(model, 0.004, 0.0, 20.0, 0.0, True)

  assert math.isclose(path_angle, 0.004 * (5.0 + (20.0 - 15.0) * (6.0 - 5.0) / (30.0 - 15.0)))
  assert math.isclose(path_offset, 0.0)


def test_lightweight_path_damps_model_c0_residual_more_at_speed():
  model = SimpleNamespace(
    position=SimpleNamespace(x=[0.0, 10.0, 20.0, 30.0], y=[0.0, 1.0, 2.0, 3.0]),
    orientation=SimpleNamespace(z=[0.0, 0.1, 0.2, 0.3]),
  )

  low_speed_offset, _ = lightweight_path_from_model(model, 0.002, 0.002, 5.0, 0.0, True)
  high_speed_offset, _ = lightweight_path_from_model(model, 0.002, 0.002, 30.0, 0.0, True)

  assert low_speed_offset > high_speed_offset


def test_lightweight_path_falls_back_to_curvature_without_model():
  assert lightweight_path_from_model(None, 0.002, 0.0, 20.0, 0.0, True) == \
         lightweight_path_from_curvature(0.002, 20.0, 0.0, True)
