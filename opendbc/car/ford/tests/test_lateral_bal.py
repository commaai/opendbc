import math
from types import SimpleNamespace

from opendbc.car.ford.lateral_bal import lightweight_path_from_curvature, lightweight_path_from_model


def _model(x, y, z):
  return SimpleNamespace(
    position=SimpleNamespace(x=x, y=y),
    orientation=SimpleNamespace(z=z),
  )


def test_lightweight_path_uses_short_c0_lookahead_at_speed():
  curvature = 0.002
  path_offset, path_angle, path_curvature = lightweight_path_from_curvature(curvature, 0.0, 20.0, 0.0, 0.0, 0.0, True)

  assert math.isclose(path_angle, curvature * 12.4)
  assert math.isclose(path_offset, 0.5 * curvature * 6.0 * 6.0 * 0.4)
  assert path_curvature > 0.0


def test_lightweight_path_uses_same_low_speed_lookahead_for_c0_and_c1():
  curvature = 0.01
  d_look = 7.0
  path_offset, path_angle, _ = lightweight_path_from_curvature(curvature, curvature, 5.0, 0.0, 0.0, curvature, True)

  assert math.isclose(path_angle, curvature * d_look)
  assert math.isclose(path_offset, 0.5 * curvature * d_look * d_look * 0.75)


def test_lightweight_path_rate_limits_large_c1_reversal():
  _, path_angle, _ = lightweight_path_from_curvature(-0.02, 0.0, 20.0, 0.497, 0.0, 0.0, True)

  assert math.isclose(path_angle, -0.003)


def test_lightweight_path_uses_model_c0_and_c1():
  model = _model(
    [0.0, 10.0, 20.0],
    [0.0, 1.0, 2.0],
    [0.0, 0.1, 0.2],
  )

  path_offset, path_angle, path_curvature = lightweight_path_from_model(model, 0.002, 0.002, 20.0, 0.0, 0.0, 0.002, True)

  assert math.isclose(path_angle, 0.124)
  assert math.isclose(path_offset, 0.24)
  assert math.isclose(path_curvature, 0.0)


def test_lightweight_path_adds_curvature_error_feedback_and_entry_c2():
  model = _model(
    [0.0, 10.0, 20.0],
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0],
  )

  path_offset, path_angle, path_curvature = lightweight_path_from_model(model, 0.004, 0.0, 20.0, 0.0, 0.0, 0.0, True)

  assert math.isclose(path_angle, 0.004 * (4.5 + (20.0 - 15.0) * (4.0 - 4.5) / (22.35 - 15.0)))
  assert math.isclose(path_offset, 0.0)
  assert path_curvature > 0.0


def test_lightweight_path_samples_nearer_heading_at_highway_speed():
  model = _model(
    [0.0, 10.0, 14.0, 18.0, 24.0],
    [0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.08, 0.16],
  )

  _, path_angle, _ = lightweight_path_from_model(model, 0.0, 0.0, 22.35, 0.0, 0.0, 0.0, True)

  assert path_angle < 0.01


def test_lightweight_path_attenuates_model_c0_bias_by_speed():
  model = _model(
    [0.0, 7.0, 20.0],
    [0.24, 0.24, 0.24],
    [0.0, 0.0, 0.0],
  )

  low_speed_offset, _, _ = lightweight_path_from_model(model, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, True)
  highway_offset, _, _ = lightweight_path_from_model(model, 0.0, 0.0, 22.35, 0.0, 0.0, 0.0, True)

  assert math.isclose(low_speed_offset, 0.24 * 0.75)
  assert math.isclose(highway_offset, 0.24 * (0.5 + (22.35 - 15.0) * (0.3 - 0.5) / (25.0 - 15.0)))
  assert abs(highway_offset) < abs(low_speed_offset)


def test_lightweight_path_falls_back_to_curvature_without_model():
  assert lightweight_path_from_model(None, 0.002, 0.0, 20.0, 0.0, 0.0, 0.0, True) == \
         lightweight_path_from_curvature(0.002, 0.0, 20.0, 0.0, 0.0, 0.0, True)


def test_lightweight_path_c2_stays_off_near_straight():
  _, _, path_curvature = lightweight_path_from_curvature(0.0003, 0.0, 20.0, 0.0, 0.0, 0.0, True)

  assert math.isclose(path_curvature, 0.0)


def test_lightweight_path_c2_pulses_only_while_turn_demand_builds():
  _, _, first_step = lightweight_path_from_curvature(0.004, 0.0, 20.0, 0.0, 0.0, 0.0, True)
  _, _, sustained = lightweight_path_from_curvature(0.004, 0.0, 20.0, 0.0, first_step, 0.004, True)

  assert math.isclose(first_step, 0.002)
  assert math.isclose(sustained, 0.0)


def test_lightweight_path_c2_stays_off_once_turn_is_established():
  _, _, path_curvature = lightweight_path_from_curvature(0.006, 0.003, 20.0, 0.0, 0.0, 0.003, True)

  assert math.isclose(path_curvature, 0.0)


def test_lightweight_path_c2_drops_out_when_path_direction_reverses():
  _, _, path_curvature = lightweight_path_from_curvature(-0.004, 0.002, 20.0, 0.0, 0.002, 0.004, True)

  assert math.isclose(path_curvature, 0.0)
