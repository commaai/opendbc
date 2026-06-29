import math

from opendbc.car.ford.lateral_bal import lightweight_path_from_curvature


def test_path_assist_adds_c0_c1_without_c2_for_high_angle_demand():
  path_offset, path_angle, path_curvature = lightweight_path_from_curvature(0.018, 0.0, 20.0, 0.0, 0.0, True)

  assert path_offset > 0.08
  assert path_angle > 0.14
  assert math.isclose(path_curvature, 0.0)


def test_path_assist_does_not_offset_established_gentle_curve():
  path_offset, path_angle, path_curvature = lightweight_path_from_curvature(0.002, 0.002, 20.0, 0.0, 0.0, True)

  assert math.isclose(path_offset, 0.0)
  assert 0.0 < path_angle < 0.03
  assert math.isclose(path_curvature, 0.0)


def test_path_assist_unwinds_c0_when_turn_is_established():
  entry_offset, _, _ = lightweight_path_from_curvature(0.008, 0.0, 20.0, 0.0, 0.0, True)
  established_offset, _, path_curvature = lightweight_path_from_curvature(0.008, 0.007, 20.0, entry_offset, 0.0, True)

  assert abs(established_offset) < abs(entry_offset) * 0.25
  assert math.isclose(path_curvature, 0.0)


def test_path_assist_does_not_fight_near_straight_yaw_error():
  path_offset, path_angle, path_curvature = lightweight_path_from_curvature(0.0, 0.003, 20.0, 0.0, 0.0, True)

  assert math.isclose(path_offset, 0.0)
  assert math.isclose(path_angle, 0.0)
  assert math.isclose(path_curvature, 0.0)


def test_path_assist_rate_limits_large_reversal():
  _, path_angle, _ = lightweight_path_from_curvature(-0.02, 0.0, 20.0, 0.0, 0.25, True)

  assert path_angle > 0.0


def test_path_assist_inactive_returns_zero():
  assert lightweight_path_from_curvature(0.018, 0.0, 20.0, 0.2, 0.2, False) == (0.0, 0.0, 0.0)
