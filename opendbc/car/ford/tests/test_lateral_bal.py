import math
from types import SimpleNamespace

from opendbc.car.ford.lateral_bal import lightweight_path_from_curvature, lightweight_path_from_model


def _model(x, y, z):
  return SimpleNamespace(
    position=SimpleNamespace(x=x, y=y),
    orientation=SimpleNamespace(z=z),
  )


def test_model_path_uses_heading_without_future_y_as_c0():
  curvature = 0.006
  x = [0.0, 5.0, 10.0, 15.0]
  model = _model(
    x,
    [0.5 * curvature * xi * xi for xi in x],
    [curvature * xi for xi in x],
  )

  path_offset, path_angle, path_curvature = lightweight_path_from_model(model, curvature, 0.0, 20.0, 0.0, 0.0, True)

  assert math.isclose(path_offset, 0.0)
  assert path_angle > 0.07
  assert math.isclose(path_curvature, 0.0)


def test_model_path_c0_represents_near_car_offset():
  model = _model(
    [0.0, 5.0, 10.0],
    [0.2, 0.2, 0.2],
    [0.0, 0.0, 0.0],
  )

  path_offset, path_angle, path_curvature = lightweight_path_from_model(model, 0.0, 0.0, 20.0, 0.0, 0.0, True)

  assert math.isclose(path_offset, 0.08)
  assert math.isclose(path_angle, 0.0)
  assert math.isclose(path_curvature, 0.0)


def test_model_path_unwinds_with_model_heading_and_lower_demand():
  entering = _model(
    [0.0, 5.0, 10.0, 15.0],
    [0.0, 0.08, 0.22, 0.42],
    [0.0, 0.04, 0.08, 0.12],
  )
  exiting = _model(
    [0.0, 5.0, 10.0, 15.0],
    [0.0, 0.04, 0.08, 0.10],
    [0.0, 0.02, 0.03, 0.035],
  )

  _, entry_angle, _ = lightweight_path_from_model(entering, 0.006, 8.0, 20.0, 0.0, 0.0, True)
  _, exit_angle, path_curvature = lightweight_path_from_model(exiting, 0.002, 2.0, 20.0, 0.0, entry_angle, True)

  assert 0.0 < exit_angle < entry_angle
  assert math.isclose(path_curvature, 0.0)


def test_model_path_zero_demand_does_not_assert_heading():
  model = _model(
    [0.0, 5.0, 10.0],
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0],
  )

  path_offset, path_angle, path_curvature = lightweight_path_from_model(model, 0.0, 0.0, 20.0, 0.0, 0.0, True)

  assert math.isclose(path_offset, 0.0)
  assert math.isclose(path_angle, 0.0)
  assert math.isclose(path_curvature, 0.0)


def test_model_path_falls_back_to_curvature_heading_without_c0():
  path_offset, path_angle, path_curvature = lightweight_path_from_model(None, 0.018, 0.0, 20.0, 0.0, 0.0, True)

  assert math.isclose(path_offset, 0.0)
  assert path_angle > 0.14
  assert math.isclose(path_curvature, 0.0)


def test_model_path_can_gain_from_desired_steering_angle():
  model = _model(
    [0.0, 5.0, 10.0, 15.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.03, 0.06, 0.09],
  )

  _, no_angle, _ = lightweight_path_from_model(model, 0.0, 0.0, 20.0, 0.0, 0.0, True)
  _, with_angle, path_curvature = lightweight_path_from_model(model, 0.0, 12.0, 20.0, 0.0, 0.0, True)

  assert math.isclose(no_angle, 0.0)
  assert with_angle > 0.07
  assert math.isclose(path_curvature, 0.0)


def test_path_assist_rate_limits_large_reversal():
  _, path_angle, _ = lightweight_path_from_curvature(-0.02, -12.0, 20.0, 0.0, 0.25, True)

  assert path_angle > 0.0


def test_path_assist_inactive_returns_zero():
  model = _model(
    [0.0, 5.0, 10.0],
    [0.2, 0.2, 0.2],
    [0.0, 0.0, 0.0],
  )

  assert lightweight_path_from_model(model, 0.018, 0.0, 20.0, 0.2, 0.2, False) == (0.0, 0.0, 0.0)
