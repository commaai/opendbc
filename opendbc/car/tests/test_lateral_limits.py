#!/usr/bin/env python3
from collections import defaultdict
import importlib
import pytest
import sys

# ISO 11270 - allowed up jerk is strictly lower than recommended limits
MAX_LAT_JERK_UP, MAX_LAT_JERK_DOWN, MAX_LAT_JERK_UP_TOLERANCE = 2.5, 5.0, 0.5
JERK_MEAS_T = 0.5


def get_platforms(lib):
  return lib.get_all_car_names()


def calculate_0_5s_jerk(control_params, torque_params, car_lib):
  step, max_lat = control_params.STEER_STEP, torque_params['MAX_LAT_ACCEL_MEASURED']
  def get_jerk(delta):
    return min(delta / control_params.STEER_MAX / step * JERK_MEAS_T / 0.01, 1.0) * max_lat / JERK_MEAS_T
  return get_jerk(control_params.STEER_DELTA_UP), get_jerk(control_params.STEER_DELTA_DOWN)


@pytest.fixture(scope="module")
def car_data(request, car_lib):
  car_model = request.param
  if car_model == 'MOCK': pytest.skip('Mock')
  CP = car_lib.interfaces[car_model].get_non_essential_params(car_model)
  if CP.steerControlType != 'torque' or CP.notCar: pytest.skip()

  mod = importlib.import_module(f'opendbc.car.{CP.brand}.values')
  return car_model, mod.CarControllerParams(CP), car_lib.interfaces_module.get_torque_params()[car_model]


@pytest.mark.parametrize("car_data", get_platforms(__import__('opendbc.car_discovery', fromlist=['get_all_car_names'])), indirect=True)
class TestLateralLimits:
  def test_jerk_limits(self, car_data, car_lib):
    up, down = calculate_0_5s_jerk(car_data[1], car_data[2], car_lib)
    assert up <= MAX_LAT_JERK_UP + MAX_LAT_JERK_UP_TOLERANCE and down <= MAX_LAT_JERK_DOWN

  def test_max_lateral_accel(self, car_data, car_lib):
    from opendbc.car.lateral import ISO_LATERAL_ACCEL
    assert car_data[2]["MAX_LAT_ACCEL_MEASURED"] <= ISO_LATERAL_ACCEL


class LatAccelReport:
  car_model_jerks = defaultdict(dict)

  def pytest_sessionfinish(self):
    print(f"\n\n---- Lateral limit report ({len(self.car_model_jerks)} cars) ----\n")
    max_len = max([len(m) for m in self.car_model_jerks] or [0])
    for m, j in sorted(self.car_model_jerks.items(), key=lambda i: i[1]['up'], reverse=True):
      v = " - VIOLATION" if j["up"] > MAX_LAT_JERK_UP + MAX_LAT_JERK_UP_TOLERANCE or j["down"] > MAX_LAT_JERK_DOWN else ""
      print(f"{m:{max_len}} - up jerk: {round(j['up'], 2):5} m/s^3, down jerk: {round(j['down'], 2):5} m/s^3{v}")

  @pytest.fixture(scope="module", autouse=True)
  def report_setup(self, request, car_data, car_lib):
    yield
    up, down = calculate_0_5s_jerk(car_data[1], car_data[2], car_lib)
    self.car_model_jerks[car_data[0]] = {"up": up, "down": down}


if __name__ == '__main__':
  sys.exit(pytest.main([__file__, '-n0', '--no-summary'], plugins=[LatAccelReport()]))
