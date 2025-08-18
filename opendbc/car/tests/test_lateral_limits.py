#!/usr/bin/env python3
from collections import defaultdict
import importlib
import pytest
import sys

from opendbc.car import DT_CTRL
from opendbc.car.car_helpers import interfaces
from opendbc.car.interfaces import get_torque_params
from opendbc.car.lateral import ISO_LATERAL_ACCEL
from opendbc.car.values import PLATFORMS

# ISO 11270 - allowed up jerk is strictly lower than recommended limits
MAX_LAT_JERK_UP = 2.5            # m/s^3
MAX_LAT_JERK_DOWN = 5.0          # m/s^3
MAX_LAT_JERK_UP_TOLERANCE = 0.5  # m/s^3

# jerk is measured over half a second
JERK_MEAS_T = 0.5


class TestLateralLimits:
  @staticmethod
  def calculate_0_5s_jerk(control_params, torque_params):
    steer_step = control_params.STEER_STEP
    max_lat_accel = torque_params['MAX_LAT_ACCEL_MEASURED']

    # Steer up/down delta per 10ms frame, in percentage of max torque
    steer_up_per_frame = control_params.STEER_DELTA_UP / control_params.STEER_MAX / steer_step
    steer_down_per_frame = control_params.STEER_DELTA_DOWN / control_params.STEER_MAX / steer_step

    # Lateral acceleration reached in 0.5 seconds, clipping to max torque
    accel_up_0_5_sec = min(steer_up_per_frame * JERK_MEAS_T / DT_CTRL, 1.0) * max_lat_accel
    accel_down_0_5_sec = min(steer_down_per_frame * JERK_MEAS_T / DT_CTRL, 1.0) * max_lat_accel

    # Convert to m/s^3
    return accel_up_0_5_sec / JERK_MEAS_T, accel_down_0_5_sec / JERK_MEAS_T

  def test_jerk_limits(self, subtests):
    for car_model in sorted(PLATFORMS):
      with subtests.test(car_model=car_model):
        CP = interfaces[car_model].get_non_essential_params(car_model)

        if car_model == 'MOCK' or CP.steerControlType != 'torque' or CP.notCar:
          continue

        CarControllerParams = importlib.import_module(f'opendbc.car.{CP.brand}.values').CarControllerParams
        control_params = CarControllerParams(CP)
        torque_params = get_torque_params()[car_model]

        up_jerk, down_jerk = self.calculate_0_5s_jerk(control_params, torque_params)
        assert up_jerk <= MAX_LAT_JERK_UP + MAX_LAT_JERK_UP_TOLERANCE
        assert down_jerk <= MAX_LAT_JERK_DOWN

  def test_max_lateral_accel(self, subtests):
    for car_model in sorted(PLATFORMS):
      with subtests.test(car_model=car_model):
        CP = interfaces[car_model].get_non_essential_params(car_model)

        if car_model == 'MOCK' or CP.steerControlType != 'torque' or CP.notCar:
          continue

        torque_params = get_torque_params()[car_model]
        assert torque_params["MAX_LAT_ACCEL_MEASURED"] <= ISO_LATERAL_ACCEL


class LatAccelReport:
  def pytest_sessionfinish(self):
    print(f"\n\n---- Lateral limit report ({len(PLATFORMS)} cars) ----\n")

    car_model_jerks: defaultdict[str, dict[str, float]] = defaultdict(dict)
    for car_model in sorted(PLATFORMS):
      CP = interfaces[car_model].get_non_essential_params(car_model)

      if car_model == 'MOCK' or CP.steerControlType != 'torque' or CP.notCar:
        continue

      CarControllerParams = importlib.import_module(f'opendbc.car.{CP.brand}.values').CarControllerParams
      control_params = CarControllerParams(CP)
      torque_params = get_torque_params()[car_model]

      up_jerk, down_jerk = TestLateralLimits.calculate_0_5s_jerk(control_params, torque_params)
      car_model_jerks[car_model] = {'up_jerk': up_jerk, 'down_jerk': down_jerk}

    max_car_model_len = max(len(car_model) for car_model in car_model_jerks) if car_model_jerks else 0
    # Sort by up_jerk descending
    for car_model, jerks in sorted(car_model_jerks.items(), key=lambda i: i[1]['up_jerk'], reverse=True):
      violation = jerks['up_jerk'] > MAX_LAT_JERK_UP + MAX_LAT_JERK_UP_TOLERANCE or jerks['down_jerk'] > MAX_LAT_JERK_DOWN
      violation_str = " - VIOLATION" if violation else ""

      print(f"{car_model:{max_car_model_len}} - up jerk: {round(jerks['up_jerk'], 2):5} m/s^3, " +
            f"down jerk: {round(jerks['down_jerk'], 2):5} m/s^3{violation_str}")


if __name__ == '__main__':
  sys.exit(pytest.main([__file__, '-n0', '--no-summary'], plugins=[LatAccelReport()]))  # noqa: TID251