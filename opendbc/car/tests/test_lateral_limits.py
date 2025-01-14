#!/usr/bin/env python3
from collections import defaultdict
import importlib
import math
from parameterized import parameterized_class
import pytest
import sys

from opendbc.car import DT_CTRL
from opendbc.car.common.numpy_fast import interp
from opendbc.car.car_helpers import interfaces
from opendbc.car.fingerprints import all_known_cars
from opendbc.car.interfaces import get_torque_params
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel  # TODO: no VM in opendbc

CAR_MODELS = all_known_cars()

# ISO 11270 - allowed up jerk is strictly lower than recommended limits
MAX_LAT_ACCEL = 3.0              # m/s^2
MAX_LAT_JERK_UP = 2.5            # m/s^3
MAX_LAT_JERK_DOWN = 5.0          # m/s^3
MAX_LAT_JERK_UP_TOLERANCE = 0.5  # m/s^3

# jerk is measured over half a second
JERK_MEAS_T = 0.5


@parameterized_class('car_model', [(c,) for c in sorted(CAR_MODELS)])
class TestLateralLimits:
  car_model: str

  @classmethod
  def setup_class(cls):
    CarInterface, _, _, _ = interfaces[cls.car_model]
    cls.CP = CarInterface.get_non_essential_params(cls.car_model)
    cls.VM = VehicleModel(cls.CP)

    if cls.CP.dashcamOnly:
      pytest.skip("Platform is behind dashcamOnly")

    # # TODO: test all platforms
    # if CP.steerControlType != 'torque':
    #   pytest.skip()

    if cls.CP.notCar:
      pytest.skip()

    CarControllerParams = importlib.import_module(f'opendbc.car.{cls.CP.carName}.values').CarControllerParams
    cls.control_params = CarControllerParams(cls.CP)
    cls.torque_params = get_torque_params()[cls.car_model]

  @staticmethod
  def calculate_0_5s_jerk(CP, VM, control_params, torque_params):
    steer_step = control_params.STEER_STEP
    max_lat_accel = torque_params['MAX_LAT_ACCEL_MEASURED']

    if CP.steerControlType == 'torque':
      # Steer up/down delta per 10ms frame, in percentage of max torque
      steer_up_per_frame = control_params.STEER_DELTA_UP / control_params.STEER_MAX / steer_step
      steer_down_per_frame = control_params.STEER_DELTA_DOWN / control_params.STEER_MAX / steer_step

      # Lateral acceleration reached in 0.5 seconds, clipping to max torque
      accel_up_0_5_sec = min(steer_up_per_frame * JERK_MEAS_T / DT_CTRL, 1.0) * max_lat_accel
      accel_down_0_5_sec = min(steer_down_per_frame * JERK_MEAS_T / DT_CTRL, 1.0) * max_lat_accel

      # Convert to m/s^3
      return accel_up_0_5_sec / JERK_MEAS_T, accel_down_0_5_sec / JERK_MEAS_T
    else:
      # We need to test the entire speed range for angle cars as jerk is speed dependent
      print(CP.carFingerprint, control_params.ANGLE_RATE_LIMIT_UP, steer_step)

      up_jerks = []
      down_jerks = []

      speeds = [1e-3, 5., 15., 35.]  # m/s
      for speed in speeds:
        up_rate = interp(speed, control_params.ANGLE_RATE_LIMIT_UP.speed_bp, control_params.ANGLE_RATE_LIMIT_UP.angle_v)
        down_rate = interp(speed, control_params.ANGLE_RATE_LIMIT_DOWN.speed_bp, control_params.ANGLE_RATE_LIMIT_DOWN.angle_v)

        # Ford is already curvature
        if CP.carName != 'ford':
          up_rate = VM.calc_curvature(math.radians(up_rate), speed, 0)
          down_rate = VM.calc_curvature(math.radians(down_rate), speed, 0)
        print(up_rate, down_rate)
        continue

        accel_up_per_frame = up_rate * speed ** 2 / steer_step
        accel_down_per_frame = down_rate * speed ** 2 / steer_step

        accel_up_0_5_sec = min(accel_up_per_frame * JERK_MEAS_T / DT_CTRL, max_lat_accel)
        accel_down_0_5_sec = min(accel_down_per_frame * JERK_MEAS_T / DT_CTRL, max_lat_accel)

        print('speed', speed)
        print('accel_up_per_frame', accel_up_per_frame, 'accel_down_per_frame', accel_down_per_frame)
        print('accel_up_0_5_sec', accel_up_0_5_sec, 'accel_down_0_5_sec', accel_down_0_5_sec)
        up_jerks.append(accel_up_0_5_sec / JERK_MEAS_T)
        down_jerks.append(accel_down_0_5_sec / JERK_MEAS_T)
        # continue
        #
        # accel_up_0_5_sec = min(curvature_up_per_frame * JERK_MEAS_T / DT_CTRL, 1.0) * MAX_LAT_ACCEL
        # accel_down_0_5_sec = min(curvature_down_per_frame * JERK_MEAS_T / DT_CTRL, 1.0) * MAX_LAT_ACCEL
        # jerk_up, jerk_down = accel_up_0_5_sec / JERK_MEAS_T, accel_down_0_5_sec / JERK_MEAS_T
        # print(up_rate, down_rate)
        # print('jerk_up', jerk_up, 'jerk_down', jerk_down)
        #
        # # curvature_up = self.VM.calc_curvature(math.radians(1.6) * (100 / steer_step), speed, 0)
        # # jerk_up = curvature * speed ** 2
        # # print('speed', speed, 'curvature', curvature, 'jerk', jerk)
      return max(up_jerks), max(down_jerks)

  def test_jerk_limits(self):
    # print(self.CP.carFingerprint, self.CP.steerControlType)
    # if self.CP.steerControlType == 'torque':
    up_jerk, down_jerk = self.calculate_0_5s_jerk(self.CP, self.VM, self.control_params, self.torque_params)
    assert up_jerk <= MAX_LAT_JERK_UP + MAX_LAT_JERK_UP_TOLERANCE
    assert down_jerk <= MAX_LAT_JERK_DOWN
    # else:
    #   steer_step = self.control_params.STEER_STEP
    #   max_lat_accel = self.torque_params['MAX_LAT_ACCEL_MEASURED']
    #
    #
    #   # We need to test the entire speed range for angle cars as jerk is speed dependent
    #   print(self.CP.carFingerprint, self.control_params.ANGLE_RATE_LIMIT_UP, self.control_params.STEER_STEP)
    #   speeds = [1e-3, 5., 15., 35.]  # m/s
    #   for speed in speeds:
    #     up_rate = interp(speed, self.control_params.ANGLE_RATE_LIMIT_UP.speed_bp, self.control_params.ANGLE_RATE_LIMIT_UP.angle_v)
    #     down_rate = interp(speed, self.control_params.ANGLE_RATE_LIMIT_DOWN.speed_bp, self.control_params.ANGLE_RATE_LIMIT_DOWN.angle_v)
    #
    #     # Ford is already curvature
    #     if self.CP.carName != 'ford':
    #       up_rate = self.VM.calc_curvature(math.radians(up_rate), speed, 0)
    #       down_rate = self.VM.calc_curvature(math.radians(down_rate), speed, 0)
    #
    #     accel_up_per_frame = up_rate * speed ** 2 / self.control_params.STEER_STEP
    #     accel_down_per_frame = down_rate * speed ** 2 / self.control_params.STEER_STEP
    #
    #     accel_up_0_5_sec = min(accel_up_per_frame * JERK_MEAS_T / DT_CTRL, max_lat_accel)
    #     accel_down_0_5_sec = min(accel_down_per_frame * JERK_MEAS_T / DT_CTRL, max_lat_accel)
    #
    #     print('speed', speed)
    #     print('accel_up_per_frame', accel_up_per_frame, 'accel_down_per_frame', accel_down_per_frame)
    #     print('accel_up_0_5_sec', accel_up_0_5_sec, 'accel_down_0_5_sec', accel_down_0_5_sec)
    #     continue
    #
    #     accel_up_0_5_sec = min(curvature_up_per_frame * JERK_MEAS_T / DT_CTRL, 1.0) * MAX_LAT_ACCEL
    #     accel_down_0_5_sec = min(curvature_down_per_frame * JERK_MEAS_T / DT_CTRL, 1.0) * MAX_LAT_ACCEL
    #     jerk_up, jerk_down = accel_up_0_5_sec / JERK_MEAS_T, accel_down_0_5_sec / JERK_MEAS_T
    #     print(up_rate, down_rate)
    #     print('jerk_up', jerk_up, 'jerk_down', jerk_down)
    #
    #
    #
    #     # curvature_up = self.VM.calc_curvature(math.radians(1.6) * (100 / self.control_params.STEER_STEP), speed, 0)
    #     # jerk_up = curvature * speed ** 2
    #     # print('speed', speed, 'curvature', curvature, 'jerk', jerk)
    #
    #   if self.CP.carFingerprint != 'TOYOTA_RAV4_TSS2_2023':
    #     return

  def test_max_lateral_accel(self):
    assert self.torque_params["MAX_LAT_ACCEL_MEASURED"] <= MAX_LAT_ACCEL


class LatAccelReport:
  car_model_jerks: defaultdict[str, dict[str, float]] = defaultdict(dict)

  def pytest_sessionfinish(self):
    print(f"\n\n---- Lateral limit report ({len(CAR_MODELS)} cars) ----\n")

    max_car_model_len = max([len(car_model) for car_model in self.car_model_jerks])
    for car_model, _jerks in sorted(self.car_model_jerks.items(), key=lambda i: i[1]['up_jerk'], reverse=True):
      violation = _jerks["up_jerk"] > MAX_LAT_JERK_UP + MAX_LAT_JERK_UP_TOLERANCE or \
                  _jerks["down_jerk"] > MAX_LAT_JERK_DOWN
      violation_str = " - VIOLATION" if violation else ""

      print(f"{car_model:{max_car_model_len}} - up jerk: {round(_jerks['up_jerk'], 2):5} " +
            f"m/s^3, down jerk: {round(_jerks['down_jerk'], 2):5} m/s^3{violation_str}")

  @pytest.fixture(scope="class", autouse=True)
  def class_setup(self, request):
    yield
    cls = request.cls
    if hasattr(cls, "control_params"):
      up_jerk, down_jerk = TestLateralLimits.calculate_0_5s_jerk(cls.CP, cls.VM, cls.control_params, cls.torque_params)
      self.car_model_jerks[cls.car_model] = {"up_jerk": up_jerk, "down_jerk": down_jerk}


if __name__ == '__main__':
  sys.exit(pytest.main([__file__, '-n0', '--no-summary'],))
                       # plugins=[LatAccelReport()]))  # noqa: TID251
