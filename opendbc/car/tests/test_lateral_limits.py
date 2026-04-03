#!/usr/bin/env python3
import unittest
import importlib
from functools import cache

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


@cache
def get_lateral_case(car_model: str):
  CarInterface = interfaces[car_model]
  CP = CarInterface.get_non_essential_params(car_model)

  if car_model == 'MOCK':
    return None

  # TODO: test all platforms
  if CP.steerControlType != 'torque':
    return None

  if CP.notCar:
    return None

  CarControllerParams = importlib.import_module(f'opendbc.car.{CP.brand}.values').CarControllerParams
  control_params = CarControllerParams(CP)
  torque_params = get_torque_params()[car_model]
  return control_params, torque_params


class TestLateralLimits(unittest.TestCase):

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

  def test_jerk_limits(self):
    for car_model in sorted(PLATFORMS):
      with self.subTest(car_model=car_model):
        case = get_lateral_case(car_model)
        if case is None:
          continue
        control_params, torque_params = case
        up_jerk, down_jerk = self.calculate_0_5s_jerk(control_params, torque_params)
        self.assertLessEqual(up_jerk, MAX_LAT_JERK_UP + MAX_LAT_JERK_UP_TOLERANCE)
        self.assertLessEqual(down_jerk, MAX_LAT_JERK_DOWN)

  def test_max_lateral_accel(self):
    for car_model in sorted(PLATFORMS):
      with self.subTest(car_model=car_model):
        case = get_lateral_case(car_model)
        if case is None:
          continue
        _, torque_params = case
        self.assertLessEqual(torque_params["MAX_LAT_ACCEL_MEASURED"], ISO_LATERAL_ACCEL)
