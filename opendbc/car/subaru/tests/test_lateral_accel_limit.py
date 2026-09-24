import math
import unittest
from collections import defaultdict
from types import SimpleNamespace

import numpy as np

from opendbc.car import DT_CTRL, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.lateral import MAX_LATERAL_ACCEL
from opendbc.car.subaru.carcontroller import get_safety_CP
from opendbc.car.subaru.values import CAR
from opendbc.car.vehicle_model import VehicleModel

# Lateral acceleration we ask for, well above anything the limiter should ever pass through
COMMANDED_LATERAL_ACCEL = 6.0  # m/s^2

# Swept speeds, starting above the 1 m/s floor the limiter clamps to internally
SPEEDS = np.arange(2.0, 40.5, 0.5)  # m/s, ~7 to ~146 km/h

# Long enough for the lateral jerk limit to ramp the command to steady state at every speed
SETTLE_FRAMES = 600  # 6s at DT_CTRL

ANGLE_CAR = CAR.SUBARU_CROSSTREK_2025


def stub_car_state(v_ego: float, steering_angle: float):
  """Minimal CarState stand-in"""
  out = structs.CarState.new_message()
  out.vEgo = v_ego
  out.vEgoRaw = v_ego
  out.steeringAngleDeg = steering_angle
  return SimpleNamespace(
    out=out, ready=True, cruise_button=0,
    es_distance_msg=defaultdict(int), es_dashstatus_msg=defaultdict(int),
    es_lkas_state_msg=defaultdict(int), es_brake_msg=defaultdict(int),
    es_status_msg=defaultdict(int), es_infotainment_msg=defaultdict(int),
  )


def command_lateral_accel(v_ego: float, sign: int) -> tuple[float, float]:
  """Command COMMANDED_LATERAL_ACCEL at v_ego, return (requested angle, worst lateral accel let through)."""
  car_interface = interfaces[ANGLE_CAR]
  cp = car_interface.get_non_essential_params(ANGLE_CAR)
  ci = car_interface(cp)

  vm_car = VehicleModel(cp)
  requested = sign * math.degrees(vm_car.get_steer_from_curvature(COMMANDED_LATERAL_ACCEL / v_ego ** 2, v_ego, 0.0))

  vm_safety = VehicleModel(get_safety_CP())

  cc = structs.CarControl.new_message(enabled=True, latActive=True)
  cc.actuators.steeringAngleDeg = requested

  applied, worst = 0.0, 0.0
  for frame in range(SETTLE_FRAMES):
    cs = stub_car_state(v_ego, applied)
    actuators, _ = ci.CC.update(cc.as_reader(), cs, frame * int(DT_CTRL * 1e9))
    applied = actuators.steeringAngleDeg
    worst = max(worst, abs(vm_safety.calc_curvature(math.radians(applied), v_ego, 0.0)) * v_ego ** 2)

  return requested, worst


class TestLateralAccelLimit(unittest.TestCase):
  def _assert_limited_to(self, bound: float):
    for v_ego in SPEEDS:
      for sign, name in ((1, "left"), (-1, "right")):
        with self.subTest(kmh=round(v_ego * 3.6), turn=name):
          requested, worst = command_lateral_accel(float(v_ego), sign)
          self.assertGreater(abs(requested), 0, "test should command a non-zero angle")
          self.assertLessEqual(worst, bound + 1e-3)

  def test_max_lateral_accel_limit(self):
    self._assert_limited_to(MAX_LATERAL_ACCEL)


if __name__ == "__main__":
  unittest.main()
