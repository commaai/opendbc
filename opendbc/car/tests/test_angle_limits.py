import unittest

from opendbc.car.lateral import apply_steer_angle_limits_vm, get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.subaru.carcontroller import get_safety_CP as subaru_safety_params
from opendbc.car.subaru.values import CarControllerParams as SubaruLimits
from opendbc.car.tesla.carcontroller import get_safety_CP as tesla_safety_params
from opendbc.car.tesla.values import CarControllerParams as TeslaLimits
from opendbc.car.vehicle_model import VehicleModel


class TestVehicleModelAngleLimits(unittest.TestCase):
  def test_recover_from_reduced_max_angle(self):
    for params, limits in ((subaru_safety_params, SubaruLimits), (tesla_safety_params, TeslaLimits)):
      vm = VehicleModel(params())
      for speed in (10, 20, 30, 40):
        bound = get_max_angle_vm(speed, vm, limits)
        delta = min(get_max_angle_delta_vm(speed, vm, limits), limits.ANGLE_LIMITS.MAX_ANGLE_RATE)
        for sign in (-1, 1):
          last = sign * min(bound + 10, limits.ANGLE_LIMITS.STEER_ANGLE_MAX)
          with self.subTest(brand=params.__module__, speed=speed, sign=sign):
            for _ in range(2000):
              angle = apply_steer_angle_limits_vm(sign * bound * 2, last, speed, last, True, limits, vm)
              self.assertLessEqual(abs(angle - last), delta + 1e-9)
              self.assertLessEqual(abs(angle), abs(last) + 1e-9)
              last = angle
              if abs(angle) <= bound + 1e-9:
                break
            self.assertAlmostEqual(abs(last), bound)

  def test_inactive_tracks_measured_angle(self):
    for params, limits in ((subaru_safety_params, SubaruLimits), (tesla_safety_params, TeslaLimits)):
      vm = VehicleModel(params())
      for measured in (-100, 0, 100):
        with self.subTest(brand=params.__module__, measured=measured):
          self.assertEqual(apply_steer_angle_limits_vm(0, 0, 30, measured, False, limits, vm), measured)
