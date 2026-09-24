import itertools
import math
import unittest

import numpy as np

from opendbc.car import Bus, structs
from opendbc.car.lateral import MAX_LATERAL_ACCEL
from opendbc.car.subaru.carcontroller import get_safety_CP
from opendbc.car.subaru.carstate import CarState
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR, CarControllerParams, SubaruFlags
from opendbc.car.vehicle_model import VehicleModel


class TestSubaruFingerprint(unittest.TestCase):
  def test_fw_version_format(self):
    for platform, fws_per_ecu in FW_VERSIONS.items():
      for (ecu, _, _), fws in fws_per_ecu.items():
        fw_size = len(fws[0])
        for fw in fws:
          assert len(fw) == fw_size, f"{platform} {ecu}: {len(fw)} {fw_size}"


class TestSubaruAngleLimits(unittest.TestCase):
  def setUp(self):
    cp = get_safety_CP()
    self.limits = CarControllerParams(cp)
    self.vm = VehicleModel(cp)

  def make_angle_controller(self, speed):
    cp = CarInterface.get_non_essential_params(CAR.SUBARU_CROSSTREK_2025)
    ci = CarInterface(cp)
    ci.update([])
    ci.CS.out = structs.CarState(vEgo=speed, vEgoRaw=speed)
    return ci

  def test_angle_filter_speed_schedule(self):
    for speed, tau in ((0.0, 0.3), (5.0, 0.3), (7.5, 0.2), (10.0, 0.1), (15.0, 0.05), (20.0, 0.0), (25.0, 0.0)):
      with self.subTest(speed=speed):
        ci = self.make_angle_controller(speed)
        cc = structs.CarControl(latActive=True)
        cc.actuators.steeringAngleDeg = 0.1
        actuators, _ = ci.CC.update(cc.as_reader(), ci.CS, 0)
        self.assertAlmostEqual(actuators.steeringAngleDeg, 0.1 * 0.01 / (tau + 0.01))

  def test_lateral_accel_limit(self):
    commanded_accel = 6.0  # m/s^2
    for speed in np.arange(2.0, 40.5, 0.5).tolist():
      for sign in (1, -1):
        with self.subTest(kmh=round(speed * 3.6), turn="left" if sign > 0 else "right"):
          ci = self.make_angle_controller(speed)
          vm_car = VehicleModel(ci.CP)
          requested = sign * math.degrees(vm_car.get_steer_from_curvature(commanded_accel / speed ** 2, speed, 0.0))

          cc = structs.CarControl(latActive=True)
          cc.actuators.steeringAngleDeg = requested

          worst = 0.0
          for _ in range(600):
            actuators, _ = ci.CC.update(cc.as_reader(), ci.CS, 0)
            curvature = self.vm.calc_curvature(math.radians(actuators.steeringAngleDeg), speed, 0.0)
            worst = max(worst, abs(curvature) * speed ** 2)

          self.assertLessEqual(worst, MAX_LATERAL_ACCEL + 1e-3)

class TestSubaruCruiseState(unittest.TestCase):
  def test_angle_cruise_uses_es_status(self):
    for platform in CAR:
      if not platform.config.flags & SubaruFlags.LKAS_ANGLE:
        continue
      cp = CarInterface.get_non_essential_params(platform)
      cs = CarState(cp)
      parsers = cs.get_can_parsers(cp)
      cruise_parser = parsers[Bus.alt if cp.flags & SubaruFlags.GLOBAL_GEN2 else Bus.cam]
      brake_parser = parsers[Bus.alt if cp.flags & SubaruFlags.GLOBAL_GEN2 else Bus.pt]
      for brake_pressed, status, brake_status in itertools.product((False, True), repeat=3):
        with self.subTest(platform=platform, brake=brake_pressed, status=status, es_brake=brake_status):
          brake_parser.vl["Brake_Status"]["Brake"] = brake_pressed
          cruise_parser.vl["ES_Status"]["Cruise_Activated"] = status
          cruise_parser.vl["ES_Brake"]["Cruise_Activated"] = brake_status
          self.assertEqual(cs.update(parsers).cruiseState.enabled, status)