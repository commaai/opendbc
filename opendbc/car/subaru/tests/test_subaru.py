import math
import unittest

import numpy as np

from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.subaru.carcontroller import apply_subaru_angle_limits, get_safety_CP
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
    self.limits = CarControllerParams(get_safety_CP())
    self.vm = VehicleModel(get_safety_CP())

  def test_low_speed_deadband(self):
    for speed, desired, expected in ((3.9, 0.99, 0.0), (3.9, 1.0, 1.0), (4.0, 0.5, 0.5)):
      with self.subTest(speed=speed, desired=desired):
        cp = CarInterface.get_non_essential_params(CAR.SUBARU_CROSSTREK_2025)
        ci = CarInterface(cp)
        ci.update([])
        ci.CS.out = structs.CarState(vEgo=speed, vEgoRaw=speed)
        cc = structs.CarControl(latActive=True)
        cc.actuators.steeringAngleDeg = desired
        actuators, _ = ci.CC.update(cc.as_reader(), ci.CS, 0)
        self.assertAlmostEqual(actuators.steeringAngleDeg, expected)

  def test_safety_model_is_conservative(self):
    for platform in CAR:
      if not platform.config.flags & SubaruFlags.LKAS_ANGLE:
        continue
      vm = VehicleModel(CarInterface.get_non_essential_params(platform))
      for speed in np.linspace(1, 60, 120):
        with self.subTest(platform=platform, speed=speed):
          angle = min(get_max_angle_vm(speed, self.vm, self.limits), CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX)
          accel = vm.calc_curvature(math.radians(angle), speed, 0) * speed ** 2
          self.assertLessEqual(accel, CarControllerParams.ANGLE_LIMITS.MAX_LATERAL_ACCEL + 1e-6)

  def test_recover_from_reduced_max_angle(self):
    for speed in (10, 20, 30, 40):
      bound = get_max_angle_vm(speed, self.vm, self.limits)
      delta = min(get_max_angle_delta_vm(speed, self.vm, self.limits), self.limits.ANGLE_LIMITS.MAX_ANGLE_RATE)
      for sign in (-1, 1):
        last = sign * min(bound + 10, self.limits.ANGLE_LIMITS.STEER_ANGLE_MAX)
        with self.subTest(speed=speed, sign=sign):
          for _ in range(math.ceil((abs(last) - bound) / delta) + 1):
            angle = apply_subaru_angle_limits(sign * bound * 2, last, speed, last, True, self.limits, self.vm)
            self.assertLessEqual(abs(angle - last), delta + 1e-9)
            self.assertLessEqual(abs(angle), abs(last) + 1e-9)
            last = angle
          self.assertAlmostEqual(abs(last), bound)

  def test_inactive_tracks_measured_angle(self):
    for measured in (-100, 0, 100):
      with self.subTest(measured=measured):
        self.assertEqual(apply_subaru_angle_limits(0, 0, 30, measured, False, self.limits, self.vm), measured)


class TestSubaruCruiseState(unittest.TestCase):
  def test_angle_cruise_uses_es_status(self):
    for platform in CAR:
      if not platform.config.flags & SubaruFlags.LKAS_ANGLE:
        continue
      cp = CarInterface.get_non_essential_params(platform)
      cs = CarState(cp)
      parsers = cs.get_can_parsers(cp)
      cruise_parser = parsers[Bus.alt if cp.flags & SubaruFlags.GLOBAL_GEN2 else Bus.cam]
      for brake_pressed in (False, True):
        parsers[Bus.alt if cp.flags & SubaruFlags.GLOBAL_GEN2 else Bus.pt].vl["Brake_Status"]["Brake"] = brake_pressed
        for status in (False, True):
          for brake_status in (False, True):
            with self.subTest(platform=platform, brake=brake_pressed, status=status, es_brake=brake_status):
              cruise_parser.vl["ES_Status"]["Cruise_Activated"] = status
              cruise_parser.vl["ES_Brake"]["Cruise_Activated"] = brake_status
              self.assertEqual(cs.update(parsers).cruiseState.enabled, status)

  def test_hybrid_cruise_uses_es_brake(self):
    for platform in CAR:
      if not platform.config.flags & SubaruFlags.HYBRID:
        continue
      cp = CarInterface.get_non_essential_params(platform)
      cs = CarState(cp)
      parsers = cs.get_can_parsers(cp)
      cruise_parser = parsers[Bus.alt if cp.flags & SubaruFlags.GLOBAL_GEN2 else Bus.cam]
      for enabled in (False, True):
        with self.subTest(platform=platform, enabled=enabled):
          cruise_parser.vl["ES_Brake"]["Cruise_Activated"] = enabled
          self.assertEqual(cs.update(parsers).cruiseState.enabled, enabled)


class TestSubaruAvailability(unittest.TestCase):
  def test_angle_control_is_development_only(self):
    for platform in CAR:
      if not platform.config.flags & SubaruFlags.LKAS_ANGLE:
        continue
      for is_release in (False, True):
        with self.subTest(platform=platform, is_release=is_release):
          cp = CarInterface.get_params(platform, gen_empty_fingerprint(), [], False, is_release, False)
          self.assertEqual(cp.dashcamOnly, is_release or platform != CAR.SUBARU_CROSSTREK_2025)
