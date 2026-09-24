import itertools
import math
import unittest

import numpy as np

from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.lateral import get_max_angle_vm
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

  def test_small_angle_converges_without_deadband(self):
    for desired in (-1.0, 1.0):
      with self.subTest(desired=desired):
        ci = self.make_angle_controller(3.0)
        cc = structs.CarControl(latActive=True)
        cc.actuators.steeringAngleDeg = desired
        for frame in range(151):
          actuators, _ = ci.CC.update(cc.as_reader(), ci.CS, frame * 10_000_000)
        self.assertAlmostEqual(actuators.steeringAngleDeg, desired, delta=0.01)

  def test_angle_filter_updates_between_steering_messages(self):
    ci = self.make_angle_controller(3.0)
    cc = structs.CarControl(latActive=True)
    cc.actuators.steeringAngleDeg = 1.0
    values = [ci.CC.update(cc.as_reader(), ci.CS, frame * 10_000_000)[0].steeringAngleDeg for frame in range(3)]
    self.assertEqual(values[0], values[1])
    self.assertAlmostEqual(values[2], 1.0 - (0.3 / 0.31) ** 3)

  def test_angle_filter_resets_while_inactive(self):
    ci = self.make_angle_controller(3.0)
    cc = structs.CarControl(latActive=True)
    cc.actuators.steeringAngleDeg = 1.0
    ci.CC.update(cc.as_reader(), ci.CS, 0)
    cc.latActive = False
    cc.actuators.steeringAngleDeg = -1.0
    ci.CC.update(cc.as_reader(), ci.CS, 10_000_000)
    cc.latActive = True
    cc.actuators.steeringAngleDeg = 0.0
    actuators, _ = ci.CC.update(cc.as_reader(), ci.CS, 20_000_000)
    self.assertAlmostEqual(actuators.steeringAngleDeg, -0.3 / 0.31)

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
