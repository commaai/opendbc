import unittest

from opendbc.car.lateral import apply_steer_angle_limits_vm, get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.subaru.carcontroller import get_safety_CP
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel


class TestSubaruFingerprint(unittest.TestCase):
  def test_fw_version_format(self):
    for platform, fws_per_ecu in FW_VERSIONS.items():
      for (ecu, _, _), fws in fws_per_ecu.items():
        fw_size = len(fws[0])
        for fw in fws:
          assert len(fw) == fw_size, f"{platform} {ecu}: {len(fw)} {fw_size}"


class TestSubaruAngleLimits(unittest.TestCase):
  def test_engagement_outside_max_angle_is_rate_limited(self):
    speed = 13.24
    angle_last = -57.61
    vm = VehicleModel(get_safety_CP())

    angle = apply_steer_angle_limits_vm(-51.60, angle_last, speed, angle_last, True, CarControllerParams, vm)

    self.assertAlmostEqual(angle - angle_last, get_max_angle_delta_vm(speed, vm, CarControllerParams))
    self.assertGreater(abs(angle), get_max_angle_vm(speed, vm, CarControllerParams))
