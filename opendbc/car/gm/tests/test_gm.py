import unittest

from opendbc.car.gm.fingerprints import FINGERPRINTS
from opendbc.car.gm.values import CAMERA_ACC_CAR, GM_RX_OFFSET

CAMERA_DIAGNOSTIC_ADDRESS = 0x24b


class TestGMFingerprint(unittest.TestCase):
  def test_can_fingerprints(self):
    for car_model, fingerprints in FINGERPRINTS.items():
      with self.subTest(car_model=car_model):
        assert len(fingerprints) > 0

        assert all(len(finger) for finger in fingerprints)

        # The camera can sometimes be communicating on startup
        if car_model in CAMERA_ACC_CAR:
          for finger in fingerprints:
            for required_addr in (CAMERA_DIAGNOSTIC_ADDRESS, CAMERA_DIAGNOSTIC_ADDRESS + GM_RX_OFFSET):
              assert finger.get(required_addr) == 8, required_addr
