import unittest

from opendbc.can import CANPacker
from opendbc.car.gm import gmcan
from opendbc.car.gm.fingerprints import FINGERPRINTS
from opendbc.car.gm.values import CAMERA_ACC_CAR, GM_RX_OFFSET
from opendbc.testing import parameterized

CAMERA_DIAGNOSTIC_ADDRESS = 0x24b


class TestGMFingerprint(unittest.TestCase):
  @parameterized("car_model, fingerprints", FINGERPRINTS.items())
  def test_can_fingerprints(self, car_model, fingerprints):
    assert len(fingerprints) > 0

    assert all(len(finger) for finger in fingerprints)

    # The camera can sometimes be communicating on startup
    if car_model in CAMERA_ACC_CAR:
      for finger in fingerprints:
        for required_addr in (CAMERA_DIAGNOSTIC_ADDRESS, CAMERA_DIAGNOSTIC_ADDRESS + GM_RX_OFFSET):
          assert finger.get(required_addr) == 8, required_addr

  def test_gas_regen_checksum(self):
    packer = CANPacker("gm_global_a_powertrain_generated")
    # Carry boundary when low byte and rolling counter are 0 (captured from 2021 Trailblazer stock camera)
    msg = gmcan.create_gas_regen_command(packer, 0, 154.0, 0, True, False)
    assert msg[1] == bytes.fromhex("0142c50000bd3b00")

    # Borrow boundary when low byte + rolling counter > 0x100
    msg = gmcan.create_gas_regen_command(packer, 0, 153.875, 2, True, False)
    assert msg[1] == bytes.fromhex("8142c4ff00bd3aff")
