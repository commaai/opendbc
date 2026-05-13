#!/usr/bin/env python3
import unittest

from opendbc.safety.tests.common import CANPackerSafety, make_msg
from opendbc.safety.tests.libsafety import libsafety_py


class TestIgnitionHook(unittest.TestCase):
  TX_MSGS: list = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.gm_packer = CANPackerSafety("gm_global_a_powertrain_generated")
    self.rivian_packer = CANPackerSafety("rivian_primary_actuator")
    self.tesla_packer = CANPackerSafety("tesla_model3_party")

  def _gm_msg(self, mode):
    return self.gm_packer.make_can_msg_safety("BCMGeneralPlatformStatus", 0,
                                              {"SystemPowerMode": mode})

  def _rivian_msg(self, counter, mode):
    return self.rivian_packer.make_can_msg_safety("VDM_OutputSignals", 0,
                                                  {"VDM_OutputSigs_Counter": counter,
                                                   "VDM_EpasPowerMode": mode})

  def _tesla_msg(self, counter, state):
    return self.tesla_packer.make_can_msg_safety("VCFRONT_LVPowerState", 0,
                                                 {"VCFRONT_LVPowerStateCounter": counter,
                                                  "VCFRONT_vehiclePowerState": state})

  def _mazda_msg(self, byte0):
    return make_msg(0, 0x9E, dat=bytes([byte0]) + b"\x00" * 7)

  # GM: SystemPowerMode 2=Run, 3=Crank Request
  def test_gm_ignition_on(self):
    self.safety.ignition_can_hook(self._gm_msg(2))
    self.assertTrue(self.safety.get_ignition_can())

  def test_gm_ignition_off(self):
    self.safety.ignition_can_hook(self._gm_msg(2))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._gm_msg(0))
    self.assertFalse(self.safety.get_ignition_can())

  # Rivian: VDM_EpasPowerMode_Drive_On=1 (counter-gated)
  def test_rivian_ignition_on(self):
    for i in range(15):
      self.safety.init_tests()
      self.safety.ignition_can_hook(self._rivian_msg(i, 1))
      self.assertFalse(self.safety.get_ignition_can())
      self.safety.ignition_can_hook(self._rivian_msg((i + 1) % 15, 1))
      self.assertTrue(self.safety.get_ignition_can())

  def test_rivian_ignition_off(self):
    self.safety.ignition_can_hook(self._rivian_msg(0, 1))
    self.safety.ignition_can_hook(self._rivian_msg(1, 1))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._rivian_msg(2, 0))
    self.safety.ignition_can_hook(self._rivian_msg(3, 0))
    self.assertFalse(self.safety.get_ignition_can())

  # Tesla: VEHICLE_POWER_STATE_DRIVE=3 (counter-gated)
  def test_tesla_ignition_on(self):
    for i in range(16):
      self.safety.init_tests()
      self.safety.ignition_can_hook(self._tesla_msg(i, 3))
      self.assertFalse(self.safety.get_ignition_can())
      self.safety.ignition_can_hook(self._tesla_msg((i + 1) % 16, 3))
      self.assertTrue(self.safety.get_ignition_can())

  def test_tesla_ignition_off(self):
    self.safety.ignition_can_hook(self._tesla_msg(0, 3))
    self.safety.ignition_can_hook(self._tesla_msg(1, 3))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._tesla_msg(2, 2))
    self.safety.ignition_can_hook(self._tesla_msg(3, 2))
    self.assertFalse(self.safety.get_ignition_can())

  # Mazda: 0x9E byte 0 high 3 bits == 6 (0xC0)
  def test_mazda_ignition_on(self):
    self.safety.ignition_can_hook(self._mazda_msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())

  def test_mazda_ignition_off(self):
    self.safety.ignition_can_hook(self._mazda_msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._mazda_msg(0x20))
    self.assertFalse(self.safety.get_ignition_can())

  def test_wrong_bus_ignored(self):
    self.safety.ignition_can_hook(make_msg(1, 0x1F1, dat=b"\x02" + b"\x00" * 7))
    self.assertFalse(self.safety.get_ignition_can())

  def test_unknown_addr_ignored(self):
    self.safety.ignition_can_hook(make_msg(0, 0x123, dat=b"\xFF" * 8))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
