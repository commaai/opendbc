#!/usr/bin/env python3
import unittest

from opendbc.safety.tests.common import CANPackerSafety, make_msg
from opendbc.safety.tests.libsafety import libsafety_py


class TestIgnitionHook(unittest.TestCase):

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.gm_packer = CANPackerSafety("gm_global_a_powertrain_generated")
    self.rivian_packer = CANPackerSafety("rivian_primary_actuator")
    self.tesla_packer = CANPackerSafety("tesla_model3_party")

  # GM: SystemPowerMode 2=Run, 3=Crank Request
  def test_gm_ignition_on(self):
    self.safety.ignition_can_hook(self.gm_packer.make_can_msg_safety("BCMGeneralPlatformStatus", 0, {"SystemPowerMode": 2}))
    self.assertTrue(self.safety.get_ignition_can())

  def test_gm_ignition_off(self):
    self.safety.ignition_can_hook(self.gm_packer.make_can_msg_safety("BCMGeneralPlatformStatus", 0, {"SystemPowerMode": 2}))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self.gm_packer.make_can_msg_safety("BCMGeneralPlatformStatus", 0, {"SystemPowerMode": 0}))
    self.assertFalse(self.safety.get_ignition_can())

  # Rivian: VDM_EpasPowerMode_Drive_On=1
  def test_rivian_ignition_on(self):
    for i in range(15):
      self.safety.init_tests()
      self.safety.ignition_can_hook(self.rivian_packer.make_can_msg_safety("VDM_OutputSignals", 0,
                                                                           {"VDM_OutputSigs_Counter": i, "VDM_EpasPowerMode": 1}))
      self.assertFalse(self.safety.get_ignition_can())
      self.safety.ignition_can_hook(self.rivian_packer.make_can_msg_safety("VDM_OutputSignals", 0,
                                                                           {"VDM_OutputSigs_Counter": (i + 1) % 15, "VDM_EpasPowerMode": 1}))
      self.assertTrue(self.safety.get_ignition_can())

  def test_rivian_ignition_off(self):
    self.safety.ignition_can_hook(self.rivian_packer.make_can_msg_safety("VDM_OutputSignals", 0,
                                                                         {"VDM_OutputSigs_Counter": 0, "VDM_EpasPowerMode": 1}))
    self.safety.ignition_can_hook(self.rivian_packer.make_can_msg_safety("VDM_OutputSignals", 0,
                                                                         {"VDM_OutputSigs_Counter": 1, "VDM_EpasPowerMode": 1}))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self.rivian_packer.make_can_msg_safety("VDM_OutputSignals", 0,
                                                                         {"VDM_OutputSigs_Counter": 2, "VDM_EpasPowerMode": 0}))
    self.safety.ignition_can_hook(self.rivian_packer.make_can_msg_safety("VDM_OutputSignals", 0,
                                                                         {"VDM_OutputSigs_Counter": 3, "VDM_EpasPowerMode": 0}))
    self.assertFalse(self.safety.get_ignition_can())

  # Tesla: VEHICLE_POWER_STATE_DRIVE=3
  def test_tesla_ignition_on(self):
    self.safety.ignition_can_hook(self.tesla_packer.make_can_msg_safety("VCFRONT_LVPowerState", 0,
                                                                        {"VCFRONT_LVPowerStateCounter": 0, "VCFRONT_vehiclePowerState": 3}))
    self.assertFalse(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self.tesla_packer.make_can_msg_safety("VCFRONT_LVPowerState", 0,
                                                                        {"VCFRONT_LVPowerStateCounter": 1, "VCFRONT_vehiclePowerState": 3}))
    self.assertTrue(self.safety.get_ignition_can())

  def test_tesla_ignition_off(self):
    self.safety.ignition_can_hook(self.tesla_packer.make_can_msg_safety("VCFRONT_LVPowerState", 0,
                                                                        {"VCFRONT_LVPowerStateCounter": 0, "VCFRONT_vehiclePowerState": 3}))
    self.safety.ignition_can_hook(self.tesla_packer.make_can_msg_safety("VCFRONT_LVPowerState", 0,
                                                                        {"VCFRONT_LVPowerStateCounter": 1, "VCFRONT_vehiclePowerState": 3}))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self.tesla_packer.make_can_msg_safety("VCFRONT_LVPowerState", 0,
                                                                        {"VCFRONT_LVPowerStateCounter": 2, "VCFRONT_vehiclePowerState": 2}))
    self.safety.ignition_can_hook(self.tesla_packer.make_can_msg_safety("VCFRONT_LVPowerState", 0,
                                                                        {"VCFRONT_LVPowerStateCounter": 3, "VCFRONT_vehiclePowerState": 2}))
    self.assertFalse(self.safety.get_ignition_can())

  # Mazda: 0x9E byte 0 high 3 bits == 6
  def test_mazda_ignition_on(self):
    self.safety.ignition_can_hook(make_msg(0, 0x9E, dat=b"\xC0" + b"\x00" * 7))
    self.assertTrue(self.safety.get_ignition_can())

  def test_mazda_ignition_off(self):
    self.safety.ignition_can_hook(make_msg(0, 0x9E, dat=b"\xC0" + b"\x00" * 7))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(make_msg(0, 0x9E, dat=b"\x20" + b"\x00" * 7))
    self.assertFalse(self.safety.get_ignition_can())

  def test_wrong_bus_ignored(self):
    self.safety.ignition_can_hook(make_msg(1, 0x1F1, dat=b"\x02" + b"\x00" * 7))
    self.assertFalse(self.safety.get_ignition_can())

  def test_unknown_addr_ignored(self):
    self.safety.ignition_can_hook(make_msg(0, 0x123, dat=b"\xFF" * 8))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
