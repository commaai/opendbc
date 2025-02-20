#!/usr/bin/env python3
import unittest

from opendbc.safety import Safety
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda


class TestRivianSafety(common.PandaCarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  TX_MSGS = [[0x120, 0]]
  STANDSTILL_THRESHOLD = 0
  RELAY_MALFUNCTION_ADDRS = {0: (0x120,)}
  FWD_BLACKLISTED_ADDRS = {2: [0x120]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  MAX_RATE_UP = 8
  MAX_RATE_DOWN = 8
  MAX_TORQUE = 350

  MAX_RT_DELTA = 300
  RT_INTERVAL = 250000

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 1

  def setUp(self):
    self.packer = CANPackerPanda("rivian_can")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(Safety.SAFETY_RIVIAN, 0)
    self.safety.init_tests()

  def _torque_driver_msg(self, torque):
    values = {"EPAS_SystemStatus": torque}
    return self.packer.make_can_msg_panda("EPAS_TorsionBarTorque", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"ACM_lkaStrToqReq": torque, "ACM_lkaActToi": steer_req}
    return self.packer.make_can_msg_panda("ACM_lkaHbaCmd", 0, values)

  def _speed_msg(self, speed):
    values = {"ESP_Status": speed * 3.6}
    return self.packer.make_can_msg_panda("ESP_Vehicle_Speed", 0, values)

  def _user_brake_msg(self, brake):
    values = {"iBESP2_BrakePedalApplied": brake}
    return self.packer.make_can_msg_panda("iBESP2", 0, values)

  def _user_gas_msg(self, gas):
    values = {"VDM_AcceleratorPedalPosition": gas}
    return self.packer.make_can_msg_panda("VDM_PropStatus", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"ACM_FeatureStatus": enable}
    return self.packer.make_can_msg_panda("ACM_Status", 2, values)

if __name__ == "__main__":
  unittest.main()
