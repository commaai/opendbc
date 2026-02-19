#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerPanda


class TestGwm(common.PandaSafetyTest):
  TX_MSGS = [[0x12B, 0], [0x147, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12B), 2: (0x147)}
  FWD_BLACKLISTED_ADDRS = {0: [0x147], 2: [0x12B]}

  MAX_RATE_UP = 3
  MAX_RATE_DOWN = 5
  MAX_TORQUE_LOOKUP = [0], [200]
  MAX_RT_DELTA = 100
  MAX_TORQUE_ERROR = 70

  def setUp(self):
    self.packer = CANPackerPanda("gwm_haval_h6_mk3_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gwm, 0)
    self.safety.init_tests()

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSURE": brake}
    return self.packer.make_can_msg_panda("BRAKE", 0, values)

  def _speed_msg(self, speed):
    values = {f"{pos}_WHEEL_SPEED": speed * 1.0 for pos in ["FRONT_LEFT", "FRONT_RIGHT", "REAR_LEFT", "REAR_RIGHT"]}
    return self.packer.make_can_msg_panda("WHEEL_SPEEDS", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_STATE": 3 if enable else 0}
    return self.packer.make_can_msg_panda("ACC_CMD", 0, values)

  def test_rx_hook(self):
    self.assertTrue(self._rx(self._speed_msg(0)))

  def _torque_meas_msg(self, torque):
    values = {"B_RX_EPS_TORQUE": torque}
    return self.packer.make_can_msg_panda("RX_STEER_RELATED", 0, values)

  def _steer_cmd_msg(self, torque, steer_req=1):
    values = {"STEER_REQUEST": steer_req, "TORQUE_CMD": torque}
    return self.packer.make_can_msg_panda("STEER_CMD", 0, values)


if __name__ == "__main__":
  unittest.main()
