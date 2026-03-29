#!/usr/bin/env python3
import unittest
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

class TestFcaGiorgio_Safety(common.CarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (0x1F6,)}

  MAX_RATE_UP = 4
  MAX_RATE_DOWN = 4
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 150

  DRIVER_TORQUE_ALLOWANCE = 80
  DRIVER_TORQUE_FACTOR = 3

  TX_MSGS = [[0x1F6, 0], [0x4AE, 0], [0x547, 0]]
  STANDSTILL_THRESHOLD = 0
  FWD_BLACKLISTED_ADDRS = {2: [0x1F6, 0x4AE, 0x547]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  def setUp(self):
    self.packer = CANPackerSafety("fca_giorgio")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.fcaGiorgio, 0)
    self.safety.init_tests()

  #def _button_msg(self, cancel=False, resume=False):
  #  pass

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_STATUS": 2 if enable else 1}
    return self.packer.make_can_msg_safety("ACC_1", 0, values)

  def _speed_msg(self, speed):
    values = {"WHEEL_SPEED_%s" % s: speed for s in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("ABS_1", 0, values)

  #def _user_gas_msg(self, gas):
  #  pass

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PEDAL_SWITCH": 1 if brake else 0}
    return self.packer.make_can_msg_safety("ABS_3", 0, values)

  def _torque_meas_msg(self, torque):
    values = {"EPS_TORQUE": torque}
    return self.packer.make_can_msg_safety("EPS_3", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKA_TORQUE": torque, "LKA_ACTIVE": steer_req}
    return self.packer.make_can_msg_safety("LKA_COMMAND", 0, values)

  def test_rx_hook(self):
    for count in range(20):
      self.assertTrue(self._rx(self._speed_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_brake_msg(False)), f"{count=}")
      self.assertTrue(self._rx(self._torque_meas_msg(0)), f"{count=}")
      #self.assertTrue(self._rx(self._user_gas_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._pcm_status_msg(False)), f"{count=}")


if __name__ == "__main__":
  unittest.main()
