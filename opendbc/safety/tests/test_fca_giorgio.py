#!/usr/bin/env python3
import unittest
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common

class TestFcaGiorgio_Safety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  DBC = "fca_giorgio"
  SAFETY_MODEL = CarParams.SafetyModel.fcaGiorgio

  RELAY_MALFUNCTION_ADDRS = {0: (0x117, 0x1F6, 0x4AE, 0x547, 0x5A2)}

  MAX_RATE_UP = 4
  MAX_RATE_DOWN = 4
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 150

  DRIVER_TORQUE_ALLOWANCE = 80
  DRIVER_TORQUE_FACTOR = 3

  TX_MSGS = [[0x117, 0], [0x1F6, 0], [0x4AE, 0], [0x547, 0], [0x5A2, 0]]
  STANDSTILL_THRESHOLD = 0
  FWD_BLACKLISTED_ADDRS = {2: [0x117, 0x1F6, 0x4AE, 0x547, 0x5A2]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  def _button_msg(self, cancel=False, resume=False):
    values = {"CANCEL": cancel, "RESUME": resume}
    return self.packer.make_can_msg_safety("ACC_BUTTON", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_STATUS": enable}
    return self.packer.make_can_msg_safety("ACC_2", 2, values)

  def _speed_msg(self, speed):
    values = {"WHEEL_SPEED_%s" % s: speed for s in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("ABS_1", 0, values)

  def _user_gas_msg(self, gas):
   values = {"ACCEL_PEDAL": gas}
   return self.packer.make_can_msg_safety("ENGINE_1", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PEDAL_SWITCH": 1 if brake else 0}
    return self.packer.make_can_msg_safety("ABS_3", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"DRIVER_TORQUE": torque}
    return self.packer.make_can_msg_safety("EPS_2", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKA_TORQUE": torque, "LKA_ACTIVE": steer_req}
    return self.packer.make_can_msg_safety("LKA_COMMAND", 0, values)

  def test_rx_hook(self):
    for count in range(20):
      self.assertTrue(self._rx(self._speed_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_brake_msg(False)), f"{count=}")
      self.assertTrue(self._rx(self._torque_driver_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_gas_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._pcm_status_msg(False)), f"{count=}")


if __name__ == "__main__":
  unittest.main()
