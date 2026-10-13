#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.subaru.values import SubaruSafetyFlags
import opendbc.safety.tests.common as common


class TestSubaruPreglobalSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  SAFETY_MODEL = CarParams.SafetyModel.subaruPreglobal

  DBC = "subaru_outback_2015_generated"
  TX_MSGS = [[0x161, 0], [0x164, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x164, 0x161)}
  FWD_BLACKLISTED_ADDRS = {2: [0x161, 0x164]}

  MAX_RATE_UP = 50
  MAX_RATE_DOWN = 70
  MAX_TORQUE_LOOKUP = [0], [2047]

  MAX_RT_DELTA = 940

  DRIVER_TORQUE_ALLOWANCE = 75
  DRIVER_TORQUE_FACTOR = 10

  def _set_prev_torque(self, t):
    self.safety.set_desired_torque_last(t)
    self.safety.set_rt_torque_last(t)

  def _torque_driver_msg(self, torque):
    values = {"Steer_Torque_Sensor": torque}
    return self.packer.make_can_msg_safety("Steering_Torque", 0, values)

  def _speed_msg(self, speed, wheels=("FR", "FL", "RR", "RL")):
    # subaru safety doesn't use the scaled value, so undo the scaling
    values = {s: speed*0.0592 if s in wheels else 0 for s in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_safety("Wheel_Speeds", 0, values)

  def _user_brake_msg(self, brake):
    values = {"Brake_Pedal": brake}
    return self.packer.make_can_msg_safety("Brake_Pedal", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Command": torque, "LKAS_Active": steer_req}
    return self.packer.make_can_msg_safety("ES_LKAS", 0, values)

  def _user_gas_msg(self, gas):
    values = {"Throttle_Pedal": gas}
    return self.packer.make_can_msg_safety("Throttle", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_safety("CruiseControl", 0, values)

  def test_vehicle_moving_any_wheel(self):
    # any single wheel reporting a non-zero speed means the vehicle is moving
    # (the safety code only reads the upper bits of FL, so it is not covered here)
    for wheel in ["FR", "RR", "RL"]:
      self._rx(self._speed_msg(0))
      self.assertFalse(self.safety.get_vehicle_moving())
      self._rx(self._speed_msg(1, wheels=(wheel,)))
      self.assertTrue(self.safety.get_vehicle_moving())


class TestSubaruPreglobalReversedDriverTorqueSafety(TestSubaruPreglobalSafety):
  SAFETY_PARAM = SubaruSafetyFlags.PREGLOBAL_REVERSED_DRIVER_TORQUE

  DBC = "subaru_outback_2019_generated"


if __name__ == "__main__":
  unittest.main()
