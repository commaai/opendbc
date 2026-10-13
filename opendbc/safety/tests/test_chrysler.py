#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.car.chrysler.values import ChryslerSafetyFlags
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common


class TestChryslerSafetyBase(common.CarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  SAFETY_MODEL = CarParams.SafetyModel.chrysler

  MAX_TORQUE_ERROR = 80

  LKAS_ACTIVE_VALUE = 1

  DAS_BUS = 0

  def _button_msg(self, cancel=False, resume=False):
    values = {"ACC_Cancel": cancel, "ACC_Resume": resume}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", self.DAS_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACC_ACTIVE": enable}
    return self.packer.make_can_msg_safety("DAS_3", self.DAS_BUS, values)

  def _speed_msg(self, speed):
    raise NotImplementedError

  def _user_gas_msg(self, gas):
    values = {"Accelerator_Position": gas}
    return self.packer.make_can_msg_safety("ECM_5", 0, values)

  def _user_brake_msg(self, brake):
    values = {"Brake_Pedal_State": 1 if brake else 0}
    return self.packer.make_can_msg_safety("ESP_1", 0, values)

  def _torque_meas_msg(self, torque):
    values = {"EPS_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("EPS_2", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEERING_TORQUE": torque, "LKAS_CONTROL_BIT": self.LKAS_ACTIVE_VALUE if steer_req else 0}
    return self.packer.make_can_msg_safety("LKAS_COMMAND", 0, values)

  def test_buttons(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)

      # resume only while controls allowed
      self.assertEqual(controls_allowed, self._tx(self._button_msg(resume=True)))

      # can always cancel
      self.assertTrue(self._tx(self._button_msg(cancel=True)))

      # only one button at a time
      self.assertFalse(self._tx(self._button_msg(cancel=True, resume=True)))
      self.assertFalse(self._tx(self._button_msg(cancel=False, resume=False)))


class TestChryslerSafety(TestChryslerSafetyBase):
  DBC = "chrysler_pacifica_2017_hybrid_generated"

  TX_MSGS = [[0x23B, 0], [0x292, 0], [0x2A6, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x292, 0x2A6)}
  FWD_BLACKLISTED_ADDRS = {2: [0x292, 0x2A6]}

  MAX_RATE_UP = 3
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [261]
  MAX_RT_DELTA = 112

  def _wheel_speeds_msg(self, speed_l, speed_r):
    values = {"SPEED_LEFT": speed_l, "SPEED_RIGHT": speed_r}
    return self.packer.make_can_msg_safety("SPEED_1", 0, values)

  def _speed_msg(self, speed):
    return self._wheel_speeds_msg(speed, speed)

  def test_vehicle_moving(self):
    super().test_vehicle_moving()

    # moving if either wheel is turning, down to a single bit of either byte of the signal
    factor = self.packer.dbc.name_to_msg["SPEED_1"].sigs["SPEED_LEFT"].factor
    for raw in (1, 16, 256):
      for speed_l, speed_r in ((raw * factor, 0), (0, raw * factor)):
        self._rx(self._wheel_speeds_msg(speed_l, speed_r))
        self.assertTrue(self.safety.get_vehicle_moving(), f"{speed_l=} {speed_r=}")


class TestChryslerRamSafetyBase(TestChryslerSafetyBase):
  DAS_BUS = 2

  LKAS_ACTIVE_VALUE = 2

  def _speed_msg(self, speed):
    values = {"Vehicle_Speed": speed}
    return self.packer.make_can_msg_safety("ESP_8", 0, values)

  def test_vehicle_moving(self):
    super().test_vehicle_moving()

    # moving at any nonzero speed, 1/128 kph per LSB
    for speed in np.arange(0, 100, 0.5):
      self._rx(self._speed_msg(speed))
      self.assertEqual(speed > 0, self.safety.get_vehicle_moving())


class TestChryslerRamDTSafety(TestChryslerRamSafetyBase):
  DBC = "chrysler_ram_dt_generated"

  SAFETY_PARAM = ChryslerSafetyFlags.RAM_DT

  TX_MSGS = [[0xB1, 2], [0xA6, 0], [0xFA, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0xA6, 0xFA)}
  FWD_BLACKLISTED_ADDRS = {2: [0xA6, 0xFA]}

  MAX_RATE_UP = 6
  MAX_RATE_DOWN = 6
  MAX_TORQUE_LOOKUP = [0], [350]
  MAX_RT_DELTA = 112


class TestChryslerRamHDSafety(TestChryslerRamSafetyBase):
  DBC = "chrysler_ram_hd_generated"

  SAFETY_PARAM = ChryslerSafetyFlags.RAM_HD

  TX_MSGS = [[0x275, 0], [0x276, 0], [0x23A, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x276, 0x275)}
  FWD_BLACKLISTED_ADDRS = {2: [0x275, 0x276]}

  MAX_TORQUE_LOOKUP = [0], [361]
  MAX_RATE_UP = 14
  MAX_RATE_DOWN = 14
  MAX_RT_DELTA = 182


if __name__ == "__main__":
  unittest.main()
