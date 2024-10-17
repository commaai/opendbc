#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.can.can_define import CANDefine
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda

MSG_CAM_MSG = 0x1be
MSG_LKAS_C2F = 0x1F0


class TestLandroverSafetyBase(common.PandaCarSafetyTest, common.AngleSteeringSafetyTest, common.LongitudinalAccelSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_CAM_MSG)}
  FWD_BLACKLISTED_ADDRS = {}
  TX_MSGS = [[MSG_LKAS_C2F, 1]]

  STANDSTILL_THRESHOLD = 0.1
  GAS_PRESSED_THRESHOLD = 3

  # Angle control limits
  STEER_ANGLE_MAX = 360  # deg
  DEG_TO_CAN = 10

  ANGLE_RATE_BP = [0., 5., 25.]
  ANGLE_RATE_UP = [2.5, 1.5, 0.2]  # windup limit
  ANGLE_RATE_DOWN = [5., 2.0, 0.3]  # unwind limit

  # Long control limits
  MAX_ACCEL = 2.0
  MIN_ACCEL = -3.48
  INACTIVE_ACCEL = 0.0

  packer: CANPackerPanda

  def setUp(self):
    self.packer = CANPackerPanda("landrover_defender_2023")
    self.define = CANDefine("landrover_defender_2023")
    #self.acc_states = {d: v for v, d in self.define.dv["DAS_control"]["DAS_accState"].items()}

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    values = {"ReqAngleTorque": angle, "EnAngle": 1 if enabled else 0}
    return self.packer.make_can_msg_panda("LKAS_OP_TO_FLEXRAY", 1, values)

  def _angle_meas_msg(self, angle: float):
    values = {"SteerAngle": angle}
    return self.packer.make_can_msg_panda("SWM_Angle", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BrakePedal": 1 if brake else 0}
    return self.packer.make_can_msg_panda("CruiseInfo", 0, values)

  def _speed_msg(self, speed):
    values = {"WheelSpeed": speed * 3.6}
    return self.packer.make_can_msg_panda("Info02", 0, values)

  def _user_gas_msg(self, gas):
    values = {"GasPedal_On": gas}
    return self.packer.make_can_msg_panda("GasPedal_ON", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"DI_cruiseState": 2 if enable else 0}
    return self.packer.make_can_msg_panda("DI_state", 0, values)


  """

  def _long_control_msg(self, set_speed, acc_state=0, jerk_limits=(0, 0), accel_limits=(0, 0), aeb_event=0, bus=0):
    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": aeb_event,
      "DAS_jerkMin": jerk_limits[0],
      "DAS_jerkMax": jerk_limits[1],
      "DAS_accelMin": accel_limits[0],
      "DAS_accelMax": accel_limits[1],
    }
    return self.packer.make_can_msg_panda("DAS_control", bus, values)
  """

  def _accel_msg(self, accel: float):
    # For common.LongitudinalAccelSafetyTest
    return self._long_control_msg(10, accel_limits=(accel, max(accel, 0)))

  def test_vehicle_speed_measurements(self):
    # OVERRIDDEN: 79.1667 is the max speed in m/s
    self._common_measurement_test(self._speed_msg, 0, 285 / 3.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)


class TestLandroverStockSafety(TestLandroverSafetyBase):

  LONGITUDINAL = False

  def setUp(self):
    super().setUp()
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.landrover, 0)
    self.safety.init_tests()

  def test_cancel(self):
    for acc_state in range(16):
      self.safety.set_controls_allowed(True)
      should_tx = acc_state == self.acc_states["ACC_CANCEL_GENERIC_SILENT"]
      self.assertFalse(self._tx(self._long_control_msg(0, acc_state=acc_state, accel_limits=(self.MIN_ACCEL, self.MAX_ACCEL))))
      self.assertEqual(should_tx, self._tx(self._long_control_msg(0, acc_state=acc_state)))

  def test_no_aeb(self):
    for aeb_event in range(4):
      self.assertEqual(self._tx(self._long_control_msg(10, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"], aeb_event=aeb_event)), aeb_event == 0)


if __name__ == "__main__":
  unittest.main()
