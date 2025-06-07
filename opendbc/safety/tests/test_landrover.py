#!/usr/bin/env python3
import unittest
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda

MSG_CAM_MSG = 0x1BE
MSG_LKAS_C2F = 0x1F0
MSG_HUD_C2F = 0x1F9


class TestLandroverSafety(common.PandaCarSafetyTest, common.AngleSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_CAM_MSG, )}
  FWD_BLACKLISTED_ADDRS = {}
  TX_MSGS = [[MSG_LKAS_C2F, 1], [MSG_HUD_C2F, 1], [MSG_CAM_MSG, 0] ]


  # Angle control limits
  STEER_ANGLE_MAX = 90 # deg
  DEG_TO_CAN  = 13.009

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
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.landrover, 0)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    values = {"ReqAngleTorque": angle,
              "EnAngle"       : 1 if enabled else 0,
             }
    return self.packer.make_can_msg_panda("LKAS_OP_TO_FLEXRAY", 1, values)

  """
  def _angle_meas_msg(self, angle: float):
    values = {"SteerAngle": angle}
    return self.packer.make_can_msg_panda("SWM_Angle", 0, values)
  """

  def _angle_meas_msg(self, angle: float):
    values = { "AngleTorque": angle}
    return self.packer.make_can_msg_panda("PSCM_Out", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BrakeDriver": brake}
    return self.packer.make_can_msg_panda("StopAndGo", 0, values)

  def _speed_msg(self, speed):
    values = {"WheelSpeed": speed * 3.6}
    return self.packer.make_can_msg_panda("Info02", 0, values)

  def _user_gas_msg(self, gas):
    values = {"GasPedalDriver": gas > 0.1 }
    return self.packer.make_can_msg_panda("GasPedal_ON", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CruiseOn": 1 if enable else 0}
    return self.packer.make_can_msg_panda("CruiseInfo", 0, values)


if __name__ == "__main__":
  unittest.main()
