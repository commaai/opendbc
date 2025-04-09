#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda

BYD_CANADDR_IPB               = 0x1F0
BYD_CANADDR_ACC_MPC_STATE     = 0x316
BYD_CANADDR_ACC_EPS_STATE     = 0x318
BYD_CANADDR_ACC_HUD_ADAS      = 0x32D
BYD_CANADDR_ACC_CMD           = 0x32E
BYD_CANADDR_PCM_BUTTONS       = 0x3B0
BYD_CANADDR_DRIVE_STATE       = 0x242
BYD_CANADDR_PEDAL             = 0x342
BYD_CANADDR_CARSPEED          = 0x121

class TestBydSafety(common.PandaCarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  #0x316 ACC_MPC_STATE
  #0x318 ACC_EPS_STATE
  #0x32E ACC_CMD
  TX_MSGS = [[BYD_CANADDR_ACC_MPC_STATE, 0],
             [BYD_CANADDR_ACC_EPS_STATE, 2],
             [BYD_CANADDR_ACC_CMD, 0]]

  RELAY_MALFUNCTION_ADDRS = {0: (BYD_CANADDR_ACC_MPC_STATE,)}
  FWD_BLACKLISTED_ADDRS = {2: [BYD_CANADDR_ACC_MPC_STATE, BYD_CANADDR_ACC_CMD],
                           0:[BYD_CANADDR_ACC_EPS_STATE]}

  STANDSTILL_THRESHOLD = 0

  MAX_RATE_UP = 17
  MAX_RATE_DOWN = 17
  MAX_TORQUE_LOOKUP = [0], [300]

  MAX_RT_DELTA = 243

  DRIVER_TORQUE_ALLOWANCE = 68
  DRIVER_TORQUE_FACTOR = 1

  def setUp(self):
    self.packer = CANPackerPanda("byd_han_dmev_2020")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, 1)
    self.safety.init_tests()

  def _torque_meas_msg(self, torque):
    values = {"MainTorque": torque}
    return self.packer.make_can_msg_panda("ACC_EPS_STATE", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Output": torque, "LKAS_Active": steer_req}
    return self.packer.make_can_msg_panda("ACC_MPC_STATE", 2, values)

  def _speed_msg(self, speed):
    values = {"CarDisplaySpeed": speed}
    return self.packer.make_can_msg_panda("CARSPEED", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BrakePedal": brake}
    return self.packer.make_can_msg_panda("PEDAL", 0, values)

  def _user_gas_msg(self, gas):
    values = {"AcceleratorPedal": gas}
    return self.packer.make_can_msg_panda("PEDAL", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"AccState": 3 if enable else 0}
    return self.packer.make_can_msg_panda("ACC_HUD_ADAS", 2, values)

  def _button_msg(self, resume=False, cancel=False):
    values = {
      "BTN_AccCancel": cancel,
      "BTN_AccUpDown_Cmd": 3 if resume else 0,
    }
    return self.packer.make_can_msg_panda("PCM_BUTTONS", 0, values)

if __name__ == "__main__":
  unittest.main()
