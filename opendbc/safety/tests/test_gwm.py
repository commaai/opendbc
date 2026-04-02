#!/usr/bin/env python3
import unittest

from opendbc.can.dbc import DBC
from opendbc.car.gwm.values import GwmSafetyFlags
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.car.gwm.gwmcan import checksum as _checksum


opendbc = "gwm_haval_h6_mk3_generated"
dbc = DBC(opendbc)


def get_signal_range(signal):
  if signal.is_signed:
    # For signed: -(2^(bits-1)) a (2^(bits-1)-1)
    max_val = (2 ** (signal.size - 1)) - 1
    min_val = -(2 ** (signal.size - 1))
  else:
    # For unsigned: 0 a (2^bits - 1)
    max_val = (2 ** signal.size) - 1
    min_val = 0

  # Apply factor and offset
  physical_max = max_val * signal.factor + signal.offset
  physical_min = min_val * signal.factor + signal.offset

  return physical_min, physical_max


def checksum(msg):
  addr, dat, bus = msg
  ret = bytearray(dat)

  if addr == 0xA1: # STEER_AND_AP_STALK
    ret[0] = _checksum(ret[1:], 0x2D)
  elif addr == 0x13B: # WHEEL_SPEEDS
    ret[0] = _checksum(ret[1:8], 0x7F)

  return addr, ret, bus


# class TestGwm(common.CarSafetyTest, common.MotorTorqueSteeringSafetyTest, common.LongitudinalGasBrakeSafetyTest,
#               common.VehicleSpeedSafetyTest):

class TestGwmSafety(common.CarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  TX_MSGS = [[0x12B, 0], [0x143, 0], [0x147, 2], [0xA1, 2]] # Steer, long, wheel touch, cancel
  RELAY_MALFUNCTION_ADDRS = {0: (0x12B, 0x143), 2: (0x147,)}
  FWD_BLACKLISTED_ADDRS = {0: [0x147], 2: [0x12B, 0x143]}

  MAX_RATE_UP = 4
  MAX_RATE_DOWN = 6
  MAX_TORQUE_LOOKUP = [0], [253]
  MAX_RT_DELTA = 100
  MAX_TORQUE_ERROR = 80
  TORQUE_MEAS_TOLERANCE = 1

  MIN_GAS = -10
  MAX_GAS = 4577
  INACTIVE_GAS = 0
  MAX_BRAKE = 107

  MAX_POSSIBLE_BRAKE = 108
  MAX_POSSIBLE_GAS = 4578  # reasonably excessive limits, not signal max
  MIN_POSSIBLE_GAS = -11

  def setUp(self):
    self.packer = CANPackerSafety(opendbc)
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gwm, GwmSafetyFlags.LONG_CONTROL)
    self.safety.init_tests()

  def _user_gas_msg(self, gas):
    values = {"GAS_POSITION": gas}
    return self.packer.make_can_msg_safety("CAR_OVERALL_SIGNALS2", 0, values)

  def _user_brake_msg(self, brake):
    values = {"PEDAL_BRAKE_PRESSED": brake}
    return self.packer.make_can_msg_safety("BRAKE2", 0, values)

  def _speed_msg(self, speed):
    values = {f"{pos}_WHEEL_SPEED": speed * 1.0 for pos in ["FRONT_LEFT", "FRONT_RIGHT", "REAR_LEFT", "REAR_RIGHT"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, values, fix_checksum=checksum)

  def _pcm_status_msg(self, enable):
    values = {"AP_ENABLE_COMMAND": enable, "AP_CANCEL_COMMAND": not enable}
    return self.packer.make_can_msg_safety("STEER_AND_AP_STALK", 0, values, fix_checksum=checksum)

  def test_main_cancel_button(self):
    self.safety.set_controls_allowed(True)
    self._rx(self.packer.make_can_msg_safety("STEER_AND_AP_STALK", 0, {"AP_CANCEL_COMMAND": 1}, fix_checksum=checksum))
    self.assertFalse(self.safety.get_controls_allowed())

  def _torque_meas_msg(self, torque):
    # 11-bit signed signal clip to not produce errors on test
    torque_signal = dbc.name_to_msg["RX_STEER_RELATED"].sigs["B_RX_EPS_TORQUE"]
    min_torque, max_torque = get_signal_range(torque_signal)
    torque = max(min(torque, max_torque), min_torque)

    values = {"B_RX_EPS_TORQUE": torque}
    return self.packer.make_can_msg_safety("RX_STEER_RELATED", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    # 10-bit signed signal clip to not produce errors on test
    torque_signal = dbc.name_to_msg["STEER_CMD"].sigs["TORQUE_CMD"]
    min_torque, max_torque = get_signal_range(torque_signal)
    torque = max(min(torque, max_torque), min_torque)

    values = {"STEER_REQUEST": steer_req, "TORQUE_CMD": torque}
    return self.packer.make_can_msg_safety("STEER_CMD", 0, values)

  def _send_brake_msg(self, brake):
    values = {"BRAKE_CMD": brake}
    return self.packer.make_can_msg_safety("ACC_CMD", 0, values)

  def _send_gas_msg(self, gas):
    values = {"GAS_CMD": gas}
    return self.packer.make_can_msg_safety("ACC_CMD", 0, values)

  def test_rx_hook(self):
    # speed
    self.assertTrue(self._rx(self._speed_msg(0)))
    # invalidate checksum
    msg = self._speed_msg(0)
    msg[0].data[0] = 0xFF
    self.assertFalse(self._rx(msg))

    # cruise
    self.assertTrue(self._rx(self._pcm_status_msg(0)))
    # invalidate checksum
    msg = self._pcm_status_msg(0)
    msg[0].data[0] = 0xFF
    self.assertFalse(self._rx(msg))


if __name__ == "__main__":
  unittest.main()
