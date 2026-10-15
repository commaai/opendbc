#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.car.gwm.gwmcan import COUNTER_CYCLE, gwm_checksum
from opendbc.car.gwm.values import GwmSafetyFlags
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common

# (CRC byte, xor_out) of the blocks validated by safety
BLOCK_CHECKSUMS = {
  0x60: ((8, 0x95),),
  0x120: ((0, 0xEE),),
  0x13b: ((0, 0x7F), (40, 0x1A)),
  0x147: ((8, 0x61),),
  0x2ab: ((16, 0x40),),
}


def checksum(msg):
  addr, dat, bus = msg
  ret = bytearray(dat)
  for crc_byte, xor_out in BLOCK_CHECKSUMS[addr]:
    ret[crc_byte] = gwm_checksum(ret[crc_byte + 1:crc_byte + 8], xor_out)
  return addr, ret, bus


class TestGwmSafetyBase(common.CarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  DBC = "gwm_haval_h6_mk3"
  SAFETY_MODEL = CarParams.SafetyModel.gwm

  TX_MSGS = [[0x12b, 0], [0x23d, 0], [0x147, 2], [0xa1, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12b, 0x23d), 2: (0x147,)}
  FWD_BLACKLISTED_ADDRS = {0: [0x147], 2: [0x12b, 0x23d]}

  MAX_RATE_UP = 4
  MAX_RATE_DOWN = 6
  MAX_TORQUE_LOOKUP = [0], [253]
  MAX_RT_DELTA = 100
  MAX_TORQUE_ERROR = 80

  def setUp(self):
    super().setUp()
    self.counters = dict.fromkeys(BLOCK_CHECKSUMS, 0)

  def _counter(self, addr):
    counter = self.counters[addr]
    self.counters[addr] = (counter + 1) % COUNTER_CYCLE
    return counter

  def _user_gas_msg(self, gas):
    values = {"GAS_POSITION": gas, "COUNTER2": self._counter(0x60)}
    return self.packer.make_can_msg_safety("CAR_OVERALL_SIGNALS2", 0, values, fix_checksum=checksum)

  def _user_brake_msg(self, brake):
    values = {"PEDAL_BRAKE_PRESSED": brake, "COUNTER": self._counter(0x120)}
    return self.packer.make_can_msg_safety("BRAKE2", 0, values, fix_checksum=checksum)

  def _speed_msg(self, speed):
    values = {f"{pos}_WHEEL_SPEED": speed * 3.6 for pos in ("FRONT_LEFT", "FRONT_RIGHT", "REAR_LEFT", "REAR_RIGHT")}
    values["COUNTER"] = self._counter(0x13b)
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, values, fix_checksum=checksum)

  def _torque_meas_msg(self, torque):
    values = {"B_RX_EPS_TORQUE": torque, "B_COUNTER": self._counter(0x147)}
    return self.packer.make_can_msg_safety("RX_STEER_RELATED", 0, values, fix_checksum=checksum)

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_STATE_2": 3 if enable else 2, "COUNTER2": self._counter(0x2ab)}
    return self.packer.make_can_msg_safety("ACC", 2, values, fix_checksum=checksum)

  def _torque_cmd_msg(self, torque, steer_req=1, invert_direction=None, reflected=None):
    # TORQUE_CMD is 10 bits, clip to not overflow it
    torque = np.clip(torque, -511, 512)
    values = {
      "TORQUE_CMD": torque,
      "STEER_REQUEST": steer_req,
      "INVERT_DIRECTION": (steer_req and torque > 0) if invert_direction is None else invert_direction,
      "TORQUE_REFLECTED": -torque if reflected is None else reflected,
    }
    return self.packer.make_can_msg_safety("STEER_CMD", 0, values)

  def _button_msg(self, **buttons):
    values = {f"AP_{name.upper()}_COMMAND": pressed for name, pressed in buttons.items()}
    return self.packer.make_can_msg_safety("STEER_AND_AP_STALK", 2, values)

  def _rx_check_msgs(self):
    return {0x60: self._user_gas_msg, 0x120: self._user_brake_msg, 0x13b: self._speed_msg,
            0x147: self._torque_meas_msg, 0x2ab: lambda _: self._pcm_status_msg(True)}

  def test_rx_hook(self):
    for addr, make_msg in self._rx_check_msgs().items():
      for crc_byte, _ in BLOCK_CHECKSUMS[addr]:
        self._reset_safety_hooks()
        self.safety.set_controls_allowed(True)
        for _ in range(10):
          self.assertTrue(self._rx(make_msg(0)))
        self.assertTrue(self.safety.get_controls_allowed())

        # bad checksum
        msg = make_msg(0)
        msg[0].data[crc_byte] ^= 0xff
        self.assertFalse(self._rx(msg))
        self.assertFalse(self.safety.get_controls_allowed())

      # repeated bad counters
      self._reset_safety_hooks()
      self.safety.set_controls_allowed(True)
      for _ in range(10):
        self.counters[addr] = 0
        valid = self._rx(make_msg(0))
      self.assertFalse(valid)
      self.assertFalse(self.safety.get_controls_allowed())

  def test_rx_counter_wraps_at_14(self):
    for addr, make_msg in self._rx_check_msgs().items():
      self._reset_safety_hooks()
      for _ in range(3 * COUNTER_CYCLE):
        self.assertTrue(self._rx(make_msg(0)))

      # 15 is never a valid counter
      for counter in [14, 15] * 3:
        self.counters[addr] = counter
        valid = self._rx(make_msg(0))
      self.assertFalse(valid)

  def test_buttons(self):
    other_buttons = ("enable", "reduce_distance", "increase_distance", "decrease_speed", "increase_speed")
    for engaged in (True, False):
      self.safety.set_cruise_engaged_prev(engaged)
      self.assertTrue(self._tx(self._button_msg()))
      self.assertEqual(engaged, self._tx(self._button_msg(cancel=True)))
      for button in other_buttons:
        self.assertFalse(self._tx(self._button_msg(**{button: True})))
        self.assertFalse(self._tx(self._button_msg(cancel=True, **{button: True})))

  def test_steer_direction_bits(self):
    self.safety.set_controls_allowed(True)
    for torque in (-3, 0, 1, 3):
      self._set_prev_torque(torque)
      self.assertTrue(self._tx(self._torque_cmd_msg(torque)))
      self.assertFalse(self._tx(self._torque_cmd_msg(torque, invert_direction=torque <= 0)))
      self.assertFalse(self._tx(self._torque_cmd_msg(torque, reflected=1 - torque)))


class TestGwmStockSafety(TestGwmSafetyBase):
  pass


class TestGwmLongSafety(TestGwmSafetyBase, common.LongitudinalGasBrakeSafetyTest):
  SAFETY_PARAM = GwmSafetyFlags.LONG_CONTROL

  TX_MSGS = TestGwmSafetyBase.TX_MSGS + [[0x143, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12b, 0x23d, 0x143), 2: (0x147,)}
  FWD_BLACKLISTED_ADDRS = {0: [0x147], 2: [0x12b, 0x23d, 0x143]}

  MIN_GAS = -192
  MAX_GAS = 4577
  MIN_POSSIBLE_GAS = -192  # signal min
  MAX_POSSIBLE_GAS = 4808  # reasonably excessive limits, not signal max
  INACTIVE_GAS = 0

  MAX_BRAKE = 107
  MAX_POSSIBLE_BRAKE = 181

  def _send_brake_msg(self, brake):
    values = {"BRAKE_CMD": -brake, "GAS_CMD": 0}
    return self.packer.make_can_msg_safety("ACC_CMD", 0, values)

  def _send_gas_msg(self, gas):
    values = {"GAS_CMD": gas, "BRAKE_CMD": 0}
    return self.packer.make_can_msg_safety("ACC_CMD", 0, values)

  def test_negative_brake(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for brake in (-1, -74):
        self.assertFalse(self._tx(self._send_brake_msg(brake)))


if __name__ == "__main__":
  unittest.main()
