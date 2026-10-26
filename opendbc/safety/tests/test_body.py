#!/usr/bin/env python3
import unittest
from opendbc.safety.tests.coverage import coverage

from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.test_defaults import _test_buses


class TestBody(common.SafetyTest):
  DBC = "comma_body"
  SAFETY_MODEL = CarParams.SafetyModel.body

  TX_MSGS = [[0x250, 0], [0x251, 0],
             [0x1, 0], [0x1, 1], [0x1, 2], [0x1, 3]]
  FWD_BUS_LOOKUP = {}

  HOST_ONLY_SUPPLEMENTS = ('TX logical bus 3: no fourth physical bus on the stock bench',)

  def required_streams(self):
    # Every valid MOTORS_DATA frame engages Body, even at zero speed. A neutral
    # background profile would invalidate the initial/disengaged assertions.
    # These tests own every input, including counter faults, without a healer.
    self._fixture_streams_declared_empty = True
    self._fixture_stream_bridge = None
    return []

  def _reset_backend(self):
    self.backend.reset(self.SAFETY_MODEL, self.SAFETY_PARAM)
    self._update_required_streams(self.required_streams())
    # Missing RX is deliberate here. Do not perform common's healthy warmup,
    # which runs the watchdog before this test's first intentional input.

  def _tx(self, msg, *, healthy=False):
    # No required stream is running before engagement or during fault probes.
    return super()._tx(msg, healthy=healthy)

  @coverage('Negative TX scan on physical buses 0..2',
            host=('Original logical-bus-3 TX domain',))
  def test_spam_can_buses(self):
    tx_msgs = {(addr, bus) for addr, bus in self.TX_MSGS}
    for bus in _test_buses(self):
      for addr in self.SCANNED_ADDRS:
        if (addr, bus) not in tx_msgs:
          self.assertFalse(self._tx(common.make_msg(bus, addr, 8)), f"allowed TX {addr=} {bus=}")

  def _engage(self):
    self.assertTrue(self._rx(self._motors_data_msg(0, 0)))

  def _disengage(self):
    self._reset_safety_hooks()

  def _motors_data_msg(self, speed_l, speed_r):
    values = {"SPEED_L": speed_l, "SPEED_R": speed_r}
    return self.packer.make_can_msg_safety("MOTORS_DATA", 0, values)

  def _torque_cmd_msg(self, torque_l, torque_r):
    values = {"TORQUE_L": torque_l, "TORQUE_R": torque_r}
    return self.packer.make_can_msg_safety("TORQUE_CMD", 0, values)

  def test_rx_hook(self):
    self.assertFalse(self._controls_allowed())

    # controls allowed when we get MOTORS_DATA message
    self.assertTrue(self._rx(self._torque_cmd_msg(0, 0)))
    self.assertFalse(self._controls_allowed())

    self.assertTrue(self._rx(self._motors_data_msg(0, 0)))
    self.assertTrue(self._controls_allowed())

    self._reset_safety_hooks()
    for _ in range(16):
      self.assertTrue(self._rx(self._motors_data_msg(0, 0), raw=True))

    msg = self._motors_data_msg(0, 0)
    msg[0].data[7] ^= 0xff
    self.assertFalse(self._rx(msg, raw=True))

    self._reset_safety_hooks()
    for _ in range(16):
      self.assertTrue(self._rx(self._motors_data_msg(0, 0), raw=True))

    msg = self._motors_data_msg(0, 0)
    for _ in range(common.MAX_WRONG_COUNTERS + 1):
      valid = self._rx(msg, raw=True)
    self.assertFalse(valid)

  def test_tx_hook(self):
    self.assertFalse(self._tx(self._torque_cmd_msg(0, 0)))
    self._set_controls_allowed(True)
    self.assertTrue(self._tx(self._torque_cmd_msg(0, 0)))

  def test_can_flasher(self):
    # CAN flasher always allowed
    self._set_controls_allowed(False)
    self.assertTrue(self._tx(common.make_msg(0, 0x1, 8)))

    # 0xdeadfaceU allowed for CAN flashing mode
    self.assertTrue(self._tx(common.make_msg(0, 0x250, dat=b'\xce\xfa\xad\xde\x1e\x0b\xb0\x0a')))
    self.assertFalse(self._tx(common.make_msg(0, 0x250, dat=b'\xcf\xfa\xad\xde\x1e\x0b\xb0\x0a')))  # wrong signature
    self.assertFalse(self._tx(common.make_msg(0, 0x250, dat=b'\xce\xfa\xad\xde\x1e\x0b\xb0')))  # not correct data/len
    self.assertFalse(self._tx(common.make_msg(0, 0x251, dat=b'\xce\xfa\xad\xde\x1e\x0b\xb0\x0a')))  # wrong address


if __name__ == "__main__":
  unittest.main()
