#!/usr/bin/env python3
import unittest
from opendbc.safety.tests.coverage import coverage

import opendbc.safety.tests.common as common
from opendbc.car.structs import CarParams


def _test_buses(case):
  """Bus 3 is an exact host-only logical-bus supplement, never a wire echo."""
  if case.backend is None:
    return range(4)
  from opendbc.safety.tests.simulated import SimulatedBackend
  return range(4) if isinstance(case.backend, SimulatedBackend) else range(3)


class TestDefaultRxHookBase(common.SafetyTest):
  FWD_BUS_LOOKUP = {}
  HOST_ONLY_SUPPLEMENTS = ('RX/TX logical bus 3: no fourth physical bus on the stock bench',)

  @coverage('Negative TX scan on physical buses 0..2',
            host=('Original logical-bus-3 TX domain',))
  def test_spam_can_buses(self):
    tx_msgs = {(addr, bus) for addr, bus in self.TX_MSGS}
    for bus in _test_buses(self):
      for addr in self.SCANNED_ADDRS:
        if (addr, bus) not in tx_msgs:
          self.assertFalse(self._tx(common.make_msg(bus, addr, 8)), f"allowed TX {addr=} {bus=}")

  @coverage('RX acceptance scan on physical buses 0..2',
            host=('Original logical-bus-3 RX domain',))
  def test_rx_hook(self):
    # default rx hook allows all msgs
    for bus in _test_buses(self):
      for addr in self.SCANNED_ADDRS:
        self.assertTrue(self._rx(common.make_msg(bus, addr, 8)), f"failed RX {addr=}")


class TestNoOutput(TestDefaultRxHookBase):
  SAFETY_MODEL = CarParams.SafetyModel.noOutput

  TX_MSGS = []


class TestSilent(TestNoOutput):
  """Test actual silent mode: TX/control needs no CAN ACK; RX/FWD needs an independent ACK participant."""
  SAFETY_MODEL = CarParams.SafetyModel.silent

  def _reset_backend(self):
    # The common profile declares no inputs for silent; its warmup checks the
    # normal stock health path without sending a CAN frame or changing mode.
    super()._reset_backend()
    health = self.backend.health()
    self.assertEqual(health['safety_mode'], int(self.SAFETY_MODEL))
    self.assertEqual(health['safety_param'], self.SAFETY_PARAM)
    from opendbc.safety.tests.simulated import SimulatedBackend
    if self._testMethodName in ('test_rx_hook', 'test_fwd_hook') and not isinstance(self.backend, SimulatedBackend):
      reason = "Silent RX/forwarding needs a verified independent ACK participant and bounded retransmission/RX evidence; "
      reason += "the two-participant backend cannot attest this bench. noOutput does not prove silent-mode behavior"
      raise AssertionError(reason)


class TestAllOutput(TestDefaultRxHookBase):
  SAFETY_MODEL = CarParams.SafetyModel.allOutput

  # Allow all messages
  TX_MSGS = [[addr, bus] for addr in common.SafetyTest.SCANNED_ADDRS
             for bus in range(4)]

  @coverage('TX allowlist scan on physical buses 0..2',
            host=('Original logical-bus-3 TX domain',))
  def test_spam_can_buses(self):
    # asserts tx allowed for all scanned addrs
    for bus in _test_buses(self):
      for addr in self.SCANNED_ADDRS:
        should_tx = [addr, bus] in self.TX_MSGS
        self.assertEqual(should_tx, self._tx(common.make_msg(bus, addr, 8)), f"allowed TX {addr=} {bus=}")

  def test_default_controls_not_allowed(self):
    # controls always allowed
    self.assertTrue(self._controls_allowed())

  def test_tx_hook_on_wrong_safety_mode(self):
    # No point, since we allow all messages
    pass


class TestAllOutputPassthrough(TestAllOutput):
  SAFETY_PARAM = 1

  FWD_BLACKLISTED_ADDRS = {}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}


class TestSafetyFramework(common.SafetyTestBase):
  SAFETY_MODEL = CarParams.SafetyModel.noOutput

  HOST_ONLY_SUPPLEMENTS = ('Exact set_safety_hooks(0xFFFF, 0) == -1 C return value',)

  @coverage('Public mode fallback and CAN inhibition',
            host=('Exact unsupported-mode C return code -1',))
  def test_unsupported_safety_mode(self):
    packer = common.CANPackerSafety("comma_body")
    if self.backend is None:
      self.safety.set_safety_hooks(CarParams.SafetyModel.body, 0)
    else:
      self.backend.reset(CarParams.SafetyModel.body, 0)
    self.assertTrue(self._rx(packer.make_can_msg_safety("MOTORS_DATA", 0, {"SPEED_L": 0, "SPEED_R": 0})))
    self.assertTrue(self._controls_allowed())
    # A valid Body command proves the transition's actuator consequence. The
    # original arbitrary-address rejection below is retained as well.
    command = packer.make_can_msg_safety("TORQUE_CMD", 0, {"TORQUE_L": 0, "TORQUE_R": 0})
    self.assertTrue(self._tx(command, healthy=False))
    if self.backend is None:
      # Exact C return value remains a host supplement, not a public Panda API result.
      self.assertEqual(self.safety.set_safety_hooks(0xFFFF, 0), -1)
    else:
      select_mode = getattr(self.backend, 'set_safety_mode', None)
      reason = "Unsupported-mode public API probe requires backend.set_safety_mode; reset() assumes a supported mode. "
      reason += "The exact C return-code check remains a host supplement"
      self.assertTrue(callable(select_mode), reason)
      select_mode(0xFFFF, 0)
      health = self.backend.health()
      self.assertIn('safety_mode', health)
      self.assertIn('safety_param', health)
      self.assertNotEqual(health['safety_mode'], 0xFFFF, "Stock firmware must fall back to a supported fail-closed mode")
    self.assertFalse(self._controls_allowed())
    self.assertFalse(self._tx(command, healthy=False))
    self.assertFalse(self._tx(common.make_msg(0, 0x123), healthy=False))


if __name__ == "__main__":
  unittest.main()
