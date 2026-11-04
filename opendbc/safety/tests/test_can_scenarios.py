#!/usr/bin/env python3
import unittest

from opendbc.safety.tests import test_toyota


class TestToyotaCanScenarios(unittest.TestCase):
  def setUp(self):
    # Reuse Toyota's CAN encoders without inheriting its platform test suite.
    self.car = test_toyota.TestToyotaSafetyTorque()
    self.car.setUp()
    self.safety = self.car.safety
    self.safety.set_timer(0)

  def _receive_required_streams(self, engaged, include_torque=True):
    # Toyota LKA checks wheel speed (0xaa), EPS torque (0x260),
    # cruise/gas (0x1d2), and brake (0x226), all on bus 0.
    messages = [self.car._speed_msg(0), self.car._user_brake_msg(False), self.car._pcm_status_msg(engaged)]
    if include_torque:
      messages.append(self.car._torque_meas_msg(0))
    for msg in messages:
      self.assertTrue(self.car._rx(msg))

  def test_required_stream_timeout_blocks_steering(self):
    self._receive_required_streams(False)
    self._receive_required_streams(True)
    self.safety.safety_tick()
    self.assertTrue(self.safety.safety_config_valid())
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.car._tx(self.car._torque_cmd_msg(1)))

    # Keep every stream except EPS torque fresh. The one-second timeout
    # boundary is still healthy; crossing it must disable actuation.
    self.safety.set_timer(1_000_000)
    self._receive_required_streams(True, include_torque=False)
    self.safety.safety_tick()
    self.assertTrue(self.safety.safety_config_valid())
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.car._tx(self.car._torque_cmd_msg(1)))

    self.safety.set_timer(1_000_001)
    self._receive_required_streams(True, include_torque=False)
    self.safety.safety_tick()
    self.assertFalse(self.safety.safety_config_valid())
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.car._tx(self.car._torque_cmd_msg(1)))

  def test_corrupt_cruise_cannot_engage(self):
    self._receive_required_streams(False)
    self.safety.safety_tick()
    self.assertTrue(self.safety.safety_config_valid())
    self.assertFalse(self.safety.get_controls_allowed())

    corrupt = self.car._pcm_status_msg(True)
    corrupt[0].data[7] ^= 1  # Toyota checksum byte
    self.assertFalse(self.car._rx(corrupt))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.car._tx(self.car._torque_cmd_msg(1)))

    self.assertTrue(self.car._rx(self.car._pcm_status_msg(True)))
    self.safety.safety_tick()
    self.assertTrue(self.safety.safety_config_valid())
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.car._tx(self.car._torque_cmd_msg(1)))


if __name__ == '__main__':
  unittest.main()
