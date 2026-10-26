"""Identical CAN/health scenarios for host C hooks and unmodified stock Panda.

Exact microsecond boundaries, ignition GPIOs and malformed USB packet metadata
remain in the supplemental host suite: they cannot be driven through real CAN.
"""
import unittest

from opendbc.safety.tests.simulated import SimulatedBackend
from opendbc.safety.tests.vehicle import Button, get_vehicle_profile

backend_factory = SimulatedBackend


class ObservableScenarios:
  profile_name = None

  def setUp(self):
    self.vehicle = get_vehicle_profile(self.profile_name)
    self.backend = backend_factory()
    self.addCleanup(self.backend.close)
    self.backend.reset(self.vehicle.safety_model, self.vehicle.safety_param)
    self.backend.streams(self.vehicle.streams())
    # Stock firmware updates RX health at 1 Hz, independently of mode changes.
    self.backend.wait(1.1)
    self.assertFalse(self.backend.health()['safety_rx_checks_invalid'])

  def update(self, **state):
    self.backend.streams(self.vehicle.update(**state))
    self.backend.wait(.15)

  def engage(self):
    for changes in self.vehicle.engage_sequence():
      self.update(**changes)
    self.assertTrue(self.backend.health()['controls_allowed'])

  def command(self, value, accepted=True, **guards):
    self.assertEqual(self.backend.tx(self.vehicle.torque(value), **guards), accepted)

  def test_sustained_health_and_controls(self):
    self.engage()
    for _ in range(3):
      self.backend.wait(1.05)
      self.command(0)

  def test_engagement_and_disengagement(self):
    self.command(1, False, controls=False)
    self.engage()
    self.command(1)
    self.update(cruise=False)
    self.command(1, False, controls=False)

  def test_brake_edge_and_moving_brake(self):
    self.engage()
    self.update(brake=True)
    self.command(1, False, controls=False)
    self.engage()  # A held brake at standstill permits re-engagement.
    self.command(0)
    self.update(speed=100)
    self.command(1, False, controls=False)

  def test_request_bit(self):
    self.engage()
    self.assertFalse(self.backend.tx(self.vehicle.torque(1, request=False)))

  def check_windup(self, sign):
    self.engage()
    self.command(sign * self.vehicle.max_rate_up)
    self.command(sign * (2 * self.vehicle.max_rate_up + 1), False)

  def test_windup_boundary(self):
    self.check_windup(1)

  def test_negative_windup_boundary(self):
    self.check_windup(-1)

  def test_zero_then_opposite_sign_rate_boundary(self):
    self.engage()
    value = 0
    for _ in range(5):
      value += self.vehicle.max_rate_up
      self.command(value)
    self.command(0)
    self.command(-self.vehicle.max_rate_up - 1, False)

  def check_absolute_limit(self, sign):
    self.engage()
    value = 0
    while value < self.vehicle.max_torque:
      value = min(value + self.vehicle.max_rate_up, self.vehicle.max_torque)
      if self.profile_name == 'toyota':
        self.update(motor_torque=sign * value)
      self.backend.wait(.012)
      self.command(sign * value)
    # Establish a fresh realtime reference at the accepted absolute boundary,
    # so the next rejection cannot come from an incidental RT-window edge.
    self.backend.wait(.35)
    self.command(sign * self.vehicle.max_torque)
    self.command(sign * (self.vehicle.max_torque + 1), False)

  def test_absolute_limit(self):
    self.check_absolute_limit(1)

  def test_negative_absolute_limit(self):
    self.check_absolute_limit(-1)

  def test_opposing_torque_requires_winddown(self):
    self.engage()
    value = 0
    target = 450 if self.profile_name == 'toyota' else 60
    while value < target:
      value += self.vehicle.max_rate_up
      if self.profile_name == 'toyota':
        self.update(motor_torque=value)
      self.command(value)
    if self.profile_name == 'toyota':
      self.update(motor_torque=0)
    else:
      self.update(driver_torque=-500)
    value -= self.vehicle.max_rate_down
    self.command(value)
    self.command(value - self.vehicle.max_rate_down + 1, False)

  def test_realtime_window_recovers_after_wait(self):
    self.engage()
    # Staying below the realtime delta is valid in either window. Waiting well
    # beyond 250 ms avoids relying on SSH latency or MCU tick phase.
    value = 0
    for _ in range(5):
      value += self.vehicle.max_rate_up
      self.command(value)
    self.backend.wait(.35)
    self.command(value)
    self.command(value + self.vehicle.max_rate_up)

  def test_required_stream_timeout(self):
    self.engage()
    self.command(1)
    self.backend.drop_stream('torque' if self.profile_name == 'toyota' else 'driver_torque')
    self.backend.wait(2.2)
    self.assertTrue(self.backend.health()['safety_rx_checks_invalid'])
    self.assertFalse(self.backend.health()['faults'])
    self.command(1, False, controls=False, healthy=False)

  def test_corrupt_checksum_and_recovery(self):
    self.engage()
    stream = 'torque' if self.profile_name == 'toyota' else 'gas'
    self.assertFalse(self.backend.rx(self.vehicle.corrupt(stream)))
    self.assertFalse(self.backend.health()['faults'])
    # The corrupt RX deliberately invalidates safety state; the remaining
    # periodic traffic continues and proves recovery without a missing stream.
    self.command(1, False, controls=False, healthy=False)
    self.backend.wait(1.1)
    self.assertFalse(self.backend.health()['safety_rx_checks_invalid'])
    self.engage()
    self.command(1)

  def test_forwarding_and_replacement_blocks(self):
    self.engage()
    payload = '0000000000000000'
    for bus, target in ((0, 2), (2, 0), (1, None)):
      self.assertTrue(self.backend.forward([0x123, payload, bus], target, controls=True))
    replaced_address = 0x2e4 if self.profile_name == 'toyota' else 0x340
    self.assertTrue(self.backend.forward([replaced_address, payload, 2], None, controls=True))

  def test_stock_ecu_disables_transmit_and_forwarding(self):
    self.engage()
    self.backend.wait(2.1)  # Let the physical relay transition grace period expire.
    self.command(0)
    self.assertTrue(self.backend.rx(self.vehicle.torque(0)))
    self.backend.wait(.2)  # Firmware publishes relay faults on its periodic tick.
    self.assertNotEqual(self.backend.health()['faults'], 0)
    self.command(0, False, healthy=False)
    self.assertTrue(self.backend.forward([0x123, '0000000000000000', 0], None, controls=True, healthy=False))

  def test_wrong_bus_and_address(self):
    self.engage()
    frame = self.vehicle.torque(0)
    self.assertFalse(self.backend.tx([frame[0], frame[1], 1]))
    self.assertFalse(self.backend.tx([0x123, frame[1], 0]))


class TestToyotaObservable(ObservableScenarios, unittest.TestCase):
  profile_name = 'toyota'

  def test_gas_inhibits_longitudinal(self):
    self.engage()
    self.assertTrue(self.backend.tx(self.vehicle.accel(.1)))
    self.update(gas=True)
    self.assertFalse(self.backend.tx(self.vehicle.accel(.1)))
    self.assertTrue(self.backend.tx(self.vehicle.accel(0)))
    self.update(gas=False)
    self.assertTrue(self.backend.tx(self.vehicle.accel(.1)))

  def test_acceleration_boundaries(self):
    values = (self.vehicle.min_accel - .001, self.vehicle.min_accel,
              self.vehicle.max_accel, self.vehicle.max_accel + .001, self.vehicle.inactive_accel)
    for engaged in (False, True):
      if engaged:
        self.engage()
      for value in values:
        with self.subTest(engaged=engaged, acceleration=value):
          allowed = value == self.vehicle.inactive_accel or (engaged and self.vehicle.min_accel <= value <= self.vehicle.max_accel)
          self.assertEqual(self.backend.tx(self.vehicle.accel(value), controls=engaged), allowed)

  def test_diagnostics(self):
    self.engage()
    for payload, allowed in ((b'\x6d\x02\x3e\x00\x00\x00\x00\x00', False),
                             (b'\x0f\x03\xaa\xaa\x00\x00\x00\x00', False),
                             (b'\x0f\x02\x3e\x00\x00\x00\x00\x00', True)):
      self.assertEqual(self.backend.tx(self.vehicle.diagnostic(payload)), allowed)

  def test_aeb_actuation_blocked(self):
    self.engage()
    self.assertTrue(self.backend.tx(self.vehicle.aeb(bytes(7))))
    for byte in range(6):
      payload = bytearray(7)
      payload[byte] = 1
      self.assertFalse(self.backend.tx(self.vehicle.aeb(payload)))

  def test_lta_only_inactive(self):
    self.engage()
    self.assertTrue(self.backend.tx(self.vehicle.lta(0, 0, 0, 0)))
    for req, req2, angle, winddown in ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, -1, 0),
                                       (0, 0, 1, 0), (0, 0, 0, 50)):
      self.assertFalse(self.backend.tx(self.vehicle.lta(req, req2, angle, winddown)))


class TestHyundaiObservable(ObservableScenarios, unittest.TestCase):
  profile_name = 'hyundai'

  def test_button_transmit_policy(self):
    self.assertFalse(self.backend.tx(self.vehicle.button(Button.RESUME), controls=False))
    self.assertFalse(self.backend.tx(self.vehicle.button(Button.SET), controls=False))
    self.assertFalse(self.backend.tx(self.vehicle.button(Button.CANCEL), controls=False))
    self.engage()
    self.assertTrue(self.backend.tx(self.vehicle.button(Button.RESUME)))
    self.assertFalse(self.backend.tx(self.vehicle.button(Button.SET)))
    self.assertTrue(self.backend.tx(self.vehicle.button(Button.CANCEL)))
    self.update(cruise=False)
    self.assertFalse(self.backend.tx(self.vehicle.button(Button.CANCEL), controls=False))

  def test_cruise_requires_recent_enable_button(self):
    self.update(cruise=True)
    self.command(1, False, controls=False)
    self.update(cruise=False)
    self.engage()
    self.command(1)


if __name__ == '__main__':
  unittest.main()
