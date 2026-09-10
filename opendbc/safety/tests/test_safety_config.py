import itertools
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py


class TestSafetyConfig(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()

  def tearDown(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.noOutput, 0)

  def configure(self, frequency=100, ignore_checksum=True, ignore_counter=True, ignore_quality_flag=True, max_counter=0, callbacks=0):
    self.safety.safety_test_configure_rx(frequency, ignore_checksum, ignore_counter, ignore_quality_flag, max_counter, callbacks)
    self.safety.set_timer(0)

  def message(self, address=0x123, bus=0, length=8, checksum=1, payload=1, counter=1, quality=1, steering_disengage=False):
    return libsafety_py.make_CANPacket(address, bus, (bytes((checksum, payload, counter, quality, steering_disengage)) + bytes(59))[:length])

  def test_rx_whitelist(self):
    # Both before and after choosing an RX descriptor, a wrong bus, address or
    # length must never reach the car hook or update its liveness timestamp.
    for seen, (address, bus, length) in itertools.product((False, True), ((0x124, 0, 8), (0x123, 1, 8), (0x123, 0, 7))):
      with self.subTest(seen=seen, address=address, bus=bus, length=length):
        self.configure()
        if seen:
          self.assertTrue(self.safety.safety_rx_hook(self.message()))
        self.safety.set_timer(1_000_001)
        self.assertTrue(self.safety.safety_rx_hook(self.message(address, bus, length)))
        self.assertEqual(self.safety.safety_test_get_rx_count(), int(seen))
        self.safety.set_controls_allowed(True)
        self.safety.safety_tick_current_safety_config()
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertTrue(self.safety.get_safety_rx_checks_invalid())

  def test_missing_checksum_callbacks(self):
    for callbacks, ignored in itertools.product(range(4), (False, True)):
      with self.subTest(callbacks=callbacks, ignored=ignored):
        self.configure(ignore_checksum=ignored, callbacks=callbacks)
        self.safety.set_controls_allowed(True)
        expected = ignored or callbacks == 3
        self.assertEqual(self.safety.safety_rx_hook(self.message()), expected)
        self.assertEqual(self.safety.get_controls_allowed(), expected)
        self.assertEqual(self.safety.safety_test_get_rx_count(), int(expected))

  def test_missing_counter_configuration(self):
    for callbacks, max_counter, ignored in itertools.product((0, 4), (0, 3), (False, True)):
      with self.subTest(callbacks=callbacks, max_counter=max_counter, ignored=ignored):
        self.configure(ignore_counter=ignored, max_counter=max_counter, callbacks=callbacks)
        self.safety.set_controls_allowed(True)
        expected = ignored or bool(callbacks and max_counter)
        self.assertEqual(self.safety.safety_rx_hook(self.message()), expected)
        self.assertEqual(self.safety.get_controls_allowed(), expected)

  def test_missing_quality_callback(self):
    for callbacks, ignored in itertools.product((0, 8), (False, True)):
      with self.subTest(callbacks=callbacks, ignored=ignored):
        self.configure(ignore_quality_flag=ignored, callbacks=callbacks)
        self.safety.set_controls_allowed(True)
        expected = ignored or bool(callbacks)
        self.assertEqual(self.safety.safety_rx_hook(self.message()), expected)
        self.assertEqual(self.safety.get_controls_allowed(), expected)

  def test_tick_lag_boundary_and_minimum_frequency(self):
    for frequency in (5, 9, 10, 100):
      threshold = max(10 * (1_000_000 // frequency), 1_000_000)
      for elapsed in (threshold - 1, threshold, threshold + 1):
        with self.subTest(frequency=frequency, elapsed=elapsed):
          self.configure(frequency=frequency)
          self.assertTrue(self.safety.safety_rx_hook(self.message()))
          self.safety.set_controls_allowed(True)
          self.safety.set_timer(elapsed)
          self.safety.safety_tick_current_safety_config()
          invalid = elapsed > threshold or frequency < 10
          self.assertEqual(self.safety.get_safety_rx_checks_invalid(), invalid)
          self.assertEqual(self.safety.get_controls_allowed(), not invalid)

  def test_tick_checks_message_validity(self):
    self.configure(ignore_checksum=False, callbacks=3)
    self.assertFalse(self.safety.safety_rx_hook(self.message(checksum=0)))
    self.safety.set_controls_allowed(True)
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.get_safety_rx_checks_invalid())
    self.assertFalse(self.safety.get_controls_allowed())

  def test_tick_without_config(self):
    self.safety.set_controls_allowed(True)
    self.safety.safety_test_tick_null()
    self.assertFalse(self.safety.get_safety_rx_checks_invalid())
    self.assertTrue(self.safety.get_controls_allowed())

  def test_unknown_safety_mode(self):
    self.safety.set_controls_allowed(True)
    self.assertEqual(self.safety.set_safety_hooks(0xFFFF, 0), -1)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_steering_override_edges(self):
    self.configure()
    for override, should_disengage in ((True, True), (True, False), (False, False), (True, True)):
      with self.subTest(override=override, should_disengage=should_disengage):
        self.safety.set_controls_allowed(True)
        self.assertTrue(self.safety.safety_rx_hook(self.message(steering_disengage=override)))
        self.assertEqual(self.safety.get_controls_allowed(), not should_disengage)

  def test_ignition_rejects_wrong_bus_or_length(self):
    for address, length, initial in itertools.product((0x1F1, 0x152, 0x221, 0x9E, 0x3C0), (0, 1, 7, 12), (False, True)):
      # Ignition parsing accepts bus 0 and exactly 8 bytes (4 for MEB).
      for bus, actual_length in ((0, length), (1, 4 if address == 0x3C0 else 8)):
        with self.subTest(address=address, bus=bus, length=actual_length, initial=initial):
          self.safety.set_ignition_can(initial)
          for counter in range(3):
            data = bytearray(12)
            if address == 0x1F1:
              data[0] = 0 if initial else 2
            elif address == 0x152:
              data[1], data[7] = counter, 0 if initial else 0x10
            elif address == 0x221:
              data[6], data[0] = counter << 4, 0 if initial else 0x60
            elif address == 0x9E:
              data[0] = 0 if initial else 0xC0
            else:
              data[1], data[2] = counter, 0 if initial else 2
            self.safety.ignition_can_hook(libsafety_py.make_CANPacket(address, bus, data[:actual_length]))
          self.assertEqual(self.safety.get_ignition_can(), initial)

  def test_interpolation_small_interval(self):
    # Preserve the small-denominator guard, including inputs below its floor.
    for midpoint in (0.00005, 0.0001, 0.1):
      with self.subTest(midpoint=midpoint):
        self.assertEqual(self.safety.safety_test_interpolate(-1, midpoint), 0)
        self.assertEqual(self.safety.safety_test_interpolate(2, midpoint), 2)
        expected = (midpoint / 2) / max(midpoint, 0.0001)
        self.assertAlmostEqual(self.safety.safety_test_interpolate(midpoint / 2, midpoint), expected, places=6)

  def test_invalid_dynamic_torque_lookup_fails_closed(self):
    for torque in (-1000, 0, 1000):
      with self.subTest(torque=torque):
        self.safety.set_safety_hooks(CarParams.SafetyModel.noOutput, 0)
        self.safety.set_controls_allowed(True)
        violation = self.safety.safety_test_dynamic_torque_limit(torque)
        self.assertEqual(violation, torque < 0)

  def test_signed_conversion_and_shift_floor(self):
    for bits in (1, 2, 8, 16, 30):
      sign = 1 << (bits - 1)
      for value, expected in ((0, 0), (sign - 1, sign - 1), (sign, -sign), ((1 << bits) - 1, -1)):
        with self.subTest(bits=bits, value=value):
          self.assertEqual(self.safety.to_signed(value, bits), expected)
    # Preserve the defensive shift floor for nonpositive widths without
    # evaluating a negative C shift (the test library also enables UBSan).
    for bits in (-1, 0):
      for value, expected in ((-1, -1), (0, 0), (1, 0), (2, 1)):
        with self.subTest(bits=bits, value=value):
          self.assertEqual(self.safety.to_signed(value, bits), expected)


if __name__ == "__main__":
  unittest.main()
