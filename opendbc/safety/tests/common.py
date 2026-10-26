import os
import abc
from functools import wraps
import math
import unittest
from opendbc.safety.tests.coverage import coverage
import importlib
import numpy as np
from collections.abc import Callable

from opendbc.can import CANPacker
from opendbc.safety import ALTERNATIVE_EXPERIENCE, DLC_TO_LEN
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.car.lateral import MAX_LATERAL_ACCEL, MAX_LATERAL_JERK
from opendbc.car.structs import CarParams

MAX_WRONG_COUNTERS = 5
MAX_SAMPLE_VALS = 6
VEHICLE_SPEED_FACTOR = 1000
RT_INTERVAL = 250000  # 250ms

# Max allowed delta between car speeds
MAX_SPEED_DELTA = 2.0  # m/s

MessageFunction = Callable[[float], libsafety_py.CANPacket]


def sign_of(a):
  return 1 if a > 0 else -1


def away_round(x):
  # non-banker's/away from zero rounding, C++ CANParser uses this style
  return math.floor(x + 0.5) if x >= 0 else math.ceil(x - 0.5)


def round_speed(v):
  return round(v * VEHICLE_SPEED_FACTOR) / VEHICLE_SPEED_FACTOR


def speeds_within_delta(speed, speed_2):
  # safety compares the speeds in integer VEHICLE_SPEED_FACTOR units
  delta = away_round(speed * VEHICLE_SPEED_FACTOR) - away_round(speed_2 * VEHICLE_SPEED_FACTOR)
  return abs(delta) <= MAX_SPEED_DELTA * VEHICLE_SPEED_FACTOR


def make_msg(bus, addr, length=8, dat=None):
  if dat is None:
    dat = b'\x00' * length
  return libsafety_py.make_CANPacket(addr, bus, dat)


class CANPackerSafety(CANPacker):
  def make_can_msg_safety(self, name_or_addr, bus, values, fix_checksum=None):
    msg = self.make_can_msg(name_or_addr, bus, values)
    if fix_checksum is not None:
      msg = fix_checksum(msg)
    addr, dat, bus = msg
    return libsafety_py.make_CANPacket(addr, bus, dat)


def add_regen_tests(cls):
  """Dynamically adds regen tests for all user brake tests."""

  # only rx/user brake tests, not brake command
  found_tests = [func for func in dir(cls) if func.startswith("test_") and "user_brake" in func]
  assert len(found_tests) >= 3, "Failed to detect known brake tests"

  for test in found_tests:
    def _make_regen_test(brake_func):
      @wraps(getattr(cls, brake_func))
      def _regen_test(self):
        # only for safety modes with a regen message
        if self._user_regen_msg(0) is None:
          raise unittest.SkipTest("Safety mode implements no _user_regen_msg")

        getter = self.safety.get_regen_braking_prev if self.backend is None else None
        getattr(self, brake_func)(self._user_regen_msg, getter)
      return _regen_test

    setattr(cls, test.replace("brake", "regen"), _make_regen_test(test))

  return cls


class PeriodicSafetyTest:
  """Continuous-traffic regressions for the two concrete smoke-tested fixtures."""

  def _periodic_backend(self):
    if self.backend is None:
      # The default host run needs periodic scheduling for these new tests too.
      self.backend = self.backend_factory('simulated')
      self.addCleanup(self.backend.close)
      self.safety = _UnavailableDeterministicHost()
      self._reset_backend()

  @coverage('Continuous RX health and accepted neutral commands over three watchdog ticks', relation='equivalent')
  def test_sustained_health_and_controls(self):
    self._periodic_backend()
    self._set_controls_allowed(True)
    for _ in range(3):
      self._wait(1.05)
      self.assertTrue(self.backend.tx(self._raw_frame(self._torque_cmd_msg(0))))

  @coverage('One missing torque stream inhibits TX while all other periodic inputs continue', relation='equivalent')
  def test_required_stream_timeout(self):
    self._periodic_backend()
    self._set_controls_allowed(True)
    self.assertTrue(self.backend.tx(self._raw_frame(self._torque_cmd_msg(1))))
    self.backend.drop_stream(self.PERIODIC_TIMEOUT_STREAM)
    self._wait(2.2)
    self.assertTrue(self.backend.health()['safety_rx_checks_invalid'])
    self.assertFalse(self.backend.health()['faults'])
    self.assertFalse(self.backend.tx(self._raw_frame(self._torque_cmd_msg(1)), controls=False, healthy=False))

  @coverage('Checksum rejection, inhibition and recovery while other periodic traffic continues', relation='equivalent')
  def test_corrupt_checksum_and_recovery(self):
    self._periodic_backend()
    self._set_controls_allowed(True)
    frame = self._raw_frame(self._checksum_fault_msg())
    data = bytearray.fromhex(frame[1])
    data[-1] ^= 0xff
    frame[1] = data.hex()
    self.assertFalse(self.backend.rx(frame))
    self.assertFalse(self.backend.health()['faults'])
    self.assertFalse(self.backend.tx(self._raw_frame(self._torque_cmd_msg(1)), controls=False, healthy=False))
    self._wait(1.1)
    self.assertFalse(self.backend.health()['safety_rx_checks_invalid'])
    self._set_controls_allowed(True)
    self.assertTrue(self.backend.tx(self._raw_frame(self._torque_cmd_msg(1))))

  @coverage('Zero command resets the rate baseline before an opposite-sign command', relation='equivalent')
  def test_zero_then_opposite_sign_rate_boundary(self):
    self._periodic_backend()
    self._set_controls_allowed(True)
    for step in range(1, 6):
      self.assertTrue(self.backend.tx(self._raw_frame(self._torque_cmd_msg(step * self.MAX_RATE_UP))))
    self.assertTrue(self.backend.tx(self._raw_frame(self._torque_cmd_msg(0))))
    self.assertFalse(self.backend.tx(self._raw_frame(self._torque_cmd_msg(-self.MAX_RATE_UP - 1))))


class _UnavailableDeterministicHost:
  def __getattr__(self, name):
    raise AssertionError(f"{name} requires deterministic host state; no stock-hardware equivalent implemented")


class SafetyTestBase(unittest.TestCase):
  safety: libsafety_py.LibSafety
  DBC: str | None = None
  packer: CANPackerSafety
  SAFETY_MODEL: int | None
  SAFETY_PARAM = 0
  IGNITION_ONLY = False

  def setUp(self):
    # Legacy is intentional: mutable packet metadata and private-state assertions
    # remain deterministic host coverage until individually migrated.
    self.backend = None
    selection = os.environ.get("SAFETY_BACKEND", "legacy")
    if self.DBC is not None:
      self.packer = CANPackerSafety(self.DBC)
    if selection == "legacy":
      self.safety = libsafety_py.libsafety
      self.safety.set_safety_hooks(self._safety_model(), self.SAFETY_PARAM)
      self.safety.init_tests()
    else:
      self.backend = self.backend_factory(selection)
      self.addCleanup(self.backend.close)
      # Observable parity never borrows private C state, even on simulation.
      self.safety = _UnavailableDeterministicHost()
      if self.IGNITION_ONLY:
        self.backend.restart()
      self._reset_backend()

  def _safety_model(self):
    # noOutput receives and ACKs CAN on stock hardware; silent cannot ACK.
    return CarParams.SafetyModel.noOutput if self.IGNITION_ONLY or self.SAFETY_MODEL is None else self.SAFETY_MODEL

  def backend_factory(self, selection):
    if selection == "simulated":
      from opendbc.safety.tests.simulated import SimulatedBackend
      return SimulatedBackend()
    if selection == "hardware":
      from opendbc.safety.tests.hardware import HardwareBackend
      return HardwareBackend(os.environ["SAFETY_DUT_COMMAND"], os.environ["SAFETY_STATION_COMMAND"])
    raise ValueError(f"Unknown SAFETY_BACKEND: {selection}")

  def required_streams(self):
    """Declare continuous valid RX cycles using test-owned fixture inputs.

    The bridge captures encoder arguments; backend scheduling owns counter phase.
    Unsupported fixture profiles fail explicitly rather than running without RX.
    """
    if self.IGNITION_ONLY:
      return []
    from opendbc.safety.tests.fixture_streams import required_streams
    return required_streams(self)

  def _reset_backend(self, alternative_experience=ALTERNATIVE_EXPERIENCE.DEFAULT):
    self.backend.reset(self._safety_model(), self.SAFETY_PARAM, alternative_experience=alternative_experience)
    self._update_required_streams(self.required_streams())
    # Stock RX watchdog health is published by its independent 1 Hz tick.
    self.backend.wait(1.1)
    self.assertFalse(self.backend.health()['safety_rx_checks_invalid'])

  def _reset_ignition(self):
    if self.backend is None:
      self.safety.init_tests()
    else:
      # Global ignition history survives safety selection: only an MCU reset
      # provides the same fresh counter state as the host test initializer.
      self.backend.restart()
      self._reset_backend()
    self.assertFalse(self._ignition_can(), "Reset must clear ignition history")

  def _ignition_rx(self, msg):
    if self.backend is None:
      self.safety.ignition_can_hook(msg)
    else:
      # Stock firmware decodes ignition on received CAN independently of safety
      # RX validity. Send the exact frame without fixture stream substitutions.
      self.backend.rx(self._raw_frame(msg))

  def _ignition_can(self):
    if self.backend is None:
      return bool(self.safety.get_ignition_can())
    health = self.backend.health()
    self.assertIn("ignition_can", health, "Backend lacks public CAN ignition health")
    return bool(health["ignition_can"])

  def _update_required_streams(self, streams):
    if streams:
      self.backend.streams(streams)

  def _drop_required_stream(self, name):
    self.backend.drop_stream(name)

  def _wait(self, seconds):
    if self.backend is None:
      self.safety.set_timer((self.safety.get_timer() + round(seconds * 1_000_000)) % 2**32)
    else:
      self.backend.wait(seconds)

  @staticmethod
  def _raw_frame(msg):
    # Never repair payload checksum/counter faults. USB-only metadata cannot be
    # represented by a wire frame, so reject it rather than silently normalizing.
    if msg[0].rejected or msg[0].returned or msg[0].checksum or msg[0].extended != (msg[0].addr >= 0x800):
      raise AssertionError("CAN packet metadata fault requires deterministic host transport")
    length = DLC_TO_LEN[msg[0].data_len_code]
    return [int(msg[0].addr), bytes(libsafety_py.ffi.buffer(msg[0].data, length)).hex(), int(msg[0].bus)]

  def _controls_allowed(self):
    if self.backend is None:
      return bool(self.safety.get_controls_allowed())
    health = self.backend.health()
    self.assertIn("controls_allowed", health, "Backend lacks public controls health")
    return bool(health["controls_allowed"])

  def _reset_relay_test(self):
    self._reset_safety_hooks()
    if self.backend is None:
      self.safety.init_tests()
    else:
      # Stock firmware grants relay detection a real startup grace interval.
      self._wait(1.1)
      self.assertFalse(self._relay_fault_observed(), "Fault present before relay injection")

  def _relay_fault_observed(self):
    if self.backend is None:
      return bool(self.safety.get_relay_malfunction())
    health = self.backend.health()
    self.assertIn("faults", health, "Backend lacks public fault health")
    # Public aggregate fault evidence, attributed by clean reset + one injection.
    # Exact private relay-latch identity remains a deterministic host supplement.
    return bool(health["faults"])

  def _assert_forward(self, bus, addr, expected_bus, *, healthy=True, rx_valid=None):
    if self.backend is None:
      self.assertEqual(expected_bus, self.safety.safety_fwd_hook(bus, addr))
    else:
      self.assertTrue(self.backend.forward(self._raw_frame(make_msg(bus, addr)),
                                          None if expected_bus == -1 else expected_bus,
                                          controls=None, healthy=healthy, rx_valid=rx_valid), (bus, addr, expected_bus))

  @classmethod
  def setUpClass(cls):
    # Classes defined in this module and classes named *Base are shared test helpers.
    if cls.__module__ == __name__ or cls.__name__.endswith('Base'):
      raise unittest.SkipTest

  def _reset_safety_hooks(self):
    if self.backend is not None:
      self._reset_backend()
      return
    self.safety.set_safety_hooks(self.safety.get_current_safety_mode(),
                                 self.safety.get_current_safety_param())

  def _engage(self):
    raise NotImplementedError("Safety mode must provide a CAN engagement sequence")

  def _disengage(self):
    raise NotImplementedError("Safety mode must provide a CAN disengagement sequence")

  def _set_controls_allowed(self, allowed):
    if allowed:
      self._engage()
    else:
      self._disengage()
    self.assertEqual(bool(allowed), self._controls_allowed())

  def _rx(self, msg, *, raw=False):
    if self.backend is None:
      return self.safety.safety_rx_hook(msg)
    from opendbc.safety.tests.fixture_streams import prepare_rx
    # Validate wire representability before the bridge can interpret the packet.
    self._raw_frame(msg)
    frame, updates, stream_name = prepare_rx(self, msg, raw=raw)
    self._update_required_streams(updates)
    return self.backend.rx(frame) if stream_name is None else self.backend.rx_stream(stream_name)

  def _rx_valid_group(self, messages):
    """Group only contiguous valid measurement inputs, never faults or TX barriers."""
    if self.backend is None:
      for message in messages:
        self.assertTrue(self._rx(message))
      return
    from opendbc.safety.tests.fixture_streams import prepare_rx
    events = []
    for message in messages:
      self._raw_frame(message)  # Reject USB-only metadata before interpreting it.
      frame, updates, stream_name = prepare_rx(self, message)
      events.append({'frame': frame, 'updates': updates, 'stream_name': stream_name})
      self.assertLessEqual(len(events), 32, 'Measurement group exceeds the backend limit')
    self.assertTrue(events, 'Empty measurement group')
    # Plans are sequential test inputs; the service applies each replacement
    # immediately before consuming that stream's real next counter phase.
    self.assertIs(self.backend.rx_group(events), True)

  def _tx(self, msg, *, healthy=True):
    # Disabled controls are ordinary test inputs; watchdog/transport faults are
    # not. Fault scenarios must explicitly opt out of the stream health guard.
    return self.safety.safety_tx_hook(msg) if self.backend is None else self.backend.tx(self._raw_frame(msg), controls=None, healthy=healthy)

  def _set_alternative_experience(self, value):
    if self.backend is None:
      self.safety.set_alternative_experience(value)
    else:
      self.backend.set_alternative_experience(value)

  @staticmethod
  def _boundary_values(boundaries, min_val, max_val, step=1, width=5, sparse_count=100):
    """Generate test values dense around boundaries and sparse across the full range."""
    values = set()
    for b in boundaries:
      for offset in range(-width, width + 1):
        v = round(b + offset * step, 2)
        if min_val <= v < max_val:
          values.add(v)
    sparse_step = max(step, (max_val - min_val) / sparse_count)
    for v in np.arange(min_val, max_val, sparse_step):
      values.add(round(v, 2))
    return sorted(values)

  def _generic_limit_safety_check(self, msg_function: MessageFunction, min_allowed_value: float, max_allowed_value: float,
                                  min_possible_value: float, max_possible_value: float, test_delta: float = 1, inactive_value: float = 0,
                                  msg_allowed = True, additional_setup: Callable[[float], None] | None = None):
    """
      Enforces that a signal within a message is only allowed to be sent within a specific range, min_allowed_value -> max_allowed_value.
      Message is also only allowed to be sent when controls_allowed is true, unless the value is equal to inactive_value.
      Message is never allowed if msg_allowed is false, for example when stock longitudinal is enabled and you are sending acceleration requests.
      additional_setup is used for extra setup before each _tx, ex: for setting the previous torque for rate limits
    """

    # Ensure that we at least test the allowed_value range
    self.assertGreater(max_possible_value, max_allowed_value)
    self.assertLessEqual(min_possible_value, min_allowed_value)

    test_values = self._boundary_values([min_allowed_value, max_allowed_value, 0, inactive_value],
                                        min_possible_value, max_possible_value, test_delta)

    for controls_allowed in [False, True]:
      for v in test_values:
        self._set_controls_allowed(controls_allowed)
        if additional_setup is not None:
          additional_setup(v)
        should_tx = controls_allowed and min_allowed_value <= v <= max_allowed_value
        should_tx = (should_tx or v == inactive_value) and msg_allowed
        self.assertEqual(self._tx(msg_function(v)), should_tx, (controls_allowed, should_tx, v))

  def _common_measurement_test(self, msg_func: Callable, min_value: float, max_value: float, factor: float,
                               meas_min_func: Callable[[], int], meas_max_func: Callable[[], int]):
    """Tests accurate measurement parsing, and that the struct is reset on safety mode init"""
    for val in np.arange(min_value, max_value, 0.5):
      for i in range(MAX_SAMPLE_VALS):
        self.assertTrue(self._rx(msg_func(val + i * 0.1)))

      # assert close by one decimal place
      self.assertAlmostEqual(meas_min_func() / factor, val, delta=0.1)
      self.assertAlmostEqual(meas_max_func() / factor - 0.5, val, delta=0.1)

      # ensure sample_t is reset on safety init
      self._reset_safety_hooks()
      self.assertEqual(meas_min_func(), 0)
      self.assertEqual(meas_max_func(), 0)


class LongitudinalAccelSafetyTest(SafetyTestBase, abc.ABC):

  LONGITUDINAL = True
  MAX_ACCEL: float = 2.0
  MIN_ACCEL: float = -3.5
  INACTIVE_ACCEL: float = 0.0

  @abc.abstractmethod
  def _accel_msg(self, accel: float):
    pass

  def test_accel_limits_correct(self):
    self.assertGreater(self.MAX_ACCEL, 0)
    self.assertLess(self.MIN_ACCEL, 0)

  def _setup_alternative_experience_phase(self, value):
    # Stock firmware accepts configuration only outside a car safety mode.
    # Rebuild periodic inputs after the explicit reset; probes engage via CAN.
    self._reset_backend(value)
    self.assertEqual(self.backend.health()['alternative_experience'], value)

  def test_accel_actuation_limits(self):
    limits = ((self.MIN_ACCEL, self.MAX_ACCEL, ALTERNATIVE_EXPERIENCE.DEFAULT),
              (self.MIN_ACCEL, self.MAX_ACCEL, ALTERNATIVE_EXPERIENCE.RAISE_LONGITUDINAL_LIMITS_TO_ISO_MAX))

    for min_accel, max_accel, alternative_experience in limits:
      if self.backend is not None:
        self._setup_alternative_experience_phase(alternative_experience)
      # enforce we don't skip over 0 or inactive accel
      for accel in np.concatenate((np.arange(min_accel - 1, max_accel + 1, 0.05), [0, self.INACTIVE_ACCEL])):
        accel = round(accel, 2)  # floats might not hit exact boundary conditions without rounding
        for controls_allowed in [True, False]:
          self._set_controls_allowed(controls_allowed)
          if self.backend is None:
            self._set_alternative_experience(alternative_experience)
          if self.LONGITUDINAL:
            should_tx = controls_allowed and min_accel <= accel <= max_accel
            should_tx = should_tx or accel == self.INACTIVE_ACCEL
          else:
            should_tx = False
          self.assertEqual(should_tx, self._tx(self._accel_msg(accel)))


class LongitudinalGasBrakeSafetyTest(SafetyTestBase, abc.ABC):

  MIN_BRAKE: int = 0
  MAX_BRAKE: int | None = None
  MAX_POSSIBLE_BRAKE: int | None = None

  MIN_GAS: int = 0
  MAX_GAS: int | None = None
  INACTIVE_GAS = 0
  MIN_POSSIBLE_GAS: int = 0
  MAX_POSSIBLE_GAS: int | None = None

  def test_gas_brake_limits_correct(self):
    self.assertIsNotNone(self.MAX_POSSIBLE_BRAKE)
    self.assertIsNotNone(self.MAX_POSSIBLE_GAS)

    assert self.MAX_BRAKE is not None and self.MAX_GAS is not None
    self.assertGreater(self.MAX_BRAKE, self.MIN_BRAKE)
    self.assertGreater(self.MAX_GAS, self.MIN_GAS)

  @abc.abstractmethod
  def _send_gas_msg(self, gas: int):
    pass

  @abc.abstractmethod
  def _send_brake_msg(self, brake: int):
    pass

  def test_brake_safety_check(self):
    self._generic_limit_safety_check(self._send_brake_msg, self.MIN_BRAKE, self.MAX_BRAKE, 0, self.MAX_POSSIBLE_BRAKE, 1)

  def test_gas_safety_check(self):
    self._generic_limit_safety_check(self._send_gas_msg, self.MIN_GAS, self.MAX_GAS, self.MIN_POSSIBLE_GAS, self.MAX_POSSIBLE_GAS, 1, self.INACTIVE_GAS)


class TorqueSteeringSafetyTestBase(SafetyTestBase, abc.ABC):

  MAX_RATE_UP = 0
  MAX_RATE_DOWN = 0
  MAX_TORQUE_LOOKUP: tuple[list[float], list[int]] = ([0], [0])
  DYNAMIC_MAX_TORQUE = False
  MAX_RT_DELTA = 0

  NO_STEER_REQ_BIT = False

  @property
  def MAX_TORQUE(self):
    return max(self.MAX_TORQUE_LOOKUP[1])

  @property
  def _torque_speed_range(self):
    if not self.DYNAMIC_MAX_TORQUE:
      return [0]
    else:
      # test with more precision inside breakpoint range
      min_speed = min(self.MAX_TORQUE_LOOKUP[0])
      max_speed = max(self.MAX_TORQUE_LOOKUP[0])
      return np.concatenate([np.arange(0, min_speed, 5), np.arange(min_speed, max_speed, 0.5), np.arange(max_speed, 40, 5)])

  def _get_max_torque(self, speed):
    # matches safety fudge
    torque = int(np.interp(speed - 1, self.MAX_TORQUE_LOOKUP[0], self.MAX_TORQUE_LOOKUP[1]) + 1)
    return min(torque, self.MAX_TORQUE)

  @abc.abstractmethod
  def _torque_cmd_msg(self, torque, steer_req=1):
    pass

  @abc.abstractmethod
  def _speed_msg(self, speed):
    pass

  def _reset_speed_measurement(self, speed):
    def messages():
      for _ in range(MAX_SAMPLE_VALS):
        yield self._speed_msg(speed)
        secondary = getattr(self, '_speed_msg_2', None)
        if secondary is not None:
          message = secondary(speed)
          if message is not None:
            yield message
    self._rx_valid_group(messages())

  def _advance_torque_time(self):
    self._wait((RT_INTERVAL + 1) / 1_000_000)

  def _prepare_torque_measurement(self, torque):
    pass

  def _set_prev_torque(self, t):
    """Reach a command through accepted CAN frames, including the realtime baseline."""
    enabled = self._controls_allowed()
    self._set_controls_allowed(False)
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))
    self._set_controls_allowed(True)
    self._prepare_torque_measurement(0)
    torque = 0
    # Driver measurements stay fixed throughout setup. Pipeline only that case;
    # motor-torque fixtures still inject each dependent measurement before TX.
    pipeline = self.backend is not None and isinstance(self, DriverTorqueSteeringSafetyTest)
    frames = []
    cadence = 0
    if self.backend is not None:
      cadence = max(0.01, (RT_INTERVAL + 1) / 1_000_000 * self.MAX_RATE_UP / (0.8 * self.MAX_RT_DELTA))
    while torque != t:
      torque += sign_of(t - torque) * min(abs(t - torque), self.MAX_RATE_UP)
      self._prepare_torque_measurement(torque)
      if pipeline:
        frames.append(self._raw_frame(self._torque_cmd_msg(torque)))
        continue
      if self.backend is None:
        self._advance_torque_time()
      else:
        # Existing torque fixtures have MAX_RATE_UP <= 0.125 * MAX_RT_DELTA.
        # An 80% ramp budget leaves room for the extra discrete window step.
        self._wait(cadence)
      self.assertTrue(self._tx(self._torque_cmd_msg(torque)), torque)
    if frames:
      result = self.backend.accepted_ramp(frames, int(np.ceil(cadence * 1_000_000)))
      self.assertEqual(result['verdicts'], [True] * len(frames))
    self._advance_torque_time()
    self.assertTrue(self._tx(self._torque_cmd_msg(t)))
    self._set_controls_allowed(enabled)

  def _scheduled_steering(self, commands, offsets):
    """Return public dispatch bounds, never pretend host timestamps are MCU time."""
    import time
    max_lateness_us = 10_000
    events = [{'offset_us': int(offset), 'frame': self._raw_frame(self._torque_cmd_msg(torque, request))}
              for (torque, request), offset in zip(commands, offsets, strict=True)]
    wall_start = time.monotonic_ns()
    result = self.backend.steering_sequence(events, max_lateness_us=max_lateness_us)
    wall_end = time.monotonic_ns()
    timing = result['timing']
    self.assertIs(timing['complete'], True, timing)
    self.assertEqual(len(result['verdicts']), len(events), result)
    self.assertEqual(len(timing['events']), len(events), timing)
    if timing.get('physical') is False:
      from opendbc.safety.tests.simulated import SimulatedBackend
      self.assertIsInstance(self.backend, SimulatedBackend, 'Physical backend returned simulated timing')
      self.assertEqual(timing['source'], 'simulated_virtual_time')
      bounds = [(event['offset_us'], event['offset_us']) for event in timing['events']]
      observation_end = timing['quiet_end_us'] - timing['start_us']
    else:
      self.assertLessEqual(timing['max_jitter_ns'], max_lateness_us * 1000, timing)
      origin = timing['events'][0]['send_start_ns']
      bounds = []
      for planned, actual in zip(events, timing['events'], strict=True):
        self.assertEqual(actual['offset_us'], planned['offset_us'], timing)
        self.assertEqual(tuple(actual['frame']), tuple(planned['frame']), timing)
        deadline = timing['start_ns'] + planned['offset_us'] * 1000
        self.assertGreaterEqual(actual['send_start_ns'], deadline, timing)
        self.assertGreaterEqual(actual['send_end_ns'], actual['send_start_ns'], timing)
        self.assertLessEqual(actual['send_end_ns'] - deadline, max_lateness_us * 1000, timing)
        bounds.append(((actual['send_start_ns'] - origin) / 1000, (actual['send_end_ns'] - origin) / 1000))
      # Receipts bound MCU processing; later wire-quiet and health checks do
      # not extend the interval in which these commands reached safety hooks.
      observation_end = (timing['receipts_observed_ns'] - origin) / 1000
    # The rejected anchor is inside this batch. RPC preflight and post-observation
    # delay cannot extend its service-local anchor-to-command dispatch interval.
    self._steering_sequence_evidence = getattr(self, '_steering_sequence_evidence', [])
    self._steering_sequence_evidence.append({'timing': timing, 'caller_elapsed_ns': wall_end - wall_start})
    return result['verdicts'], bounds, observation_end

  def _coarse_realtime_torque(self):
    """Scheduled coarse behavior; original dense zero-time sweep is host-only."""
    self.assertGreater(self.MAX_RATE_UP, 0)
    values = list(range(self.MAX_RATE_UP, self.MAX_RT_DELTA + 1, self.MAX_RATE_UP))
    values.append(self.MAX_RT_DELTA + 1)
    self.assertTrue(values)
    self.assertLessEqual(max(np.diff([0, *values])), self.MAX_RATE_UP)
    self.assertLess(values[-1], self.MAX_TORQUE, 'Need a command beyond RT but below absolute limit')
    spacing = 6000 if len(values) * 6000 + 10_000 < RT_INTERVAL else 4000
    self.assertLess(len(values) * spacing + 10_000, RT_INTERVAL, 'Coarse sequence cannot fit the RT window')
    for sign in (-1, 1):
      for cross_limit in (True, False):
        self._reset_safety_hooks()
        self._set_controls_allowed(True)
        if isinstance(self, MotorTorqueSteeringSafetyTest):
          # Motor error limits are directional: zero remains allowed while
          # winding toward a same-sign measurement. Use the ramp endpoint as a
          # constant reference, so even a narrow error band cannot constrain the
          # tested direction. The post-wait positive control proves that the
          # rejected crossing is otherwise a legitimate command.
          self._reset_torque_measurement(sign * values[-1])
        else:
          self._reset_torque_driver_measurement(0)
        ramp = values if cross_limit else values[:-1]
        # Absolute-cap violation resets the RT baseline after all slow setup.
        commands = [(sign * (self.MAX_TORQUE + 1), 1), *((sign * value, 1) for value in ramp)]
        verdicts, bounds, observation_end = self._scheduled_steering(commands, [i * spacing for i in range(len(commands))])
        self.assertLess(observation_end, RT_INTERVAL, (bounds, observation_end))
        expected = [False] + [True] * len(ramp)
        if cross_limit:
          expected[-1] = False
        self.assertEqual(verdicts, expected, (sign, ramp, bounds))
        if not cross_limit:
          # Rebase after a broad real wait and prove the same crossing is allowed.
          self._wait((RT_INTERVAL + 50_000) / 1_000_000)
          self.assertTrue(self._tx(self._torque_cmd_msg(sign * ramp[-1])))
          self.assertTrue(self._tx(self._torque_cmd_msg(sign * values[-1])))

  def _coarse_request_realtime(self):
    """Broad negative/positive request intervals; no exact edge coverage claim."""
    count = self.MIN_VALID_STEERING_FRAMES
    self.assertLessEqual(count + self.MAX_INVALID_STEERING_FRAMES, min(self.MAX_TORQUE, self.MAX_RT_DELTA))
    negative_cut_us = (count + 1) * 6000
    self.assertLess(negative_cut_us + 10_000, self.MIN_VALID_STEERING_RT_INTERVAL,
                    'Unique 6ms request frames do not fit a discriminating negative interval')
    for allowed in (False, True):
      self._reset_safety_hooks()
      self._set_controls_allowed(True)
      if isinstance(self, MotorTorqueSteeringSafetyTest):
        self._reset_torque_measurement(round(count / 2))
      else:
        self._reset_torque_driver_measurement(0)
      start = self.MIN_VALID_STEERING_RT_INTERVAL + 50_000 if allowed else 0
      commands = [(self.MAX_TORQUE + 1, 1)] + [(i, 1) for i in range(1, count + 1)]
      offsets = [0] + [start + i * 6000 for i in range(1, count + 1)]
      cuts = self.MAX_INVALID_STEERING_FRAMES if allowed else 1
      commands += [(count + i, 0) for i in range(cuts)]
      offsets += [start + (count + 1 + i) * 6000 for i in range(cuts)]
      verdicts, bounds, observation_end = self._scheduled_steering(commands, offsets)
      self.assertEqual(verdicts, [False] + [True] * count + [allowed] * cuts, bounds)
      first_cut = bounds[count + 1]
      if allowed:
        # First anchor completion is at most 10ms late; use a conservative lower bound.
        self.assertGreater(first_cut[0] - bounds[0][1], self.MIN_VALID_STEERING_RT_INTERVAL)
      else:
        self.assertLess(observation_end, self.MIN_VALID_STEERING_RT_INTERVAL)
      self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=1)))
      self.assertTrue(self._tx(self._torque_cmd_msg(1, steer_req=1)))

  def _steering_timing_test(self):
    if self.backend is not None:
      from opendbc.safety.tests.simulated import SimulatedBackend
      if not isinstance(self.backend, SimulatedBackend):
        raise AssertionError("Exact steering timing needs timestamped TX sequences with bounded jitter; host clock supplement required")

  def _steering_test_init(self):
    if self.backend is None:
      self.safety.init_tests()
    else:
      # RX health warmup consumes real time. Establish a fresh observable timing
      # baseline with a rejected command instead of assuming reset means time 0.
      self.assertFalse(self._tx(self._torque_cmd_msg(1, steer_req=0)))

  def _steering_test_elapsed(self, microseconds):
    if self.backend is None:
      self.safety.set_timer(int(microseconds))
    else:
      self._wait(int(microseconds) / 1_000_000)

  def test_steer_safety_check(self):
    for speed in self._torque_speed_range:
      self._reset_speed_measurement(speed)
      max_torque = self._get_max_torque(speed)
      for sign in (-1, 1):
        self._set_controls_allowed(True)
        self._set_prev_torque(sign * max_torque)
        self.assertTrue(self._tx(self._torque_cmd_msg(sign * max_torque)))
        self.assertFalse(self._tx(self._torque_cmd_msg(sign * (max_torque + 1))))
        self._set_prev_torque(sign * max_torque)
        self.assertFalse(self._tx(self._torque_cmd_msg(sign * int(max_torque * 1.5))))
        self._set_controls_allowed(False)
        self.assertTrue(self._tx(self._torque_cmd_msg(0)))
        for torque in (sign, sign * max_torque, sign * (max_torque + 1)):
          self.assertFalse(self._tx(self._torque_cmd_msg(torque)))

  def test_non_realtime_limit_up(self):
    self._set_controls_allowed(True)

    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(-self.MAX_RATE_UP)))

    self._set_prev_torque(0)
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP + 1)))
    self._set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertFalse(self._tx(self._torque_cmd_msg(-self.MAX_RATE_UP - 1)))

  def test_steer_req_bit(self):
    """Asserts all torque safety modes check the steering request bit"""
    if self.NO_STEER_REQ_BIT:
      raise unittest.SkipTest("No steering request bit")

    self._reset_safety_hooks()
    self._set_controls_allowed(True)
    self.assertTrue(self._tx(self._torque_cmd_msg(1, 1)))
    self.assertFalse(self._tx(self._torque_cmd_msg(1, 0)))
    for _ in range(10):
      self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_TORQUE, 1)))


def hold_measurement_stream(case, packet, max_idle_seconds=1.0, *, clock=None):
  """Hold only this measurement's periodic input; explicit RX still advances it.

  Call the yielded guard immediately BEFORE each explicit RX. The guard imposes
  the stock one-second minimum watchdog budget between refreshes/probes; the backend's
  watchdog and all other input streams remain active. Resume even on failure.
  """
  from contextlib import contextmanager
  import time

  clock = time.monotonic if clock is None else clock

  @contextmanager
  def hold():
    if case.backend is None:
      yield lambda: None
      return
    frame = case._raw_frame(packet)
    bridge = case._fixture_stream_bridge
    name = bridge.keys.get((frame[0], frame[2]))
    case.assertIsNotNone(name, f'No required measurement stream for {frame[0]:#x} bus {frame[2]}')
    last_refresh = clock()

    def refresh():
      nonlocal last_refresh
      now = clock()
      case.assertLess(now - last_refresh, max_idle_seconds, 'Measurement hold exceeded refresh deadline')
      last_refresh = now

    case.backend.pause_stream(name)
    try:
      yield refresh
      case.assertLess(clock() - last_refresh, max_idle_seconds, 'Measurement probes exceeded refresh deadline')
    finally:
      case.backend.resume_stream(name)

  return hold()


class SteerRequestCutSafetyTest(TorqueSteeringSafetyTestBase, abc.ABC):

  # Safety around steering request bit mismatch tolerance
  MIN_VALID_STEERING_FRAMES: int
  MAX_INVALID_STEERING_FRAMES: int
  STEER_STEP: int = 1

  @property
  def MIN_VALID_STEERING_RT_INTERVAL(self):
    # a ~10% buffer
    return int((self.MIN_VALID_STEERING_FRAMES + 1) * self.STEER_STEP * 10000 * 0.9)

  @coverage('Request mismatch counts and recovery after a CAN-established timing baseline',
            host=('Original frozen-clock execution and synthetic timing baseline',))
  def test_steer_req_bit_frames(self):
    """
      Certain safety modes implement some tolerance on their steer request bits matching the
      requested torque to avoid a steering fault or lockout and maintain torque. This tests:
        - We can't cut torque for more than one frame
        - We can't cut torque until at least the minimum number of matching steer_req messages
        - We can always recover from violations if steer_req=1
    """

    # dense around the threshold, sparse elsewhere (the valid frame counter saturates at MIN_VALID_STEERING_FRAMES)
    valid_frames = self._boundary_values([self.MIN_VALID_STEERING_FRAMES], 0, self.MIN_VALID_STEERING_FRAMES * 2, sparse_count=10)
    for min_valid_steer_frames in map(int, valid_frames):
      # Reset match count and rt timer to allow cut (valid_steer_req_count, ts_steer_req_mismatch_last)
      self._reset_safety_hooks()
      self._steering_test_init()
      self._steering_test_elapsed(self.MIN_VALID_STEERING_RT_INTERVAL)

      # Allow torque cut
      self._set_controls_allowed(True)
      for _ in range(min_valid_steer_frames):
        self.assertTrue(self._tx(self._torque_cmd_msg(1, steer_req=1)))

      # should tx if we've sent enough valid frames, and we're not cutting torque for too many frames consecutively
      should_tx = min_valid_steer_frames >= self.MIN_VALID_STEERING_FRAMES
      for idx in range(self.MAX_INVALID_STEERING_FRAMES * 2):
        tx = self._tx(self._torque_cmd_msg(1, steer_req=0))
        self.assertEqual(should_tx and idx < self.MAX_INVALID_STEERING_FRAMES, tx)

      # Keep blocking after one steer_req mismatch
      for _ in range(10):
        self.assertFalse(self._tx(self._torque_cmd_msg(1, steer_req=0)))

      # Make sure we can recover
      self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=1)))
      self.assertTrue(self._tx(self._torque_cmd_msg(1, steer_req=1)))

  @coverage('Request mismatch counts and recovery after a CAN-established timing baseline',
            host=('Original frozen-clock execution and synthetic timing baseline',))
  def test_steer_req_bit_multi_invalid(self):
    """
      For safety modes allowing multiple consecutive invalid frames, this ensures that once a valid frame
      is sent after an invalid frame (even without sending the max number of allowed invalid frames),
      all counters are reset.
    """
    for max_invalid_steer_frames in range(1, self.MAX_INVALID_STEERING_FRAMES * 2):
      self._reset_safety_hooks()
      self._steering_test_init()
      self._steering_test_elapsed(self.MIN_VALID_STEERING_RT_INTERVAL)

      # Allow torque cut
      self._set_controls_allowed(True)
      for _ in range(self.MIN_VALID_STEERING_FRAMES):
        self.assertTrue(self._tx(self._torque_cmd_msg(1, steer_req=1)))

      # Send partial amount of allowed invalid frames
      for idx in range(max_invalid_steer_frames):
        should_tx = idx < self.MAX_INVALID_STEERING_FRAMES
        self.assertEqual(should_tx, self._tx(self._torque_cmd_msg(1, steer_req=0)))

      # Send one valid frame, and subsequent invalid should now be blocked
      self.assertTrue(self._tx(self._torque_cmd_msg(1, steer_req=1)))
      for _ in range(self.MIN_VALID_STEERING_FRAMES + 1):
        self.assertFalse(self._tx(self._torque_cmd_msg(1, steer_req=0)))

  @coverage('Broad request-mismatch rejection and recovery intervals',
            host=('Original synthetic elapsed-time grid and MCU clock baseline',))
  def test_steer_req_bit_realtime(self):
    """
      Realtime safety for cutting steer request bit. This tests:
        - That we allow messages with mismatching steer request bit if time from last is >= MIN_VALID_STEERING_RT_INTERVAL
        - That frame mismatch safety does not interfere with this test
    """
    if self.backend is not None:
      self._coarse_request_realtime()
      return
    self._steering_timing_test()
    for rt_us in np.arange(self.MIN_VALID_STEERING_RT_INTERVAL - 50000, self.MIN_VALID_STEERING_RT_INTERVAL + 50000, 10000):
      # Reset match count and rt timer (valid_steer_req_count, ts_steer_req_mismatch_last)
      self._reset_safety_hooks()
      self._steering_test_init()

      # Make sure valid_steer_req_count doesn't affect this test
      self._set_controls_allowed(True)
      for _ in range(self.MIN_VALID_STEERING_FRAMES):
        self.assertTrue(self._tx(self._torque_cmd_msg(1, steer_req=1)))

      # Normally, sending MIN_VALID_STEERING_FRAMES valid frames should always allow
      self._steering_test_elapsed(max(rt_us, 0))
      should_tx = rt_us >= self.MIN_VALID_STEERING_RT_INTERVAL
      for _ in range(self.MAX_INVALID_STEERING_FRAMES):
        self.assertEqual(should_tx, self._tx(self._torque_cmd_msg(1, steer_req=0)))

      # Keep blocking after one steer_req mismatch
      for _ in range(10):
        self.assertFalse(self._tx(self._torque_cmd_msg(1, steer_req=0)))

      # Make sure we can recover
      self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=1)))
      self.assertTrue(self._tx(self._torque_cmd_msg(1, steer_req=1)))


class DriverTorqueSteeringSafetyTest(TorqueSteeringSafetyTestBase, abc.ABC):

  DRIVER_TORQUE_ALLOWANCE = 0
  DRIVER_TORQUE_FACTOR = 0

  @abc.abstractmethod
  def _torque_driver_msg(self, torque):
    pass

  def _reset_torque_driver_measurement(self, torque):
    self._rx_valid_group(self._torque_driver_msg(torque) for _ in range(MAX_SAMPLE_VALS))

  def _set_prev_torque(self, torque):
    self._reset_torque_driver_measurement(0)
    super()._set_prev_torque(torque)

  def _probe_driver_window(self, history):
    samples = history[-MAX_SAMPLE_VALS:]
    # Translate both orientations across the allowance: neither original
    # extremum's eviction can hide behind the driver's deadband.
    for sign in (-1, 1):
      for orientation in (-1, 1):
        offset = self.DRIVER_TORQUE_ALLOWANCE + 50
        extreme = max(orientation * sample for sample in samples)
        boundary = max(0, self.MAX_TORQUE + (extreme - 50) * self.DRIVER_TORQUE_FACTOR)
        for command, allowed in ((boundary, True), (boundary + 1, False)):
          self._set_controls_allowed(True)
          self._set_prev_torque(sign * min(command, self.MAX_TORQUE))
          # Replay the full six/seven/eight-frame prefix; only the expected
          # extrema are restricted to six samples. Setup zeros are not history.
          first = self._torque_driver_msg(sign * (orientation * history[0] - offset))
          with hold_measurement_stream(self, first) as refresh:
            for index, torque in enumerate(history):
              packet = first if index == 0 else self._torque_driver_msg(sign * (orientation * torque - offset))
              refresh()
              self.assertTrue(self._rx(packet))
            self.assertEqual(bool(self._tx(self._torque_cmd_msg(sign * command))), allowed,
                             (history, sign, orientation, command))

  @coverage('Signed driver-torque boundary and eviction probes',
            host=('Exact private driver-torque extrema',))
  def test_driver_torque_measurements(self):
    if self.backend is not None:
      history = []
      for torque in (50, -50, *([0] * (MAX_SAMPLE_VALS - 2)), 0, 0):
        history.append(torque)
        if len(history) >= MAX_SAMPLE_VALS:
          self._probe_driver_window(history)
      return
    for torque in (50, -50, *([0] * (MAX_SAMPLE_VALS - 2))):
      self.assertTrue(self._rx(self._torque_driver_msg(torque)))

    self.assertEqual(-50, self.safety.get_torque_driver_min())
    self.assertEqual(50, self.safety.get_torque_driver_max())

    self.assertTrue(self._rx(self._torque_driver_msg(0)))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(-50, self.safety.get_torque_driver_min())

    self.assertTrue(self._rx(self._torque_driver_msg(0)))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(0, self.safety.get_torque_driver_min())

  def test_against_torque_driver(self):
    # Tests down limits and driver torque blending
    self._set_controls_allowed(True)

    for speed in self._torque_speed_range:
      self._reset_speed_measurement(speed)
      if self.backend is not None:
        # Changing two live speed sources can transiently cancel engagement.
        # Re-establish the test precondition after both sources match, before
        # torque setup preserves the current controls state.
        self._set_controls_allowed(True)
      max_torque = self._get_max_torque(speed)

      # Cannot stay at MAX_TORQUE if above DRIVER_TORQUE_ALLOWANCE
      for sign in [-1, 1]:
        previous_tx = False
        for driver_torque in self._boundary_values([self.DRIVER_TORQUE_ALLOWANCE], 0, self.DRIVER_TORQUE_ALLOWANCE * 2, sparse_count=10):
          if not previous_tx:
            self._set_prev_torque(max_torque * sign)
          self._reset_torque_driver_measurement(-driver_torque * sign)
          should_tx = abs(driver_torque) <= self.DRIVER_TORQUE_ALLOWANCE
          self.assertEqual(should_tx, self._tx(self._torque_cmd_msg(max_torque * sign)))
          previous_tx = should_tx

      # arbitrary high driver torque to ensure max steer torque is allowed
      max_driver_torque = int(max_torque / self.DRIVER_TORQUE_FACTOR + self.DRIVER_TORQUE_ALLOWANCE + 1)

      # spot check some individual cases
      for sign in [-1, 1]:
        # Ensure we wind down factor units for every unit above allowance
        driver_torque = (self.DRIVER_TORQUE_ALLOWANCE + 10) * sign
        torque_desired = (max_torque - 10 * self.DRIVER_TORQUE_FACTOR) * sign
        delta = 1 * sign
        self._set_prev_torque(torque_desired)
        self._reset_torque_driver_measurement(-driver_torque)
        self.assertTrue(self._tx(self._torque_cmd_msg(torque_desired)))
        self._set_prev_torque(torque_desired + delta)
        self._reset_torque_driver_measurement(-driver_torque)
        self.assertFalse(self._tx(self._torque_cmd_msg(torque_desired + delta)))

        # If we're well past the allowance, minimum wind down is MAX_RATE_DOWN
        self._set_prev_torque(max_torque * sign)
        self._reset_torque_driver_measurement(-max_driver_torque * sign)
        self.assertTrue(self._tx(self._torque_cmd_msg((max_torque - self.MAX_RATE_DOWN) * sign)))
        self._set_prev_torque(max_torque * sign)
        self._reset_torque_driver_measurement(-max_driver_torque * sign)
        self.assertTrue(self._tx(self._torque_cmd_msg(0)))
        self._set_prev_torque(max_torque * sign)
        self._reset_torque_driver_measurement(-max_driver_torque * sign)
        self.assertFalse(self._tx(self._torque_cmd_msg((max_torque - self.MAX_RATE_DOWN + 1) * sign)))

  @coverage('Coarse realtime torque rejection and recovery',
            host=('Original dense frozen-clock sweep and exact MCU rollover timing',))
  def test_realtime_limits(self):
    if self.backend is not None:
      self._coarse_realtime_torque()
      return
    self._steering_timing_test()
    self._set_controls_allowed(True)

    for sign in [-1, 1]:
      self._reset_safety_hooks()
      self._steering_test_init()
      self._set_controls_allowed(True)
      self._reset_torque_driver_measurement(0)
      for t in np.arange(0, self.MAX_RT_DELTA, 1):
        t *= sign
        self.assertTrue(self._tx(self._torque_cmd_msg(t)))
      self.assertFalse(self._tx(self._torque_cmd_msg(sign * (self.MAX_RT_DELTA + 1))))

      self.assertTrue(self._tx(self._torque_cmd_msg(0)))
      for t in np.arange(0, self.MAX_RT_DELTA, 1):
        t *= sign
        self.assertTrue(self._tx(self._torque_cmd_msg(t)))

      # Increase timer to update rt_torque_last
      self._steering_test_elapsed(RT_INTERVAL + 1)
      self.assertTrue(self._tx(self._torque_cmd_msg(sign * (self.MAX_RT_DELTA - 1))))
      self.assertTrue(self._tx(self._torque_cmd_msg(sign * (self.MAX_RT_DELTA + 1))))

  @coverage('Neutral-input torque boundary after reset and RX warmup',
            host=('Immediate zero driver-torque extrema before RX warmup',))
  def test_reset_driver_torque_measurements(self):
    if self.backend is not None:
      # Public evidence proves the post-reset neutral-input boundary. Stream
      # warmup cannot prove immediately empty sample_t; retain exact host checks.
      for t in np.linspace(-self.MAX_TORQUE, self.MAX_TORQUE, MAX_SAMPLE_VALS):
        self.assertTrue(self._rx(self._torque_driver_msg(t)))
      for sign in (-1, 1):
        self._set_controls_allowed(True)
        self._set_prev_torque(sign * self.MAX_TORQUE)
        self._reset_torque_driver_measurement(-sign * (self.DRIVER_TORQUE_ALLOWANCE + 50))
        self.assertFalse(self._tx(self._torque_cmd_msg(sign * self.MAX_TORQUE)))
        self._reset_safety_hooks()
        self._set_controls_allowed(True)
        # This ramp must not replace the just-reset driver samples.
        for torque in range(0, self.MAX_TORQUE + 1, self.MAX_RATE_UP):
          self._advance_torque_time()
          self.assertTrue(self._tx(self._torque_cmd_msg(sign * torque)))
        self.assertTrue(self._tx(self._torque_cmd_msg(sign * self.MAX_TORQUE)))
      return
    # Tests that the driver torque measurement sample_t is reset on safety mode init
    for t in np.linspace(-self.MAX_TORQUE, self.MAX_TORQUE, MAX_SAMPLE_VALS):
      self.assertTrue(self._rx(self._torque_driver_msg(t)))

    self.assertNotEqual(self.safety.get_torque_driver_min(), 0)
    self.assertNotEqual(self.safety.get_torque_driver_max(), 0)

    self._reset_safety_hooks()
    self.assertEqual(self.safety.get_torque_driver_min(), 0)
    self.assertEqual(self.safety.get_torque_driver_max(), 0)


class MotorTorqueSteeringSafetyTest(TorqueSteeringSafetyTestBase, abc.ABC):

  MAX_TORQUE_ERROR = 0
  TORQUE_MEAS_TOLERANCE = 0

  @abc.abstractmethod
  def _torque_meas_msg(self, torque):
    pass

  def _reset_torque_measurement(self, torque):
    self._rx_valid_group(self._torque_meas_msg(torque) for _ in range(MAX_SAMPLE_VALS))

  def _prepare_torque_measurement(self, torque):
    self.assertTrue(self._rx(self._torque_meas_msg(torque)))

  def test_non_realtime_limit_down(self):
    for speed in self._torque_speed_range:
      self._reset_speed_measurement(speed)
      max_torque = self._get_max_torque(speed)
      torque_meas = max_torque - self.MAX_TORQUE_ERROR - 50

      for sign in (-1, 1):
        for delta in (self.MAX_RATE_DOWN, self.MAX_RATE_DOWN - 1):
          self._set_controls_allowed(True)
          self._set_prev_torque(sign * max_torque)
          self._reset_torque_measurement(sign * torque_meas)
          self.assertEqual(delta == self.MAX_RATE_DOWN, self._tx(self._torque_cmd_msg(sign * (max_torque - delta))))

  def test_exceed_torque_sensor(self):
    self._set_controls_allowed(True)

    for sign in [-1, 1]:
      self._set_prev_torque(0)
      self._reset_torque_measurement(0)
      for t in np.arange(0, self.MAX_TORQUE_ERROR + 2, 2):  # step needs to be smaller than MAX_TORQUE_ERROR
        t *= sign
        self.assertTrue(self._tx(self._torque_cmd_msg(t)))

      self.assertFalse(self._tx(self._torque_cmd_msg(sign * (self.MAX_TORQUE_ERROR + 2))))

  @coverage('Coarse realtime torque rejection and recovery',
            host=('Original dense frozen-clock sweep and exact MCU rollover timing',))
  def test_realtime_limit_up(self):
    if self.backend is not None:
      self._coarse_realtime_torque()
      return
    self._steering_timing_test()
    self._set_controls_allowed(True)

    for sign in [-1, 1]:
      self._reset_safety_hooks()
      self._steering_test_init()
      self._set_controls_allowed(True)
      for t in np.arange(0, self.MAX_RT_DELTA + 1, 1):
        t *= sign
        self._reset_torque_measurement(t)
        self.assertTrue(self._tx(self._torque_cmd_msg(t)))
      self.assertFalse(self._tx(self._torque_cmd_msg(sign * (self.MAX_RT_DELTA + 1))))

      self.assertTrue(self._tx(self._torque_cmd_msg(0)))
      for t in np.arange(0, self.MAX_RT_DELTA + 1, 1):
        t *= sign
        self._reset_torque_measurement(t)
        self.assertTrue(self._tx(self._torque_cmd_msg(t)))

      # Increase timer to update rt_torque_last
      self._steering_test_elapsed(RT_INTERVAL + 1)
      self.assertTrue(self._tx(self._torque_cmd_msg(sign * self.MAX_RT_DELTA)))
      self.assertTrue(self._tx(self._torque_cmd_msg(sign * (self.MAX_RT_DELTA + 1))))

  def _probe_motor_window(self, history):
    samples = history[-MAX_SAMPLE_VALS:]
    for sign in (-1, 1):
      extreme = max(0, max(sign * value for value in samples))
      boundary = self.MAX_TORQUE_ERROR + extreme
      for command, allowed in ((boundary, True), (boundary + self.TORQUE_MEAS_TOLERANCE + 1, False)):
        self._set_controls_allowed(True)
        self._set_prev_torque(sign * command)
        # Flush setup, then replay the FULL original prefix. Replaying only
        # its last six samples would hide a seventh-slot retention regression.
        self._reset_torque_measurement(0)
        first = self._torque_meas_msg(history[0])
        with hold_measurement_stream(self, first) as refresh:
          for index, torque in enumerate(history):
            packet = first if index == 0 else self._torque_meas_msg(torque)
            refresh()
            self.assertTrue(self._rx(packet))
          self.assertEqual(bool(self._tx(self._torque_cmd_msg(sign * command))), allowed,
                           (history, sign, command))

  @coverage('Signed motor-torque boundary and eviction probes',
            host=('Exact private motor-torque extrema within parser tolerance',))
  def test_torque_measurements(self):
    if self.backend is not None:
      history = []
      for torque in (50, -50, 0, 0, 0, 0, 0, 0):
        history.append(torque)
        if len(history) >= MAX_SAMPLE_VALS:
          self._probe_motor_window(history)
      return
    trq = 50
    for t in [trq, -trq, 0, 0, 0, 0]:
      self.assertTrue(self._rx(self._torque_meas_msg(t)))

    max_range = range(trq, trq + self.TORQUE_MEAS_TOLERANCE + 1)
    min_range = range(-(trq + self.TORQUE_MEAS_TOLERANCE), -trq + 1)
    self.assertTrue(self.safety.get_torque_meas_min() in min_range)
    self.assertTrue(self.safety.get_torque_meas_max() in max_range)

    max_range = range(self.TORQUE_MEAS_TOLERANCE + 1)
    min_range = range(-(trq + self.TORQUE_MEAS_TOLERANCE), -trq + 1)
    self.assertTrue(self._rx(self._torque_meas_msg(0)))
    self.assertTrue(self.safety.get_torque_meas_min() in min_range)
    self.assertTrue(self.safety.get_torque_meas_max() in max_range)

    max_range = range(self.TORQUE_MEAS_TOLERANCE + 1)
    min_range = range(-self.TORQUE_MEAS_TOLERANCE, 0 + 1)
    self.assertTrue(self._rx(self._torque_meas_msg(0)))
    self.assertTrue(self.safety.get_torque_meas_min() in min_range)
    self.assertTrue(self.safety.get_torque_meas_max() in max_range)

  @coverage('Neutral-input motor-torque boundary after RX warmup',
            host=('Immediate zero motor-torque extrema before RX warmup',))
  def test_reset_torque_measurements(self):
    if self.backend is not None:
      # Public evidence proves the post-reset neutral-input boundary. Stream
      # warmup cannot prove immediately empty sample_t; retain exact host checks.
      for sign in (-1, 1):
        target = sign * (self.MAX_TORQUE_ERROR + 50)
        self._set_controls_allowed(True)
        self._set_prev_torque(target)
        samples = np.linspace(-self.MAX_TORQUE, self.MAX_TORQUE, MAX_SAMPLE_VALS)
        first = self._torque_meas_msg(samples[0])
        # Keep periodic inputs from evicting the loaded motor samples
        # while the physical RX calls run. Other streams and guards stay active.
        with hold_measurement_stream(self, first) as refresh:
          for index, torque in enumerate(samples):
            refresh()
            self.assertTrue(self._rx(first if index == 0 else self._torque_meas_msg(torque)))
          self.assertTrue(self._tx(self._torque_cmd_msg(target)))
        self._reset_safety_hooks()
        self._set_controls_allowed(True)
        # Warmup replaces samples: this probes the neutral-input boundary, not immediate sample_t reset.
        for torque in range(0, self.MAX_TORQUE_ERROR + 1, self.MAX_RATE_UP):
          self._advance_torque_time()
          self.assertTrue(self._tx(self._torque_cmd_msg(sign * torque)))
        self.assertTrue(self._tx(self._torque_cmd_msg(sign * self.MAX_TORQUE_ERROR)))
        self.assertFalse(self._tx(self._torque_cmd_msg(sign * (self.MAX_TORQUE_ERROR + self.TORQUE_MEAS_TOLERANCE + 1))))
      return
    # Tests that the torque measurement sample_t is reset on safety mode init
    for t in np.linspace(-self.MAX_TORQUE, self.MAX_TORQUE, MAX_SAMPLE_VALS):
      self.assertTrue(self._rx(self._torque_meas_msg(t)))

    self.assertNotEqual(self.safety.get_torque_meas_min(), 0)
    self.assertNotEqual(self.safety.get_torque_meas_max(), 0)

    self._reset_safety_hooks()
    self.assertEqual(self.safety.get_torque_meas_min(), 0)
    self.assertEqual(self.safety.get_torque_meas_max(), 0)


class VehicleSpeedSafetyTest(SafetyTestBase):
  @abc.abstractmethod
  def _speed_msg(self, speed):
    pass

  @coverage('Consumed-speed boundary brackets and neutral TX after warmup',
            host=('Exact numeric speed extrema, unconsumed maximum and immediate reset',))
  def test_vehicle_speed_measurements(self):
    if self.backend is None:
      # Exact extrema and immediate reset are host-only supplements. TX exposes
      # the consumed minimum below, not the unconsumed maximum/sample layout.
      self._common_measurement_test(self._speed_msg, 0, 80, 1, self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

    from contextlib import ExitStack, contextmanager
    from opendbc.car.lateral import get_max_angle_delta_vm

    torque = getattr(self, 'DYNAMIC_MAX_TORQUE', False)
    curvature = isinstance(self, CurvatureSteeringSafetyTest)
    angle = isinstance(self, AngleSteeringSafetyTest)
    self.assertTrue(torque or curvature or angle, 'Consumed-speed probe needs a speed-dependent steering boundary')

    def neutral():
      if torque:
        return self._torque_cmd_msg(0)
      if curvature:
        return self._curvature_cmd_msg(0, True)
      return self._angle_cmd_msg(0, True)

    def limit(speed):
      if torque:
        return self._get_max_torque(speed)
      if curvature:
        return int(MAX_LATERAL_JERK / max(speed - 1, 1)**2 * self.SEND_RATE * self.CURVATURE_TO_CAN) + 1
      if hasattr(self, 'ANGLE_RATE_BP'):
        return int(np.interp(speed - 1, self.ANGLE_RATE_BP, self.ANGLE_RATE_UP) * self.DEG_TO_CAN) + 1
      # VM angle fixtures use the same speed floor as steer_angle_cmd_checks_vm.
      params = importlib.import_module(type(self).__module__).CarControllerParams
      return int(get_max_angle_delta_vm(max(speed - 1, 1), self.VM, params) * self.DEG_TO_CAN) + 1

    hold_options = {}
    if self.backend is not None:
      from opendbc.safety.tests.simulated import SimulatedBackend
      if isinstance(self.backend, SimulatedBackend):
        # CPU scheduling cannot advance simulated C time. Physical backends
        # keep the helper's default monotonic wall clock without an override.
        hold_options['clock'] = lambda: self.backend.now / 1_000_000

    periodic_speed = self.backend is not None and torque and self.SAFETY_MODEL == CarParams.SafetyModel.rivian

    @contextmanager
    def window(val, builders):
      if periodic_speed:
        from opendbc.safety.tests.fixture_streams import periodic_speed_window
        with periodic_speed_window(self, builders, [val + i * 0.1 for i in range(MAX_SAMPLE_VALS)]) as checkpoint:
          self._set_controls_allowed(True)
          yield checkpoint
        return
      # Engagement and command-baseline setup run with periodic RX active.
      # Hold only while injecting the six samples and probing their boundary.
      with ExitStack() as stack:
        refreshers = [stack.enter_context(hold_measurement_stream(self, builder(val), **hold_options)) for builder in builders]

        def samples():
          for refresh in refreshers:
            refresh()
          self._rx_valid_group(builder(val + i * 0.1) for i in range(MAX_SAMPLE_VALS) for builder in builders)
          for refresh in refreshers:
            refresh()

        samples()
        yield samples

    for val in np.arange(0, 80, 0.5):
      builders = [self._speed_msg]
      secondary = getattr(self, '_speed_msg_2', None)
      if secondary is not None and secondary(val) is not None:
        builders.append(secondary)
      # Preserve the original 0.1 m/s parser tolerance. Adjacent CAN commands
      # bracket its possible consumed limits; no test cache is a DUT reading.
      bounds = [limit(max(val + offset, 0)) for offset in (-0.1, 0, 0.1)]
      accepted, rejected = min(bounds), max(bounds) + 1
      for sign in (-1, 1):
        if torque:
          self._set_controls_allowed(False)
          self.assertTrue(self._tx(self._torque_cmd_msg(0)))
          if not periodic_speed:
            self._set_controls_allowed(True)
          with window(val, builders) as samples:
            current = 0
            frames = []
            if periodic_speed:
              samples()
            while current < accepted:
              current = min(current + self.MAX_RATE_UP, accepted)
              if periodic_speed:
                frames.append(self._raw_frame(self._torque_cmd_msg(sign * current)))
                continue
              samples()
              if self.backend is None:
                self._advance_torque_time()
              else:
                self._wait(max(0.01, (RT_INTERVAL + 1) / 1_000_000 * self.MAX_RATE_UP / (0.8 * self.MAX_RT_DELTA)))
              self.assertTrue(self._tx(self._torque_cmd_msg(sign * current)), (val, sign, current))
            if frames:
              cadence = max(0.01, (RT_INTERVAL + 1) / 1_000_000 * self.MAX_RATE_UP / (0.8 * self.MAX_RT_DELTA))
              cadence_us = int(np.ceil(cadence * 1_000_000))
              # accepted_ramp dispatches immediately; retain the first step's wait.
              self._wait(cadence_us / 1_000_000)
              result = self.backend.accepted_ramp(frames, cadence_us)
              self.assertEqual(result['verdicts'], [True] * len(frames))
            samples()
            self._advance_torque_time()
            self.assertTrue(self._tx(self._torque_cmd_msg(sign * accepted)))
            # Rejection must isolate the absolute limit, not a rate/RT limit.
            for command in range(accepted + 1, rejected + 1):
              samples()
              self._advance_torque_time()
              if not self._tx(self._torque_cmd_msg(sign * command)):
                break
            else:
              self.fail(f'No consumed-speed torque boundary at {val=} {sign=}')
        else:
          for allowed, delta in ((True, accepted), (False, rejected)):
            self._set_controls_allowed(True)
            if curvature:
              self.assertTrue(self._tx(self._curvature_cmd_msg(0, True)))
              command = sign * delta / self.CURVATURE_TO_CAN
            else:
              anchor = sign * 10 if hasattr(self, 'ANGLE_RATE_BP') else 0
              self._set_prev_desired_angle(anchor)
              command = anchor + sign * delta / self.DEG_TO_CAN
            with window(val, builders):
              packet = self._curvature_cmd_msg(command, True) if curvature else self._angle_cmd_msg(command, True)
              self.assertEqual(allowed, bool(self._tx(packet)), (val, sign, delta))
              if curvature and allowed:
                self.assertTrue(self._tx(self._curvature_cmd_msg(0, True)))
      self._reset_safety_hooks()
      self._set_controls_allowed(True)
      # Narrow reset observation: neutral TX after normal RX warmup. This does
      # not establish immediately empty sample_t or replace host reset checks.
      self.assertTrue(self._tx(neutral()))

class AngleSteeringSafetyTest(VehicleSpeedSafetyTest):

  STEER_ANGLE_MAX: float = 300
  DEG_TO_CAN: float
  ANGLE_RATE_BP: list[float]
  ANGLE_RATE_UP: list[float]  # windup limit
  ANGLE_RATE_DOWN: list[float]  # unwind limit

  # Real time limits
  LATERAL_FREQUENCY: int = -1  # Hz

  @abc.abstractmethod
  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    pass

  @abc.abstractmethod
  def _angle_meas_msg(self, angle: float):
    pass

  def _get_steer_cmd_angle_max(self, speed):
    return self.STEER_ANGLE_MAX

  def _set_prev_desired_angle(self, t):
    self._reset_angle_measurement(t)
    self.assertTrue(self._tx(self._angle_cmd_msg(t, False)))

  def _reset_angle_measurement(self, angle):
    self._rx_valid_group(self._angle_meas_msg(angle) for _ in range(MAX_SAMPLE_VALS))

  def _reset_speed_measurement(self, speed):
    self._rx_valid_group(self._speed_msg(speed) for _ in range(MAX_SAMPLE_VALS))

  def _inactive_angle_probe_msg(self, angle):
    import inspect
    if 'increment_timer' in inspect.signature(self._angle_cmd_msg).parameters:
      return self._angle_cmd_msg(angle, False, increment_timer=False)
    return self._angle_cmd_msg(angle, False)

  @coverage('Inactive-angle boundary probes and neutral reset after warmup',
            host=('Exact numeric angle extrema and immediate reset before warmup',))
  def test_steering_angle_measurements(self):
    if self.backend is not None:
      margin = max(1, 3 / self.DEG_TO_CAN)
      for val in np.arange(-self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX, 0.5):
        first = self._angle_meas_msg(val)
        with hold_measurement_stream(self, first) as refresh:
          for i in range(MAX_SAMPLE_VALS):
            packet = first if i == 0 else self._angle_meas_msg(val + i * 0.1)
            refresh()
            self.assertTrue(self._rx(packet))
          # Inactive commands expose both endpoints before periodic eviction.
          for command, allowed in ((val, True), (min(val + 0.5, self.STEER_ANGLE_MAX), True),
                                   (val - margin, False), (val + 0.5 + margin, False)):
            self.assertEqual(bool(self._tx(self._inactive_angle_probe_msg(command))), allowed, (val, command))
        self._reset_safety_hooks()
        self.assertTrue(self._tx(self._inactive_angle_probe_msg(0)))
        self.assertFalse(self._tx(self._inactive_angle_probe_msg(margin)))
      return
    self._common_measurement_test(self._angle_meas_msg, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX, self.DEG_TO_CAN,
                                  self.safety.get_angle_meas_min, self.safety.get_angle_meas_max)

  def test_angle_cmd_when_enabled(self):
    # when controls are allowed, angle cmd rate limit is enforced
    speeds = [0., 1., 5., 10., 15., 50.]
    angles = np.concatenate((np.arange(-self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX, 5), [0, self.STEER_ANGLE_MAX]))
    for a in angles:
      for s in speeds:
        max_delta_up = np.interp(s, self.ANGLE_RATE_BP, self.ANGLE_RATE_UP)
        max_delta_down = np.interp(s, self.ANGLE_RATE_BP, self.ANGLE_RATE_DOWN)

        # first test against false positives
        self._reset_angle_measurement(a)
        self._reset_speed_measurement(s)

        self._set_prev_desired_angle(a)
        self._set_controls_allowed(1)

        # Stay within limits
        # Up
        self.assertTrue(self._tx(self._angle_cmd_msg(a + sign_of(a) * max_delta_up, True)))
        self.assertTrue(self._controls_allowed())

        # Don't change
        self.assertTrue(self._tx(self._angle_cmd_msg(a, True)))
        self.assertTrue(self._controls_allowed())

        # Down
        self.assertTrue(self._tx(self._angle_cmd_msg(a - sign_of(a) * max_delta_down, True)))
        self.assertTrue(self._controls_allowed())

        # Inject too high rates
        # Up
        self.assertFalse(self._tx(self._angle_cmd_msg(a + sign_of(a) * (max_delta_up + 1.1), True)))

        # Don't change
        self._set_controls_allowed(1)
        self._set_prev_desired_angle(a)
        self.assertTrue(self._controls_allowed())
        self.assertTrue(self._tx(self._angle_cmd_msg(a, True)))
        self.assertTrue(self._controls_allowed())

        # Down
        self.assertFalse(self._tx(self._angle_cmd_msg(a - sign_of(a) * (max_delta_down + 1.1), True)))

        # Check desired steer should be the same as steer angle when controls are off
        self._set_controls_allowed(0)
        self.assertTrue(self._tx(self._angle_cmd_msg(a, False)))

  def test_inactive_angle_absolute_limit(self):
    for enabled in (False, True):
      self._set_controls_allowed(enabled)
      for sign in (-1, 1):
        boundary = sign * self.STEER_ANGLE_MAX
        self._reset_angle_measurement(boundary)
        self.assertTrue(self._tx(self._angle_cmd_msg(boundary, False)))
        self._reset_angle_measurement(boundary + sign * 5)
        self.assertFalse(self._tx(self._angle_cmd_msg(boundary + sign * 5, False)))

  def test_angle_cmd_when_disabled(self):
    # Tests that only angles close to the meas are allowed while
    # steer actuation bit is 0, regardless of controls allowed.
    for controls_allowed in (True, False):
      self._set_controls_allowed(controls_allowed)

      for steer_control_enabled in (True, False):
        for angle_meas in np.arange(-90, 91, 10):
          self._reset_angle_measurement(angle_meas)

          for angle_cmd in np.arange(-90, 91, 10):
            self._set_prev_desired_angle(angle_cmd)
            self._reset_angle_measurement(angle_meas)

            # controls_allowed is checked if actuation bit is 1, else the angle must be close to meas (inactive)
            should_tx = controls_allowed if steer_control_enabled else angle_cmd == angle_meas
            self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(angle_cmd, steer_control_enabled)))

  def test_angle_violation(self):
    # If violation occurs, angle cmd is blocked until reset to 0. Matches behavior of torque safety modes
    self._set_controls_allowed(True)

    for speed in (0., 1., 5., 10., 15., 50.):
      self._reset_angle_measurement(0)
      self._set_prev_desired_angle(0)
      self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
      self._reset_speed_measurement(speed)

      for _ in range(20):
        self.assertFalse(self._tx(self._angle_cmd_msg(self._get_steer_cmd_angle_max(speed), True)))
      self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

  @coverage('Scheduled command burst and recovery with bounded timing',
            host=('Exact MCU clock edges and zero-time command bursts',))
  def test_rt_limits(self):
    # TODO: remove and check all safety modes
    if self.LATERAL_FREQUENCY == -1:
      raise unittest.SkipTest("No real time limits")

    if self.backend is not None:
      # Coarse frequency behavior only. The original one-microsecond edge and
      # zero-time burst below remain an explicit deterministic host supplement.
      def run_sequence(packets, budget):
        max_lateness_us = 10_000
        events = [{'offset_us': i * 6000, 'frame': self._raw_frame(packet)} for i, packet in enumerate(packets)]
        self.assertEqual(len({tuple(event['frame']) for event in events}), len(events), 'Nonunique burst identity')
        result = self.backend.steering_sequence(events, max_lateness_us=max_lateness_us)
        timing = result['timing']
        self.assertIs(timing['complete'], True, timing)
        self.assertEqual(len(result['verdicts']), len(events), result)
        self.assertEqual(len(timing['events']), len(events), timing)
        if timing.get('physical') is False:
          from opendbc.safety.tests.simulated import SimulatedBackend
          self.assertIsInstance(self.backend, SimulatedBackend)
          self.assertEqual(timing['source'], 'simulated_virtual_time')
          elapsed = timing['quiet_end_us'] - timing['start_us']
          self.assertEqual(timing['events'], events)
        else:
          self.assertLessEqual(timing['max_jitter_ns'], max_lateness_us * 1000, timing)
          origin = timing['events'][0]['send_start_ns']
          for planned, actual in zip(events, timing['events'], strict=True):
            self.assertEqual(actual['offset_us'], planned['offset_us'], timing)
            self.assertEqual(tuple(actual['frame']), tuple(planned['frame']), timing)
            deadline = timing['start_ns'] + planned['offset_us'] * 1000
            self.assertGreaterEqual(actual['send_start_ns'], deadline, timing)
            self.assertGreaterEqual(actual['send_end_ns'], actual['send_start_ns'], timing)
            self.assertLessEqual(actual['send_end_ns'] - deadline, max_lateness_us * 1000, timing)
          elapsed = (timing['receipts_observed_ns'] - origin) / 1000
        self.assertLess(elapsed, budget, timing)
        return result['verdicts']

      self._reset_safety_hooks()
      self._set_controls_allowed(True)
      self._reset_speed_measurement(0)
      self._reset_angle_measurement(0)
      max_rt_msgs = int(self.LATERAL_FREQUENCY * RT_INTERVAL / 1e6 * 1.2 + 1)
      angles = [i / self.DEG_TO_CAN for i in range(1, 2 * max_rt_msgs + 1)]

      def command(angle, enabled=True):
        return self._angle_cmd_msg(angle, enabled, increment_timer=False)

      # Independently prove every varying frame is legitimate from neutral,
      # rather than allowing angle/rate checks to explain frequency rejection.
      for angle in angles:
        self.assertTrue(self._tx(command(0, False)))
        self._wait((RT_INTERVAL + 50_000) / 1_000_000)
        self.assertTrue(self._tx(command(angle)))
      self.assertTrue(self._tx(command(0, False)))
      self._wait((RT_INTERVAL + 50_000) / 1_000_000)
      # Anchor executes after a broad wait: current check passes, THEN C resets
      # the count. Put it in the same batch to avoid an unbounded RPC gap.
      verdicts = run_sequence([command(0)] + [command(angle) for angle in angles], RT_INTERVAL)
      self.assertEqual(verdicts, [True] + [i <= max_rt_msgs for i in range(len(angles))])
      self._wait((RT_INTERVAL + 50_000) / 1_000_000)
      # C checks the overflowing count before resetting it on this transition.
      self.assertFalse(self._tx(command(0)))
      self.assertTrue(self._tx(command(angles[0])))
      self.assertTrue(self._tx(command(angles[1])))
      return

    # Angle safety enforces real time limits by checking the message send frequency in a 250ms time window
    self.safety.set_timer(0)
    self._set_controls_allowed(True)
    max_rt_msgs = int(self.LATERAL_FREQUENCY * RT_INTERVAL / 1e6 * 1.2 + 1)  # 1.2x buffer

    for i in range(max_rt_msgs * 2):
      should_tx = i <= max_rt_msgs
      self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))

    # One under RT interval should do nothing
    self.safety.set_timer(RT_INTERVAL - 1)
    for _ in range(5):
      self.assertFalse(self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))

    # Increment timer and send 1 message to reset RT window
    self.safety.set_timer(RT_INTERVAL)
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))
    for _ in range(5):
      self.assertTrue(self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))


class CurvatureSteeringSafetyTest(VehicleSpeedSafetyTest):
  MAX_CURVATURE: float
  MAX_CURVATURE_TEST: float
  CURVATURE_TO_CAN: float
  SEND_RATE: float

  @abc.abstractmethod
  def _curvature_cmd_msg(self, curvature: float, steer_req: bool):
    pass

  @abc.abstractmethod
  def _curvature_meas_msg(self, curvature: float):
    pass

  @coverage('Accepted CAN curvature setup',
            host=('Private desired-curvature equality after every setup ramp step',))
  def _set_prev_desired_curvature(self, curvature: float):
    enabled = self._controls_allowed()
    self._set_controls_allowed(True)
    if self.backend is not None:
      # A zero command may first violate the previous rate boundary. Establish
      # zero through a subsequent accepted command, without reading private state
      # or assuming the fixture's default steer power is valid while inactive.
      self._tx(self._curvature_cmd_msg(0, True))
      self._wait((RT_INTERVAL + 1) / 1_000_000)
      self.assertTrue(self._tx(self._curvature_cmd_msg(0, True)))
      self._reset_curvature_measurement(curvature)
      current = 0  # planning only, never firmware-state evidence
      speed = getattr(self, '_issued_speed', 0)
    else:
      self._reset_curvature_measurement(curvature)
      speed = self.safety.get_vehicle_speed_min()
      current = self.safety.get_desired_curvature_last()
    speed = max(speed - 1, 1)
    delta = max(1, int(MAX_LATERAL_JERK / speed**2 * self.SEND_RATE * self.CURVATURE_TO_CAN))
    target = round(curvature * self.CURVATURE_TO_CAN)
    for _ in range(math.ceil(abs(target - current) / delta)):
      current += int(np.clip(target - current, -delta, delta))
      self._wait((RT_INTERVAL + 1) / 1_000_000)
      self.assertTrue(self._tx(self._curvature_cmd_msg(current / self.CURVATURE_TO_CAN, True)))
      if self.backend is None:
        self.assertEqual(current, self.safety.get_desired_curvature_last())
    self._set_controls_allowed(enabled)

  def _reset_curvature_measurement(self, curvature: float):
    for _ in range(MAX_SAMPLE_VALS):
      self.assertTrue(self._rx(self._curvature_meas_msg(curvature)))

  def _reset_speed_measurement(self, speed: float):
    self._issued_speed = speed
    self._rx_valid_group(builder(speed) for _ in range(MAX_SAMPLE_VALS) for builder in (self._speed_msg, self._speed_msg_2))

  @coverage('No backend measurement-window replacement implemented',
            host=('Exact curvature extrema and immediate reset',), relation='host-only')
  def test_curvature_measurements(self):
    if self.backend is not None:
      raise AssertionError("Curvature sample-window probes require a fixture-specific measurement-error boundary; a cached RX value is not evidence")
    self._common_measurement_test(self._curvature_meas_msg, -self.MAX_CURVATURE, self.MAX_CURVATURE, self.CURVATURE_TO_CAN,
                                  self.safety.get_curvature_meas_min, self.safety.get_curvature_meas_max)

  def _check_curvature_limit(self, speed, curvature, delta):
    self._reset_speed_measurement(speed)
    self._set_controls_allowed(True)
    self._set_prev_desired_curvature(curvature)
    for command in (curvature, curvature - delta, curvature):
      self.assertTrue(self._tx(self._curvature_cmd_msg(command, True)), f"{speed} {curvature} {delta}")
      self.assertTrue(self._controls_allowed())
    self.assertFalse(self._tx(self._curvature_cmd_msg(curvature + delta, True)), f"{speed} {curvature} {delta}")

  @coverage('Curvature command limit probes',
            host=(),
            uses=(_set_prev_desired_curvature,))
  def test_curvature_limit(self):
    v = 1
    for sign in (1, -1):
      max_curvature = self.MAX_CURVATURE_TEST * sign
      max_curvature_rate = MAX_LATERAL_JERK / v**2
      max_curvature_delta = max_curvature_rate * self.SEND_RATE * sign

      self._check_curvature_limit(v, max_curvature, max_curvature_delta)

  @coverage('ISO lateral-acceleration command limit probes',
            host=(),
            uses=(_set_prev_desired_curvature,))
  def test_iso_accel_limit(self):
    speeds = [2., 5., 10., 15., 50.]
    for v in speeds:
      for sign in (1, -1):
        max_curvature = np.clip(((MAX_LATERAL_ACCEL / (v - 1)**2) * sign), -self.MAX_CURVATURE_TEST, self.MAX_CURVATURE_TEST)
        max_curvature_rate = MAX_LATERAL_JERK / (v - 1)**2
        max_curvature_delta = max_curvature_rate * self.SEND_RATE * sign

        self._check_curvature_limit(v, max_curvature, max_curvature_delta)

  @coverage('ISO lateral-jerk command limit probes',
            host=(),
            uses=(_set_prev_desired_curvature,))
  def test_iso_jerk_limit(self):
    speeds = [2., 5., 10., 15., 50.]
    for v in speeds:
      max_curvature_rate = MAX_LATERAL_JERK / (v - 1)**2
      max_curvature_delta = max_curvature_rate * self.SEND_RATE

      self._reset_speed_measurement(v)
      self._set_controls_allowed(True)
      self._set_prev_desired_curvature(max_curvature_delta)

      self.assertTrue(self._tx(self._curvature_cmd_msg(max_curvature_delta, True)))
      self.assertTrue(self._controls_allowed())

      self.assertTrue(self._tx(self._curvature_cmd_msg(0, True)))
      self.assertTrue(self._controls_allowed())

      self.assertTrue(self._tx(self._curvature_cmd_msg(-max_curvature_delta, True)))
      self.assertTrue(self._controls_allowed())

      self.assertFalse(self._tx(self._curvature_cmd_msg(max_curvature_delta, True)))

      # after violation, prev is reset to 0, going past the jerk limit must fail
      self._set_controls_allowed(True)
      self.assertFalse(self._tx(self._curvature_cmd_msg(2 * max_curvature_delta, True)))


class SafetyTest(SafetyTestBase):
  TX_MSGS: list[list[int]] = []
  SCANNED_ADDRS = [*range(0x800),                      # Entire 11-bit CAN address space
                   *range(0x18DA00F1, 0x18DB00F1, 0x100),   # 29-bit UDS physical addressing
                   *range(0x18DB00F1, 0x18DC00F1, 0x100),   # 29-bit UDS functional addressing
                   *range(0x3300, 0x3400)]                  # Honda
  FWD_BLACKLISTED_ADDRS: dict[int, list[int]] = {}  # {bus: [addr]}
  FWD_BUS_LOOKUP: dict[int, int] = {0: 2, 2: 0}

  # ***** standard tests for all safety modes *****

  def test_tx_msg_in_scanned_range(self):
    # the relay malfunction, fwd hook, and spam can tests don't exhaustively
    # scan the entire 29-bit address space, only some known important ranges
    # make sure SCANNED_ADDRS stays up to date with car port TX_MSGS; new
    # model ports should expand the range if needed
    for msg in self.TX_MSGS:
      self.assertTrue(msg[0] in self.SCANNED_ADDRS, f"{msg[0]=:#x}")

  def test_fwd_hook(self):
    if self.backend is not None:
      from opendbc.safety.tests.fixture_streams import suspend_required_streams
      self._scan_coverage = {'operation': 'forward', 'physical_buses': [0, 1, 2],
                             'addresses_per_bus': len(self.SCANNED_ADDRS), 'forward_cases_verified': 0, 'host_supplements': []}

      def clean_reset():
        self._reset_relay_test()
        suspend_required_streams(self)

      def scan(cases):
        if cases:
          self.assertIs(self.backend.forward_scan(cases), True)
          self._scan_coverage['forward_cases_verified'] += len(cases)

      clean_reset()
      pending = []
      relay_addresses = getattr(self, 'RELAY_MALFUNCTION_ADDRS', None) or {}
      for bus in range(3):
        for addr in self.SCANNED_ADDRS:
          target = self.FWD_BUS_LOOKUP.get(bus, -1)
          if addr in self.FWD_BLACKLISTED_ADDRS.get(bus, ()):
            target = -1
          pending.append((self._raw_frame(make_msg(bus, addr, 8)), None if target == -1 else target))
          if addr in relay_addresses.get(bus, ()):
            # This final frame forwards BEFORE RX latches its relay fault. No
            # later frame may inherit that fault while testing normal routing.
            scan(pending)
            pending = []
            clean_reset()
      scan(pending)
      self.assertEqual(self._scan_coverage['forward_cases_verified'], 3 * len(self.SCANNED_ADDRS))
      return
    # some safety modes don't forward anything, while others blacklist msgs
    for bus in range(3):
      for addr in self.SCANNED_ADDRS:
        # assume len 8
        fwd_bus = self.FWD_BUS_LOOKUP.get(bus, -1)
        if bus in self.FWD_BLACKLISTED_ADDRS and addr in self.FWD_BLACKLISTED_ADDRS[bus]:
          fwd_bus = -1
        # Physical forwarding precedes RX. A stock ECU address can therefore
        # forward this frame and then latch a relay fault for subsequent frames.
        self._assert_forward(bus, addr, fwd_bus, healthy=self.backend is None)
        if self.backend is not None and addr in getattr(self, 'RELAY_MALFUNCTION_ADDRS', {}).get(bus, ()):
          self._reset_backend()

  @coverage('Negative TX scan on physical buses 0..2',
            host=('Original logical-bus-3 scan domain',))
  def test_spam_can_buses(self):
    if self.backend is not None:
      from opendbc.safety.tests.fixture_streams import suspend_required_streams
      self._scan_coverage = {
        'operation': 'negative_tx', 'physical_buses': [0, 1, 2], 'addresses_per_bus': len(self.SCANNED_ADDRS),
        'host_supplements': [{'requirement': 'logical_bus_3', 'reason': 'Stock physical CAN exposes buses 0..2; legacy range(4) retained'}],
      }
      self._reset_relay_test()
      suspend_required_streams(self)
      tx_msgs = {(addr, bus) for addr, bus in self.TX_MSGS}
      frames = [self._raw_frame(make_msg(bus, addr, 8)) for bus in range(3) for addr in self.SCANNED_ADDRS
                if (addr, bus) not in tx_msgs]
      verdicts = self.backend.tx_scan(frames)
      self.assertEqual(len(verdicts), len(frames), 'Missing exhaustive negative TX verdicts')
      self.assertTrue(all(type(verdict) is bool for verdict in verdicts), 'TX scan must return explicit bool verdicts')
      self.assertEqual(verdicts, [False] * len(frames))
      self._scan_coverage['negative_tx_verified'] = len(frames)
      return
    tx_msgs = {(addr, bus) for addr, bus in self.TX_MSGS}
    for bus in range(4):
      for addr in self.SCANNED_ADDRS:
        if (addr, bus) not in tx_msgs:
          self.assertFalse(self._tx(make_msg(bus, addr, 8)), f"allowed TX {addr=} {bus=}")

  def test_default_controls_not_allowed(self):
    self.assertFalse(self._controls_allowed())

  @coverage('Other safety modes\' command rejection on physical buses 0..2',
            host=('Original logical-bus-3 wrong-mode TX candidates',))
  def test_tx_hook_on_wrong_safety_mode(self):
    files = os.listdir(os.path.dirname(os.path.realpath(__file__)))
    test_files = [f for f in files if f.startswith("test_") and f.endswith(".py")]

    current_test = self.__class__.__name__

    all_tx = []
    for tf in test_files:
      test = importlib.import_module("opendbc.safety.tests."+tf[:-3])
      for attr in dir(test):
        tc = getattr(test, attr)
        if isinstance(tc, type) and issubclass(tc, SafetyTest) and attr != current_test:
          tx = tc.TX_MSGS
          if tx is not None and not attr.endswith('Base'):
            # No point in comparing different Tesla safety modes
            if 'Tesla' in attr and 'Tesla' in current_test:
              continue
            # No point in comparing to ALLOUTPUT which allows all messages
            if attr.startswith('TestAllOutput'):
              continue
            if attr.startswith('TestToyota') and current_test.startswith('TestToyota'):
              continue
            if attr.startswith('TestSubaruGen') and current_test.startswith('TestSubaruGen'):
              continue
            if attr.startswith('TestSubaruPreglobal') and current_test.startswith('TestSubaruPreglobal'):
              continue
            if {attr, current_test}.issubset({'TestVolkswagenPqSafety', 'TestVolkswagenPqStockSafety', 'TestVolkswagenPqLongSafety'}):
              continue
            if {attr, current_test}.issubset({'TestGmCameraSafety', 'TestGmCameraLongitudinalSafety', 'TestGmAscmSafety',
                                              'TestGmCameraEVSafety', 'TestGmCameraLongitudinalEVSafety', 'TestGmAscmEVSafety'}):
              continue
            if attr.startswith('TestFord') and current_test.startswith('TestFord'):
              continue
            if attr.startswith('TestHyundaiCanfd') and current_test.startswith('TestHyundaiCanfd'):
              continue
            if {attr, current_test}.issubset({'TestHyundaiLongitudinalSafety', 'TestHyundaiLongitudinalSafetyCameraSCC', 'TestHyundaiSafetyFCEVLong'}):
              continue
            volkswagen_shared = ('TestVolkswagenMqb', 'TestVolkswagenMlb', 'TestVolkswagenMeb')
            if attr.startswith(volkswagen_shared) and current_test.startswith(volkswagen_shared):
              continue

            # overlapping TX addrs, but they're not actuating messages for either car
            if attr == 'TestHyundaiCanfdLKASteeringLongEV' and current_test.startswith('TestToyota'):
              tx = list(filter(lambda m: m[0] not in [0x160, ], tx))

            # Volkswagen MQB longitudinal actuating message overlaps with the Subaru lateral actuating message
            if attr == 'TestVolkswagenMqbLongSafety' and current_test.startswith('TestSubaru'):
              tx = list(filter(lambda m: m[0] not in [0x122, ], tx))

            # Volkswagen MQB and Honda Nidec ACC HUD messages overlap
            if attr == 'TestVolkswagenMqbLongSafety' and current_test.startswith('TestHondaNidec'):
              tx = list(filter(lambda m: m[0] not in [0x30c, ], tx))

            # Volkswagen MQB and Honda Bosch Radarless ACC HUD messages overlap
            if attr == 'TestVolkswagenMqbLongSafety' and current_test.startswith('TestHondaBoschRadarless'):
              tx = list(filter(lambda m: m[0] not in [0x30c, ], tx))

            # TODO: Temporary, should be fixed in panda firmware, safety_honda.h
            if attr.startswith('TestHonda'):
              # exceptions for common msgs across different hondas
              tx = list(filter(lambda m: m[0] not in [0x1FA, 0x30C, 0x33D, 0x33DB], tx))

            if attr.startswith('TestHyundaiLongitudinal'):
              # exceptions for common msgs across different Hyundai CAN platforms
              tx = list(filter(lambda m: m[0] not in [0x420, 0x50A, 0x389, 0x4A2], tx))
            all_tx.append([[m[0], m[1], attr] for m in tx])

    # make sure we got all the msgs
    self.assertTrue(len(all_tx) >= len(test_files)-1)

    for tx_msgs in all_tx:
      for addr, bus, test_name in tx_msgs:
        if self.backend is not None and bus not in (0, 1, 2):
          # The original host sweep retains the logical fourth bus.
          continue
        msg = make_msg(bus, addr)
        if self.SAFETY_MODEL not in (CarParams.SafetyModel.noOutput, CarParams.SafetyModel.silent):
          self._set_controls_allowed(1)
        # TODO: this should be blocked
        if current_test in ["TestNissanSafety", "TestNissanSafetyAltEpsBus", "TestNissanLeafSafety"] and [addr, bus] in self.TX_MSGS:
          continue
        self.assertFalse(self._tx(msg), f"transmit of {addr=:#x} {bus=} from {test_name} during {current_test} was allowed")


@add_regen_tests
class CarSafetyTest(SafetyTest):
  STANDSTILL_THRESHOLD: float = 0.0
  GAS_PRESSED_THRESHOLD = 0
  RELAY_MALFUNCTION_ADDRS: dict[int, tuple[int, ...]] | None = None

  @abc.abstractmethod
  def _user_brake_msg(self, brake):
    pass

  def _user_regen_msg(self, regen):
    pass

  @abc.abstractmethod
  def _speed_msg(self, speed):
    pass

  @abc.abstractmethod
  def _speed_msg_2(self, speed: float):
    pass

  # Safety modes can override if vehicle_moving is driven by a different message
  def _vehicle_moving_msg(self, speed: float):
    return self._speed_msg(speed)

  @abc.abstractmethod
  def _user_gas_msg(self, gas):
    pass

  @abc.abstractmethod
  def _pcm_status_msg(self, enable):
    pass

  def _engage(self):
    self.assertTrue(self._rx(self._pcm_status_msg(False)))
    self.assertTrue(self._rx(self._pcm_status_msg(True)))

  def _disengage(self):
    self.assertTrue(self._rx(self._pcm_status_msg(False)))

  # ***** standard tests for all car-specific safety modes *****

  @coverage('Public relay fault and exhaustive CAN protection scans',
            host=('Exact private relay-latch identity',))
  def test_relay_malfunction(self):
    if self.backend is not None:
      from opendbc.safety.tests.fixture_streams import suspend_required_streams
      self._scan_coverage = {'operation': 'relay', 'physical_buses': [0, 1, 2],
                             'addresses_per_bus': len(self.SCANNED_ADDRS),
                             'host_supplements': [{'requirement': 'private_relay_latch_identity',
                                                   'reason': 'Public faults plus exhaustive TX/forward block are physical evidence'}]}

      def clean_reset():
        self._reset_relay_test()
        suspend_required_streams(self)

      candidates = [(bus, addr) for bus in range(3) for addr in self.SCANNED_ADDRS]
      positives = [(bus, addr) for bus, addr in candidates if addr in self.RELAY_MALFUNCTION_ADDRS.get(bus, ())]
      negatives = [self._raw_frame(make_msg(bus, addr, 8)) for bus, addr in candidates
                   if addr not in self.RELAY_MALFUNCTION_ADDRS.get(bus, ())]
      clean_reset()
      # Relay checks depend only on static address/bus configuration and latch
      # until reset. A final clear fault therefore excludes every negative input,
      # even though checksums/counters/watchdogs are deliberately invalid here.
      health = self.backend.rx_scan(negatives)
      self.assertIn('faults', health, 'RX scan lacks public final fault health')
      self.assertFalse(health['faults'], health)
      self._wait(.15)  # Allow the final candidate's public 8Hz fault publication.
      self.assertFalse(self._relay_fault_observed())
      for bus, addr in positives:
        clean_reset()
        self._rx(make_msg(bus, addr, 8))
        self._wait(.15)
        self.assertTrue(self._relay_fault_observed(), (bus, hex(addr)))

      # Fresh positive fault for the exhaustive protection scans, not sampling.
      self.assertTrue(positives, 'Car fixture must declare a relay-trigger address')
      clean_reset()
      bus, addr = positives[0]
      self._rx(make_msg(bus, addr, 8))
      self._wait(.15)
      self.assertTrue(self._relay_fault_observed())
      frames = [self._raw_frame(make_msg(bus, addr, 8)) for bus, addr in candidates]
      verdicts = self.backend.tx_scan(frames)
      self.assertEqual(len(verdicts), len(frames), 'Missing exhaustive relay TX verdicts')
      self.assertTrue(all(type(verdict) is bool for verdict in verdicts), 'TX scan must return explicit bool verdicts')
      self.assertEqual(verdicts, [False] * len(frames))
      self.assertIs(self.backend.forward_scan([(frame, None) for frame in frames]), True)
      self.assertTrue(self._relay_fault_observed())
      self._scan_coverage.update(negative_rx_candidates_verified=len(negatives), positive_candidates_verified=len(positives),
                                 protection_tx_verified=len(frames), protection_forward_verified=len(frames))
      return
    # each car has an addr that is used to detect relay malfunction
    # if that addr is seen on specified bus, triggers the relay malfunction
    # protection logic: both tx_hook and fwd_hook are expected to return failure
    self.assertFalse(self._relay_fault_observed())
    for bus in range(3):
      for addr in self.SCANNED_ADDRS:
        self._reset_relay_test()
        self._rx(make_msg(bus, addr, 8))
        if self.backend is not None:
          self._wait(.15)  # Stock firmware publishes relay faults at 8 Hz.
        should_relay_malfunction = addr in self.RELAY_MALFUNCTION_ADDRS.get(bus, ())
        self.assertEqual(should_relay_malfunction, self._relay_fault_observed(), (bus, hex(addr)))

    # test relay malfunction protection logic
    self._reset_relay_test()
    bus, addresses = next((bus, addresses) for bus, addresses in self.RELAY_MALFUNCTION_ADDRS.items() if addresses)
    self._rx(make_msg(bus, addresses[0], 8))
    if self.backend is not None:
      self._wait(.15)
    self.assertTrue(self._relay_fault_observed())
    for bus in range(3):
      for addr in self.SCANNED_ADDRS:
        self.assertFalse(self._tx(make_msg(bus, addr, 8), healthy=False))
        self._assert_forward(bus, addr, -1, healthy=False)

  @coverage('Public lateral and available longitudinal command probes',
            host=('Exact private longitudinal permission; no actuator probe in lateral-only modes',))
  def _assert_pedal_permissions(self, lateral, longitudinal):
    self.assertEqual(lateral, self._controls_allowed())
    if isinstance(self, LongitudinalGasBrakeSafetyTest):
      self.assertTrue(self._tx(self._send_gas_msg(self.INACTIVE_GAS)))
      self.assertTrue(self._tx(self._send_brake_msg(0)))
      self.assertEqual(longitudinal, bool(self._tx(self._send_brake_msg(self.MAX_BRAKE))))
    elif isinstance(self, LongitudinalAccelSafetyTest) and self.LONGITUDINAL:
      active = min(self.MAX_ACCEL, max(0.1, self.MAX_ACCEL / 2))
      self.assertNotEqual(active, self.INACTIVE_ACCEL)
      self.assertTrue(self._tx(self._accel_msg(self.INACTIVE_ACCEL)))
      self.assertEqual(longitudinal, bool(self._tx(self._accel_msg(active))))
    # Lateral-only modes have no downstream longitudinal command to probe.
    # Public lateral permission is checked above; the unconsumed private
    # longitudinal flag has no stock CAN equivalent. Its exact host assertion
    # below supplements that behavior without claiming hardware flag coverage.
    if self.backend is None:
      self.assertEqual(longitudinal, bool(self.safety.get_longitudinal_allowed()))

  @coverage('Gas-dependent pedal permission probes',
            host=('Exact previous-gas latch and initial state',),
            uses=(_assert_pedal_permissions,))
  def test_prev_gas(self):
    if self.backend is None:
      self.assertFalse(self.safety.get_gas_pressed_prev())
    self._set_controls_allowed(True)
    for pressed in [self.GAS_PRESSED_THRESHOLD + 1, 0]:
      self._rx(self._user_gas_msg(pressed))
      self._assert_pedal_permissions(True, not bool(pressed))
      if self.backend is None:
        self.assertEqual(bool(pressed), self.safety.get_gas_pressed_prev())

  def test_allow_engage_with_gas_pressed(self):
    self._rx(self._user_gas_msg(1))
    self._set_controls_allowed(True)
    self._rx(self._user_gas_msg(1))
    self.assertTrue(self._controls_allowed())
    self._rx(self._user_gas_msg(1))
    self.assertTrue(self._controls_allowed())

  @coverage('Gas longitudinal inhibition and recovery where actuation exists',
            host=(),
            uses=(_assert_pedal_permissions,))
  def test_no_disengage_on_gas(self):
    self._rx(self._user_gas_msg(0))
    self._set_controls_allowed(True)
    self._rx(self._user_gas_msg(self.GAS_PRESSED_THRESHOLD + 1))
    # Test we allow lateral, but not longitudinal
    self.assertTrue(self._controls_allowed())
    self._assert_pedal_permissions(self._controls_allowed(), False)
    # Make sure we can re-gain longitudinal actuation
    self._rx(self._user_gas_msg(0))
    self._assert_pedal_permissions(self._controls_allowed(), True)

  @coverage('Pedal disengagement and permission probes',
            host=('Exact previous-pedal latch and initial state',),
            uses=(_assert_pedal_permissions,))
  def test_prev_user_brake(self, _user_brake_msg=None, get_brake_pressed_prev=None):
    if _user_brake_msg is None:
      _user_brake_msg = self._user_brake_msg
      if self.backend is None:
        get_brake_pressed_prev = self.safety.get_brake_pressed_prev

    if self.backend is None:
      self.assertFalse(get_brake_pressed_prev())
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 1))
    for pressed in [True, False]:
      for state in [not pressed, pressed]:
        # Establish permission before each sample so rejection is attributable
        # to this brake/regen frame rather than a prior disengagement.
        self._rx(_user_brake_msg(False))
        self._set_controls_allowed(True)
        self._rx(_user_brake_msg(state))
        self._assert_pedal_permissions(not state, not state)
        if self.backend is None:
          self.assertEqual(state, get_brake_pressed_prev())

  def test_enable_control_allowed_from_cruise(self):
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self._controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self._controls_allowed())

  def test_disable_control_allowed_from_cruise(self):
    self._set_controls_allowed(1)
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self._controls_allowed())

  @coverage('Repeated-PCM inhibition and fresh-edge engagement',
            host=('Exact previous-cruise latch sequence',))
  def test_cruise_engaged_prev(self):
    # Preserve the exact original state sequence as a host supplement. A PCM
    # falling edge need not disable controls (Honda Nidec keeps braking).
    for engaged in [True, False]:
      self._rx(self._pcm_status_msg(engaged))
      if self.backend is None:
        self.assertEqual(engaged, self.safety.get_cruise_engaged_prev())
      self._rx(self._pcm_status_msg(not engaged))
      if self.backend is None:
        self.assertEqual(not engaged, self.safety.get_cruise_engaged_prev())

    # Disengage independently of PCM while leaving its last sample true. A
    # repeated true must preserve inhibition; only a fresh rising edge enables.
    self._rx(self._user_brake_msg(False))
    self._set_controls_allowed(True)
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self._controls_allowed())
    self._rx(self._user_brake_msg(False))
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self._controls_allowed(), "Repeated PCM true re-enabled controls")
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self._controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self._controls_allowed(), "PCM rising edge failed to enable controls")

  @coverage('Held-pedal and rising-edge behavior at standstill',
            host=(),
            uses=(_assert_pedal_permissions,))
  def test_allow_user_brake_at_zero_speed(self, _user_brake_msg=None, get_brake_pressed_prev=None):
    if _user_brake_msg is None:
      _user_brake_msg = self._user_brake_msg

    # Brake was already pressed
    self._rx(self._vehicle_moving_msg(0))
    self._rx(_user_brake_msg(1))
    self._set_controls_allowed(1)
    self._rx(_user_brake_msg(1))
    self.assertTrue(self._controls_allowed())
    self._assert_pedal_permissions(self._controls_allowed(), True)
    self._rx(_user_brake_msg(0))
    self.assertTrue(self._controls_allowed())
    self._assert_pedal_permissions(self._controls_allowed(), True)
    # rising edge of brake should disengage
    self._rx(_user_brake_msg(1))
    self.assertFalse(self._controls_allowed())
    self._assert_pedal_permissions(self._controls_allowed(), False)
    self._rx(_user_brake_msg(0))  # reset no brakes

  @coverage('Moving-pedal permission probes',
            host=(),
            uses=(_assert_pedal_permissions,))
  def test_not_allow_user_brake_when_moving(self, _user_brake_msg=None, get_brake_pressed_prev=None):
    if _user_brake_msg is None:
      _user_brake_msg = self._user_brake_msg

    # Brake was already pressed
    self._rx(_user_brake_msg(1))
    self._set_controls_allowed(1)
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD))
    self._rx(_user_brake_msg(1))
    self.assertTrue(self._controls_allowed())
    self._assert_pedal_permissions(self._controls_allowed(), True)
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 1))
    self._rx(_user_brake_msg(1))
    self.assertFalse(self._controls_allowed())
    self._assert_pedal_permissions(self._controls_allowed(), False)
    self._rx(self._vehicle_moving_msg(0))

  @coverage('Held-brake moving threshold behavior',
            host=('Exact vehicle-moving latch and initial state',),
            uses=(_assert_pedal_permissions,))
  def test_vehicle_moving(self):
    if self.backend is None:
      self.assertFalse(self.safety.get_vehicle_moving())

    for speed, moving in [(0, False), (self.STANDSTILL_THRESHOLD, False),
                          (self.STANDSTILL_THRESHOLD + 1, True)]:
      # A held brake permits engagement at standstill but inhibits permission
      # on the next brake frame when moving. Avoid a brake rising edge here.
      self._rx(self._vehicle_moving_msg(0))
      self._rx(self._user_brake_msg(True))
      self._set_controls_allowed(True)
      self._rx(self._vehicle_moving_msg(speed))
      self._rx(self._user_brake_msg(True))
      self._assert_pedal_permissions(not moving, not moving)
      if self.backend is None:
        self.assertEqual(moving, self.safety.get_vehicle_moving())

  def test_rx_hook_speed_mismatch(self):
    if self._speed_msg_2(0) is None:
      raise unittest.SkipTest("No second speed message for this safety mode")

    for speed in np.arange(0, 40, 0.5):
      for speed_delta in np.arange(-5, 5, 0.1):
        speed_2 = round(max(speed + speed_delta, 0), 1)
        # Set controls allowed in between rx since first message can reset it
        self._rx(self._speed_msg(speed))
        if self.backend is not None:
          # Periodic secondary traffic still carries the previous pair's fault.
          # Restore a matching baseline before engagement; inject this pair's
          # intended mismatch only after controls have been verified allowed.
          self.assertTrue(self._rx(self._speed_msg_2(speed)))
        self._set_controls_allowed(True)
        self._rx(self._speed_msg_2(speed_2))

        within_delta = speeds_within_delta(speed, speed_2)
        self.assertEqual(self._controls_allowed(), within_delta)

  @coverage('Natural watchdog inhibition and public health',
            host=('Synchronous safety_tick and config-valid assertions at synthetic timestamps',))
  def test_safety_tick(self):
    # Preserve the exact immediate tick assertion as a host supplement.
    for elapsed in (0, int(2e6)):
      with self.subTest(elapsed=elapsed):
        self._reset_safety_hooks()
        if self.backend is None:
          self.safety.set_timer(elapsed)
          self._set_controls_allowed(True)
          self.safety.safety_tick()
          self.assertFalse(self._controls_allowed())
          self.assertFalse(self.safety.safety_config_valid())
        else:
          # First prove scheduled traffic survives the same interval. A stale
          # or always-invalid health report must not satisfy the missing test.
          self._set_controls_allowed(True)
          self._wait(2.0)
          health = self.backend.health()
          self.assertIn("safety_rx_checks_invalid", health)
          self.assertFalse(health["safety_rx_checks_invalid"])
          self.assertTrue(self._controls_allowed())
          streams = self.required_streams()
          self.assertTrue(streams, "Watchdog test requires monitored RX streams")
          for stream in streams:
            self._drop_required_stream(stream["name"])
          self._wait(elapsed / 1e6)
          self._wait(2.0)
          self.assertFalse(self._controls_allowed())
          self.assertTrue(self.backend.health()["safety_rx_checks_invalid"])
