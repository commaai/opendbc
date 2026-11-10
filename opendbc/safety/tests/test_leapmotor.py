#!/usr/bin/env python3
"""Panda safety tests for opendbc/safety/modes/leapmotor.h: the LKA torque cap by speed (6 points, no speed fudge), rate up 15,
down 30, rt 225, driver torque limited, and engaging by double-tapping the right roller on the ICU side."""
import unittest

import numpy as np

from opendbc.car.leapmotor.values import CarControllerParams, LkaState
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py

VEHICLE_SPEED_FACTOR = 1000.  # safety/declarations.h

# leapmotor.h, microseconds
TAP_GAP_TIME = 30000
TAP_WINDOW_TIME = 1500000
TAP_FRAMES = 3     # a roller press is always 3 frames, however long it's held (commit 7c2e688)
TAP_STEP = 20000   # 20 ms between those frames


class TestLeapmotorSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  DBC = "leapmotor_t03_pt"
  SAFETY_MODEL = CarParams.SafetyModel.leapmotor

  TX_MSGS = [[0x365, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x365,)}
  FWD_BLACKLISTED_ADDRS = {2: [0x365]}

  MAX_RATE_UP = 15
  MAX_RATE_DOWN = 30
  # leapmotor_max_torque(): openpilot's STEER_MAX_LOOKUP + 10, at the lowest speed sample, at most 650
  PANDA_TORQUE_LOOKUP = ([9.722, 10.416, 11.111, 12.5, 15.277, 16.666, 18.055, 19.444, 20.833, 21.527],
                         [660, 603, 565, 540, 540, 513, 490, 470, 453, 443])
  MAX_TORQUE_LOOKUP = PANDA_TORQUE_LOOKUP[0], [650] + PANDA_TORQUE_LOOKUP[1][1:]
  DYNAMIC_MAX_TORQUE = True
  MAX_RT_DELTA = 225
  DRIVER_TORQUE_ALLOWANCE = 80
  DRIVER_TORQUE_FACTOR = 2

  def setUp(self):
    super().setUp()
    self.ts = 0

  def _get_max_torque(self, speed):
    # the speed as the panda has it: VEHICLE_SPEED's resolution (0.05625 km/h), then VEHICLE_SPEED_FACTOR, in float32
    raw = round(speed * 3.6 / 0.05625)
    v = np.float32(round(raw * 0.05625 / 3.6 * VEHICLE_SPEED_FACTOR)) / np.float32(VEHICLE_SPEED_FACTOR)
    x, y = (np.asarray(a, dtype=np.float32) for a in self.PANDA_TORQUE_LOOKUP)
    if v <= x[0]:
      torque = y[0]
    else:
      torque = y[-1]
      for i in range(1, len(x)):
        if v < x[i]:
          torque = y[i - 1] + ((v - x[i - 1]) * (y[i] - y[i - 1])) / (x[i] - x[i - 1])
          break
    return min(int(torque), 650)

  # openpilot is engaged by double-tapping the right roller, not by the stock ACC
  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def _pcm_status_msg(self, enable):
    return self.packer.make_can_msg_safety("STATE_MIRROR_1", 0, {"ACC_STATE": 5 if enable else 2})

  def _roller_msg(self, button="RIGHT_ROLLER_PRESS", bus=2):
    return self.packer.make_can_msg_safety("STEERING_BUTTONS", bus, {button: 1})

  def _wait(self, us):
    self.ts += us
    self.safety.set_timer(self.ts)

  def _tap(self, button="RIGHT_ROLLER_PRESS", bus=2, steps=(TAP_STEP,) * TAP_FRAMES):
    for step in steps:
      self._wait(step)
      self._rx(self._roller_msg(button, bus))

  def _double_tap(self, gap=200000, **kwargs):
    self._tap(**kwargs)
    self._wait(gap)
    self._tap(**kwargs)

  def _lka_cmd_msg(self, torque, lka_state=LkaState.ACTIVE, angle=0):
    values = {"LKA_STATE": lka_state, "LKA_TORQUE_REQ": torque, "STEER_ANGLE_REQ": angle}
    return self.packer.make_can_msg_safety("EPS_LKA_CMD", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    return self._lka_cmd_msg(torque, LkaState.ACTIVE if steer_req else LkaState.READY)

  def _torque_driver_msg(self, torque):
    values = {"STEER_TORQUE_DRIVER": torque + CarControllerParams.STEER_DRIVER_TORQUE_ZERO}
    return self.packer.make_can_msg_safety("STEER_1", 0, values)

  def _speed_msg(self, speed):
    return self.packer.make_can_msg_safety("VEHICLE_SPEED", 0, {"VEHICLE_SPEED": speed * 3.6})

  def _speed_msg_2(self, speed):
    values = {"WHEEL_SPEED_FL": speed * 3.6, "WHEEL_SPEED_FR": speed * 3.6}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS_FRONT", 0, values)

  def _user_brake_msg(self, brake):
    return self.packer.make_can_msg_safety("BRAKE_PEDAL", 0, {"BRAKE_PRESSED": brake})

  def _user_gas_msg(self, gas):
    return self.packer.make_can_msg_safety("ACCEL_PEDAL", 0, {"ACCEL_PEDAL_POS": gas})

  # ---- engagement by the roller ----
  def test_double_tap_engages(self):
    self._double_tap()
    self.assertTrue(self.safety.get_controls_allowed())

  def test_double_tap_while_allowed_does_not_cancel(self):
    # documents the behavior: the panda can't tell an engage from a cancel double tap, it only ever allows.
    # openpilot cancels, the panda keeps controls_allowed until the heartbeat mismatch (~3 s) clears it
    self._double_tap()
    self._wait(TAP_WINDOW_TIME + 1)
    self._double_tap()
    self.assertTrue(self.safety.get_controls_allowed())

  def test_single_tap_does_nothing(self):
    for _ in range(10):
      self._tap()
      self._wait(TAP_WINDOW_TIME + 1)
      self.assertFalse(self.safety.get_controls_allowed())

  def test_single_tap_with_a_late_frame(self):
    # one press whose 3rd frame comes 31 ms after the 2nd (bus jitter): the panda counts it as a second tap.
    # carstate.py needs > 3 idle 10 ms frames (>= 40 ms of silence) for a new tap, so openpilot would not engage.
    self._tap(steps=(TAP_STEP, TAP_STEP, TAP_GAP_TIME + 1000))
    self.assertTrue(self.safety.get_controls_allowed())  # documents the asymmetry (panda more permissive)

  def test_held_roller_is_one_tap(self):
    for _ in range(50):
      self._wait(TAP_STEP)
      self._rx(self._roller_msg())
    self.assertFalse(self.safety.get_controls_allowed())

  def test_double_tap_timing(self):
    for gap in (TAP_GAP_TIME + TAP_STEP, 180000, 200000, 240000, TAP_WINDOW_TIME - 4 * TAP_STEP):
      with self.subTest(gap=gap):
        self.safety.set_controls_allowed(False)
        self._wait(TAP_WINDOW_TIME + 1)
        self._double_tap(gap=gap)
        self.assertTrue(self.safety.get_controls_allowed())

  def test_other_buttons_and_car_bus_dont_engage(self):
    for button in ("LEFT_ROLLER_UP", "LEFT_ROLLER_DOWN", "LEFT_ROLLER_PRESS", "RIGHT_ROLLER_UP", "RIGHT_ROLLER_DOWN"):
      for _ in range(5):
        self._double_tap(button=button)
      self.assertFalse(self.safety.get_controls_allowed(), button)
    for _ in range(5):
      self._double_tap(bus=0)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_brake_while_moving_cancels_and_blocks_tap_engage(self):
    self._rx(self._speed_msg(10))
    self._double_tap()
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self._double_tap()  # brake still held while moving: generic_rx_checks clears it again on every message
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  # ---- the command ----
  def test_steer_safety_check(self):
    # unlike the common test, ACTIVE isn't allowed while disengaged even without torque
    for controls_allowed in (False, True):
      for t in range(-self.MAX_TORQUE * 2, self.MAX_TORQUE * 2, 7):
        self.safety.set_controls_allowed(controls_allowed)
        self._set_prev_torque(t)
        self._reset_speed_measurement(0)
        should_tx = controls_allowed and abs(t) <= self._get_max_torque(0)
        self.assertEqual(should_tx, self._tx(self._torque_cmd_msg(t)), (controls_allowed, t))

  def test_inactive_lka_states(self):
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for lka_state in (LkaState.OFF, LkaState.STANDBY, LkaState.READY):
        self.assertTrue(self._tx(self._lka_cmd_msg(0, lka_state)))
        self.assertFalse(self._tx(self._lka_cmd_msg(1, lka_state)))

  def test_angle_request_blocked(self):
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for angle in (-10, -0.1, 0.1, 10):
        self.assertFalse(self._tx(self._lka_cmd_msg(0, LkaState.READY, angle)))

  def test_limits_match_openpilot(self):
    p = CarControllerParams(None)
    self.assertEqual(p.STEER_MAX, self.MAX_TORQUE)
    self.assertEqual(p.STEER_DELTA_UP, self.MAX_RATE_UP)
    # the panda only demands at least MAX_RATE_DOWN toward zero when over the driver limit; openpilot may go faster
    self.assertGreaterEqual(p.STEER_DELTA_DOWN, self.MAX_RATE_DOWN)
    self.assertEqual(p.STEER_DRIVER_ALLOWANCE, self.DRIVER_TORQUE_ALLOWANCE)
    self.assertEqual(p.STEER_DRIVER_MULTIPLIER, self.DRIVER_TORQUE_FACTOR)
    self.assertEqual(p.STEER_DRIVER_FACTOR, 1)
    for v in np.arange(0., 40., 0.01):
      # openpilot's limit at v never exceeds the panda's at v (the panda takes its lowest recent speed: higher)
      self.assertLessEqual(p.steer_max(v), self._get_max_torque(v), v)
    # 12.5 frames of STEER_DELTA_UP per 250 ms real-time window must stay under max_rt_delta (with 1 frame of jitter)
    self.assertLessEqual(p.STEER_DELTA_UP * 14, self.MAX_RT_DELTA)


MAX_WRONG_COUNTERS = 5


class Bus:
  """realistic periodic traffic with correct counters, 10 ms ticks"""
  PERIODIC = {  # name: (period in 10 ms ticks, bus)
    "STEER_1": (1, 0), "VEHICLE_SPEED": (2, 0), "WHEEL_SPEEDS_FRONT": (2, 0), "BRAKE_PEDAL": (1, 0),
    "ACCEL_PEDAL": (2, 0), "STATE_MIRROR_1": (2, 0),
  }

  def __init__(self, tc, speed=10.):
    self.tc, self.speed = tc, speed
    self.counters = dict.fromkeys(self.PERIODIC, 0)
    self.tick = 0
    self.driver_torque = 0
    self.brake = 0

  def msg(self, name, counter=None, **extra):
    values = {"COUNTER": self.counters[name] if counter is None else counter}
    if name == "STEER_1":
      values["STEER_TORQUE_DRIVER"] = self.driver_torque + 10
    elif name == "VEHICLE_SPEED":
      values["VEHICLE_SPEED"] = self.speed * 3.6
    elif name == "WHEEL_SPEEDS_FRONT":
      values.update({"WHEEL_SPEED_FL": self.speed * 3.6, "WHEEL_SPEED_FR": self.speed * 3.6})
    elif name == "BRAKE_PEDAL":
      values["BRAKE_PRESSED"] = self.brake
    values.update(extra)
    return self.tc.packer.make_can_msg_safety(name, self.PERIODIC[name][1], values)

  def step(self, n=1):
    for _ in range(n):
      self.tick += 1
      self.tc.ts += 10000
      self.tc.safety.set_timer(self.tc.ts & 0xFFFFFFFF)
      for name, (period, _) in self.PERIODIC.items():
        if self.tick % period == 0:
          self.counters[name] = (self.counters[name] + 1) % 16
          self.tc.assertTrue(self.tc.safety.safety_rx_hook(self.msg(name)), name)


class TestLeapmotorBus(unittest.TestCase):
  """on a realistic bus: counters and checksums, tap timing with STEER_1 traffic, the cap at the lowest recent speed,
  safety_tick with the event-driven STEERING_BUTTONS, every raw torque value"""
  def setUp(self):
    self.packer = CANPackerSafety("leapmotor_t03_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.leapmotor, 0)
    self.safety.init_tests()
    self.ts = 0
    self.bus = Bus(self)
    self.bus.step(20)

  def roller(self):
    return self.packer.make_can_msg_safety("STEERING_BUTTONS", 2, {"RIGHT_ROLLER_PRESS": 1})

  def press(self):
    """one press: 3 frames 20 ms apart, with the periodic bus traffic in between"""
    for _ in range(3):
      self.safety.safety_rx_hook(self.roller())
      self.bus.step(2)

  def lka(self, torque, state=19, angle=0):
    return self.packer.make_can_msg_safety("EPS_LKA_CMD", 0, {"LKA_STATE": state, "LKA_TORQUE_REQ": torque, "STEER_ANGLE_REQ": angle})

  # ---- engagement on a realistic bus ----
  def test_double_tap_engages_with_bus_traffic(self):
    self.press()
    self.bus.step(15)  # ~200 ms between the presses
    self.press()
    self.assertTrue(self.safety.get_controls_allowed())

  def test_taps_further_apart_than_window_do_not_engage_with_bus_traffic(self):
    self.press()
    self.bus.step(160)  # 1.6 s > 1.5 s window, STEER_1 keeps coming
    self.press()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_old_tap_forgotten_before_timer_wraps(self):
    # tap, then the bus runs on past the window: STEER_1 forgets it; a tap ~2^32 us later must not pair with it
    self.press()
    self.bus.step(200)
    self.ts += (1 << 32) - 2_000_000 - 50_000  # lands 50 ms after the first tap, modulo the 32-bit timer
    self.safety.set_timer(self.ts & 0xFFFFFFFF)
    self.press()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_tap_resets_heartbeat_mismatches_characterization(self):
    # documents what the explicit reset in leapmotor.h does beyond safety_rx_hook's rising-edge reset:
    # a double tap while already allowed (e.g. the cancel double tap) restarts the ~3 s heartbeat timeout
    self.safety.set_controls_allowed(True)
    self.safety.set_heartbeat_engaged_mismatches(2)
    self.press()
    self.bus.step(15)
    self.press()
    self.assertEqual(0, self.safety.get_heartbeat_engaged_mismatches())

  # ---- rx checks: counters and checksums, from the DBC ----
  def _stuck_counter_invalidates(self, name):
    self.safety.set_controls_allowed(True)
    ok = [self.safety.safety_rx_hook(self.bus.msg(name, counter=3)) for _ in range(MAX_WRONG_COUNTERS + 2)]
    self.assertFalse(ok[-1], f"{name}: a stuck counter must make the message invalid")
    self.assertFalse(self.safety.get_controls_allowed(), name)

  def test_stuck_counter_steer_1(self):
    self._stuck_counter_invalidates("STEER_1")

  def test_stuck_counter_vehicle_speed(self):
    self._stuck_counter_invalidates("VEHICLE_SPEED")

  def test_stuck_counter_brake(self):
    self._stuck_counter_invalidates("BRAKE_PEDAL")

  def test_stuck_counter_accel_pedal(self):
    self._stuck_counter_invalidates("ACCEL_PEDAL")

  def test_stuck_counter_state_mirror_1(self):
    self._stuck_counter_invalidates("STATE_MIRROR_1")

  def test_stuck_counter_wheel_speeds(self):
    self._stuck_counter_invalidates("WHEEL_SPEEDS_FRONT")

  def test_bad_checksum_invalidates(self):
    for name in ("STEER_1", "VEHICLE_SPEED", "BRAKE_PEDAL", "WHEEL_SPEEDS_FRONT"):
      with self.subTest(name=name):
        self.safety.set_controls_allowed(True)
        self.bus.counters[name] = (self.bus.counters[name] + 1) % 16
        m = self.bus.msg(name)
        m[0].data[7] = (m[0].data[7] + 1) & 0xFF
        self.assertFalse(self.safety.safety_rx_hook(m), name)
        self.assertFalse(self.safety.get_controls_allowed(), name)
        self.bus.step(10)

  def test_counter_position_matches_dbc(self):
    # incrementing DBC counters keep every message valid for a long time (Bus.step asserts rx ok)
    self.safety.set_controls_allowed(True)
    self.bus.step(300)
    self.assertTrue(self.safety.get_controls_allowed())

  # ---- torque cap ----
  def test_cap_uses_lowest_recent_speed(self):
    self.safety.set_controls_allowed(True)
    self.bus.speed = 5.
    self.bus.step(12)   # 6 VEHICLE_SPEED samples at 5 m/s
    self.bus.speed = 25.
    self.bus.step(2)    # one sample at 25 m/s: the lowest recent speed is still 5 m/s -> cap 650
    self.safety.set_desired_torque_last(640)
    self.safety.set_rt_torque_last(640)
    self.assertTrue(self.safety.safety_tx_hook(self.lka(640)))
    self.bus.step(12)   # now only 25 m/s samples -> cap 310
    self.safety.set_desired_torque_last(640)
    self.safety.set_rt_torque_last(640)
    self.assertFalse(self.safety.safety_tx_hook(self.lka(640)))

  def test_speed_mismatch_uses_front_average(self):
    self.safety.set_controls_allowed(True)
    for fl, fr, ok in ((7., 13., True), (10., 15.5, False)):
      with self.subTest(fl=fl, fr=fr):
        self.safety.set_controls_allowed(True)
        self.safety.safety_rx_hook(self.bus.msg("VEHICLE_SPEED", VEHICLE_SPEED=10. * 3.6))
        self.safety.safety_rx_hook(self.bus.msg("WHEEL_SPEEDS_FRONT", WHEEL_SPEED_FL=fl * 3.6, WHEEL_SPEED_FR=fr * 3.6))
        self.assertEqual(ok, self.safety.get_controls_allowed())

  # ---- safety_tick with the event-driven STEERING_BUTTONS (non_periodic, the port's change to safety.h) ----
  def test_tick_keeps_controls_without_any_button_message(self):
    # all periodic rx messages flow, the roller was never touched: the 1 Hz tick must not disable controls
    self.safety.set_controls_allowed(True)
    for _ in range(5):
      self.bus.step(100)
      self.safety.safety_tick()
      self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.safety_config_valid())

  def test_tick_keeps_controls_long_after_a_button_message(self):
    self.press()
    self.bus.step(15)
    self.press()
    self.assertTrue(self.safety.get_controls_allowed())
    for _ in range(5):  # 5 s without any STEERING_BUTTONS frame
      self.bus.step(100)
      self.safety.safety_tick()
      self.assertTrue(self.safety.get_controls_allowed())

  def test_tick_disables_controls_when_a_periodic_message_stops(self):
    self.safety.set_controls_allowed(True)
    self.bus.step(100)
    self.safety.safety_tick()
    self.assertTrue(self.safety.get_controls_allowed())
    self.bus.PERIODIC = {k: v for k, v in Bus.PERIODIC.items() if k != "STEER_1"}
    self.bus.step(250)  # 2.5 s without STEER_1 (100 Hz: lag threshold 1 s)
    self.safety.safety_tick()
    self.assertFalse(self.safety.get_controls_allowed())

  # ---- the command ----
  def test_full_raw_torque_range(self):
    # LKA_TORQUE_REQ is 12 bits (DBC 32|12, factor -1, offset 1000: -3095..1000). Every raw value whose torque is over the
    # cap must be blocked, also those the generic +-2x cap sweep never reaches
    self.bus.speed = 0.
    self.bus.step(12)
    for raw in range(4096):
      t = 1000 - raw
      self.safety.set_controls_allowed(True)
      self.safety.set_desired_torque_last(t)
      self.safety.set_rt_torque_last(t)
      dat = bytearray(bytes(self.lka(0)[0].data)[:8])  # CANPacket_t.data is 64 bytes
      dat[4] = raw & 0xFF
      dat[5] = (dat[5] & 0xF0) | (raw >> 8)
      dat[7] = (0xFF - sum(dat[:7])) & 0xFF
      ok = self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x365, 0, bytes(dat)))
      self.assertEqual(abs(t) <= 650, ok, (raw, t))

  def test_only_known_lka_states(self):
    # OFF, STANDBY and READY pass without controls, ACTIVE only with them; the ICU only ever sent 0 and openpilot 0/2/19
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      allowed = [s for s in range(256) if self.safety.safety_tx_hook(self.lka(0, state=s))]
      self.assertEqual([0, 1, 2, 19] if controls_allowed else [0, 1, 2], allowed)


if __name__ == "__main__":
  unittest.main()
