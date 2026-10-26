#!/usr/bin/env python3
import itertools
import unittest
from opendbc.safety.tests.coverage import coverage
import numpy as np

from opendbc.car.byd.carcontroller import get_safety_CP
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import away_round

STEERING_MODULE_ADAS = 0x1E2
LKAS_HUD_ADAS = 0x316
PCM_BUTTONS = 0x3B0


def safety_max_can(max_angle_float, can_offset=0):
  # Matches C: max_angle_can = (int)(max_angle * 10 + 1.) which is floor(max_angle * 10) + 1
  return int(max_angle_float * 10 + 1.) + can_offset


class TestBydSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  DBC = "byd_atto3"
  SAFETY_MODEL = CarParams.SafetyModel.byd

  RELAY_MALFUNCTION_ADDRS = {0: (STEERING_MODULE_ADAS, LKAS_HUD_ADAS)}
  FWD_BLACKLISTED_ADDRS = {2: [STEERING_MODULE_ADAS, LKAS_HUD_ADAS]}
  TX_MSGS = [[STEERING_MODULE_ADAS, 0], [LKAS_HUD_ADAS, 0], [PCM_BUTTONS, 0]]

  MAIN_BUS = 0
  CAM_BUS = 2

  STEER_ANGLE_MAX = 390  # deg, EPS fault limit
  DEG_TO_CAN = 10

  # Real time limits
  LATERAL_FREQUENCY = 50  # Hz

  def _get_steer_cmd_angle_max(self, speed):
    return get_max_angle_vm(max(speed, 1), self.VM, CarControllerParams)

  def setUp(self):
    super().setUp()
    self.VM = VehicleModel(get_safety_CP())

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    values = {"STEER_ANGLE": angle, "STEER_REQ": 1 if enabled else 0, "STEER_REQ_ACTIVE_LOW": 0 if enabled else 1}
    if increment_timer:
      if self.backend is None:
        self.safety.set_timer(self.__class__.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      else:
        self._wait(1 / self.LATERAL_FREQUENCY)
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("STEERING_MODULE_ADAS", self.MAIN_BUS, values)

  cnt_angle_cmd = 0

  def _angle_meas_msg(self, angle: float):
    values = {"STEER_ANGLE_2": angle}
    return self.packer.make_can_msg_safety("STEER_MODULE_2", self.MAIN_BUS, values)

  def _pcm_status_msg(self, enable):
    # ACC_STATE: 0=OFF, 3=ACC_ACTIVE
    values = {"ACC_STATE": 3 if enable else 0}
    return self.packer.make_can_msg_safety("ACC_HUD_ADAS", self.CAM_BUS, values)

  def _speed_msg(self, speed):
    values = {"WHEELSPEED_CLEAN": speed * 3.6}
    return self.packer.make_can_msg_safety("WHEELSPEED_CLEAN", self.MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSED": 1 if brake else 0}
    return self.packer.make_can_msg_safety("DRIVE_STATE", self.MAIN_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"RAW_THROTTLE": int(gas * 100)}
    return self.packer.make_can_msg_safety("DRIVE_STATE", self.MAIN_BUS, values)

  def test_cruise_buttons(self):
    buttons = ("SET_BTN", "RES_BTN", "LKAS_ON_BTN", "DEC_DISTANCE_BTN", "INC_DISTANCE_BTN", "ACC_ON_BTN")
    # Cruise off cannot coexist with controls allowed. Gas can disengage controls
    # while cruise remains active, which independently exercises the button gate.
    for cruise_engaged, controls_allowed in ((False, False), (True, True), (True, False)):
      self.assertTrue(self._rx(self._user_gas_msg(0)))
      self._set_controls_allowed(cruise_engaged)
      if cruise_engaged and not controls_allowed:
        self.assertTrue(self._rx(self._user_gas_msg(1)))
      for pressed in itertools.product((False, True), repeat=len(buttons)):
        values = dict(zip(buttons, pressed, strict=True))
        values.update(SET_ME_1_1=1, SET_ME_1_2=1)
        with self.subTest(cruise_engaged=cruise_engaged, controls_allowed=controls_allowed, buttons=pressed):
          msg = self.packer.make_can_msg_safety("PCM_BUTTONS", self.MAIN_BUS, values)
          should_tx = not any(pressed[:-1]) and (not pressed[-1] or cruise_engaged)
          self.assertEqual(should_tx, self._tx(msg))

  @coverage('Checksum rejection, controls-off observations, and high-speed jerk checks after recovery',
            host=('Exact stored speed min/max preservation across rejected frames and unreseeded high-speed jerk check',))
  def test_rx_checksums(self):
    for name, bus, signal, initial, corrupt in (
      ("WHEELSPEED_CLEAN", self.MAIN_BUS, "WHEELSPEED_CLEAN", 72, 0),
      ("ACC_HUD_ADAS", self.CAM_BUS, "ACC_STATE", 0, 3),
    ):
      for byte in range(8):
        with self.subTest(message=name, byte=byte):
          self._reset_safety_hooks()
          if self.backend is None:
            self.safety.init_tests()
          for _ in range(16):
            # Valid setup must also update the test-owned periodic input.
            msg = self._speed_msg(initial / 3.6) if name == "WHEELSPEED_CLEAN" else self._pcm_status_msg(False)
            self.assertTrue(self._rx(msg))
          if self.backend is None:  # Exact sample preservation has no public getter.
            speed_min = self.safety.get_vehicle_speed_min()
            speed_max = self.safety.get_vehicle_speed_max()
          msg = self.packer.make_can_msg_safety(name, bus, {signal: corrupt})
          msg[0].data[byte] ^= 0xFF
          self._set_controls_allowed(name == "WHEELSPEED_CLEAN")
          if self.backend is not None:
            from opendbc.safety.tests.fixture_streams import suspend_required_streams
            # A checksum fault can also change the counter tracker. Keep the
            # exact faulty frame isolated through rejection and controls-off
            # observations; the speed case explicitly ends isolation below.
            suspend_required_streams(self, 'speed' if name == "WHEELSPEED_CLEAN" else 'pcm')
          self.assertFalse(self._rx(msg, raw=True))
          self.assertFalse(self._controls_allowed())
          if self.backend is None:  # Exact speed retention across the rejected frame, before any recovery.
            self.assertEqual(speed_min, self.safety.get_vehicle_speed_min())
            self.assertEqual(speed_max, self.safety.get_vehicle_speed_max())
          if name == "WHEELSPEED_CLEAN":
            if self.backend is not None:
              # End raw-fault isolation before the healthy jerk probe. Reinstall
              # the same high speed and let the 1 Hz RX watchdog recover.
              self._update_required_streams(self.required_streams())
              for _ in range(16):
                self.assertTrue(self._rx(self._speed_msg(initial / 3.6)))
              self._wait(1.1)
            # Host checks retained speed without reseeding after the corrupt frame;
            # backend checks the jerk gate at the restored healthy high speed.
            # Re-engage via CAN, then seed zero command without changing speed.
            self._set_controls_allowed(True)
            self._set_prev_desired_angle(0)
            self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
            limit = get_max_angle_delta_vm(19, self.VM, CarControllerParams)
            over_limit = safety_max_can(limit, 1) / self.DEG_TO_CAN
            self.assertFalse(self._tx(self._angle_cmd_msg(over_limit, True)))

  def test_rx_counters(self):
    for name, bus, signal, value in (
      ("WHEELSPEED_CLEAN", self.MAIN_BUS, "WHEELSPEED_CLEAN", 72),
      ("ACC_HUD_ADAS", self.CAM_BUS, "ACC_STATE", 3),
    ):
      with self.subTest(message=name):
        if self.backend is not None:
          self._reset_safety_hooks()
        self._set_controls_allowed(True)
        if self.backend is not None:
          from opendbc.safety.tests.fixture_streams import suspend_required_streams
          # Keep background counters from colliding with or healing the raw sequence.
          suspend_required_streams(self, 'speed' if name == "WHEELSPEED_CLEAN" else 'pcm')
        # Check both counter locations and rollover with valid checksums.
        for _ in range(32):
          msg = self.packer.make_can_msg_safety(name, bus, {signal: value})
          self.assertTrue(self._rx(msg))
        # Replayed frames are rejected after the common counter tolerance.
        for i in range(common.MAX_WRONG_COUNTERS + 1):
          should_rx = i < common.MAX_WRONG_COUNTERS - 1
          self.assertEqual(should_rx, self._rx(msg, raw=True))
          self.assertEqual(should_rx, self._controls_allowed())
        # A valid sequence clears the counter faults.
        for _ in range(common.MAX_WRONG_COUNTERS):
          msg = self.packer.make_can_msg_safety(name, bus, {signal: 0})
          self.assertTrue(self._rx(msg))

  # BYD uses get_max_angle_delta_vm and get_max_angle_vm for lateral accel and jerk limits
  def test_angle_cmd_when_enabled(self):
    # We properly test lateral acceleration and jerk below
    pass

  def test_lateral_accel_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      # match both CAN encoding (0.1 kph/LSB) and VEHICLE_SPEED_FACTOR=1000 rounding in UPDATE_VEHICLE_SPEED
      sent = speed + 1
      sent_can = away_round(sent / 0.1 * 3.6) * 0.1 / 3.6
      speed = round(sent_can * 1000) / 1000 - 1
      for sign in (-1, 1):
        self._set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed

        max_angle_float = get_max_angle_vm(speed, self.VM, CarControllerParams)

        # at limit (safety tolerance adds 1 CAN unit)
        max_angle_can = safety_max_can(max_angle_float)
        max_angle_can = min(max_angle_can, self.STEER_ANGLE_MAX * self.DEG_TO_CAN)
        max_angle = sign * max_angle_can / self.DEG_TO_CAN
        self._set_prev_desired_angle(max_angle)

        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle, True)))

        # 1 unit above limit
        over_can = safety_max_can(max_angle_float, 1)
        over_can_clipped = min(over_can, self.STEER_ANGLE_MAX * self.DEG_TO_CAN)
        over_angle = sign * over_can_clipped / self.DEG_TO_CAN
        self._tx(self._angle_cmd_msg(over_angle, True))

        # at low speeds max angle is above STEER_ANGLE_MAX, so adding 1 has no effect
        should_tx = over_can >= self.STEER_ANGLE_MAX * self.DEG_TO_CAN
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(over_angle, True)))

  def test_lateral_jerk_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      # match both CAN encoding (0.1 kph/LSB) and VEHICLE_SPEED_FACTOR=1000 rounding in UPDATE_VEHICLE_SPEED
      sent = speed + 1
      sent_can = away_round(sent / 0.1 * 3.6) * 0.1 / 3.6
      speed = round(sent_can * 1000) / 1000 - 1
      for sign in (-1, 1):
        self._set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed
        self._tx(self._angle_cmd_msg(0, True))

        max_delta_float = get_max_angle_delta_vm(speed, self.VM, CarControllerParams)

        # Stay within limits
        # Up
        max_delta_can = safety_max_can(max_delta_float)
        max_angle_delta = sign * max_delta_can / self.DEG_TO_CAN
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        # Inject too high rates
        # Up
        over_delta_can = safety_max_can(max_delta_float, 1)
        max_angle_delta = sign * over_delta_can / self.DEG_TO_CAN
        self.assertFalse(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self._set_prev_desired_angle(max_angle_delta)
        self._reset_angle_measurement(0)
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

        # Recover
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))


if __name__ == "__main__":
  unittest.main()
