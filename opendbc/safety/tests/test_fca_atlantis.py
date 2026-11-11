#!/usr/bin/env python3
import itertools
import unittest
from opendbc.car.structs import CarParams
from opendbc.car.fca_atlantis.fca_atlantiscan import lka_command_torque
import opendbc.safety.tests.common as common

class TestFcaAtlantis_Safety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  DBC = "fca_atlantis"
  SAFETY_MODEL = CarParams.SafetyModel.fcaAtlantis

  RELAY_MALFUNCTION_ADDRS = {0: (0x117, 0x1F6, 0x547, 0x5A2)}

  # LKA_COMMAND_2 units (4x LKA_COMMAND)
  MAX_RATE_UP = 12
  MAX_RATE_DOWN = 12
  MAX_TORQUE_LOOKUP = [0], [1200]
  MAX_RT_DELTA = 448

  DRIVER_TORQUE_ALLOWANCE = 80
  DRIVER_TORQUE_FACTOR = 8

  TX_MSGS = [[0x117, 0], [0x1F6, 0], [0x547, 0], [0x5A2, 0], [0x2FA, 2]]
  STANDSTILL_THRESHOLD = 0
  FWD_BLACKLISTED_ADDRS = {2: [0x117, 0x1F6, 0x547, 0x5A2]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  def _button_msg(self, cancel=False, resume=False, **extra):
    values = {"CANCEL": cancel, "RESUME": resume, "UNKNOWN_1": 3, "UNKNOWN_2": 0x0E, **extra}
    return self.packer.make_can_msg_safety("ACC_BUTTON", 2, values)

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_STATUS": enable}
    return self.packer.make_can_msg_safety("ACC_2", 2, values)

  def _speed_msg(self, speed):
    values = {"WHEEL_SPEED_%s" % s: speed for s in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("ABS_1", 0, values)

  def _user_gas_msg(self, gas):
    values = {"ACCEL_PEDAL_USER": gas}
    return self.packer.make_can_msg_safety("ENGINE_2", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PEDAL_SWITCH": 1 if brake else 0}
    return self.packer.make_can_msg_safety("ABS_3", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"DRIVER_TORQUE": torque}
    return self.packer.make_can_msg_safety("EPS_2", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKA_TORQUE": torque, "LKA_ACTIVE": steer_req}
    return self.packer.make_can_msg_safety("LKA_COMMAND_2", 0, values)

  def _torque_cmd_1_msg(self, torque, steer_req=1):
    values = {"LKA_TORQUE": torque, "LKA_ACTIVE": steer_req}
    return self.packer.make_can_msg_safety("LKA_COMMAND", 0, values)

  TORQUES = (0, 5, 6, 12, 22, 10, 0, -2, -6, -12, -22)

  def test_lka_command_mirrors_lka_command_2(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self._rx(self._torque_driver_msg(0))
    for torque in self.TORQUES:
      self.assertTrue(self._tx(self._torque_cmd_msg(torque)))
      # anything but LKA_COMMAND_2 / 4 (rounded either way) is blocked, and leaves the copy allowed
      for torque_1 in range(torque // 4 - 2, torque // 4 + 3):
        if abs(torque_1 * 4 - torque) > 2:
          self.assertFalse(self._tx(self._torque_cmd_1_msg(torque_1)), f"{torque=} {torque_1=}")
      self.assertFalse(self._tx(self._torque_cmd_1_msg(lka_command_torque(torque), steer_req=0)))
      # what carcontroller sends is allowed, once
      self.assertTrue(self._tx(self._torque_cmd_1_msg(lka_command_torque(torque))))
      self.assertFalse(self._tx(self._torque_cmd_1_msg(lka_command_torque(torque))))

  def test_lka_command_rounding(self):
    # either rounding direction is allowed
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self._rx(self._torque_driver_msg(0))
    for torque in self.TORQUES:
      for torque_1 in range(torque // 4 - 2, torque // 4 + 3):
        if abs(torque_1 * 4 - torque) <= 2:
          self.assertTrue(self._tx(self._torque_cmd_msg(torque)))
          self.assertTrue(self._tx(self._torque_cmd_1_msg(torque_1)), f"{torque=} {torque_1=}")

  def test_lka_command_zero_after_blocked_lka_command_2(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self._rx(self._torque_driver_msg(0))
    self.assertTrue(self._tx(self._torque_cmd_msg(12)))
    # rate limit violation blocks LKA_COMMAND_2; LKA_COMMAND can't carry that torque or the last allowed one
    self.assertFalse(self._tx(self._torque_cmd_msg(12 + self.MAX_RATE_UP + 4)))
    self.assertFalse(self._tx(self._torque_cmd_1_msg((12 + self.MAX_RATE_UP + 4) // 4)))
    self.assertFalse(self._tx(self._torque_cmd_1_msg(12 // 4)))
    self.assertTrue(self._tx(self._torque_cmd_1_msg(0, steer_req=0)))

  def test_lka_command_without_controls(self):
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._torque_cmd_1_msg(25)))
    self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=0)))
    self.assertTrue(self._tx(self._torque_cmd_1_msg(0, steer_req=0)))
    self.assertFalse(self._tx(self._torque_cmd_1_msg(1, steer_req=0)))

  def _engage_with_torque(self, torque):
    self.safety.set_controls_allowed(True)
    self._rx(self._torque_driver_msg(0))
    self._set_prev_torque(torque)
    self.assertTrue(self._tx(self._torque_cmd_msg(torque)))

  def test_winddown_after_disengage(self):
    for sign in (1, -1):
      self._reset_safety_hooks()
      self.safety.init_tests()
      self._engage_with_torque(sign * 120)
      self.safety.set_controls_allowed(False)
      # magnitude may hold or shrink, never grow
      self.assertTrue(self._tx(self._torque_cmd_msg(sign * 120)))
      self.assertFalse(self._tx(self._torque_cmd_msg(sign * 121)))
      for torque in range(108, 0, -12):
        self.assertTrue(self._tx(self._torque_cmd_msg(sign * torque)))
      # down to the smallest torque, which may also hold
      for torque in (8, 4, 1, 1):
        self.assertTrue(self._tx(self._torque_cmd_msg(sign * torque)))
      # winddown ends at zero
      self.assertTrue(self._tx(self._torque_cmd_msg(0)))
      self.assertFalse(self._tx(self._torque_cmd_msg(sign * 12)))

  def test_winddown_limits(self):
    # no sign change, and LKA_ACTIVE must stay set
    self._engage_with_torque(120)
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._torque_cmd_msg(-12)))
    self.assertFalse(self._tx(self._torque_cmd_msg(108, steer_req=0)))
    # blocked frames don't end the winddown (openpilot may send a frame or two before it sees the brake)
    self.assertTrue(self._tx(self._torque_cmd_msg(108)))

  def test_winddown_limit_over_time(self):
    # the limit falls from the last torque allowed while engaged, MAX_RATE_DOWN per frame after the slack,
    # and reaches zero by itself
    self._engage_with_torque(1200)
    self.safety.set_controls_allowed(False)
    frame = 1
    while True:
      limit = min(1200, 1200 - self.MAX_RATE_DOWN * (frame - self.WINDDOWN_SLACK_FRAMES))
      t = frame * self.FRAME_US
      self.assertFalse(self._tx_frame(t, limit + 1), f"{frame=}")
      if limit <= 0:
        break
      self.assertTrue(self._tx_frame(t, limit), f"{frame=}")
      frame += 1
    self.assertEqual(frame, 1200 // self.MAX_RATE_DOWN + self.WINDDOWN_SLACK_FRAMES)

  def test_winddown_tolerates_late_frames(self):
    # frames up to the slack late (e.g. openpilot stalls), then on schedule again, are still allowed
    self._engage_with_torque(1200)
    self.safety.set_controls_allowed(False)
    for frame, torque in enumerate(range(1188, 0, -self.MAX_RATE_DOWN), start=1 + self.WINDDOWN_SLACK_FRAMES):
      self.assertTrue(self._tx_frame(frame * self.FRAME_US, torque), f"{torque=}")

  def test_no_winddown_after_violation(self):
    self._engage_with_torque(120)
    # rate limit violation while engaged
    self.assertFalse(self._tx(self._torque_cmd_msg(120 + self.MAX_RATE_UP + 1)))
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._torque_cmd_msg(108)))

  def test_no_winddown_without_prior_torque(self):
    self._engage_with_torque(0)
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._torque_cmd_msg(12)))

  def test_winddown_on_brake(self):
    self._rx(self._speed_msg(100))
    self._rx(self._user_brake_msg(False))
    self._engage_with_torque(120)
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._tx(self._torque_cmd_msg(132)))
    for torque in range(120, -1, -12):
      self.assertTrue(self._tx(self._torque_cmd_msg(torque)))
      self.assertFalse(self._tx(self._torque_cmd_1_msg(torque // 4 + 1)))
      self.assertTrue(self._tx(self._torque_cmd_1_msg(torque // 4)))

  def test_reengage_during_winddown(self):
    for sign in (1, -1):
      self._reset_safety_hooks()
      self.safety.init_tests()
      self._engage_with_torque(sign * 120)
      self.safety.set_controls_allowed(False)
      self.assertTrue(self._tx(self._torque_cmd_msg(sign * 108)))
      self.safety.set_controls_allowed(True)
      # rate limits continue from the last winddown frame
      self.assertTrue(self._tx(self._torque_cmd_msg(sign * (108 + self.MAX_RATE_UP))))
      self.assertFalse(self._tx(self._torque_cmd_msg(sign * (120 + self.MAX_RATE_UP + 1))))

  # Known bugs. Frames are 10 ms apart, as openpilot sends them.
  FRAME_US = 10000

  def _tx_frame(self, t, torque, steer_req=1):
    self.safety.set_timer(t)
    return self._tx(self._torque_cmd_msg(torque, steer_req))

  def test_reengage_after_blocked_frame(self):
    # A frame blocked after the brake press (one openpilot sent before it saw the brake) must not leave the rate
    # limits in a state where re-engaging mid-winddown, still above MAX_RT_DELTA, is blocked indefinitely.
    self._rx(self._speed_msg(100))
    self._rx(self._user_brake_msg(False))
    self._engage_with_torque(800)
    self._rx(self._user_brake_msg(True))
    t = self.FRAME_US
    self.assertFalse(self._tx_frame(t, 812))
    for torque in (800, 788, 776, 764):
      t += self.FRAME_US
      self.assertTrue(self._tx_frame(t, torque))
    self._rx(self._user_brake_msg(False))
    self.safety.set_controls_allowed(True)
    # openpilot ramps back up from its last torque
    for torque in (776, 788, 800, 800, 800):
      t += self.FRAME_US
      self.assertTrue(self._tx_frame(t, torque), f"{torque=}")

  def test_winddown_state_expires(self):
    # The winddown must not leave rate limit state behind: after openpilot goes silent, re-engaging must start
    # from zero torque, not continue from the last winddown frame.
    self._engage_with_torque(800)
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx_frame(self.FRAME_US, 800))
    self.safety.set_controls_allowed(True)
    t = 5_000_000
    self.assertFalse(self._tx_frame(t, 800))
    self.assertTrue(self._tx_frame(t + self.FRAME_US, self.MAX_RATE_UP))

  WINDDOWN_SLACK_FRAMES = 5  # frames of delay tolerated before torque must fall

  def test_winddown_ramps_down(self):
    # After a disengage, torque must keep falling at MAX_RATE_DOWN per frame, not hold
    for sign in (1, -1):
      self._reset_safety_hooks()
      self.safety.init_tests()
      self._engage_with_torque(sign * 1200)
      self.safety.set_controls_allowed(False)
      # it may hold within the slack (openpilot may not have seen the disengage yet, or be late), then not
      for frame in range(1, self.WINDDOWN_SLACK_FRAMES + 1):
        self.assertTrue(self._tx_frame(frame * self.FRAME_US, sign * 1200), f"{frame=}")
      self.assertFalse(self._tx_frame((self.WINDDOWN_SLACK_FRAMES + 1) * self.FRAME_US, sign * 1200))

      # the full ramp is allowed, down to zero within about a second
      self._reset_safety_hooks()
      self.safety.init_tests()
      self._engage_with_torque(sign * 1200)
      self.safety.set_controls_allowed(False)
      for frame, torque in enumerate(range(1188, 0, -self.MAX_RATE_DOWN), start=1):
        self.assertTrue(self._tx_frame(frame * self.FRAME_US, sign * torque), f"{torque=}")

  def test_lka_command_once_per_lka_command_2(self):
    # Each allowed LKA_COMMAND_2 allows one LKA_COMMAND copy, so repeating a copy can't keep torque applied after
    # openpilot stops sending LKA_COMMAND_2.
    self._engage_with_torque(120)
    self.assertTrue(self._tx(self._torque_cmd_1_msg(30)))
    self.assertFalse(self._tx(self._torque_cmd_1_msg(30)))
    self.safety.set_controls_allowed(False)
    for _ in range(10):
      self.assertFalse(self._tx(self._torque_cmd_1_msg(30)))

  def test_buttons(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)

      # only cancel is allowed, regardless of controls allowed
      self.assertTrue(self._tx(self._button_msg(cancel=True)))
      self.assertFalse(self._tx(self._button_msg(resume=True)))
      self.assertFalse(self._tx(self._button_msg(cancel=True, resume=True)))
      self.assertFalse(self._tx(self._button_msg(cancel=False, resume=False)))
      # cancel with another button, or a constant bit not as sent by the car, in the second byte
      self.assertFalse(self._tx(self._button_msg(cancel=True, DIST_PLUS=1)))
      self.assertFalse(self._tx(self._button_msg(cancel=True, SPEED_LIMITER=1)))
      self.assertFalse(self._tx(self._button_msg(cancel=True, UNKNOWN_1=2)))

  def test_gas_pressed_smallest_step(self):
    # ACCEL_PEDAL_USER is 0.4% per bit
    self._rx(self._user_gas_msg(0.4))
    self.assertTrue(self.safety.get_gas_pressed_prev())

  def test_vehicle_moving_any_wheel(self):
    # moving when any wheel turns, by the smallest step (0.017 m/s per bit)
    for moving in itertools.product((False, True), repeat=4):
      values = {f"WHEEL_SPEED_{w}": 0.017 if m else 0 for w, m in zip(("FL", "FR", "RL", "RR"), moving, strict=True)}
      self._rx(self.packer.make_can_msg_safety("ABS_1", 0, values))
      self.assertEqual(any(moving), self.safety.get_vehicle_moving(), moving)

  def test_rx_hook(self):
    for count in range(20):
      self.assertTrue(self._rx(self._speed_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_brake_msg(False)), f"{count=}")
      self.assertTrue(self._rx(self._torque_driver_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_gas_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._pcm_status_msg(False)), f"{count=}")

  def test_rx_checksum_counter(self):
    for name, bus, length in (("ABS_1", 0, 8), ("ABS_3", 0, 8), ("ENGINE_2", 0, 8), ("EPS_2", 0, 7), ("ACC_2", 2, 8)):
      with self.subTest(msg=name):
        # an expected counter sequence is always accepted, and clears the wrong counter count
        for _ in range(common.MAX_WRONG_COUNTERS + 1):
          self.assertTrue(self._rx(self.packer.make_can_msg_safety(name, bus, {})))

        # a bad checksum (the last byte) fails, and disables controls
        self.safety.set_controls_allowed(True)
        msg = self.packer.make_can_msg_safety(name, bus, {})
        msg[0].data[length - 1] ^= 0xFF
        self.assertFalse(self._rx(msg))
        self.assertFalse(self.safety.get_controls_allowed())

        # a stuck counter fails as well, its checksum is still correct
        msg[0].data[length - 1] ^= 0xFF
        for i in range(common.MAX_WRONG_COUNTERS):
          should_rx = i < common.MAX_WRONG_COUNTERS - 1
          self.assertEqual(should_rx, self._rx(msg))


if __name__ == "__main__":
  unittest.main()
