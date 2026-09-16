#!/usr/bin/env python3
import itertools
import unittest
import numpy as np

from opendbc.car.byd.carcontroller import get_safety_CP
from opendbc.car.byd.values import BydSafetyFlags, CarControllerParams
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
  SAFETY_PARAM = int(BydSafetyFlags.LKS_ON)

  RELAY_MALFUNCTION_ADDRS = {0: (STEERING_MODULE_ADAS, LKAS_HUD_ADAS)}
  # Stock 0x1E2/0x316 are forwarded until OP sends STEER_REQ=1.
  FWD_BLACKLISTED_ADDRS: dict[int, list[int]] = {}
  TX_MSGS = [[STEERING_MODULE_ADAS, 0], [LKAS_HUD_ADAS, 0], [PCM_BUTTONS, 0], [PCM_BUTTONS, 2]]

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
      self.safety.set_timer(self.__class__.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("STEERING_MODULE_ADAS", self.MAIN_BUS, values)

  # start past t=0: byd_fwd_hook reads a zero timestamp as "openpilot never steered"
  cnt_angle_cmd = 1

  def _angle_meas_msg(self, angle: float):
    values = {"STEER_ANGLE_2": angle}
    return self.packer.make_can_msg_safety("STEER_MODULE_2", self.MAIN_BUS, values)

  def _pcm_status_msg(self, enable):
    # ACC_STATE: 0=OFF, 3=ACC_ACTIVE
    values = {"ACC_STATE": 3 if enable else 0}
    return self.packer.make_can_msg_safety("ACC_HUD_ADAS", self.CAM_BUS, values)

  def _lkas_hud_msg(self, lkas_state: int):
    values = {"LKAS_STATE": lkas_state}
    return self.packer.make_can_msg_safety("LKAS_HUD_ADAS", self.CAM_BUS, values)

  def _lks_btn_msg(self, pressed: bool, bus=None):
    values = {"LKAS_ON_BTN": 1 if pressed else 0, "SET_ME_1_1": 1, "SET_ME_1_2": 1}
    return self.packer.make_can_msg_safety("PCM_BUTTONS", self.MAIN_BUS if bus is None else bus, values)

  def _speed_msg(self, speed):
    values = {"WHEELSPEED_CLEAN": speed * 3.6}
    return self.packer.make_can_msg_safety("WHEELSPEED_CLEAN", self.MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSED": 1 if brake else 0}
    return self.packer.make_can_msg_safety("DRIVE_STATE", self.MAIN_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL": gas}
    return self.packer.make_can_msg_safety("PEDAL", self.MAIN_BUS, values)

  def test_cruise_buttons(self):
    buttons = ("SET_BTN", "RES_BTN", "LKAS_ON_BTN", "DEC_DISTANCE_BTN", "INC_DISTANCE_BTN", "ACC_ON_BTN")
    for cruise_engaged, controls_allowed in itertools.product((False, True), repeat=2):
      self.assertTrue(self._rx(self._pcm_status_msg(cruise_engaged)))
      self.safety.set_controls_allowed(controls_allowed)
      for pressed in itertools.product((False, True), repeat=len(buttons)):
        values = dict(zip(buttons, pressed, strict=True))
        values.update(SET_ME_1_1=1, SET_ME_1_2=1)
        set_res, lkas_on, dec, inc, cancel = pressed[0] or pressed[1], pressed[2], pressed[3], pressed[4], pressed[5]
        with self.subTest(cruise_engaged=cruise_engaged, controls_allowed=controls_allowed, buttons=pressed):
          # bus 0: cancel only while stock cruise is engaged
          msg = self.packer.make_can_msg_safety("PCM_BUTTONS", self.MAIN_BUS, values)
          should_tx = not (set_res or lkas_on or dec or inc) and (not cancel or cruise_engaged)
          self.assertEqual(should_tx, self._tx(msg))
          # bus 2: only the camera LKS spoof
          msg = self.packer.make_can_msg_safety("PCM_BUTTONS", self.CAM_BUS, values)
          should_tx = not (set_res or dec or inc or cancel)
          self.assertEqual(should_tx, self._tx(msg))

  def test_rx_checksums(self):
    for name, bus, signal, initial, corrupt in (
      ("WHEELSPEED_CLEAN", self.MAIN_BUS, "WHEELSPEED_CLEAN", 72, 0),
      ("ACC_HUD_ADAS", self.CAM_BUS, "ACC_STATE", 0, 3),
    ):
      for byte in range(8):
        with self.subTest(message=name, byte=byte):
          self.safety.set_safety_hooks(CarParams.SafetyModel.byd, 0)
          self.safety.init_tests()
          for _ in range(16):
            msg = self.packer.make_can_msg_safety(name, bus, {signal: initial})
            self.assertTrue(self._rx(msg))
          speed_min = self.safety.get_vehicle_speed_min()
          speed_max = self.safety.get_vehicle_speed_max()
          msg = self.packer.make_can_msg_safety(name, bus, {signal: corrupt})
          msg[0].data[byte] ^= 0xFF
          self.safety.set_controls_allowed(name == "WHEELSPEED_CLEAN")
          self.assertFalse(self._rx(msg))
          self.assertFalse(self.safety.get_controls_allowed())
          self.assertEqual(speed_min, self.safety.get_vehicle_speed_min())
          self.assertEqual(speed_max, self.safety.get_vehicle_speed_max())

  def test_rx_counters(self):
    for name, bus, signal, value in (
      ("WHEELSPEED_CLEAN", self.MAIN_BUS, "WHEELSPEED_CLEAN", 72),
      ("ACC_HUD_ADAS", self.CAM_BUS, "ACC_STATE", 3),
    ):
      with self.subTest(message=name):
        # Check both counter locations and rollover with valid checksums.
        for _ in range(32):
          msg = self.packer.make_can_msg_safety(name, bus, {signal: value})
          self.assertTrue(self._rx(msg))
        self.safety.set_controls_allowed(True)
        # Replayed frames are rejected after the common counter tolerance.
        for i in range(common.MAX_WRONG_COUNTERS + 1):
          should_rx = i < common.MAX_WRONG_COUNTERS - 1
          self.assertEqual(should_rx, self._rx(msg))
          self.assertEqual(should_rx, self.safety.get_controls_allowed())
        # A valid sequence clears the counter faults.
        for _ in range(common.MAX_WRONG_COUNTERS):
          msg = self.packer.make_can_msg_safety(name, bus, {signal: 0})
          self.assertTrue(self._rx(msg))

  def _driver_torque_msg(self, torque: float):
    values = {"DRIVER_TORQUE": torque}
    return self.packer.make_can_msg_safety("STEERING_TORQUE", self.MAIN_BUS, values)

  def test_steering_wheel_disengage(self):
    # BYD disengages when the driver holds the wheel against angle control. The EPS measurement
    # is attenuated too far to see that, so this reads column torque and debounces it.
    frames = CarControllerParams.STEER_DRIVER_DISENGAGE_FRAMES
    for sign in (-1, 1):
      for torque in (CarControllerParams.STEER_DRIVER_DISENGAGE, CarControllerParams.STEER_DRIVER_DISENGAGE + 5):
        should_disengage = torque > CarControllerParams.STEER_DRIVER_DISENGAGE

        self.safety.set_controls_allowed(True)
        for _ in range(frames - 1):
          self.assertTrue(self._rx(self._driver_torque_msg(sign * torque)))
          self.assertFalse(self.safety.get_steering_disengage_prev())
          self.assertTrue(self.safety.get_controls_allowed())

        self.assertTrue(self._rx(self._driver_torque_msg(sign * torque)))
        self.assertEqual(should_disengage, self.safety.get_steering_disengage_prev())
        self.assertEqual(not should_disengage, self.safety.get_controls_allowed())

        # one frame under the threshold resets the debounce, and controls stay disallowed
        self.assertTrue(self._rx(self._driver_torque_msg(0)))
        self.assertFalse(self.safety.get_steering_disengage_prev())
        self.assertEqual(not should_disengage, self.safety.get_controls_allowed())

  def test_rx_checksum(self):
    # 8-byte BYD frames use (~sum) in the last byte. STEER_MODULE_2 is 4-bit and
    # DRIVE_STATE has no checksum, so those stay ignored.
    checked = (
      self._driver_torque_msg(0),
      self._speed_msg(0),
      self._pcm_status_msg(False),
      self._lkas_hud_msg(1),
    )
    for msg in checked:
      self.assertTrue(self._rx(msg))
      msg.data[7] ^= 0xFF
      self.assertFalse(self._rx(msg))

    ignored = (
      self._angle_meas_msg(0),
      self._user_brake_msg(False),
    )
    for msg in ignored:
      self.assertTrue(self._rx(msg))
      msg.data[len(msg.data) - 1] ^= 0x0F
      self.assertTrue(self._rx(msg))

  # BYD uses get_max_angle_delta_vm and get_max_angle_vm for lateral accel and jerk limits
  def test_stock_steer_passthrough(self):
    # Idle: camera steer and HUD reach the car. Any OP 0x1E2 takes the steer away
    # from the camera, and the idle STEER_REQ=0 heartbeat holds it for the whole
    # engagement. HUD stays blocked ~2s after the last OP 0x1E2 so our cluster
    # nag bits are not overwritten by the camera.
    self.assertEqual(0, self.safety.safety_fwd_hook(2, STEERING_MODULE_ADAS))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, LKAS_HUD_ADAS))

    self.safety.set_controls_allowed(True)
    self._reset_speed_measurement(10)
    self._reset_angle_measurement(0)
    self.safety.set_desired_angle_last(0)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, STEERING_MODULE_ADAS))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, LKAS_HUD_ADAS))

    # lateral off but still engaged: the heartbeat keeps the camera off the EPS
    for _ in range(25):
      self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))
      self.assertEqual(-1, self.safety.safety_fwd_hook(2, STEERING_MODULE_ADAS))

    t = (self.__class__.cnt_angle_cmd - 1) * int(1e6 / self.LATERAL_FREQUENCY)
    self.safety.set_timer(t + 201000)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, STEERING_MODULE_ADAS))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, LKAS_HUD_ADAS))
    self.safety.set_timer(t + 2001000)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, LKAS_HUD_ADAS))

  def test_rejected_steer_keeps_camera_blocked(self):
    # A rate limited request still owns lateral; the camera must not steer bus 0 meanwhile.
    self.safety.set_controls_allowed(True)
    self._reset_speed_measurement(10)
    self._reset_angle_measurement(0)
    self.safety.set_desired_angle_last(0)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, STEERING_MODULE_ADAS))

    # way past the jerk limit, so every one of these is refused
    for _ in range(25):
      self.assertFalse(self._tx(self._angle_cmd_msg(self.STEER_ANGLE_MAX, True)))
      self.assertEqual(-1, self.safety.safety_fwd_hook(2, STEERING_MODULE_ADAS))

  def test_camera_blocked_when_controls_not_allowed(self):
    # Toggling LKS while ACC already holds gives openpilot an enable that panda
    # does not see, so it asks to steer with controls_allowed false. Every frame is
    # refused, but the camera must not pick up LKS behind openpilot's back.
    self.safety.set_controls_allowed(False)
    self._reset_speed_measurement(10)
    self._reset_angle_measurement(0)
    for _ in range(25):
      self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))
      self.assertEqual(-1, self.safety.safety_fwd_hook(2, STEERING_MODULE_ADAS))

  def test_lks_switch_gates_controls(self):
    # Latch starts on from safetyParam. ACC holding enables. The driver LKS
    # button on bus 0 is a rising-edge toggle; camera LKAS_STATE is ignored.
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())

    self.assertTrue(self._rx(self._lks_btn_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._lks_btn_msg(False)))
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed())

    self.assertTrue(self._rx(self._lks_btn_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_lks_button_on_camera_bus_does_not_toggle(self):
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._lks_btn_msg(True, bus=self.CAM_BUS)))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_camera_lkas_states_keep_controls(self):
    # Camera LKAS_STATE 0/4 are moods, not the switch.
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())

    for state in (2, 3, 1, 4, 0, 1, 3, 2):
      self.assertTrue(self._rx(self._lkas_hud_msg(state)))
      self.assertTrue(self._rx(self._pcm_status_msg(True)))
      self.assertTrue(self.safety.get_controls_allowed(), f"LKAS_STATE {state} dropped controls")

  def test_lks_off_param_blocks_acc(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, 0)
    self.safety.init_tests()
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_idle_heartbeat_tracks_angle(self):
    # The STEER_REQ=0 heartbeat carries the measured angle, so panda's rate limit
    # reference follows the wheel while lateral is off instead of going stale.
    self.safety.set_controls_allowed(True)
    self._reset_speed_measurement(10)
    self.safety.set_desired_angle_last(0)

    for angle in (0, 30, 90, -90, 0):
      self._reset_angle_measurement(angle)
      self.assertTrue(self._tx(self._angle_cmd_msg(angle, False)))
      self.assertEqual(angle * self.DEG_TO_CAN, self.safety.get_desired_angle_last())

    # so the first frame after lateral comes back is within one jerk step
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

  def test_angle_cmd_when_enabled(self):
    # We properly test lateral acceleration and jerk below
    pass

  def test_lateral_accel_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      # match both CAN encoding (WHEELSPEED_TO_KPH) and VEHICLE_SPEED_FACTOR=1000 rounding in UPDATE_VEHICLE_SPEED
      sent = speed + 1
      sent_can = away_round(sent / CarControllerParams.WHEELSPEED_TO_KPH * 3.6) * CarControllerParams.WHEELSPEED_TO_KPH / 3.6
      speed = round(sent_can * 1000) / 1000 - 1
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed

        max_angle_float = get_max_angle_vm(speed, self.VM, CarControllerParams)

        # at limit (safety tolerance adds 1 CAN unit)
        max_angle_can = safety_max_can(max_angle_float)
        max_angle_can = min(max_angle_can, self.STEER_ANGLE_MAX * self.DEG_TO_CAN)
        max_angle = sign * max_angle_can / self.DEG_TO_CAN
        self.safety.set_desired_angle_last(sign * max_angle_can)

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
      # match both CAN encoding (WHEELSPEED_TO_KPH) and VEHICLE_SPEED_FACTOR=1000 rounding in UPDATE_VEHICLE_SPEED
      sent = speed + 1
      sent_can = away_round(sent / CarControllerParams.WHEELSPEED_TO_KPH * 3.6) * CarControllerParams.WHEELSPEED_TO_KPH / 3.6
      speed = round(sent_can * 1000) / 1000 - 1
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
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
        self.safety.set_desired_angle_last(sign * over_delta_can)
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

        # Recover
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))


class TestBydIgnition(common.SafetyTestBase):
  DBC = "byd_atto3"
  SAFETY_MODEL = None

  TX_MSGS: list = []

  def _msg(self, gear, bus=0):
    return self.packer.make_can_msg_safety("DRIVE_STATE", bus, {"GEAR": gear})

  def test_ignition_on_for_every_valid_gear(self):
    for gear in (1, 2, 3, 4):
      self.safety.ignition_can_hook(self._msg(gear))
      self.assertTrue(self.safety.get_ignition_can(), f"gear {gear}")

  def test_ignition_stays_on_when_gear_zero(self):
    # Off is the 2s timeout, not GEAR==0
    self.safety.ignition_can_hook(self._msg(4))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_ignores_other_bus(self):
    self.safety.ignition_can_hook(self._msg(4, bus=2))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
