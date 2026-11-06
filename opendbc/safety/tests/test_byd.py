#!/usr/bin/env python3
import itertools
import unittest
import numpy as np

from opendbc.car.byd.carcontroller import get_safety_CP
from opendbc.car.byd.values import BydSafetyFlags, CarControllerParams, SAFETY_PLATFORM_PARAMS
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import away_round

STEERING_MODULE_ADAS = 0x1E2
LKAS_HUD_ADAS = 0x316
ACC_HUD_ADAS = 0x32D
ACC_CMD = 0x32E
PCM_BUTTONS = 0x3B0


def safety_max_can(max_angle_float, can_offset=0):
  # Matches C: max_angle_can = (int)(max_angle * 10 + 1.) which is floor(max_angle * 10) + 1
  return int(max_angle_float * 10 + 1.) + can_offset


class TestBydSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  DBC = "byd_atto3"
  SAFETY_MODEL = CarParams.SafetyModel.byd
  SAFETY_PARAM = int(BydSafetyFlags.LKS_ON) | int(SAFETY_PLATFORM_PARAMS["BYD_ATTO_3"].safety_param)

  RELAY_MALFUNCTION_ADDRS = {0: (STEERING_MODULE_ADAS, LKAS_HUD_ADAS)}
  # Stock 0x1E2/0x316 are forwarded until OP sends any 0x1E2 (heartbeat included).
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

  def _reinit_with_param(self, param):
    self.safety.set_safety_hooks(self.SAFETY_MODEL, param)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self._reset_speed_measurement(10)
    self._reset_angle_measurement(50)
    self._set_prev_desired_angle(50)

  def test_platform_bit_default_falls_back_to_atto3(self):
    # No platform bit keeps today's limits
    self._reinit_with_param(int(BydSafetyFlags.LKS_ON))
    self.assertTrue(self._tx(self._angle_cmd_msg(50, True)))
    self.assertFalse(self._tx(self._angle_cmd_msg(391, True)))

  def test_platform_bit_unknown_denies_steer(self):
    self._reinit_with_param(int(BydSafetyFlags.LKS_ON) | 0x200)
    self.assertFalse(self._tx(self._angle_cmd_msg(50, True)))

  def test_platform_bit_atto3_explicit(self):
    self._reinit_with_param(int(BydSafetyFlags.LKS_ON) | int(SAFETY_PLATFORM_PARAMS["BYD_ATTO_3"].safety_param))
    self.assertTrue(self._tx(self._angle_cmd_msg(50, True)))
    self.assertFalse(self._tx(self._angle_cmd_msg(391, True)))

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
          # bus 2: with no stock frame latched, only LKAS_ON_BTN
          msg = self.packer.make_can_msg_safety("PCM_BUTTONS", self.CAM_BUS, values)
          should_tx = not (set_res or dec or inc or cancel)
          self.assertEqual(should_tx, self._tx(msg))

  def _button_values(self, **buttons):
    values = {"SET_ME_1_1": 1, "SET_ME_1_2": 1}
    values.update(buttons)
    return values

  def test_bus2_buttons_follow_stock(self):
    # Bus 2 may repeat SET/RES/DEC/INC/ACC_ON from the last two stock frames, plus LKAS_ON_BTN.
    stock = self.packer.make_can_msg_safety("PCM_BUTTONS", self.MAIN_BUS, self._button_values(SET_BTN=1))
    self.assertTrue(self._rx(stock))
    self.assertTrue(self._tx(self.packer.make_can_msg_safety("PCM_BUTTONS", self.CAM_BUS, self._button_values(SET_BTN=1))))
    self.assertTrue(self._tx(self.packer.make_can_msg_safety("PCM_BUTTONS", self.CAM_BUS, self._button_values(SET_BTN=1, LKAS_ON_BTN=1))))
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("PCM_BUTTONS", self.CAM_BUS, self._button_values(SET_BTN=1, RES_BTN=1))))

    self.assertTrue(self._rx(self.packer.make_can_msg_safety("PCM_BUTTONS", self.MAIN_BUS, self._button_values())))
    self.assertTrue(self._tx(self.packer.make_can_msg_safety("PCM_BUTTONS", self.CAM_BUS, self._button_values(SET_BTN=1))))
    self.assertTrue(self._rx(self.packer.make_can_msg_safety("PCM_BUTTONS", self.MAIN_BUS, self._button_values())))
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("PCM_BUTTONS", self.CAM_BUS, self._button_values(SET_BTN=1))))

  def test_long_msgs_blocked_without_long(self):
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("ACC_CMD", self.MAIN_BUS, {"ACCEL_CMD": 0.0})))
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("ACC_HUD_ADAS", self.MAIN_BUS, {"ACC_STATE": 2})))
    self.assertEqual(0, self.safety.safety_fwd_hook(self.CAM_BUS, ACC_CMD))
    self.assertEqual(0, self.safety.safety_fwd_hook(self.CAM_BUS, ACC_HUD_ADAS))
    self.assertEqual(2, self.safety.safety_fwd_hook(self.MAIN_BUS, PCM_BUTTONS))

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
    # 8-byte BYD frames use (~sum) in the last byte. STEER_MODULE_2 is 4-bit, so that stays ignored.
    checked = (
      self._driver_torque_msg(0),
      self._speed_msg(0),
      self._pcm_status_msg(False),
      self._lkas_hud_msg(1),
      self._lks_btn_msg(False),
      self._user_gas_msg(0),
      self._user_brake_msg(False),
    )
    for msg in checked:
      self.assertTrue(self._rx(msg))
      msg.data[7] ^= 0xFF
      self.assertFalse(self._rx(msg))

    ignored = (
      self._angle_meas_msg(0),
    )
    for msg in ignored:
      self.assertTrue(self._rx(msg))
      msg.data[len(msg.data) - 1] ^= 0x0F
      self.assertTrue(self._rx(msg))

  # BYD uses get_max_angle_delta_vm and get_max_angle_vm for lateral accel and jerk limits
  def _lkas_hud_tx(self, increment_timer: bool = True):
    if increment_timer:
      self.safety.set_timer(self.__class__.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("LKAS_HUD_ADAS", self.MAIN_BUS, {"LKAS_STATE": 1})

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

  def test_hud_only_tx_blocks_hud_not_steer(self):
    # Long-mode HUD hold: OP paints 0x316 only so camera 0x1E2 can reach the EPS.
    self.assertEqual(0, self.safety.safety_fwd_hook(2, STEERING_MODULE_ADAS))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, LKAS_HUD_ADAS))
    self.assertTrue(self._tx(self._lkas_hud_tx()))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, STEERING_MODULE_ADAS))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, LKAS_HUD_ADAS))
    t = (self.__class__.cnt_angle_cmd - 1) * int(1e6 / self.LATERAL_FREQUENCY)
    self.safety.set_timer(t + 2001000)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, LKAS_HUD_ADAS))

  def test_hud_tx_extends_hud_block_after_steer(self):
    # After the last OP 0x1E2, 50 Hz 0x316 keeps camera HUD blocked through the
    # hold while camera 0x1E2 unblocks at 200 ms.
    self.safety.set_controls_allowed(True)
    self._reset_speed_measurement(10)
    self._reset_angle_measurement(0)
    self.safety.set_desired_angle_last(0)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))
    t_steer = (self.__class__.cnt_angle_cmd - 1) * int(1e6 / self.LATERAL_FREQUENCY)
    controls_allowed = self.safety.get_controls_allowed()
    cruise = self.safety.get_cruise_engaged_prev()

    for _ in range(150):  # 3 s at 50 Hz
      self.assertTrue(self._tx(self._lkas_hud_tx()))
      self.assertEqual(-1, self.safety.safety_fwd_hook(2, LKAS_HUD_ADAS))
    t_hud = (self.__class__.cnt_angle_cmd - 1) * int(1e6 / self.LATERAL_FREQUENCY)

    self.safety.set_timer(t_steer + 201000)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, STEERING_MODULE_ADAS))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, LKAS_HUD_ADAS))
    self.assertEqual(controls_allowed, self.safety.get_controls_allowed())
    self.assertEqual(cruise, self.safety.get_cruise_engaged_prev())

    self.safety.set_timer(t_hud + 2001000)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, LKAS_HUD_ADAS))

  def test_steer_tx_still_blocks_hud(self):
    # OP 0x1E2 alone still blocks camera HUD for 2 s (superset of the old rule).
    self.safety.set_controls_allowed(True)
    self._reset_speed_measurement(10)
    self._reset_angle_measurement(0)
    self.safety.set_desired_angle_last(0)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))
    t = (self.__class__.cnt_angle_cmd - 1) * int(1e6 / self.LATERAL_FREQUENCY)
    self.safety.set_timer(t + 201000)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, STEERING_MODULE_ADAS))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, LKAS_HUD_ADAS))
    self.safety.set_timer(t + 2001000)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, LKAS_HUD_ADAS))

  def test_hud_timers_reset_on_init(self):
    self.assertTrue(self._tx(self._lkas_hud_tx()))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, LKAS_HUD_ADAS))
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, self.SAFETY_PARAM)
    self.safety.init_tests()
    self.__class__.cnt_angle_cmd = 1
    self.assertEqual(0, self.safety.safety_fwd_hook(2, STEERING_MODULE_ADAS))
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

  def test_acc_states_gate_controls(self):
    # ACC_STATE 2 is main (standby), not active - does not enable.
    self.assertTrue(self._rx(self.packer.make_can_msg_safety("ACC_HUD_ADAS", self.CAM_BUS, {"ACC_STATE": 2})))
    self.assertFalse(self.safety.get_controls_allowed())

    # ACC_STATE 5 (FORCE_ACCEL) enables like state 3 when LKS is on.
    self.assertTrue(self._rx(self.packer.make_can_msg_safety("ACC_HUD_ADAS", self.CAM_BUS, {"ACC_STATE": 5})))
    self.assertTrue(self.safety.get_controls_allowed())

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
        self.safety.set_desired_angle_last(sign * max_angle_can)
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


class TestBydLongitudinalSafety(TestBydSafety, common.LongitudinalAccelSafetyTest):
  SAFETY_PARAM = int(BydSafetyFlags.LKS_ON | BydSafetyFlags.LONG_CONTROL) | int(SAFETY_PLATFORM_PARAMS["BYD_ATTO_3"].safety_param)
  TX_MSGS = [[STEERING_MODULE_ADAS, 0], [LKAS_HUD_ADAS, 0], [ACC_HUD_ADAS, 0], [ACC_CMD, 0], [PCM_BUTTONS, 0], [PCM_BUTTONS, 2]]
  FWD_BLACKLISTED_ADDRS = {0: [PCM_BUTTONS], 2: [ACC_HUD_ADAS, ACC_CMD]}
  RELAY_MALFUNCTION_ADDRS = {0: (STEERING_MODULE_ADAS, LKAS_HUD_ADAS, ACC_HUD_ADAS, ACC_CMD)}
  MAX_ACCEL = 2.0
  MIN_ACCEL = -3.5
  INACTIVE_ACCEL = 0.0

  def _accel_msg(self, accel: float):
    values = {"ACCEL_CMD": accel, "ACC_ON_1": 1, "ACC_ON_2": 1, "SET_ME_XF": 0xF}
    return self.packer.make_can_msg_safety("ACC_CMD", self.MAIN_BUS, values)

  def _acc_cmd(self, accel, decel_factor=1, bus=None):
    values = {"ACCEL_CMD": accel, "DECEL_FACTOR": decel_factor, "SET_ME_XF": 0xF}
    return self.packer.make_can_msg_safety("ACC_CMD", self.CAM_BUS if bus is None else bus, values)

  def test_gas_pressed_blocks_accel(self):
    self.safety.set_controls_allowed(True)
    self.safety.set_gas_pressed_prev(True)
    self.assertTrue(self._tx(self._accel_msg(0.0)))
    self.assertFalse(self._tx(self._accel_msg(-0.5)))
    self.assertFalse(self._tx(self._accel_msg(0.5)))

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_lks_switch_gates_controls(self):
    pass

  def test_acc_states_gate_controls(self):
    # Long mode does not engage from ACC rising edge.
    pass

  def test_long_msgs_blocked_without_long(self):
    pass

  def test_bus2_buttons_follow_stock(self):
    pass

  def test_platform_bit_unknown_denies_accel(self):
    self._reinit_with_param(int(BydSafetyFlags.LKS_ON | BydSafetyFlags.LONG_CONTROL) | 0x200)
    self.assertFalse(self._tx(self._accel_msg(0.0)))

  def test_lks_button_on_camera_bus_does_not_toggle(self):
    self._arm(2, 0)
    self._tap(SET_BTN=1)
    self._tap()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._lks_btn_msg(True, bus=self.CAM_BUS)))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_camera_lkas_states_keep_controls(self):
    self._arm(2, 0)
    self._tap(SET_BTN=1)
    self._tap()
    self.assertTrue(self.safety.get_controls_allowed())
    for i, state in enumerate((2, 3, 1, 4, 0, 1, 3, 2)):
      self.assertTrue(self._rx(self._lkas_hud_msg(state)))
      self._arm(3, i + 1)
      self.assertTrue(self.safety.get_controls_allowed(), f"LKAS_STATE {state} dropped controls")

  def test_cruise_buttons(self):
    # Stock cruise state does not engage, so bus 0 cancel stays blocked.
    buttons = ("SET_BTN", "RES_BTN", "LKAS_ON_BTN", "DEC_DISTANCE_BTN", "INC_DISTANCE_BTN", "ACC_ON_BTN")
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    for pressed in itertools.product((False, True), repeat=len(buttons)):
      values = self._button_values(**dict(zip(buttons, pressed, strict=True)))
      set_res, lkas_on, dec, inc, cancel = pressed[0] or pressed[1], pressed[2], pressed[3], pressed[4], pressed[5]
      with self.subTest(buttons=pressed):
        msg = self.packer.make_can_msg_safety("PCM_BUTTONS", self.MAIN_BUS, values)
        should_tx = not (set_res or lkas_on or dec or inc or cancel)
        self.assertEqual(should_tx, self._tx(msg))
        msg = self.packer.make_can_msg_safety("PCM_BUTTONS", self.CAM_BUS, values)
        should_tx = not (set_res or dec or inc or cancel)
        self.assertEqual(should_tx, self._tx(msg))

  def _acc_state(self, state, counter=0):
    values = {
      "ACC_STATE": state,
      "ACC_ON1": 1 if state in (2, 3, 5) else 0,
      "ACC_ON2": (state >> 1) & 1,
      "SET_ME_B2_LO": 4,
      "COUNTER": counter,
    }
    return self.packer.make_can_msg_safety("ACC_HUD_ADAS", self.CAM_BUS, values)

  def _arm(self, state=2, counter=0):
    self.assertTrue(self._rx(self._acc_state(state, counter)))

  def _tap(self, **buttons):
    self.assertTrue(self._rx(self.packer.make_can_msg_safety("PCM_BUTTONS", self.MAIN_BUS, self._button_values(**buttons))))

  def test_set_resume_engage(self):
    for button in ("SET_BTN", "RES_BTN"):
      self._reset_safety_hooks()
      self.safety.init_tests()
      self._arm()
      self.assertFalse(self.safety.get_controls_allowed())
      self._tap(**{button: 1})
      self.assertFalse(self.safety.get_controls_allowed())
      self._tap()
      self.assertTrue(self.safety.get_controls_allowed())

    self._tap(ACC_ON_BTN=1)
    self.assertFalse(self.safety.get_controls_allowed())

    self._reset_safety_hooks()
    self.safety.init_tests()
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_long_engage_edges(self):
    # SET with ACC main off does not engage.
    self._tap(SET_BTN=1)
    self._tap()
    self.assertFalse(self.safety.get_controls_allowed())

    # Standby (state 2) is enough. Active state 3 is not required, because
    # the camera never sees SET in long mode.
    self._arm(2, 0)
    self._tap(SET_BTN=1)
    self._tap()
    self.assertTrue(self.safety.get_controls_allowed())

    # Same frame as the SET release, ACC_ON wins.
    self._tap(SET_BTN=1)
    self._tap(ACC_ON_BTN=1)
    self.assertFalse(self.safety.get_controls_allowed())

    # Re-engage, then ACC main off drops it and blocks the next SET.
    self._tap()
    self._tap(SET_BTN=1)
    self._tap()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._acc_state(0, 1)))
    self.assertFalse(self.safety.get_controls_allowed())
    self._tap(SET_BTN=1)
    self._tap()
    self.assertFalse(self.safety.get_controls_allowed())

    # State 7 is not main either.
    self._reset_safety_hooks()
    self.safety.init_tests()
    self._arm(7, 0)
    self._tap(SET_BTN=1)
    self._tap()
    self.assertFalse(self.safety.get_controls_allowed())

    # State 5 (FORCE_ACCEL) is main and allows SET engage.
    self._reset_safety_hooks()
    self.safety.init_tests()
    self._arm(5, 0)
    self._tap(SET_BTN=1)
    self._tap()
    self.assertTrue(self.safety.get_controls_allowed())

    # LKS off drops controls and a later SET does nothing until LKS is on again.
    self._arm(2, 1)
    self._tap(SET_BTN=1)
    self._tap()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._lks_btn_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed())
    self._tap(SET_BTN=1)
    self._tap()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._lks_btn_msg(False)))
    self.assertTrue(self._rx(self._lks_btn_msg(True)))
    self._tap(SET_BTN=1)
    self._tap()
    self.assertTrue(self.safety.get_controls_allowed())

    # Rolling with the brake held cannot engage. Stopped with the brake held can.
    self._reset_safety_hooks()
    self.safety.init_tests()
    self._arm(2, 0)
    self.assertTrue(self._rx(self._speed_msg(10)))
    self.assertTrue(self._rx(self._user_brake_msg(True)))
    self._tap(SET_BTN=1)
    self._tap()
    self.assertFalse(self.safety.get_controls_allowed())

    self._reset_safety_hooks()
    self.safety.init_tests()
    self._arm(2, 0)
    self.assertTrue(self._rx(self._speed_msg(0)))
    self.assertTrue(self._rx(self._user_brake_msg(True)))
    self._tap(SET_BTN=1)
    self._tap()
    self.assertTrue(self.safety.get_controls_allowed())

    # Rising brake while moving disengages. Gas blocks non-zero accel only.
    self.assertTrue(self._rx(self._user_brake_msg(False)))
    self.assertTrue(self._rx(self.packer.make_can_msg_safety(
      "WHEELSPEED_CLEAN", self.MAIN_BUS, {"WHEELSPEED_CLEAN": 36, "COUNTER": 1})))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._user_brake_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed())

    self._reset_safety_hooks()
    self.safety.init_tests()
    self._arm(2, 0)
    self._tap(SET_BTN=1)
    self._tap()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._user_gas_msg(20)))
    self.assertFalse(self._tx(self._accel_msg(1.0)))
    self.assertTrue(self._tx(self._accel_msg(0.0)))
    self.assertTrue(self._rx(self._user_gas_msg(0)))
    self.assertTrue(self._tx(self._accel_msg(1.0)))
    self.assertFalse(self._tx(self._accel_msg(2.05)))
    self.assertFalse(self._tx(self._accel_msg(-3.55)))
    self.assertTrue(self._tx(self._accel_msg(2.0)))
    self.assertTrue(self._tx(self._accel_msg(-3.5)))

  def test_bus2_buttons_strip_acc(self):
    self.assertTrue(self._rx(self.packer.make_can_msg_safety("PCM_BUTTONS", self.MAIN_BUS, self._button_values(SET_BTN=1, ACC_ON_BTN=1))))
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("PCM_BUTTONS", self.CAM_BUS, self._button_values(SET_BTN=1))))
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("PCM_BUTTONS", self.CAM_BUS, self._button_values(RES_BTN=1))))
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("PCM_BUTTONS", self.CAM_BUS, self._button_values(DEC_DISTANCE_BTN=1))))
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("PCM_BUTTONS", self.CAM_BUS, self._button_values(INC_DISTANCE_BTN=1))))
    self.assertTrue(self._tx(self.packer.make_can_msg_safety("PCM_BUTTONS", self.CAM_BUS, self._button_values(ACC_ON_BTN=1))))
    self.assertTrue(self._tx(self.packer.make_can_msg_safety("PCM_BUTTONS", self.CAM_BUS, self._button_values(ACC_ON_BTN=1, LKAS_ON_BTN=1))))

  def test_camera_acc_cmd_passthrough(self):
    hard = -4.5
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._rx(self._acc_cmd(hard, 1)))
    self.assertTrue(self._tx(self._acc_cmd(hard, 1, bus=self.MAIN_BUS)))
    self.assertFalse(self._tx(self._acc_cmd(hard, 2, bus=self.MAIN_BUS)))

    self.assertTrue(self._rx(self._acc_cmd(hard, 2)))
    self.assertTrue(self._rx(self._acc_cmd(hard, 3)))
    self.assertTrue(self._tx(self._acc_cmd(hard, 1, bus=self.MAIN_BUS)))
    self.assertTrue(self._rx(self._acc_cmd(hard, 4)))
    self.assertFalse(self._tx(self._acc_cmd(hard, 1, bus=self.MAIN_BUS)))

    bad = self._acc_cmd(hard, 5)
    bad[0].data[7] ^= 0xFF
    self.assertFalse(self._rx(bad))
    self.assertFalse(self._tx(self._acc_cmd(hard, 5, bus=self.MAIN_BUS)))

  def test_acc_cmd_checksum_and_counter(self):
    for _ in range(16):
      self.assertTrue(self._rx(self._acc_cmd(0.0)))
    msg = self._acc_cmd(-1.0)
    msg[0].data[0] ^= 0xFF
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._rx(msg))
    self.assertFalse(self.safety.get_controls_allowed())

    self._reset_safety_hooks()
    self.safety.init_tests()
    msg = None
    for _ in range(32):
      msg = self._acc_cmd(0.0)
      self.assertTrue(self._rx(msg))
    self.safety.set_controls_allowed(True)
    for i in range(common.MAX_WRONG_COUNTERS + 1):
      should_rx = i < common.MAX_WRONG_COUNTERS - 1
      self.assertEqual(should_rx, self._rx(msg))

  def test_acc_hud_tx(self):
    self.assertTrue(self._tx(self.packer.make_can_msg_safety("ACC_HUD_ADAS", self.MAIN_BUS, {"ACC_STATE": 3, "SET_SPEED": 80})))


class TestBydIgnition(common.SafetyTestBase):
  DBC = "byd_atto3"
  SAFETY_MODEL = None

  TX_MSGS: list = []

  def _msg(self, d4, bus=0):
    dat = bytes([0, 0, 0, 0, d4, 0, 0, 0])
    return common.make_msg(bus, 0x055, dat=dat)

  def test_ignition_on_for_0x82(self):
    self.safety.ignition_can_hook(self._msg(0x82))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off_for_0x80(self):
    self.safety.ignition_can_hook(self._msg(0x82))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0x80))
    self.assertFalse(self.safety.get_ignition_can())

  def test_ignition_ignores_other_bus(self):
    self.safety.ignition_can_hook(self._msg(0x82, bus=2))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
