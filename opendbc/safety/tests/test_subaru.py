#!/usr/bin/env python3
import enum
import unittest
import numpy as np

from functools import partial

from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.subaru.values import CAR, CarControllerParams, SubaruSafetyFlags
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.structs import CarParams, CarControl, CarState
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


class SubaruMsg(enum.IntEnum):
  Brake_Status      = 0x13c
  CruiseControl     = 0x240
  Throttle          = 0x40
  Steering_Torque   = 0x119
  Wheel_Speeds      = 0x13a
  ES_LKAS           = 0x122
  ES_LKAS_ANGLE     = 0x124
  ES_Brake          = 0x220
  ES_Status         = 0x222
  ES_Distance       = 0x221
  ES_DashStatus     = 0x321
  ES_LKAS_State     = 0x322
  ES_Infotainment   = 0x323


SUBARU_MAIN_BUS = 0
SUBARU_ALT_BUS  = 1
SUBARU_CAM_BUS  = 2


def lkas_tx_msgs(alt_bus, lkas_msg=SubaruMsg.ES_LKAS):
  return [[lkas_msg,                    SUBARU_MAIN_BUS],
          [SubaruMsg.ES_Distance,       alt_bus],
          [SubaruMsg.ES_DashStatus,     SUBARU_MAIN_BUS],
          [SubaruMsg.ES_LKAS_State,     SUBARU_MAIN_BUS],
          [SubaruMsg.ES_Infotainment,   SUBARU_MAIN_BUS]]


def fwd_blacklisted_addr(lkas_msg=SubaruMsg.ES_LKAS):
  return {SUBARU_CAM_BUS: [lkas_msg, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State, SubaruMsg.ES_Infotainment]}


class TestSubaruSafetyBase(common.CarSafetyTest):
  FLAGS = 0
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment)}
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr()

  MAX_RT_DELTA = 940

  DRIVER_TORQUE_ALLOWANCE = 60
  DRIVER_TORQUE_FACTOR = 50

  ALT_MAIN_BUS = SUBARU_MAIN_BUS
  ALT_CAM_BUS = SUBARU_CAM_BUS

  DEG_TO_CAN = 100

  INACTIVE_GAS = 1818

  def setUp(self):
    self.packer = CANPackerSafety("subaru_global_2017_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
    self.safety.init_tests()

  def _set_prev_torque(self, t):
    self.safety.set_desired_torque_last(t)
    self.safety.set_rt_torque_last(t)

  def _torque_driver_msg(self, torque):
    values = {"Steer_Torque_Sensor": torque}
    return self.packer.make_can_msg_safety("Steering_Torque", 0, values)

  def _speed_msg(self, speed):
    values = {s: speed for s in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_safety("Wheel_Speeds", self.ALT_MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"Brake": brake}
    return self.packer.make_can_msg_safety("Brake_Status", self.ALT_MAIN_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"Throttle_Pedal": gas}
    return self.packer.make_can_msg_safety("Throttle", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_safety("CruiseControl", self.ALT_MAIN_BUS, values)

  def test_rx_wrong_bus(self):
    signals = [
      (self._torque_driver_msg, self.safety.get_torque_driver_max),
      (self._speed_msg, self.safety.get_vehicle_speed_max),
      (self._user_brake_msg, self.safety.get_brake_pressed_prev),
      (self._user_gas_msg, self.safety.get_gas_pressed_prev),
      (self._pcm_status_msg, self.safety.get_controls_allowed),
    ]
    if self.FLAGS & SubaruSafetyFlags.LKAS_ANGLE:
      signals.append((self._angle_meas_msg, self.safety.get_angle_meas_max))
    for make_msg, get_value in signals:
      for bus in range(3):
        self.setUp()
        msg = make_msg(1)
        if bus != msg[0].bus:
          with self.subTest(message=msg[0].addr, bus=bus):
            msg[0].bus = bus
            self._rx(msg)
            self.assertEqual(get_value(), 0)


class TestSubaruStockLongitudinalSafetyBase(TestSubaruSafetyBase):
  def _cancel_msg(self, cancel, cruise_throttle=0):
    values = {"Cruise_Cancel": cancel, "Cruise_Throttle": cruise_throttle}
    return self.packer.make_can_msg_safety("ES_Distance", self.ALT_MAIN_BUS, values)

  def test_cancel_message(self):
    # test that we can only send the cancel message (ES_Distance) with inactive throttle (1818) and Cruise_Cancel=1
    for cancel in [True, False]:
      self._generic_limit_safety_check(partial(self._cancel_msg, cancel), self.INACTIVE_GAS, self.INACTIVE_GAS, 0, 2**12, 1, self.INACTIVE_GAS, cancel)


class TestSubaruTorqueSafetyBase(TestSubaruSafetyBase, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):
  MAX_RATE_UP = 50
  MAX_RATE_DOWN = 70
  MAX_TORQUE_LOOKUP = [0], [2047]

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 7
  MAX_INVALID_STEERING_FRAMES = 1
  STEER_STEP = 2

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Output": torque, "LKAS_Request": steer_req}
    return self.packer.make_can_msg_safety("ES_LKAS", SUBARU_MAIN_BUS, values)


class TestSubaruGen1TorqueStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruTorqueSafetyBase):
  FLAGS = 0
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS)


class TestSubaruGen2TorqueSafetyBase(TestSubaruTorqueSafetyBase):
  ALT_MAIN_BUS = SUBARU_ALT_BUS
  ALT_CAM_BUS = SUBARU_ALT_BUS

  MAX_RATE_UP = 40
  MAX_RATE_DOWN = 40
  MAX_TORQUE_LOOKUP = [0], [1000]


class TestSubaruGen2TorqueStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruGen2TorqueSafetyBase):
  FLAGS = SubaruSafetyFlags.GEN2
  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS)


class TestSubaruAngleSafetyBase(TestSubaruSafetyBase, common.AngleSteeringSafetyTest):
  STEER_ANGLE_MAX = 190
  DEG_TO_CAN = 100

  # VM-based limits, not breakpoint-based
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  LATERAL_FREQUENCY = 50

  cnt_angle_cmd = 0

  def setUp(self):
    self.__class__.cnt_angle_cmd = 0
    super().setUp()
    from opendbc.car.subaru.carcontroller import get_safety_CP
    self.VM = VehicleModel(get_safety_CP())
    self.limits = CarControllerParams(get_safety_CP())

  def _speed_msg(self, speed):
    # speed is in m/s for angle tests, convert to kph for DBC
    speed_kph = speed * 3.6
    values = {s: speed_kph for s in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_safety("Wheel_Speeds", self.ALT_MAIN_BUS, values)

  def _angle_cmd_msg(self, angle, enabled, increment_timer=True):
    values = {"LKAS_Output": angle, "LKAS_Request": enabled, "SET_3": 3}
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("ES_LKAS_ANGLE", SUBARU_MAIN_BUS, values)

  def _angle_meas_msg(self, angle):
    values = {"Steering_Angle": angle}
    return self.packer.make_can_msg_safety("Steering_2", SUBARU_MAIN_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_safety("ES_Status", self.ALT_MAIN_BUS, values)

  def test_stale_es_brake_cannot_engage(self):
    self._rx(self._speed_msg(0))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(False))
    self._rx(self._user_brake_msg(False))
    # ES_Brake may remain high after ACC cancels at a stop.
    self._rx(self.packer.make_can_msg_safety("ES_Brake", self.ALT_MAIN_BUS, {"Cruise_Activated": 1}))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_controller_curve_engagement(self):
    platform = CAR.SUBARU_CROSSTREK_2025 if self.FLAGS & SubaruSafetyFlags.GEN2 else CAR.SUBARU_FORESTER_2022
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        self.setUp()
        ci = CarInterface(CarInterface.get_non_essential_params(platform))
        ci.update([])
        angle = sign * 57.61
        ci.CS.out = CarState(vEgo=13.24, vEgoRaw=13.24, steeringAngleDeg=angle)
        self._reset_speed_measurement(13.24)
        for _ in range(6):
          self._rx(self._angle_meas_msg(angle))
        cc = CarControl()
        cc.actuators.steeringAngleDeg = sign * 51.60
        for frame in range(100):
          cc.latActive = frame > 0
          self.safety.set_controls_allowed(cc.latActive)
          self.safety.set_timer(frame * 10000)
          _, messages = ci.CC.update(cc.as_reader(), ci.CS, frame * 10000000)
          for addr, data, bus in messages:
            if addr == SubaruMsg.ES_LKAS_ANGLE:
              self.assertTrue(self._tx(libsafety_py.make_CANPacket(addr, bus, data)), f"frame {frame}")

  def test_angle_cmd_when_enabled(self):
    # VM-based limits are tested below
    pass

  def test_lateral_limits(self):
    for speed in np.linspace(0, 50, 101):
      self._reset_speed_measurement(speed)
      # Use the decoded wheel speed, including safety's 1 m/s tolerance.
      limit_speed = max(self.safety.get_vehicle_speed_min() - 1, 1)
      for sign in (-1, 1):
        for jerk in (False, True):
          limit = (get_max_angle_delta_vm if jerk else get_max_angle_vm)(limit_speed, self.VM, self.limits)
          # Bracket the boundary with room for C float rounding and the one CAN-unit tolerance.
          for offset, allowed in ((-1, True), (2, False)):
            angle_can = int(limit * self.DEG_TO_CAN) + offset
            if angle_can > self.STEER_ANGLE_MAX * self.DEG_TO_CAN:
              continue
            with self.subTest(speed=speed, sign=sign, jerk=jerk, offset=offset):
              self.safety.set_controls_allowed(True)
              self.safety.set_desired_angle_last(0 if jerk else angle_can * sign)
              self.assertEqual(allowed, self._tx(self._angle_cmd_msg(angle_can / self.DEG_TO_CAN * sign, True)))


class TestSubaruGen1AngleStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruAngleSafetyBase):
  FLAGS = SubaruSafetyFlags.LKAS_ANGLE
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS, SubaruMsg.ES_LKAS_ANGLE)
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS_ANGLE, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment)}
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr(SubaruMsg.ES_LKAS_ANGLE)


class TestSubaruGen2AngleStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruAngleSafetyBase):
  ALT_MAIN_BUS = SUBARU_ALT_BUS
  FLAGS = SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.LKAS_ANGLE
  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS, SubaruMsg.ES_LKAS_ANGLE)
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS_ANGLE, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment)}
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr(SubaruMsg.ES_LKAS_ANGLE)


class TestSubaruGen1LongitudinalDisabled(TestSubaruGen1TorqueStockLongitudinalSafety):
  FLAGS = SubaruSafetyFlags.LONG


class TestSubaruGen2LongitudinalDisabled(TestSubaruGen2TorqueStockLongitudinalSafety):
  FLAGS = SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.LONG


if __name__ == "__main__":
  unittest.main()
