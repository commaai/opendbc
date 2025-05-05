#!/usr/bin/env python3
import unittest

from opendbc.car.tesla.values import TeslaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.can.can_define import CANDefine
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda

MSG_DAS_steeringControl = 0x488
MSG_APS_eacMonitor = 0x27d
MSG_DAS_Control = 0x2b9


class TestTeslaSafetyBase(common.PandaCarSafetyTest, common.AngleSteeringSafetyTest, common.LongitudinalAccelSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_DAS_steeringControl, MSG_APS_eacMonitor)}
  FWD_BLACKLISTED_ADDRS = {2: [MSG_DAS_steeringControl, MSG_APS_eacMonitor]}
  TX_MSGS = [[MSG_DAS_steeringControl, 0], [MSG_APS_eacMonitor, 0], [MSG_DAS_Control, 0]]

  STANDSTILL_THRESHOLD = 0.1
  GAS_PRESSED_THRESHOLD = 3

  # Angle control limits
  STEER_ANGLE_MAX = 360  # deg
  DEG_TO_CAN = 10

  ANGLE_RATE_BP = [0., 5., 25.]
  ANGLE_RATE_UP = [2.5, 1.5, 0.2]  # windup limit
  ANGLE_RATE_DOWN = [5., 2.0, 0.3]  # unwind limit

  # Long control limits
  MAX_ACCEL = 2.0
  MIN_ACCEL = -3.48
  INACTIVE_ACCEL = 0.0

  cnt_epas = 0

  packer: CANPackerPanda

  def setUp(self):
    self.packer = CANPackerPanda("tesla_model3_party")
    self.define = CANDefine("tesla_model3_party")
    self.acc_states = {d: v for v, d in self.define.dv["DAS_control"]["DAS_accState"].items()}
    self.autopark_states = {d: v for v, d in self.define.dv["DI_state"]["DI_autoparkState"].items()}

    self.active_autopark_states = [self.autopark_states[s] for s in ('ACTIVE', 'COMPLETE', 'SELFPARK_STARTED')]

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    values = {"DAS_steeringAngleRequest": angle, "DAS_steeringControlType": 1 if enabled else 0}
    return self.packer.make_can_msg_panda("DAS_steeringControl", 0, values)

  def _angle_meas_msg(self, angle: float, hands_on_level: int = 0, eac_status: int = 1, eac_error_code: int = 0):
    values = {"EPAS3S_internalSAS": angle, "EPAS3S_handsOnLevel": hands_on_level,
              "EPAS3S_eacStatus": eac_status, "EPAS3S_eacErrorCode": eac_error_code,
              "EPAS3S_sysStatusCounter": self.cnt_epas % 16}
    self.__class__.cnt_epas += 1
    return self.packer.make_can_msg_panda("EPAS3S_sysStatus", 0, values)

  def _user_brake_msg(self, brake):
    values = {"IBST_driverBrakeApply": 2 if brake else 1}
    return self.packer.make_can_msg_panda("IBST_status", 0, values)

  def _speed_msg(self, speed):
    values = {"DI_vehicleSpeed": speed * 3.6}
    return self.packer.make_can_msg_panda("DI_speed", 0, values)

  def _vehicle_moving_msg(self, speed: float):
    values = {"DI_cruiseState": 3 if speed <= self.STANDSTILL_THRESHOLD else 2}
    return self.packer.make_can_msg_panda("DI_state", 0, values)

  def _user_gas_msg(self, gas):
    values = {"DI_accelPedalPos": gas}
    return self.packer.make_can_msg_panda("DI_systemStatus", 0, values)

  def _pcm_status_msg(self, enable, autopark_state=0):
    values = {
      "DI_cruiseState": 2 if enable else 0,
      "DI_autoparkState": autopark_state,
    }
    return self.packer.make_can_msg_panda("DI_state", 0, values)

  def _long_control_msg(self, set_speed, acc_state=0, jerk_limits=(0, 0), accel_limits=(0, 0), aeb_event=0, bus=0):
    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": aeb_event,
      "DAS_jerkMin": jerk_limits[0],
      "DAS_jerkMax": jerk_limits[1],
      "DAS_accelMin": accel_limits[0],
      "DAS_accelMax": accel_limits[1],
    }
    return self.packer.make_can_msg_panda("DAS_control", bus, values)

  def _accel_msg(self, accel: float):
    # For common.LongitudinalAccelSafetyTest
    return self._long_control_msg(10, accel_limits=(accel, max(accel, 0)))

  def test_vehicle_speed_measurements(self):
    # OVERRIDDEN: 79.1667 is the max speed in m/s
    self._common_measurement_test(self._speed_msg, 0, 285 / 3.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  def test_steering_wheel_disengage(self):
    # Tesla disengages when the user forcibly overrides the locked-in angle steering control
    # Either when the hands on level is high, or if there is a high angle rate fault
    for hands_on_level in range(4):
      for eac_status in range(8):
        for eac_error_code in range(16):
          self.safety.set_controls_allowed(True)

          should_disengage = hands_on_level >= 3 or (eac_status == 0 and eac_error_code == 9)
          self.assertTrue(self._rx(self._angle_meas_msg(0, hands_on_level=hands_on_level, eac_status=eac_status,
                                                        eac_error_code=eac_error_code)))
          self.assertNotEqual(should_disengage, self.safety.get_controls_allowed())
          self.assertEqual(should_disengage, self.safety.get_steering_disengage_prev())

          # Should not recover
          self.assertTrue(self._rx(self._angle_meas_msg(0, hands_on_level=0, eac_status=1, eac_error_code=0)))
          self.assertNotEqual(should_disengage, self.safety.get_controls_allowed())
          self.assertFalse(self.safety.get_steering_disengage_prev())

  def test_autopark_summon_while_enabled(self):
    # We should not respect Autopark that activates while controls are allowed
    self.safety.set_controls_allowed(True)

    self._rx(self._pcm_status_msg(True, self.autopark_states["SELFPARK_STARTED"]))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
    self.assertTrue(self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"])))

    # We should still not respect Autopark if we disengage cruise
    self._rx(self._pcm_status_msg(False, self.autopark_states["SELFPARK_STARTED"]))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))
    self.assertTrue(self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"])))

  def test_autopark_summon_behavior(self):
    for autopark_state in range(16):
      self._rx(self._pcm_status_msg(False, 0))

      # We shouldn't allow controls if Autopark is an active state
      autopark_active = autopark_state in self.active_autopark_states
      self._rx(self._pcm_status_msg(False, autopark_state))
      self._rx(self._pcm_status_msg(True, autopark_state))
      self.assertNotEqual(autopark_active, self.safety.get_controls_allowed())

      # We should also start blocking all inactive/active openpilot msgs
      self.assertNotEqual(autopark_active, self._tx(self._angle_cmd_msg(0, False)))
      self.assertNotEqual(autopark_active, self._tx(self._angle_cmd_msg(0, True)))
      self.assertNotEqual(autopark_active, self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"])))
      self.assertNotEqual(autopark_active or not self.LONGITUDINAL, self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_ON"])))

      # Regain controls when Autopark disables
      self._rx(self._pcm_status_msg(True, 0))
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))
      self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
      self.assertTrue(self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"])))
      self.assertEqual(self.LONGITUDINAL, self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_ON"])))


class TestTeslaStockSafety(TestTeslaSafetyBase):

  LONGITUDINAL = False

  def setUp(self):
    super().setUp()
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, 0)
    self.safety.init_tests()

  def test_cancel(self):
    for acc_state in range(16):
      self.safety.set_controls_allowed(True)
      should_tx = acc_state == self.acc_states["ACC_CANCEL_GENERIC_SILENT"]
      self.assertFalse(self._tx(self._long_control_msg(0, acc_state=acc_state, accel_limits=(self.MIN_ACCEL, self.MAX_ACCEL))))
      self.assertEqual(should_tx, self._tx(self._long_control_msg(0, acc_state=acc_state)))

  def test_no_aeb(self):
    for aeb_event in range(4):
      should_tx = aeb_event == 0
      ret = self._tx(self._long_control_msg(10, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"], aeb_event=aeb_event))
      self.assertEqual(ret, should_tx)

  def test_stock_aeb_no_cancel(self):
    # No passthrough logic since we always forward DAS_control,
    # but ensure we can't send cancel cmd while stock AEB is active
    no_aeb_msg = self._long_control_msg(10, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"], aeb_event=0)
    no_aeb_msg_cam = self._long_control_msg(10, aeb_event=0, bus=2)
    aeb_msg_cam = self._long_control_msg(10, aeb_event=1, bus=2)

    # stock system sends no AEB -> no forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, no_aeb_msg_cam.addr))
    self.assertTrue(self._tx(no_aeb_msg))

    # stock system sends AEB -> forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, aeb_msg_cam.addr))
    self.assertFalse(self._tx(no_aeb_msg))


class TestTeslaLongitudinalSafety(TestTeslaSafetyBase):
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_DAS_steeringControl, MSG_APS_eacMonitor, MSG_DAS_Control)}
  FWD_BLACKLISTED_ADDRS = {2: [MSG_DAS_steeringControl, MSG_APS_eacMonitor, MSG_DAS_Control]}

  def setUp(self):
    super().setUp()
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, TeslaSafetyFlags.LONG_CONTROL)
    self.safety.init_tests()

  def test_no_aeb(self):
    for aeb_event in range(4):
      self.assertEqual(self._tx(self._long_control_msg(10, aeb_event=aeb_event)), aeb_event == 0)

  def test_stock_aeb_passthrough(self):
    no_aeb_msg = self._long_control_msg(10, aeb_event=0)
    no_aeb_msg_cam = self._long_control_msg(10, aeb_event=0, bus=2)
    aeb_msg_cam = self._long_control_msg(10, aeb_event=1, bus=2)

    # stock system sends no AEB -> no forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_aeb_msg_cam))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, no_aeb_msg_cam.addr))
    self.assertTrue(self._tx(no_aeb_msg))

    # stock system sends AEB -> forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, aeb_msg_cam.addr))
    self.assertFalse(self._tx(no_aeb_msg))

  def test_prevent_reverse(self):
    # Note: Tesla can reverse while at a standstill if both accel_min and accel_max are negative.
    self.safety.set_controls_allowed(True)

    # accel_min and accel_max are positive
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(1.1, 0.8))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(1.1, 0.8))))

    # accel_min and accel_max are both zero
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(0, 0))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0, 0))))

    # accel_min and accel_max have opposing signs
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-0.8, 1.3))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0.8, -1.3))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0, -1.3))))

    # accel_min and accel_max are negative
    self.assertFalse(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-1.1, -0.6))))
    self.assertFalse(self._tx(self._long_control_msg(set_speed=0, accel_limits=(-0.6, -1.1))))
    self.assertFalse(self._tx(self._long_control_msg(set_speed=0, accel_limits=(-0.1, -0.1))))


if __name__ == "__main__":
  unittest.main()
