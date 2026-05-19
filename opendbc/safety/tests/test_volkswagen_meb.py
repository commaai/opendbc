#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags

MAX_ACCEL = 2.0
MIN_ACCEL = -3.5

# MEB message IDs
MSG_LH_EPS_03  = 0x9F
MSG_ESC_51     = 0xFC
MSG_Motor_51   = 0x10B
MSG_GRA_ACC_01 = 0x12B
MSG_QFK_01     = 0x13D
MSG_ACC_18     = 0x14D
MSG_KLR_01     = 0x25D
MSG_MEB_ACC_01 = 0x300
MSG_HCA_03     = 0x303
MSG_LDW_02     = 0x397
MSG_MOTOR_14   = 0x3BE


class TestVolkswagenMebSafetyBase(common.CarSafetyTest, common.CurvatureSteeringSafetyTest):
  STANDSTILL_THRESHOLD = 0
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02), 2: (MSG_KLR_01,)}

  MAX_CURVATURE = 29105
  MAX_CURVATURE_TEST = 0.195
  CURVATURE_TO_CAN = 149253.7313
  INACTIVE_CURVATURE_IS_ZERO = True
  MAX_POWER = 125
  MAX_POWER_TEST = 50
  SEND_RATE = 0.02

  def _speed_msg(self, speed_mps: float):
    spd_kph = speed_mps * 3.6
    values = {f"{s}_Radgeschw": spd_kph for s in ("VL", "VR", "HL", "HR")}
    return self.packer.make_can_msg_safety("ESC_51", 0, values)

  def _motor_14_msg(self, brake):
    values = {"MO_Fahrer_bremst": brake}
    return self.packer.make_can_msg_safety("Motor_14", 0, values)

  def _user_brake_msg(self, brake):
    return self._motor_14_msg(brake)

  def _user_gas_msg(self, gas):
    values = {"Accel_Pedal_Pressure": 1 if gas else 0, "TSK_Status": 3}
    return self.packer.make_can_msg_safety("Motor_51", 0, values)

  def _vehicle_moving_msg(self, speed_mps: float):
    return self._speed_msg(speed_mps)

  def _curvature_meas_msg(self, curvature):
    values = {"Curvature": abs(curvature), "Curvature_VZ": curvature > 0}
    return self.packer.make_can_msg_safety("QFK_01", 0, values)

  def _curvature_cmd_msg(self, curvature, steer_req=1, power=50):
    values = {
      "Curvature": abs(curvature),
      "Curvature_VZ": curvature > 0,
      "RequestStatus": 4 if steer_req else 0,
      "Power": power,
    }
    return self.packer.make_can_msg_safety("HCA_03", 0, values)

  def _tsk_status_msg(self, enable, main_switch=True):
    if main_switch:
      tsk_status = 3 if enable else 2
    else:
      tsk_status = 0
    values = {"TSK_Status": tsk_status}
    return self.packer.make_can_msg_safety("Motor_51", 0, values)

  def _pcm_status_msg(self, enable):
    return self._tsk_status_msg(enable)

  def _torque_driver_msg(self, torque):
    values = {"EPS_Lenkmoment": abs(torque), "EPS_VZ_Lenkmoment": torque < 0}
    return self.packer.make_can_msg_safety("LH_EPS_03", 0, values)

  def _button_msg(self, cancel=0, resume=0, _set=0, bus=2):
    values = {"GRA_Abbrechen": cancel, "GRA_Tip_Setzen": _set, "GRA_Tip_Wiederaufnahme": resume}
    return self.packer.make_can_msg_safety("GRA_ACC_01", bus, values)

  def _accel_msg(self, accel):
    values = {"ACC_Sollbeschleunigung_02": accel}
    return self.packer.make_can_msg_safety("ACC_18", 0, values)

  def test_curvature_measurements(self):
    self._rx(self._curvature_meas_msg(0.15))
    self._rx(self._curvature_meas_msg(-0.1))
    self._rx(self._curvature_meas_msg(0))
    self._rx(self._curvature_meas_msg(0))
    self._rx(self._curvature_meas_msg(0))
    self._rx(self._curvature_meas_msg(0))

    self.assertEqual(int(-0.1 * self.CURVATURE_TO_CAN), self.safety.get_curvature_meas_min())
    self.assertEqual(int(0.15 * self.CURVATURE_TO_CAN), self.safety.get_curvature_meas_max())

    self._rx(self._curvature_meas_msg(0))
    self.assertEqual(0, self.safety.get_curvature_meas_max())
    self.assertEqual(int(-0.1 * self.CURVATURE_TO_CAN), self.safety.get_curvature_meas_min())

    self._rx(self._curvature_meas_msg(0))
    self.assertEqual(0, self.safety.get_curvature_meas_max())
    self.assertEqual(0, self.safety.get_curvature_meas_min())

  def test_brake_signal(self):
    self._rx(self._user_brake_msg(False))
    self.assertFalse(self.safety.get_brake_pressed_prev())
    self._rx(self._user_brake_msg(True))
    self.assertTrue(self.safety.get_brake_pressed_prev())

  def test_torque_driver_measurements(self):
    for t in (0, 100, -100, 250, -250):
      self._rx(self._torque_driver_msg(t))

  def test_main_switch_off_disables_controls(self):
    self.safety.set_controls_allowed(True)
    self._rx(self._tsk_status_msg(False, main_switch=False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_cancel_button_rising_edge(self):
    self.safety.set_controls_allowed(True)
    self._rx(self._button_msg(cancel=1, bus=0))
    self.assertFalse(self.safety.get_controls_allowed())


class TestVolkswagenMebStockSafety(TestVolkswagenMebSafetyBase):
  FWD_BLACKLISTED_ADDRS = {0: [MSG_KLR_01], 2: [MSG_HCA_03, MSG_LDW_02]}
  TX_MSGS = [[MSG_HCA_03, 0], [MSG_LDW_02, 0], [MSG_GRA_ACC_01, 0], [MSG_GRA_ACC_01, 2],
             [MSG_KLR_01, 0], [MSG_KLR_01, 2]]

  def setUp(self):
    self.packer = CANPackerSafety("vw_meb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMeb, 0)
    self.safety.init_tests()

  def test_spam_cancel_safety_check(self):
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=1)))
    self.assertFalse(self._tx(self._button_msg(resume=1)))
    self.assertFalse(self._tx(self._button_msg(_set=1)))
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(resume=1)))


class TestVolkswagenMebLongSafety(TestVolkswagenMebSafetyBase):
  TX_MSGS = [[MSG_HCA_03, 0], [MSG_LDW_02, 0], [MSG_GRA_ACC_01, 0], [MSG_GRA_ACC_01, 2],
             [MSG_KLR_01, 0], [MSG_KLR_01, 2], [MSG_MEB_ACC_01, 0], [MSG_ACC_18, 0]]
  FWD_BLACKLISTED_ADDRS = {0: [MSG_KLR_01], 2: [MSG_HCA_03, MSG_LDW_02, MSG_MEB_ACC_01, MSG_ACC_18]}
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02, MSG_MEB_ACC_01, MSG_ACC_18), 2: (MSG_KLR_01,)}

  INACTIVE_ACCEL = 3.01

  def setUp(self):
    self.packer = CANPackerSafety("vw_meb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMeb, VolkswagenSafetyFlags.LONG_CONTROL)
    self.safety.init_tests()

  # stock cruise controls are entirely bypassed under openpilot longitudinal control
  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_set_and_resume_buttons(self):
    for button in ["set", "resume"]:
      # ACC main switch must be on, engage on falling edge
      self.safety.set_controls_allowed(0)
      self._rx(self._tsk_status_msg(False, main_switch=False))
      self._rx(self._button_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} with main switch off")
      self._rx(self._tsk_status_msg(False, main_switch=True))
      self._rx(self._button_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} rising edge")
      self._rx(self._button_msg(bus=0))
      self.assertTrue(self.safety.get_controls_allowed(), f"controls not allowed on {button} falling edge")

  def test_cancel_button(self):
    # Disable on rising edge of cancel button
    self._rx(self._tsk_status_msg(False, main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._button_msg(cancel=True, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after cancel")

  def test_main_switch(self):
    # Disable as soon as main switch turns off
    self._rx(self._tsk_status_msg(False, main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._tsk_status_msg(False, main_switch=False))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after ACC main switch off")

  def test_accel_safety_check(self):
    for controls_allowed in [True, False]:
      # enforce we don't skip over 0 or inactive accel
      for accel in np.concatenate((np.arange(MIN_ACCEL - 2, MAX_ACCEL + 2, 0.03), [0, self.INACTIVE_ACCEL])):
        accel = round(accel, 2)  # floats might not hit exact boundary conditions without rounding
        is_inactive_accel = accel == self.INACTIVE_ACCEL
        send = (controls_allowed and MIN_ACCEL <= accel <= MAX_ACCEL) or is_inactive_accel
        self.safety.set_controls_allowed(controls_allowed)
        # accel request used by ECU
        self.assertEqual(send, self._tx(self._accel_msg(accel)), (controls_allowed, accel))


class TestVolkswagenMebIgnition(unittest.TestCase):
  TX_MSGS: list = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.packer = CANPackerSafety("vw_meb")

  def _msg(self, counter, ign):
    return self.packer.make_can_msg_safety("Klemmen_Status_01", 0,
                                           {"Klemmen_Status_01_BZ": counter,
                                            "ZAS_Kl_15": ign})

  def test_ignition_on(self):
    for i in range(16):
      self.safety.init_tests()
      self.safety.ignition_can_hook(self._msg(i, 1))
      self.assertFalse(self.safety.get_ignition_can())
      self.safety.ignition_can_hook(self._msg((i + 1) % 16, 1))
      self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(0, 1))
    self.safety.ignition_can_hook(self._msg(1, 1))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(2, 0))
    self.safety.ignition_can_hook(self._msg(3, 0))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
