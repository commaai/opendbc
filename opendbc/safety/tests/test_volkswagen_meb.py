#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


# MEB message IDs
MSG_LH_EPS_03  = 0x9F
MSG_ESC_51     = 0xFC
MSG_Motor_51   = 0x10B
MSG_GRA_ACC_01 = 0x12B
MSG_QFK_01     = 0x13D
MSG_KLR_01     = 0x25D
MSG_HCA_03     = 0x303
MSG_LDW_02     = 0x397
MSG_MOTOR_14   = 0x3BE


class TestVolkswagenMebSafetyBase(common.CarSafetyTest):
  STANDSTILL_THRESHOLD = 0
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02), 2: (MSG_KLR_01,)}

  CURVATURE_TO_CAN = 149253.7313

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

  def _button_msg(self, cancel=0, resume=0, set=0, bus=2):
    values = {"GRA_Abbrechen": cancel, "GRA_Tip_Setzen": set, "GRA_Tip_Wiederaufnahme": resume}
    return self.packer.make_can_msg_safety("GRA_ACC_01", bus, values)

  def test_brake_signal(self):
    self._rx(self._user_brake_msg(False))
    self.assertFalse(self.safety.get_brake_pressed_prev())
    self._rx(self._user_brake_msg(True))
    self.assertTrue(self.safety.get_brake_pressed_prev())

  def test_curvature_measurements(self):
    for c in (0.0, 0.05, -0.05, 0.15, -0.15):
      self._rx(self._curvature_meas_msg(c))

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

  def test_curvature_cmd_safety_check(self):
    # Exercise tx HCA_03 path with both signs and steer_req states
    for c in (0.0, 0.01, -0.01, 0.1, -0.1):
      for steer_req in (0, 1):
        self.safety.set_controls_allowed(True)
        self._tx(self._curvature_cmd_msg(c, steer_req=steer_req))

    # When controls are not allowed, only steer_req=0 with small curvature is allowed
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._curvature_cmd_msg(0.0, steer_req=0)))
    self.assertFalse(self._tx(self._curvature_cmd_msg(0.1, steer_req=1)))


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
    self.assertFalse(self._tx(self._button_msg(set=1)))
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(resume=1)))


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
