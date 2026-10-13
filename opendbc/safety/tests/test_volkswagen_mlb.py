#!/usr/bin/env python3
import unittest
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common

MSG_LS_01 = 0x10B       # TX by OP, ACC control buttons for cancel/resume
MSG_HCA_01 = 0x126      # TX by OP, Heading Control Assist steering torque
MSG_LDW_02 = 0x397      # TX by OP, Lane line recognition and text alerts


class TestVolkswagenMlbSafetyBase(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_01, MSG_LDW_02)}

  MAX_RATE_UP = 9
  MAX_RATE_DOWN = 10
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 169

  DRIVER_TORQUE_ALLOWANCE = 60
  DRIVER_TORQUE_FACTOR = 3

  # Wheel speeds
  def _esp_03_msg(self, speeds: dict[str, float]):
    values = {f"ESP_{s}_Radgeschw": speed for s, speed in speeds.items()}
    return self.packer.make_can_msg_safety("ESP_03", 0, values)

  def _speed_msg(self, speed):
    return self._esp_03_msg(dict.fromkeys(("HL", "HR", "VL", "VR"), speed))

  # Driver brake pressure over threshold
  def _esp_05_msg(self, brake):
    values = {"ESP_Fahrer_bremst": brake}
    return self.packer.make_can_msg_safety("ESP_05", 0, values)

  # Brake pedal switch
  def _motor_03_msg(self, brake_signal=False, gas_signal=0):
    values = {
      "MO_Fahrer_bremst": brake_signal,
      "MO_Fahrpedalrohwert_01": gas_signal,
    }
    return self.packer.make_can_msg_safety("Motor_03", 0, values)

  def _user_brake_msg(self, brake):
    return self._motor_03_msg(brake_signal=brake)

  def _user_gas_msg(self, gas):
    return self._motor_03_msg(gas_signal=gas)

  # ACC engagement status
  def _tsk_04_msg(self, tsk_status):
    values = {"TSK_Status_GRA_ACC_02": tsk_status}
    return self.packer.make_can_msg_safety("TSK_04", 1, values)

  def _tsk_status_msg(self, enable):
    return self._tsk_04_msg(1 if enable else 0)

  def _pcm_status_msg(self, enable):
    return self._tsk_status_msg(enable)

  # Driver steering input torque
  def _torque_driver_msg(self, torque):
    values = {"EPS_Lenkmoment": abs(torque), "EPS_VZ_Lenkmoment": torque < 0}
    return self.packer.make_can_msg_safety("LH_EPS_03", 0, values)

  # openpilot steering output torque
  def _torque_cmd_msg(self, torque, steer_req=1, hca_status=7):
    values = {"HCA_01_LM_Offset": abs(torque),
              "HCA_01_LM_OffSign": torque < 0,
              "HCA_01_Sendestatus": steer_req,
              "HCA_01_Status_HCA": hca_status if steer_req else 3}
    return self.packer.make_can_msg_safety("HCA_01", 0, values)

  # Cruise control buttons
  def _ls_01_msg(self, cancel=0, resume=0, _set=0, bus=2):
    values = {"LS_Abbrechen": cancel, "LS_Tip_Setzen": _set, "LS_Tip_Wiederaufnahme": resume}
    return self.packer.make_can_msg_safety("LS_01", bus, values)

  # Verify brake_pressed is true if either the switch or pressure threshold signals are true
  def test_redundant_brake_signals(self):
    test_combinations = [(True, True, True), (True, True, False), (True, False, True), (False, False, False)]
    for brake_pressed, motor_03_signal, esp_05_signal in test_combinations:
      self._rx(self._motor_03_msg(brake_signal=False))
      self._rx(self._esp_05_msg(False))
      self.assertFalse(self.safety.get_brake_pressed_prev())
      self._rx(self._motor_03_msg(brake_signal=motor_03_signal))
      self._rx(self._esp_05_msg(esp_05_signal))
      self.assertEqual(brake_pressed, self.safety.get_brake_pressed_prev(),
                       f"expected {brake_pressed=} with {motor_03_signal=} and {esp_05_signal=}")

  def test_vehicle_moving_any_wheel(self):
    # any single wheel turning, at the smallest speed the ABS reports, means the vehicle is moving
    for wheel in ("HL", "HR", "VL", "VR"):
      self._rx(self._speed_msg(0))
      self.assertFalse(self.safety.get_vehicle_moving())
      self._rx(self._esp_03_msg({wheel: 0.1}))
      self.assertTrue(self.safety.get_vehicle_moving(), f"vehicle not moving with only {wheel} turning")

  def test_tsk_status(self):
    # ACC active (1) and driver override (2) are the engaged states, off (0) and fault (3) are not
    for tsk_status in range(4):
      self._rx(self._tsk_04_msg(0))
      self.assertFalse(self.safety.get_controls_allowed())
      self._rx(self._tsk_04_msg(tsk_status))
      self.assertEqual(tsk_status in (1, 2), self.safety.get_controls_allowed(), f"{tsk_status=}")
      self.assertEqual(tsk_status in (1, 2), self.safety.get_cruise_engaged_prev(), f"{tsk_status=}")

  def test_torque_cmd_enable_variants(self):
    # The EPS rack accepts either 5 or 7 for an enabled status, torque with any other status is blocked
    for hca_status in range(16):
      self.safety.set_controls_allowed(True)
      self._set_prev_torque(0)
      should_tx = hca_status in (5, 7)
      self.assertEqual(should_tx, self._tx(self._torque_cmd_msg(self.MAX_RATE_UP, hca_status=hca_status)),
                       f"torque cmd with {hca_status=}")


class TestVolkswagenMlbStockSafety(TestVolkswagenMlbSafetyBase):
  DBC = "vw_mlb"
  SAFETY_MODEL = CarParams.SafetyModel.volkswagenMlb

  TX_MSGS = [[MSG_HCA_01, 0], [MSG_LDW_02, 0], [MSG_LS_01, 0], [MSG_LS_01, 2]]
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_01, MSG_LDW_02]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  def test_spam_cancel_safety_check(self):
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._ls_01_msg(cancel=1)))
    self.assertFalse(self._tx(self._ls_01_msg(resume=1)))
    self.assertFalse(self._tx(self._ls_01_msg(_set=1)))
    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._ls_01_msg(resume=1)))

  def test_cancel_button(self):
    # Disable on rising edge of cancel button
    self._rx(self._tsk_status_msg(False))
    self.safety.set_controls_allowed(1)
    self._rx(self._ls_01_msg(bus=0))
    self.assertTrue(self.safety.get_controls_allowed(), "controls not allowed without cancel")
    self._rx(self._ls_01_msg(cancel=True, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after cancel")


if __name__ == "__main__":
  unittest.main()
