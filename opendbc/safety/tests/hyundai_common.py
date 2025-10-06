import unittest

from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
import opendbc.safety.tests.common as common
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import make_msg


class Buttons:
  NONE = 0
  RESUME = 1
  SET = 2
  CANCEL = 4


PREV_BUTTON_SAMPLES = 8
ENABLE_BUTTONS = (Buttons.RESUME, Buttons.SET, Buttons.CANCEL)


class HyundaiButtonBase:
  BUTTONS_TX_BUS = 0  # tx on this bus, rx on 0
  SCC_BUS = 0  # rx on this bus

  def test_button_sends(self):
    """
      RES, SET and CANCEL buttons are allowed
      - RES and SET allowed while controls allowed
      - CANCEL allowed while cruise is enabled
    """
    self.safety.set_controls_allowed(0)
    self.assertFalse(self._tx(self._button_msg(Buttons.RESUME, bus=self.BUTTONS_TX_BUS)))
    self.assertFalse(self._tx(self._button_msg(Buttons.SET, bus=self.BUTTONS_TX_BUS)))

    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(Buttons.RESUME, bus=self.BUTTONS_TX_BUS)))
    self.assertTrue(self._tx(self._button_msg(Buttons.SET, bus=self.BUTTONS_TX_BUS)))

    for enabled in (True, False):
      self._rx(self._pcm_status_msg(enabled))
      self.assertEqual(enabled, self._tx(self._button_msg(Buttons.CANCEL, bus=self.BUTTONS_TX_BUS)))

  def test_enable_control_allowed_from_cruise(self):
    """
      Hyundai non-longitudinal only enables on PCM rising edge and recent button press. Tests PCM enabling with:
      - disallowed: No buttons
      - disallowed: Buttons that don't enable cruise
      - allowed: Buttons that do enable cruise
      - allowed: Main button with all above combinations
    """
    for main_button in (0, 1):
      for btn in range(8):
        for _ in range(PREV_BUTTON_SAMPLES):  # reset
          self._rx(self._button_msg(Buttons.NONE))

        self._rx(self._pcm_status_msg(False))
        self.assertFalse(self.safety.get_controls_allowed())
        self._rx(self._button_msg(btn, main_button=main_button))
        self._rx(self._pcm_status_msg(True))
        controls_allowed = btn in ENABLE_BUTTONS or main_button
        self.assertEqual(controls_allowed, self.safety.get_controls_allowed())

  def test_sampling_cruise_buttons(self):
    """
      Test that we allow controls on recent button press, but not as button leaves sliding window
    """
    self._rx(self._button_msg(Buttons.SET))
    for i in range(2 * PREV_BUTTON_SAMPLES):
      self._rx(self._pcm_status_msg(False))
      self.assertFalse(self.safety.get_controls_allowed())
      self._rx(self._pcm_status_msg(True))
      controls_allowed = i < PREV_BUTTON_SAMPLES
      self.assertEqual(controls_allowed, self.safety.get_controls_allowed())
      self._rx(self._button_msg(Buttons.NONE))


class HyundaiLongitudinalBase(common.LongitudinalAccelSafetyTest):

  DISABLED_ECU_UDS_MSG: tuple[int, int]
  DISABLED_ECU_ACTUATION_MSG: tuple[int, int]

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "HyundaiLongitudinalBase":
      cls.safety = None
      raise unittest.SkipTest

  # override these tests from PandaCarSafetyTest, hyundai longitudinal uses button enable
  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_sampling_cruise_buttons(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_button_sends(self):
    pass

  def _pcm_status_msg(self, enable):
    raise Exception

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    raise NotImplementedError

  def _acc_state_msg(self, enable):
    raise NotImplementedError

  def _tx_acc_state_msg(self, enable):
    raise NotImplementedError

  def test_pcm_main_cruise_state_availability(self):
    pass

  def test_set_resume_buttons(self):
    """
      SET and RESUME enter controls allowed on their falling edge.
    """
    for btn_prev in range(8):
      for btn_cur in range(8):
        self._rx(self._button_msg(Buttons.NONE))
        self.safety.set_controls_allowed(0)
        for _ in range(10):
          self._rx(self._button_msg(btn_prev))
          self.assertFalse(self.safety.get_controls_allowed())

        # should enter controls allowed on falling edge and not transitioning to cancel
        should_enable = btn_cur != btn_prev and \
                        btn_cur != Buttons.CANCEL and \
                        btn_prev in (Buttons.RESUME, Buttons.SET)

        self._rx(self._button_msg(btn_cur))
        self.assertEqual(should_enable, self.safety.get_controls_allowed())

  def test_cancel_button(self):
    self.safety.set_controls_allowed(1)
    self._rx(self._button_msg(Buttons.CANCEL))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_main_cruise_button(self):
    """Test that main cruise button correctly toggles acc_main_on state"""
    default_safety_mode = self.safety.get_current_safety_mode()
    default_safety_param = self.safety.get_current_safety_param()
    default_safety_param_sp = self.safety.get_current_safety_param_sp()

    for enable_mads in (True, False):
      with self.subTest("enable_mads", mads_enabled=enable_mads):
        for main_cruise_toggleable in (True, False):
          with self.subTest("main_cruise_toggleable", main_cruise_toggleable=main_cruise_toggleable):
            main_cruise_toggleable_flag = HyundaiSafetyFlagsSP.LONG_MAIN_CRUISE_TOGGLEABLE if main_cruise_toggleable else 0
            self.safety.set_current_safety_param_sp(default_safety_param_sp | main_cruise_toggleable_flag)
            self.safety.set_safety_hooks(default_safety_mode, default_safety_param)

            # Test initial state
            self.safety.set_mads_params(enable_mads, False, False)

            self.assertFalse(self.safety.get_acc_main_on())

            self._rx(self._main_cruise_button_msg(0))
            self._rx(self._main_cruise_button_msg(1))
            self.assertEqual(enable_mads and main_cruise_toggleable, self.safety.get_controls_allowed_lat())

            self._rx(self._main_cruise_button_msg(0))
            self.assertEqual(enable_mads and main_cruise_toggleable, self.safety.get_controls_allowed_lat())

            self._rx(self._main_cruise_button_msg(1))
            self.assertFalse(self.safety.get_controls_allowed_lat())

            for _ in range(10):
              self._rx(self._main_cruise_button_msg(1))
              self.assertFalse(self.safety.get_controls_allowed_lat())
    self.safety.set_current_safety_param_sp(default_safety_param_sp)

  def test_acc_main_sync_mismatches_reset(self):
    """Test that acc_main_on_mismatches resets properly on rising edge of main button"""
    default_safety_mode = self.safety.get_current_safety_mode()
    default_safety_param = self.safety.get_current_safety_param()
    default_safety_param_sp = self.safety.get_current_safety_param_sp()

    for enable_mads in (True, False):
      with self.subTest("enable_mads", mads_enabled=enable_mads):
        main_cruise_toggleable_flag = HyundaiSafetyFlagsSP.LONG_MAIN_CRUISE_TOGGLEABLE
        self.safety.set_current_safety_param_sp(default_safety_param_sp | main_cruise_toggleable_flag)
        self.safety.set_safety_hooks(default_safety_mode, default_safety_param)

        self.safety.set_mads_params(enable_mads, False, False)

        # Initial state
        self._rx(self._main_cruise_button_msg(0))
        self.assertFalse(self.safety.get_acc_main_on())

        # Set up mismatch condition
        self._rx(self._main_cruise_button_msg(1))  # Press button - acc_main_on = True
        self._rx(self._main_cruise_button_msg(0))  # Release button
        self._tx(self._tx_acc_state_msg(False))  # acc_main_on_tx = False
        self.assertTrue(self.safety.get_acc_main_on())
        self.assertEqual(1, self.safety.get_acc_main_on_mismatches())

        # Rising edge of acc_main_on should reset
        self._rx(self._main_cruise_button_msg(1))  # Press button again
        self._rx(self._main_cruise_button_msg(0))  # Release button
        self._tx(self._tx_acc_state_msg(False))  # acc_main_on_tx = False
        self.assertFalse(self.safety.get_acc_main_on())
        self.assertEqual(0, self.safety.get_acc_main_on_mismatches())
    self.safety.set_current_safety_param_sp(default_safety_param_sp)

  def test_acc_main_sync_mismatch_counter(self):
    """Test mismatch counter behavior and disengagement"""
    default_safety_mode = self.safety.get_current_safety_mode()
    default_safety_param = self.safety.get_current_safety_param()
    default_safety_param_sp = self.safety.get_current_safety_param_sp()

    for enable_mads in (True, False):
      with self.subTest("enable_mads", mads_enabled=enable_mads):
        main_cruise_toggleable_flag = HyundaiSafetyFlagsSP.LONG_MAIN_CRUISE_TOGGLEABLE
        self.safety.set_current_safety_param_sp(default_safety_param_sp | main_cruise_toggleable_flag)
        self.safety.set_safety_hooks(default_safety_mode, default_safety_param)

        self.safety.set_mads_params(enable_mads, False, False)
        self.safety.set_controls_allowed_lat(True)

        # Start with matched states
        self._rx(self._main_cruise_button_msg(0))
        self._tx(self._tx_acc_state_msg(False))
        self.assertEqual(0, self.safety.get_acc_main_on_mismatches())

        # Create mismatch by enabling acc_main_on
        self._rx(self._main_cruise_button_msg(1))  # Press button
        self._rx(self._main_cruise_button_msg(0))  # Release button
        self._tx(self._tx_acc_state_msg(False))  # acc_main_on_tx stays false
        self.assertTrue(self.safety.get_acc_main_on())
        self.assertEqual(1, self.safety.get_acc_main_on_mismatches())

        # Second mismatch
        self._tx(self._tx_acc_state_msg(False))
        self.assertTrue(self.safety.get_acc_main_on())
        self.assertEqual(2, self.safety.get_acc_main_on_mismatches())

        # Third mismatch should trigger disengagement
        self._tx(self._tx_acc_state_msg(False))
        self.assertFalse(self.safety.get_acc_main_on())
        self.assertFalse(self.safety.get_controls_allowed_lat())
        # Counter should reset after disengagement
        self._tx(self._tx_acc_state_msg(False))
        self.assertEqual(0, self.safety.get_acc_main_on_mismatches())
    self.safety.set_current_safety_param_sp(default_safety_param_sp)

  def test_acc_main_sync_mismatch_recovery(self):
    default_safety_mode = self.safety.get_current_safety_mode()
    default_safety_param = self.safety.get_current_safety_param()
    default_safety_param_sp = self.safety.get_current_safety_param_sp()

    """Test that mismatch counter resets when states resync"""
    for enable_mads in (True, False):
      with self.subTest("enable_mads", mads_enabled=enable_mads):
        main_cruise_toggleable_flag = HyundaiSafetyFlagsSP.LONG_MAIN_CRUISE_TOGGLEABLE
        self.safety.set_current_safety_param_sp(default_safety_param_sp | main_cruise_toggleable_flag)
        self.safety.set_safety_hooks(default_safety_mode, default_safety_param)

        self.safety.set_mads_params(enable_mads, False, False)

        # Create initial mismatch
        self._rx(self._main_cruise_button_msg(1))  # Press button
        self._rx(self._main_cruise_button_msg(0))  # Release button
        self._tx(self._tx_acc_state_msg(False))  # acc_main_on_tx = False
        self.assertEqual(1, self.safety.get_acc_main_on_mismatches())

        # Sync states
        self._tx(self._tx_acc_state_msg(True))  # Match acc_main_on_tx to acc_main_on
        self.assertEqual(0, self.safety.get_acc_main_on_mismatches())
    self.safety.set_current_safety_param_sp(default_safety_param_sp)

  def test_tester_present_allowed(self, ecu_disable: bool = True):
    """
      Ensure tester present diagnostic message is allowed to keep ECU knocked out
      for longitudinal control.
    """

    addr, bus = self.DISABLED_ECU_UDS_MSG
    for should_tx, msg in ((True, b"\x02\x3E\x80\x00\x00\x00\x00\x00"),
                           (False, b"\x03\xAA\xAA\x00\x00\x00\x00\x00")):
      tester_present = libsafety_py.make_CANPacket(addr, bus, msg)
      self.assertEqual(should_tx and ecu_disable, self._tx(tester_present))

  def test_disabled_ecu_alive(self):
    """
      If the ECU knockout failed, make sure the relay malfunction is shown
    """

    addr, bus = self.DISABLED_ECU_ACTUATION_MSG
    self.assertFalse(self.safety.get_relay_malfunction())
    self._rx(make_msg(bus, addr, 8))
    self.assertTrue(self.safety.get_relay_malfunction())
