from parameterized import parameterized
import abc
import unittest

from opendbc.safety.tests.libsafety import libsafety_py


class MadsSafetyTestBase(unittest.TestCase):
  safety: libsafety_py.Panda

  @abc.abstractmethod
  def _lkas_button_msg(self, enabled):
    raise NotImplementedError

  @abc.abstractmethod
  def _acc_state_msg(self, enabled):
    raise NotImplementedError

  def teardown_method(self, method):
    self.safety.set_mads_button_press(-1)
    self.safety.set_controls_allowed_lat(False)
    self.safety.set_controls_requested_lat(False)
    self.safety.set_acc_main_on(False)
    self.safety.set_mads_params(False, False, False)
    self.safety.set_heartbeat_engaged_mads(True)

  def test_heartbeat_engaged_mads_check(self):
    """Test MADS heartbeat engaged check behavior"""
    for boolean in (True, False):
      # If boolean is True, the heartbeat is engaged and should remain engaged, otherwise it should disengage.
      with self.subTest(heartbeat_engaged=boolean, should_remain_engaged=boolean):
        # Setup initial conditions
        self.safety.set_mads_params(True, False, False)  # Enable MADS
        self.safety.set_controls_allowed_lat(True)
        self.assertTrue(self.safety.get_controls_allowed_lat())

        # Set heartbeat engaged state based on test case
        self.safety.set_heartbeat_engaged_mads(boolean)

        # Call the heartbeat check function multiple times
        # We know from the implementation that it takes 3 mismatches to disengage
        for _ in range(4):  # More than 3 times to ensure we pass the threshold
          self.safety.mads_heartbeat_engaged_check()

        # Verify engagement state matches expectation
        self.assertEqual(self.safety.get_controls_allowed_lat(), boolean,
                         f"Expected controls_allowed_lat to be [{boolean}] but got [{self.safety.get_controls_allowed_lat()}]")

  def test_enable_control_allowed_with_mads_button(self):
    """Toggle MADS with MADS button"""
    try:
      self._lkas_button_msg(False)
    except NotImplementedError as err:
      raise unittest.SkipTest("Skipping test because MADS button is not supported") from err

    for enable_mads in (True, False):
      with self.subTest("enable_mads", mads_enabled=enable_mads):
        self.safety.set_mads_params(enable_mads, False, False)
        self.assertEqual(enable_mads, self.safety.get_enable_mads())

        self._rx(self._lkas_button_msg(True))
        self._rx(self._lkas_button_msg(False))
        self.assertEqual(enable_mads, self.safety.get_controls_allowed_lat())

  def test_enable_control_allowed_with_manual_acc_main_on_state(self):
    try:
      self._acc_state_msg(False)
    except NotImplementedError as err:
      raise unittest.SkipTest("Skipping test because _acc_state_msg is not implemented for this car") from err

    for enable_mads in (True, False):
      with self.subTest("enable_mads", mads_enabled=enable_mads):
        self.safety.set_mads_params(enable_mads, False, False)
        self._rx(self._acc_state_msg(True))
        self.assertEqual(enable_mads, self.safety.get_controls_allowed_lat())

  def test_enable_control_allowed_with_manual_mads_button_state(self):
    for enable_mads in (True, False):
      with self.subTest("enable_mads", mads_enabled=enable_mads):
        for mads_button_press in (-1, 0, 1):
          with self.subTest("mads_button_press", button_state=mads_button_press):
            self.safety.set_mads_params(enable_mads, False, False)

            self.safety.set_mads_button_press(mads_button_press)
            self._rx(self._speed_msg(0))
            self.assertEqual(enable_mads and mads_button_press == 1, self.safety.get_controls_allowed_lat())

  def test_enable_control_allowed_from_acc_main_on(self):
    """Test that lateral controls are allowed when ACC main is enabled and disabled when ACC main is disabled"""
    for enable_mads in (True, False):
      with self.subTest("enable_mads", mads_enabled=enable_mads):
        for acc_main_on in (True, False):
          with self.subTest("initial_acc_main", initial_acc_main=acc_main_on):
            self.safety.set_mads_params(enable_mads, False, False)

            # Set initial state
            self.safety.set_acc_main_on(acc_main_on)
            self._rx(self._speed_msg(0))
            expected_lat = enable_mads and acc_main_on
            self.assertEqual(expected_lat, self.safety.get_controls_allowed_lat(),
                             f"Expected lat: [{expected_lat}] when acc_main_on goes to [{acc_main_on}]")

            # Test transition to opposite state
            self.safety.set_acc_main_on(not acc_main_on)
            self._rx(self._speed_msg(0))
            expected_lat = enable_mads and not acc_main_on
            self.assertEqual(expected_lat, self.safety.get_controls_allowed_lat(),
                             f"Expected lat: [{expected_lat}] when acc_main_on goes from [{acc_main_on}] to [{not acc_main_on}]")

            # Test transition back to initial state
            self.safety.set_acc_main_on(acc_main_on)
            self._rx(self._speed_msg(0))
            expected_lat = enable_mads and acc_main_on
            self.assertEqual(expected_lat, self.safety.get_controls_allowed_lat(),
                             f"Expected lat: [{expected_lat}] when acc_main_on goes from [{not acc_main_on}] to [{acc_main_on}]")

  def test_mads_with_acc_main_on(self):
    for enable_mads in (True, False):
      with self.subTest("enable_mads", mads_enabled=enable_mads):
        self.safety.set_mads_params(enable_mads, False, False)

        self.safety.set_acc_main_on(True)
        self._rx(self._speed_msg(0))
        self.assertEqual(enable_mads, self.safety.get_controls_allowed_lat())

        self.safety.set_acc_main_on(False)
        self._rx(self._speed_msg(0))
        self.assertFalse(self.safety.get_controls_allowed_lat())

  def test_pause_lateral_on_brake_setup(self):
    for enable_mads in (True, False):
      with self.subTest("enable_mads", enable_mads=enable_mads):
        for pause_lateral_on_brake in (True, False):
          with self.subTest("pause_lateral_on_brake", pause_lateral_on_brake=pause_lateral_on_brake):
            self.safety.set_mads_params(enable_mads, False, pause_lateral_on_brake)
            self.assertEqual(enable_mads and pause_lateral_on_brake, self.safety.get_pause_lateral_on_brake())

  def test_pause_lateral_on_brake(self):
    self.safety.set_mads_params(True, False, True)

    self._rx(self._user_brake_msg(False))
    self.safety.set_controls_requested_lat(True)
    self.safety.set_controls_allowed_lat(True)

    self._rx(self._user_brake_msg(True))
    # Test we pause lateral
    self.assertFalse(self.safety.get_controls_allowed_lat())
    # Make sure we can re-gain lateral actuation
    self._rx(self._user_brake_msg(False))
    self.assertTrue(self.safety.get_controls_allowed_lat())

  def test_no_pause_lateral_on_brake(self):
    self.safety.set_mads_params(True, False, False)

    self._rx(self._user_brake_msg(False))
    self.safety.set_controls_requested_lat(True)
    self.safety.set_controls_allowed_lat(True)

    self._rx(self._user_brake_msg(True))
    self.assertTrue(self.safety.get_controls_allowed_lat())

  @parameterized.expand(["mads_button", "acc_main_on"])
  def test_engage_with_brake_pressed(self, engage_method):
    if engage_method == "mads_button":
      try:
        self._lkas_button_msg(False)
      except NotImplementedError as err:
        raise unittest.SkipTest("Skipping test because MADS button is not supported") from err
    elif engage_method == "acc_main_on":
      try:
        self._acc_state_msg(False)
      except NotImplementedError as err:
        raise unittest.SkipTest("Skipping test because ACC main is not supported") from err

    for enable_mads in (True, False):
      with self.subTest("enable_mads", enable_mads=enable_mads):
        for pause_lateral_on_brake in (True, False):
          with self.subTest("pause_lateral_on_brake", pause_lateral_on_brake=pause_lateral_on_brake):
            with self.subTest(engage_method):
              self.safety.set_mads_params(enable_mads, False, pause_lateral_on_brake)

              # Brake press rising edge
              self._rx(self._user_brake_msg(True))

              if engage_method == "mads_button":
                self._rx(self._lkas_button_msg(True))
              elif engage_method == "acc_main_on":
                self.safety.set_acc_main_on(True)
                self.assertTrue(self.safety.get_acc_main_on())
              else:
                raise ValueError(f"Invalid engage_method: {engage_method}")
              self._rx(self._speed_msg(0))

              self.assertEqual(enable_mads and not pause_lateral_on_brake, self.safety.get_controls_allowed_lat())

              # Continuous braking after the first frame of brake press rising edge
              for _ in range(400):
                self.assertEqual(enable_mads and not pause_lateral_on_brake, self.safety.get_controls_allowed_lat())

  def test_pause_lateral_on_brake_with_pressed_and_released(self):
    for enable_mads in (True, False):
      with self.subTest("enable_mads", enable_mads=enable_mads):
        for pause_lateral_on_brake in (True, False):
          with self.subTest("pause_lateral_on_brake", pause_lateral_on_brake=pause_lateral_on_brake):
            self.safety.set_mads_params(enable_mads, False, pause_lateral_on_brake)

            # Set controls_allowed_lat rising edge
            self.safety.set_controls_requested_lat(True)
            self._rx(self._speed_msg(0))
            self.assertEqual(enable_mads, self.safety.get_controls_allowed_lat())

            # User brake press, validate controls_allowed_lat is false
            self._rx(self._user_brake_msg(True))
            self.assertEqual(enable_mads and not pause_lateral_on_brake, self.safety.get_controls_allowed_lat())

            # User brake release, validate controls_allowed_lat is true
            self._rx(self._user_brake_msg(False))
            self.assertEqual(enable_mads, self.safety.get_controls_allowed_lat())

  def test_pause_lateral_on_brake_persistent_control_allowed_off(self):
    self.safety.set_mads_params(True, False, True)

    self.safety.set_controls_requested_lat(True)

    # Vehicle moving, validate controls_allowed_lat is true
    for _ in range(10):
      self._rx(self._speed_msg(10))
      self.assertTrue(self.safety.get_controls_allowed_lat())

    # User braked, vehicle slowed down in 10 frames, then stopped for 10 frames
    # Validate controls_allowed_lat is false
    self._rx(self._user_brake_msg(True))
    for _ in range(10):
      self._rx(self._speed_msg(5))
      self.assertFalse(self.safety.get_controls_allowed_lat())
    for _ in range(10):
      self._rx(self._speed_msg(0))
      self.assertFalse(self.safety.get_controls_allowed_lat())

  def test_enable_lateral_control_with_controls_allowed_rising_edge(self):
    for enable_mads in (True, False):
      with self.subTest("enable_mads", enable_mads=enable_mads):
        self.safety.set_mads_params(enable_mads, False, False)

        self.safety.set_controls_allowed(False)
        self._rx(self._speed_msg(0))
        self.safety.set_controls_allowed(True)
        self._rx(self._speed_msg(0))
        self.assertTrue(self.safety.get_controls_allowed())

  def test_enable_control_allowed_with_mads_button_and_disable_with_main_cruise(self):
    """Tests main cruise and MADS button state transitions.

      Sequence:
      1. Main cruise off -> on
      2. MADS button engage
      3. Main cruise off

    """
    try:
      self._lkas_button_msg(False)
    except NotImplementedError as err:
      raise unittest.SkipTest("Skipping test because MADS button is not supported") from err

    try:
      self._acc_state_msg(False)
    except NotImplementedError as err:
      raise unittest.SkipTest("Skipping test because _acc_state_msg is not implemented for this car") from err

    for enable_mads in (True, False):
      with self.subTest("enable_mads", enable_mads=enable_mads):
        self.safety.set_mads_params(enable_mads, False, False)

        self._rx(self._lkas_button_msg(True))
        self._rx(self._lkas_button_msg(False))
        self.assertEqual(enable_mads, self.safety.get_controls_allowed_lat())

        self._rx(self._acc_state_msg(True))
        self.assertEqual(enable_mads, self.safety.get_controls_allowed_lat())

        self._rx(self._acc_state_msg(False))
        self.assertFalse(self.safety.get_controls_allowed_lat())

  def test_brake_disengage_with_control_request(self):
    """Tests behavior when controls are requested while brake is engaged

    Sequence:
    1. Enable MADS with pause lateral on brake
    2. Brake to pause lateral control
    3. Set control request while braking
    4. Release brake
    5. Verify controls become allowed
    """
    self.safety.set_mads_params(True, False, True)  # enable MADS with pause lateral on brake

    # Initial state
    self.safety.set_controls_allowed_lat(True)
    self._rx(self._speed_msg(0))
    self.assertTrue(self.safety.get_controls_allowed_lat())

    # Brake press disengages lateral
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed_lat())

    # Request controls while braking
    self.safety.set_controls_requested_lat(True)
    self.assertFalse(self.safety.get_controls_allowed_lat())

    # Release brake - should enable since controls were requested
    self._rx(self._user_brake_msg(False))
    self.assertTrue(self.safety.get_controls_allowed_lat())

  def test_brake_disengage_with_acc_main_off(self):
    """Tests behavior when ACC main is turned off while brake is engaged

    Sequence:
    1. Enable MADS with pause lateral on brake
    2. Brake to pause lateral control
    3. Turn ACC main off while braking
    4. Release brake
    5. Verify controls remain disengaged
    """
    self.safety.set_mads_params(True, False, True)  # enable MADS with pause lateral on brake

    # Initial state - enable with ACC main
    self.safety.set_acc_main_on(True)
    self._rx(self._speed_msg(0))
    self.assertTrue(self.safety.get_controls_allowed_lat())

    # Brake press disengages lateral
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed_lat())

    # Turn ACC main off while braking
    self.safety.set_acc_main_on(False)
    self._rx(self._speed_msg(0))
    self.assertFalse(self.safety.get_controls_allowed_lat())

    # Release brake - should remain disabled since ACC main is off
    self._rx(self._user_brake_msg(False))
    self.assertFalse(self.safety.get_controls_allowed_lat())

  def test_steering_disengage_with_control_request(self):
    self.safety.set_mads_params(True, False, False)

    self.safety.set_controls_allowed_lat(True)
    self._rx(self._speed_msg(0))
    self.assertTrue(self.safety.get_controls_allowed_lat())

    self.safety.set_steering_disengage(True)
    self._rx(self._speed_msg(0))
    self.assertFalse(self.safety.get_controls_allowed_lat())

  def test_disengage_on_brake(self):
    for disengage_on_brake in (True, False):
      self.safety.set_mads_params(True, disengage_on_brake, False)

      self.safety.set_controls_allowed_lat(True)
      self._rx(self._speed_msg(0))
      self.assertTrue(self.safety.get_controls_allowed_lat())

      self._rx(self._user_brake_msg(True))
      self.assertEqual(not disengage_on_brake, self.safety.get_controls_allowed_lat())

      self._rx(self._user_brake_msg(False))
      self.assertEqual(not disengage_on_brake, self.safety.get_controls_allowed_lat())

  # TODO-SP: controls_allowed and controls_allowed_lat check for steering safety tests
