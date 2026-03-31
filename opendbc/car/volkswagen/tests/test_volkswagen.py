import random
import re
import unittest
from types import SimpleNamespace

from opendbc.car import DT_CTRL
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.carcontroller import HCAMitigation, MQBStandstillManager
from opendbc.car.volkswagen.values import CAR, CarControllerParams as CCP, FW_QUERY_CONFIG, WMI
from opendbc.car.volkswagen.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu

CHASSIS_CODE_PATTERN = re.compile('[A-Z0-9]{2}')
# TODO: determine the unknown groups
SPARE_PART_FW_PATTERN = re.compile(b'\xf1\x87(?P<gateway>[0-9][0-9A-Z]{2})(?P<unknown>[0-9][0-9A-Z][0-9])(?P<unknown2>[0-9A-Z]{2}[0-9])([A-Z0-9]| )')


class TestVolkswagenHCAMitigation(unittest.TestCase):
  STUCK_TORQUE_FRAMES = round(CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP))

  def test_same_torque_mitigation(self):
    """Same-torque nudge fires at the threshold, in the correct direction, and resets cleanly."""
    hca_mitigation = HCAMitigation(CCP)

    for actuator_value in (-CCP.STEER_MAX, -1, 0, 1, CCP.STEER_MAX):
      hca_mitigation.update(0, 0)  # Reset mitigation state
      for frame in range(self.STUCK_TORQUE_FRAMES + 2):
        should_nudge = actuator_value != 0 and frame == self.STUCK_TORQUE_FRAMES
        expected_torque = actuator_value - (1, -1)[actuator_value < 0] if should_nudge else actuator_value
        assert hca_mitigation.update(actuator_value, actuator_value) == expected_torque, f"{frame=}"

class TestVolkswagenMQBStandstillManager(unittest.TestCase):
  HOLD_MAX_FRAMES = MQBStandstillManager.HOLD_MAX_FRAMES
  HOLD_RELEASE_TOTAL_FRAMES = MQBStandstillManager.HOLD_RELEASE_TOTAL_FRAMES
  RELEASE_START_FRAME = HOLD_MAX_FRAMES - HOLD_RELEASE_TOTAL_FRAMES + 1  # first frame of release window

  def _cs(self, *, esp_hold_confirmation=False, esp_stopping=False, rolling_backward=False,
          rolling_forward=False, grade=0.0, brake_pressed=False, standstill=True, v_ego=0.0):
    out = SimpleNamespace(brakePressed=brake_pressed, standstill=standstill, vEgo=v_ego)
    return SimpleNamespace(out=out, esp_hold_confirmation=esp_hold_confirmation,
                           esp_stopping=esp_stopping, rolling_backward=rolling_backward,
                           rolling_forward=rolling_forward, grade=grade)

  def test_brake_pressed_disables_long_active(self):
    """Brake input overrides long_active to prevent faults when pre-enabled."""
    mgr = MQBStandstillManager()
    long_active, *_ = mgr.update(self._cs(brake_pressed=True), long_active=True, accel=0.0, stopping=True, starting=False)
    assert not long_active

  def test_hold_max_frames_disables_long_active(self):
    """long_active is suppressed after HOLD_MAX_FRAMES of confirmed hold to avoid a cruise fault."""
    mgr = MQBStandstillManager()
    cs = self._cs(esp_hold_confirmation=True)
    for _ in range(self.HOLD_MAX_FRAMES + 1):
      long_active, *_ = mgr.update(cs, long_active=True, accel=-1.0, stopping=True, starting=False)
    assert not long_active

  def test_can_stop_forever_flat_stop(self):
    """ESP stopping procedure triggers can_stop_forever, overriding starting/stopping signals to hold indefinitely."""
    mgr = MQBStandstillManager()
    *_, esp_starting_override, esp_stopping_override = \
      mgr.update(self._cs(esp_stopping=True), long_active=True, accel=-1.0, stopping=True, starting=False)
    assert mgr.can_stop_forever
    assert esp_starting_override is True
    assert esp_stopping_override is False

  def test_can_stop_forever_cleared_by_hold_confirmation(self):
    """can_stop_forever is cleared when ESP confirms a hold (indefinite hold is no longer possible)."""
    mgr = MQBStandstillManager()
    mgr.update(self._cs(esp_stopping=True), long_active=True, accel=-1.0, stopping=True, starting=False)
    assert mgr.can_stop_forever
    mgr.update(self._cs(esp_hold_confirmation=True), long_active=True, accel=-1.0, stopping=True, starting=False)
    assert not mgr.can_stop_forever

  def test_can_stop_forever_cleared_by_steep_grade(self):
    """can_stop_forever is cleared on grades >= 10 where rollback risk is too high."""
    mgr = MQBStandstillManager()
    mgr.update(self._cs(esp_stopping=True), long_active=True, accel=-1.0, stopping=True, starting=False)
    assert mgr.can_stop_forever
    mgr.update(self._cs(grade=10), long_active=True, accel=-1.0, stopping=True, starting=False)
    assert not mgr.can_stop_forever

  def test_can_stop_forever_cleared_when_not_stopping_or_starting(self):
    """can_stop_forever is cleared when neither stopping nor starting, e.g. during a resume."""
    mgr = MQBStandstillManager()
    mgr.update(self._cs(esp_stopping=True), long_active=True, accel=-1.0, stopping=True, starting=False)
    assert mgr.can_stop_forever
    mgr.update(self._cs(), long_active=True, accel=0.5, stopping=False, starting=False)
    assert not mgr.can_stop_forever

  def test_can_stop_forever_cleared_when_long_inactive(self):
    """can_stop_forever is cleared when long control is inactive."""
    mgr = MQBStandstillManager()
    mgr.update(self._cs(esp_stopping=True), long_active=True, accel=-1.0, stopping=True, starting=False)
    assert mgr.can_stop_forever
    mgr.update(self._cs(), long_active=False, accel=-1.0, stopping=True, starting=False)
    assert not mgr.can_stop_forever

  def test_launch_boost_by_grade(self):
    """Accel is boosted when grade alone exceeds the rollback threshold (grade > 5), no rollback needed."""
    grade = 10.0  # desired_launch_accel = 0.2 * 10 - 1 = 1.0
    mgr = MQBStandstillManager()
    _, accel, *_ = mgr.update(self._cs(grade=grade), long_active=True, accel=0.5, stopping=False, starting=True)
    assert accel == 0.2 * grade - 1

  def test_launch_boost_not_applied_above_speed_threshold(self):
    """Accel is not boosted when vEgo >= 0.25 (no longer near-standstill)."""
    mgr = MQBStandstillManager()
    _, accel, *_ = mgr.update(self._cs(grade=10.0, v_ego=0.25), long_active=True, accel=0.5, stopping=False, starting=True)
    assert accel == 0.5

  def test_rollback_brake_protection(self):
    """Accel is forced to -3.5 when rollback is active and accel is not positive."""
    mgr = MQBStandstillManager()
    mgr.update(self._cs(rolling_backward=True), long_active=True, accel=0.0, stopping=True, starting=False)
    assert mgr.rollback_protection_active
    _, accel, *_ = mgr.update(self._cs(), long_active=True, accel=-0.5, stopping=True, starting=False)
    assert accel == -3.5

  def test_rollback_protection_clears_on_rolling_forward(self):
    """rollback_protection_active is cleared when the car rolls forward."""
    mgr = MQBStandstillManager()
    mgr.update(self._cs(rolling_backward=True), long_active=True, accel=0.0, stopping=True, starting=False)
    assert mgr.rollback_protection_active
    mgr.update(self._cs(rolling_forward=True), long_active=True, accel=0.0, stopping=True, starting=False)
    assert not mgr.rollback_protection_active

  def test_hill_hold_first_frame_skip(self):
    """First frame with hold confirmed skips hill_accel to avoid a check engine light, but still sets overrides."""
    grade = 8.0
    cs = self._cs(esp_hold_confirmation=True, grade=grade)
    mgr = MQBStandstillManager()
    _, accel, _, _, esp_starting_override, esp_stopping_override = \
      mgr.update(cs, long_active=True, accel=-1.0, stopping=True, starting=False)
    assert accel == -1.0  # hill_accel not yet applied
    assert esp_starting_override is False
    assert esp_stopping_override is True

  def test_hill_hold_accel_and_overrides(self):
    """Engine torque is built via hill_accel and ESP braking is held when stopped on a grade."""
    grade = 8.0
    cs = self._cs(esp_hold_confirmation=True, grade=grade)
    mgr = MQBStandstillManager()
    for _ in range(2):
      mgr.update(cs, long_active=True, accel=-1.0, stopping=True, starting=False)
    _, accel, stopping, starting, esp_starting_override, esp_stopping_override = \
      mgr.update(cs, long_active=True, accel=-1.0, stopping=True, starting=False)
    assert accel == 0.045 * grade + 0.0625
    assert starting is True
    assert stopping is False
    assert esp_starting_override is False
    assert esp_stopping_override is True

  def test_hill_hold_accel_suppressed_on_flat(self):
    """hill_accel is suppressed on grades <= 3% to prevent unnecessary engine torque on flat ground."""
    for grade in (0.0, 1.0, 2.0, 3.0):
      cs = self._cs(esp_hold_confirmation=True, grade=grade)
      mgr = MQBStandstillManager()
      for _ in range(2):
        mgr.update(cs, long_active=True, accel=-1.0, stopping=True, starting=False)
      _, accel, *_ = mgr.update(cs, long_active=True, accel=-1.0, stopping=True, starting=False)
      assert accel == -1.0, f"Expected no hill_accel boost at {grade=}"

  def test_hill_hold_progressive_release_pattern(self):
    """Progressive release pulses follow the 1-on/3-off, 2-on/3-off, 3-on/3-off, hold pattern near HOLD_MAX_FRAMES."""
    cs = self._cs(esp_hold_confirmation=True, grade=0.0)
    mgr = MQBStandstillManager()
    # Expected esp_starting_override for each frame 1..HOLD_MAX_FRAMES
    # Frames before release window: always False
    # Release window phases: 0=T, 1-3=F, 4-5=T, 6-8=F, 9-11=T, 12-14=F, 15+=T
    excluded_phases = {1, 2, 3, 6, 7, 8, 12, 13, 14}
    for frame in range(1, self.HOLD_MAX_FRAMES + 1):
      *_, esp_starting_override, _ = mgr.update(cs, long_active=True, accel=-1.0, stopping=True, starting=False)
      phase = frame - self.RELEASE_START_FRAME
      expected = phase >= 0 and phase not in excluded_phases
      assert esp_starting_override is expected, f"{frame=} {phase=}"

  def test_timer_resets_when_moving_without_hold(self):
    """Hold frame counter resets when wheels move without an ESP hold confirmation."""
    mgr = MQBStandstillManager()
    cs_held = self._cs(esp_hold_confirmation=True)
    for _ in range(5):
      mgr.update(cs_held, long_active=True, accel=-1.0, stopping=True, starting=False)
    assert mgr.esp_hold_frames == 5
    mgr.update(self._cs(v_ego=1.0, standstill=False), long_active=True, accel=0.5, stopping=False, starting=True)
    assert mgr.esp_hold_frames == 0

  def test_timer_resets_after_starting_with_hold(self):
    """Hold frame counter resets when hold drops after a starting attempt was sent."""
    mgr = MQBStandstillManager()
    cs_held = self._cs(esp_hold_confirmation=True, grade=0.0)
    # Run into release window so esp_starting_override=True, setting hold_timer_can_reset
    for _ in range(self.RELEASE_START_FRAME):
      mgr.update(cs_held, long_active=True, accel=-1.0, stopping=True, starting=False)
    assert mgr.hold_timer_can_reset
    # Hold releases while car remains at standstill
    mgr.update(self._cs(esp_hold_confirmation=False), long_active=True, accel=-1.0, stopping=True, starting=False)
    assert mgr.esp_hold_frames == 1

  def test_hold_timer_can_reset_clears_on_inactive(self):
    """hold_timer_can_reset is cleared when long control disengages, preventing a stale reset on re-engagement."""
    mgr = MQBStandstillManager()
    cs_held = self._cs(esp_hold_confirmation=True, grade=0.0)
    for _ in range(self.RELEASE_START_FRAME):
      mgr.update(cs_held, long_active=True, accel=-1.0, stopping=True, starting=False)
    assert mgr.hold_timer_can_reset
    mgr.update(cs_held, long_active=False, accel=-1.0, stopping=True, starting=False)
    assert not mgr.hold_timer_can_reset


class TestVolkswagenPlatformConfigs(unittest.TestCase):
  def test_spare_part_fw_pattern(self):
    # Relied on for determining if a FW is likely VW
    for platform, ecus in FW_VERSIONS.items():
      with self.subTest(platform=platform.value):
        for fws in ecus.values():
          for fw in fws:
            assert SPARE_PART_FW_PATTERN.match(fw) is not None, f"Bad FW: {fw}"

  def test_chassis_codes(self):
    for platform in CAR:
      with self.subTest(platform=platform.value):
        assert len(platform.config.wmis) > 0, "WMIs not set"
        assert len(platform.config.chassis_codes) > 0, "Chassis codes not set"
        assert all(CHASSIS_CODE_PATTERN.match(cc) for cc in
                   platform.config.chassis_codes), "Bad chassis codes"

        # No two platforms should share chassis codes
        for comp in CAR:
          if platform == comp:
            continue
          assert set() == platform.config.chassis_codes & comp.config.chassis_codes, \
                           f"Shared chassis codes: {comp}"

  def test_custom_fuzzy_fingerprinting(self):
    all_radar_fw = list({fw for ecus in FW_VERSIONS.values() for fw in ecus[Ecu.fwdRadar, 0x757, None]})

    for platform in CAR:
      with self.subTest(platform=platform.name):
        for wmi in WMI:
          for chassis_code in platform.config.chassis_codes | {"00"}:
            vin = ["0"] * 17
            vin[0:3] = wmi
            vin[6:8] = chassis_code
            vin = "".join(vin)

            # Check a few FW cases - expected, unexpected
            for radar_fw in random.sample(all_radar_fw, 5) + [b'\xf1\x875Q0907572G \xf1\x890571', b'\xf1\x877H9907572AA\xf1\x890396']:
              should_match = ((wmi in platform.config.wmis and chassis_code in platform.config.chassis_codes) and
                              radar_fw in all_radar_fw)

              live_fws = {(0x757, None): [radar_fw]}
              matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fws, vin, FW_VERSIONS)

              expected_matches = {platform} if should_match else set()
              assert expected_matches == matches, "Bad match"
