import random
import re
import unittest
from types import SimpleNamespace

from opendbc.car import DT_CTRL
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.carcontroller import HCAMitigation, MQBStandstillManager
from opendbc.car.volkswagen.mqbcan import ESPOverride
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

  def _cs(self, *, esp_hold_confirmation=False, esp_stopping=False, rolling_backward=False,
          rolling_forward=False, grade=0.0, brake_pressed=False, standstill=True, v_ego=0.0,
          sum_wegimpulse=0):
    out = SimpleNamespace(brakePressed=brake_pressed, standstill=standstill, vEgo=v_ego)
    return SimpleNamespace(out=out, esp_hold_confirmation=esp_hold_confirmation,
                           esp_stopping=esp_stopping, rolling_backward=rolling_backward,
                           rolling_forward=rolling_forward, grade=grade,
                           sum_wegimpulse=sum_wegimpulse)

  def _run(self, mgr, cs, *, long_active=True, accel=0.0, stopping=False, starting=False, max_planned_speed=10.0):
    """Convenience wrapper for update() with sensible defaults."""
    return mgr.update(cs, long_active=long_active, accel=accel, stopping=stopping, starting=starting,
                      max_planned_speed=max_planned_speed)

  # ── brake press ──────────────────────────────────────────────────────────────

  def test_brake_pressed_disables_long_active(self):
    """Brake input overrides long_active to prevent faults when pre-enabled."""
    mgr = MQBStandstillManager()
    long_active, *_ = self._run(mgr, self._cs(brake_pressed=True), long_active=True)
    assert not long_active

  # ── rollback detection ───────────────────────────────────────────────────────

  def test_rollback_detected_on_rolling_backward(self):
    """rollback_detected latches True when the car rolls backward."""
    mgr = MQBStandstillManager()
    self._run(mgr, self._cs(rolling_backward=True))
    assert mgr.rollback_detected

  def test_rollback_clears_on_rolling_forward(self):
    """rollback_detected clears when the car rolls forward."""
    mgr = MQBStandstillManager()
    self._run(mgr, self._cs(rolling_backward=True))
    assert mgr.rollback_detected
    self._run(mgr, self._cs(rolling_forward=True))
    assert not mgr.rollback_detected

  def test_rollback_forces_brake(self):
    """Accel is forced to -3.5 when rolling backward and accel is not positive."""
    mgr = MQBStandstillManager()
    self._run(mgr, self._cs(rolling_backward=True))
    _, accel, *_ = self._run(mgr, self._cs(), accel=-0.5)
    assert accel == -3.5

  def test_rollback_softer_brake_when_rolling_forward(self):
    """Accel is only -0.5 when rollback is active but car is now rolling forward (recovering)."""
    mgr = MQBStandstillManager()
    self._run(mgr, self._cs(rolling_backward=True))
    _, accel, *_ = self._run(mgr, self._cs(rolling_forward=True), accel=-0.5)
    assert accel == -0.5

  def test_rollback_forces_start_when_positive_accel(self):
    """start_commit is applied when rollback is detected but accel target is positive."""
    mgr = MQBStandstillManager()
    self._run(mgr, self._cs(rolling_backward=True))
    _, accel, stopping, starting, _ = self._run(mgr, self._cs(), accel=0.5)
    assert starting
    assert not stopping

  # ── stop commit ──────────────────────────────────────────────────────────────

  def test_stop_commit_not_triggered_on_flat(self):
    """stop_commit never triggers on flat ground (theoretical_safe_stop_speed == 0)."""
    mgr = MQBStandstillManager()
    self._run(mgr, self._cs(grade=0.0, v_ego=0.0))
    assert not mgr.stop_commit_active

  def test_stop_commit_triggers_below_safe_speed(self):
    """stop_commit_active triggers when vEgo drops below the grade-based safe stop speed."""
    mgr = MQBStandstillManager()
    grade = 20.0
    safe_speed = mgr.get_theoretical_safe_speed(grade, 0.0)
    self._run(mgr, self._cs(grade=grade, v_ego=safe_speed * 0.5))
    assert mgr.stop_commit_active

  def test_stop_commit_not_triggered_above_safe_speed(self):
    """stop_commit_active does not trigger when vEgo is above the safe stop speed."""
    mgr = MQBStandstillManager()
    grade = 20.0
    safe_speed = mgr.get_theoretical_safe_speed(grade, 0.0)
    self._run(mgr, self._cs(grade=grade, v_ego=safe_speed * 2.0))
    assert not mgr.stop_commit_active

  def test_stop_commit_forces_brake(self):
    """stop_commit_active forces accel to -3.5, stopping=True, starting=False."""
    mgr = MQBStandstillManager()
    grade = 20.0
    safe_speed = mgr.get_theoretical_safe_speed(grade, 0.0)
    _, accel, stopping, starting, _ = self._run(mgr, self._cs(grade=grade, v_ego=safe_speed * 0.5), accel=0.0)
    assert accel == -3.5
    assert stopping
    assert not starting

  def test_stop_commit_transitions_to_start_commit(self):
    """stop_commit transitions to start_commit when max_planned_speed allows leaving and accel > 0."""
    mgr = MQBStandstillManager()
    grade = 20.0
    safe_speed = mgr.get_theoretical_safe_speed(grade, 0.0)
    # Trigger stop commit
    self._run(mgr, self._cs(grade=grade, v_ego=safe_speed * 0.5), accel=-1.0, max_planned_speed=0.0)
    assert mgr.stop_commit_active
    # Now want to leave (max_planned_speed > safe_speed, accel > 0)
    self._run(mgr, self._cs(grade=grade, v_ego=safe_speed * 0.5), accel=0.5, max_planned_speed=safe_speed * 2.0)
    assert mgr.start_commit_active
    assert not mgr.stop_commit_active

  def test_stop_commit_clears_on_long_inactive(self):
    """stop_commit_active resets when long control disengages."""
    mgr = MQBStandstillManager()
    grade = 20.0
    safe_speed = mgr.get_theoretical_safe_speed(grade, 0.0)
    self._run(mgr, self._cs(grade=grade, v_ego=safe_speed * 0.5))
    assert mgr.stop_commit_active
    self._run(mgr, self._cs(grade=grade), long_active=False)
    assert not mgr.stop_commit_active

  # ── start commit ─────────────────────────────────────────────────────────────

  def test_start_commit_on_pre_enable(self):
    """esp_hold_confirmation while long_active triggers start_commit (pre-enable scenario)."""
    mgr = MQBStandstillManager()
    self._run(mgr, self._cs(esp_hold_confirmation=True))
    assert mgr.start_commit_active
    assert not mgr.stop_commit_active

  def test_start_commit_forces_accel(self):
    """start_commit_active boosts accel to at least hill_launch_accel and 0.2."""
    mgr = MQBStandstillManager()
    grade = 10.0
    expected_min = max(0.2 * grade - 1, 0.2)
    self._run(mgr, self._cs(esp_hold_confirmation=True, grade=grade))
    _, accel, stopping, starting, _ = self._run(mgr, self._cs(esp_hold_confirmation=True, grade=grade), accel=0.0)
    assert accel >= expected_min
    assert starting
    assert not stopping

  def test_start_commit_clears_above_safe_speed_while_moving(self):
    """start_commit_active clears when vEgo exceeds safe stop speed and wheels are moving."""
    mgr = MQBStandstillManager()
    grade = 20.0
    safe_speed = mgr.get_theoretical_safe_speed(grade, 0.0)
    # Enter start commit
    self._run(mgr, self._cs(esp_hold_confirmation=True, grade=grade))
    assert mgr.start_commit_active
    # Move above safe speed with changing wegimpulse (not at_standstill)
    for i in range(MQBStandstillManager.WEGIMPULSE_STILLNESS_FRAMES + 1):
      self._run(mgr, self._cs(grade=grade, v_ego=safe_speed * 2.0, sum_wegimpulse=i))
    assert not mgr.start_commit_active
    assert not mgr.stop_commit_active

  def test_start_commit_persists_at_standstill(self):
    """start_commit_active does not clear if wheels are not moving (at_standstill), even above safe speed."""
    mgr = MQBStandstillManager()
    grade = 20.0
    safe_speed = mgr.get_theoretical_safe_speed(grade, 0.0)
    # Enter start_commit, then let at_standstill build up while vEgo is safely below threshold
    self._run(mgr, self._cs(esp_hold_confirmation=True, grade=grade))
    assert mgr.start_commit_active
    for _ in range(MQBStandstillManager.WEGIMPULSE_STILLNESS_FRAMES + 1):
      self._run(mgr, self._cs(grade=grade, v_ego=safe_speed * 0.5, sum_wegimpulse=0))
    # at_standstill is now True — high vEgo should not clear start_commit
    self._run(mgr, self._cs(grade=grade, v_ego=safe_speed * 2.0, sum_wegimpulse=0))
    assert mgr.start_commit_active

  # ── can_stop_forever / ESP override ──────────────────────────────────────────

  def test_can_stop_forever_latches_on_esp_stopping(self):
    """can_stop_forever latches True and returns ESPOverride.START when esp_stopping is seen."""
    mgr = MQBStandstillManager()
    *_, esp_override = self._run(mgr, self._cs(esp_stopping=True), accel=-1.0, stopping=True)
    assert mgr.can_stop_forever
    assert esp_override == ESPOverride.START

  def test_can_stop_forever_persists(self):
    """can_stop_forever continues returning ESPOverride.START once latched, without esp_stopping."""
    mgr = MQBStandstillManager()
    self._run(mgr, self._cs(esp_stopping=True), accel=-1.0, stopping=True)
    *_, esp_override = self._run(mgr, self._cs(), accel=-1.0, stopping=True)
    assert mgr.can_stop_forever
    assert esp_override == ESPOverride.START

  def test_can_stop_forever_cleared_by_hold_confirmation(self):
    """can_stop_forever is cleared when ESP confirms a hold."""
    mgr = MQBStandstillManager()
    self._run(mgr, self._cs(esp_stopping=True), accel=-1.0, stopping=True)
    assert mgr.can_stop_forever
    self._run(mgr, self._cs(esp_hold_confirmation=True), accel=-1.0, stopping=True)
    assert not mgr.can_stop_forever

  def test_can_stop_forever_cleared_when_moving(self):
    """can_stop_forever clears when vEgo > 1 and esp_stopping is not active."""
    mgr = MQBStandstillManager()
    self._run(mgr, self._cs(esp_stopping=True), accel=-1.0, stopping=True)
    assert mgr.can_stop_forever
    self._run(mgr, self._cs(v_ego=1.1), accel=0.5)
    assert not mgr.can_stop_forever

  def test_can_stop_forever_cleared_when_long_inactive(self):
    """can_stop_forever is cleared when long control is inactive."""
    mgr = MQBStandstillManager()
    self._run(mgr, self._cs(esp_stopping=True), accel=-1.0, stopping=True)
    assert mgr.can_stop_forever
    self._run(mgr, self._cs(), long_active=False)
    assert not mgr.can_stop_forever

  def test_esp_override_stop_at_standstill(self):
    """ESPOverride.STOP is requested when wheels have been still for WEGIMPULSE_STILLNESS_FRAMES."""
    mgr = MQBStandstillManager()
    esp_override = None
    for _ in range(MQBStandstillManager.WEGIMPULSE_STILLNESS_FRAMES + 1):
      *_, esp_override = self._run(mgr, self._cs(sum_wegimpulse=0), accel=-1.0, stopping=True)
    assert esp_override == ESPOverride.STOP

  def test_esp_override_stop_during_stop_commit(self):
    """ESPOverride.STOP is also requested when stop_commit_active, before reaching full standstill."""
    mgr = MQBStandstillManager()
    grade = 20.0
    safe_speed = mgr.get_theoretical_safe_speed(grade, 0.0)
    # Use changing sum_wegimpulse so at_standstill stays False
    *_, esp_override = self._run(mgr, self._cs(grade=grade, v_ego=safe_speed * 0.5, sum_wegimpulse=0),
                                 accel=-1.0, stopping=True, max_planned_speed=0.0)
    assert esp_override == ESPOverride.STOP

  def test_esp_override_none_when_inactive(self):
    """No ESP override is issued when long control is inactive."""
    mgr = MQBStandstillManager()
    *_, esp_override = self._run(mgr, self._cs(), long_active=False)
    assert esp_override is None

  # ── wegimpulse stillness ──────────────────────────────────────────────────────

  def test_wegimpulse_at_standstill_after_stillness_frames(self):
    """at_standstill becomes True after WEGIMPULSE_STILLNESS_FRAMES with constant sum_wegimpulse."""
    mgr = MQBStandstillManager()
    # First frame resets frames_since (prev is None), so need STILLNESS_FRAMES + 1 total
    for _ in range(MQBStandstillManager.WEGIMPULSE_STILLNESS_FRAMES + 1):
      self._run(mgr, self._cs(sum_wegimpulse=0))
    assert mgr.frames_since_wegimpulse_change >= MQBStandstillManager.WEGIMPULSE_STILLNESS_FRAMES

  def test_wegimpulse_resets_on_change(self):
    """frames_since_wegimpulse_change resets when sum_wegimpulse changes."""
    mgr = MQBStandstillManager()
    for _ in range(MQBStandstillManager.WEGIMPULSE_STILLNESS_FRAMES):
      self._run(mgr, self._cs(sum_wegimpulse=0))
    self._run(mgr, self._cs(sum_wegimpulse=1))
    assert mgr.frames_since_wegimpulse_change == 0


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
