import math
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

  def _mgr(self):
    return MQBStandstillManager(vehicle_mass=1540.0, accel_min=CCP.ACCEL_MIN)

  def _cs(self, *, esp_hold_confirmation=False, esp_stopping=False, rolling_backward=False,
          rolling_forward=False, brake_pressed=False, gas_pressed=False, standstill=True, v_ego=0.0,
          sum_wegimpulse=0):
    out = SimpleNamespace(brakePressed=brake_pressed, gasPressed=gas_pressed, standstill=standstill, vEgo=v_ego)
    return SimpleNamespace(out=out, esp_hold_confirmation=esp_hold_confirmation,
                           esp_stopping=esp_stopping, rolling_backward=rolling_backward,
                           rolling_forward=rolling_forward, sum_wegimpulse=sum_wegimpulse)

  def _run(self, mgr, cs, *, long_active=True, accel=0.0, stopping=False, starting=False,
           max_planned_speed=0.0, grade_pct=0.0, tsk_brake_torque=0.0):
    """Convenience wrapper for update() with sensible defaults."""
    return mgr.update(cs, long_active=long_active, accel=accel, stopping=stopping, starting=starting,
                      max_planned_speed=max_planned_speed, pitch=self._pitch(grade_pct),
                      tsk_brake_torque=tsk_brake_torque)

  def _safe_speed(self, mgr, grade_pct, brake_torque=0.0):
    return mgr.get_safe_speed_for_brake_torque(self._pitch(grade_pct), brake_torque)

  def _pitch(self, grade_pct):
    return math.atan(grade_pct / 100.0)

  # ── brake press ──────────────────────────────────────────────────────────────

  def test_brake_pressed_disables_long_active(self):
    """Brake input overrides long_active to prevent faults when pre-enabled."""
    mgr = self._mgr()
    long_active, *_ = self._run(mgr, self._cs(brake_pressed=True), long_active=True)
    assert not long_active

  # ── rollback detection ───────────────────────────────────────────────────────

  def test_rollback_detected_on_rolling_backward(self):
    """rollback_detected latches True when the car rolls backward."""
    mgr = self._mgr()
    self._run(mgr, self._cs(rolling_backward=True))
    assert mgr.rollback_detected

  def test_rollback_clears_on_rolling_forward(self):
    """rollback_detected clears when the car rolls forward."""
    mgr = self._mgr()
    self._run(mgr, self._cs(rolling_backward=True))
    assert mgr.rollback_detected
    self._run(mgr, self._cs(rolling_forward=True))
    assert not mgr.rollback_detected

  def test_rollback_forces_brake(self):
    """Accel is forced to ACCEL_MIN when rolling backward and accel is not positive."""
    mgr = self._mgr()
    self._run(mgr, self._cs(rolling_backward=True))
    _, accel, *_ = self._run(mgr, self._cs(), accel=-0.5)
    assert accel == CCP.ACCEL_MIN

  def test_rollback_cleared_when_rolling_forward_passes_through_accel(self):
    """Rolling forward clears rollback state, so no rollback brake override is applied."""
    mgr = self._mgr()
    self._run(mgr, self._cs(rolling_backward=True))
    _, accel, *_ = self._run(mgr, self._cs(rolling_forward=True, standstill=False), accel=-0.5)
    assert accel == -0.5

  def test_rollback_forces_brake_with_positive_accel(self):
    """Actual rollback forces hard braking even when raw accel target is positive."""
    mgr = self._mgr()
    self._run(mgr, self._cs(rolling_backward=True))
    _, accel, stopping, starting, _ = self._run(mgr, self._cs(), accel=0.5)
    assert accel == CCP.ACCEL_MIN
    assert stopping
    assert not starting

  # ── safe speed braking ───────────────────────────────────────────────────────

  def test_flat_ground_passes_raw_accel(self):
    """Flat ground has no rollback-risk speed threshold, so accel/states pass through."""
    mgr = self._mgr()
    _, accel, stopping, starting, _ = self._run(mgr, self._cs(v_ego=0.0), accel=-0.55,
                                                stopping=True, starting=False, grade_pct=0.0)
    assert accel == -0.55
    assert stopping
    assert not starting

  def test_current_brake_torque_reduces_safe_speed(self):
    """Current TSK brake torque reduces the rollback-risk speed threshold."""
    mgr = self._mgr()
    grade = 20.0
    zero_brake_safe_speed = self._safe_speed(mgr, grade)
    current_brake_safe_speed = self._safe_speed(mgr, grade, 500.0)
    assert 0.0 < current_brake_safe_speed < zero_brake_safe_speed

  def test_current_brake_torque_can_eliminate_safe_speed(self):
    """Sufficient current brake torque eliminates the remaining rollback-risk speed threshold."""
    mgr = self._mgr()
    assert self._safe_speed(mgr, 20.0, 10000.0) == 0.0

  def test_safe_speed_is_capped_at_10_kph(self):
    """Very steep pitch does not raise rollback-risk logic above the ESP grant speed."""
    mgr = self._mgr()
    assert self._safe_speed(mgr, 100.0) == MQBStandstillManager.MAX_SAFE_STOPPING_SPEED

  def test_esp_stopping_passes_raw_accel_on_flat(self):
    """esp_stopping latches ESP behavior without forcing a brake command on flat ground."""
    mgr = self._mgr()
    _, accel, stopping, starting, esp_override = self._run(mgr, self._cs(esp_stopping=True, v_ego=0.2), accel=-0.55,
                                                           stopping=True, starting=False)
    assert accel == -0.55
    assert stopping
    assert not starting
    assert esp_override == ESPOverride.START

  def test_below_safe_speed_blends_brake(self):
    """Below safe speed with missing brake torque blends raw accel toward ACCEL_MIN."""
    mgr = self._mgr()
    grade = 20.0
    safe_speed = self._safe_speed(mgr, grade)
    required_torque = mgr.get_hill_hold_decel_deficit(self._pitch(grade), 0.0) * mgr.vehicle_mass * mgr.ASSUMED_WHEEL_RADIUS
    _, accel, stopping, starting, _ = self._run(mgr, self._cs(v_ego=safe_speed * 0.5), accel=-0.55,
                                                grade_pct=grade, tsk_brake_torque=required_torque * 0.5)
    assert CCP.ACCEL_MIN < accel < -0.55
    assert stopping
    assert not starting

  def test_sufficient_brake_torque_passes_raw_accel(self):
    """When current TSK brake torque covers rollback risk, raw openpilot accel/states pass through."""
    mgr = self._mgr()
    grade = 20.0
    safe_speed = self._safe_speed(mgr, grade)
    _, accel, stopping, starting, _ = self._run(mgr, self._cs(v_ego=safe_speed * 0.5), accel=-0.2,
                                                stopping=True, starting=False, grade_pct=grade, tsk_brake_torque=10000.0)
    assert accel == -0.2
    assert stopping
    assert not starting

  def test_below_safe_speed_with_low_planned_speed_uses_blended_braking(self):
    """Below safe speed with low planned speed keeps stopping behavior and uses blended braking."""
    mgr = self._mgr()
    grade = 20.0
    safe_speed = self._safe_speed(mgr, grade)
    _, accel, stopping, starting, _ = self._run(mgr, self._cs(v_ego=safe_speed * 0.5), accel=-0.55,
                                                max_planned_speed=safe_speed * 0.5, grade_pct=grade)
    assert CCP.ACCEL_MIN <= accel < -0.55
    assert stopping
    assert not starting

  def test_below_safe_speed_with_high_planned_speed_and_negative_accel_uses_blended_braking(self):
    """Below safe speed, planned drive-away intent still brakes until openpilot requests positive accel."""
    mgr = self._mgr()
    grade = 20.0
    safe_speed = self._safe_speed(mgr, grade)
    _, accel, stopping, starting, _ = self._run(mgr, self._cs(v_ego=safe_speed * 0.5), accel=-0.55,
                                                max_planned_speed=safe_speed * 2.0, grade_pct=grade)
    assert CCP.ACCEL_MIN <= accel < -0.55
    assert stopping
    assert not starting

  def test_below_safe_speed_with_high_planned_speed_and_positive_accel_uses_hill_takeoff(self):
    """Below safe speed, positive openpilot accel with planned drive-away intent uses hill launch."""
    mgr = self._mgr()
    grade = 20.0
    safe_speed = self._safe_speed(mgr, grade)
    _, accel, stopping, starting, _ = self._run(mgr, self._cs(v_ego=safe_speed * 0.5), accel=0.1,
                                                max_planned_speed=safe_speed * 2.0, grade_pct=grade)
    assert accel == max(0.1, 0.1 * grade, 0.2)
    assert starting
    assert not stopping

  # ── start commit ─────────────────────────────────────────────────────────────

  def test_start_commit_on_pre_enable(self):
    """esp_hold_confirmation while long_active triggers start_commit (pre-enable scenario)."""
    mgr = self._mgr()
    self._run(mgr, self._cs(esp_hold_confirmation=True))
    assert mgr.start_commit_active

  def test_start_commit_forces_accel(self):
    """start_commit_active boosts accel to at least hill_launch_accel and 0.2."""
    mgr = self._mgr()
    grade = 10.0
    expected_min = max(0.1 * grade, 0.2)
    self._run(mgr, self._cs(esp_hold_confirmation=True), grade_pct=grade)
    _, accel, stopping, starting, _ = self._run(mgr, self._cs(esp_hold_confirmation=True), accel=0.0,
                                                grade_pct=grade)
    assert accel >= expected_min
    assert starting
    assert not stopping

  def test_start_commit_clears_above_safe_speed_while_moving(self):
    """start_commit_active clears once vEgo exceeds safe stop speed."""
    mgr = self._mgr()
    grade = 20.0
    safe_speed = self._safe_speed(mgr, grade)
    self._run(mgr, self._cs(esp_hold_confirmation=True), grade_pct=grade)
    assert mgr.start_commit_active
    self._run(mgr, self._cs(v_ego=safe_speed * 2.0, standstill=True), grade_pct=grade)
    assert not mgr.start_commit_active

  def test_start_commit_persists_below_safe_speed(self):
    """start_commit_active holds launch ownership until vEgo exceeds safe stop speed."""
    mgr = self._mgr()
    grade = 20.0
    safe_speed = self._safe_speed(mgr, grade)
    self._run(mgr, self._cs(esp_hold_confirmation=True), grade_pct=grade)
    assert mgr.start_commit_active
    _, accel, stopping, starting, _ = self._run(mgr, self._cs(v_ego=safe_speed * 0.5), accel=-0.55,
                                                grade_pct=grade)
    assert mgr.start_commit_active
    assert accel == max(-0.55, 0.1 * grade, 0.2)
    assert starting
    assert not stopping

  # ── can_stop_forever / ESP override ──────────────────────────────────────────

  def test_can_stop_forever_latches_on_esp_stopping(self):
    """can_stop_forever latches True and returns ESPOverride.START when esp_stopping is seen."""
    mgr = self._mgr()
    *_, esp_override = self._run(mgr, self._cs(esp_stopping=True), accel=-1.0, stopping=True)
    assert mgr.can_stop_forever
    assert esp_override == ESPOverride.START

  def test_can_stop_forever_persists(self):
    """can_stop_forever continues returning ESPOverride.START once latched, without esp_stopping."""
    mgr = self._mgr()
    self._run(mgr, self._cs(esp_stopping=True), accel=-1.0, stopping=True)
    *_, esp_override = self._run(mgr, self._cs(), accel=-1.0, stopping=True)
    assert mgr.can_stop_forever
    assert esp_override == ESPOverride.START

  def test_can_stop_forever_cleared_by_hold_confirmation(self):
    """can_stop_forever is cleared when ESP confirms a hold."""
    mgr = self._mgr()
    self._run(mgr, self._cs(esp_stopping=True), accel=-1.0, stopping=True)
    assert mgr.can_stop_forever
    self._run(mgr, self._cs(esp_hold_confirmation=True), accel=-1.0, stopping=True)
    assert not mgr.can_stop_forever

  def test_hold_confirmation_recovers_stop_forever_after_launch_commit(self):
    """A hold confirmation starts recovery, then requests STOP once launch commit clears while moving."""
    mgr = self._mgr()
    grade = 20.0
    safe_speed = self._safe_speed(mgr, grade)

    *_, esp_override = self._run(mgr, self._cs(esp_hold_confirmation=True, sum_wegimpulse=0), accel=0.0,
                                 grade_pct=grade)
    assert mgr.start_commit_active
    assert mgr.hold_recovery_active
    assert esp_override == ESPOverride.START

    *_, esp_override = self._run(mgr, self._cs(v_ego=safe_speed * 2.0, standstill=False, sum_wegimpulse=1), accel=0.0,
                                 grade_pct=grade)
    assert not mgr.start_commit_active
    assert mgr.hold_recovery_active
    assert esp_override == ESPOverride.STOP

    *_, esp_override = self._run(mgr, self._cs(esp_stopping=True, v_ego=safe_speed * 2.0, standstill=False,
                                               sum_wegimpulse=2), accel=0.0, grade_pct=grade)
    assert mgr.can_stop_forever
    assert not mgr.hold_recovery_active
    assert esp_override == ESPOverride.START

  def test_can_stop_forever_cleared_when_moving(self):
    """can_stop_forever clears above the ESP grant speed when esp_stopping is not active."""
    mgr = self._mgr()
    self._run(mgr, self._cs(esp_stopping=True), accel=-1.0, stopping=True)
    assert mgr.can_stop_forever
    self._run(mgr, self._cs(v_ego=MQBStandstillManager.ESP_OVERRIDE_SPEED + 0.01), accel=0.5)
    assert not mgr.can_stop_forever

  def test_can_stop_forever_cleared_when_long_inactive(self):
    """can_stop_forever is cleared when long control is inactive."""
    mgr = self._mgr()
    self._run(mgr, self._cs(esp_stopping=True), accel=-1.0, stopping=True)
    assert mgr.can_stop_forever
    self._run(mgr, self._cs(), long_active=False)
    assert not mgr.can_stop_forever

  def test_esp_override_stop_at_standstill(self):
    """ESPOverride.STOP is requested once wheel impulses have been still long enough."""
    mgr = self._mgr()
    esp_override = None
    for _ in range(MQBStandstillManager.WEGIMPULSE_STILLNESS_FRAMES + 1):
      *_, esp_override = self._run(mgr, self._cs(sum_wegimpulse=0), accel=-1.0, stopping=True)
    assert esp_override == ESPOverride.STOP

  def test_esp_override_stop_persists_at_standstill(self):
    """ESPOverride.STOP remains requested while wheel impulses are still and no hold procedure is detected."""
    mgr = self._mgr()
    for _ in range(MQBStandstillManager.WEGIMPULSE_STILLNESS_FRAMES + 1):
      self._run(mgr, self._cs(sum_wegimpulse=0), accel=-1.0, stopping=True)
    *_, esp_override = self._run(mgr, self._cs(sum_wegimpulse=0), accel=-1.0, stopping=True)
    assert esp_override == ESPOverride.STOP

  def test_esp_override_stop_not_sent_while_esp_stopping(self):
    """Once ESP reports stopping, START is sent instead of another STOP pulse."""
    mgr = self._mgr()
    *_, esp_override = self._run(mgr, self._cs(esp_stopping=True, sum_wegimpulse=0),
                                 accel=-1.0, stopping=True)
    assert mgr.can_stop_forever
    assert esp_override == ESPOverride.START

  def test_esp_override_stop_not_requested_while_moving(self):
    """Below 10 kph, ESPOverride.START is the default while wheel impulses are changing."""
    mgr = self._mgr()
    esp_override = None
    for sum_wegimpulse in range(MQBStandstillManager.WEGIMPULSE_STILLNESS_FRAMES + 1):
      *_, esp_override = self._run(mgr, self._cs(sum_wegimpulse=sum_wegimpulse), accel=-1.0, stopping=True)
    assert esp_override == ESPOverride.START

  def test_esp_override_none_when_inactive(self):
    """Below 10 kph, ESPOverride.START remains the default even when long control is inactive."""
    mgr = self._mgr()
    *_, esp_override = self._run(mgr, self._cs(), long_active=False)
    assert esp_override == ESPOverride.START

  def test_wegimpulse_at_standstill_after_stillness_frames(self):
    """Wheel impulse stillness is tracked after WEGIMPULSE_STILLNESS_FRAMES with constant sum_wegimpulse."""
    mgr = self._mgr()
    for _ in range(MQBStandstillManager.WEGIMPULSE_STILLNESS_FRAMES + 1):
      self._run(mgr, self._cs(sum_wegimpulse=0))
    assert mgr.frames_since_last_wheel_pulse >= MQBStandstillManager.WEGIMPULSE_STILLNESS_FRAMES

  def test_wegimpulse_resets_on_change(self):
    """Wheel impulse stillness resets when sum_wegimpulse changes."""
    mgr = self._mgr()
    for _ in range(MQBStandstillManager.WEGIMPULSE_STILLNESS_FRAMES):
      self._run(mgr, self._cs(sum_wegimpulse=0))
    self._run(mgr, self._cs(sum_wegimpulse=1))
    assert mgr.frames_since_last_wheel_pulse == 0

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
