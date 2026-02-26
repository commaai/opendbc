import random
import re

from opendbc.car import DT_CTRL
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.carcontroller import HCAMitigation
from opendbc.car.volkswagen.values import CAR, FW_QUERY_CONFIG, WMI
from opendbc.car.volkswagen.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu

CHASSIS_CODE_PATTERN = re.compile('[A-Z0-9]{2}')
# TODO: determine the unknown groups
SPARE_PART_FW_PATTERN = re.compile(b'\xf1\x87(?P<gateway>[0-9][0-9A-Z]{2})(?P<unknown>[0-9][0-9A-Z][0-9])(?P<unknown2>[0-9A-Z]{2}[0-9])([A-Z0-9]| )')


class TestVWHCAMitigation:
  # HCA runs at 50Hz (STEER_STEP=2 at DT_CTRL=0.01s), so 1 HCA frame = DT_CTRL * 2 seconds.
  # Threshold values derived from HCAMitigation class constants.
  STUCK_TORQUE_FRAMES = round(HCAMitigation.STEER_TIME_STUCK_TORQUE / (DT_CTRL * 2))
  ALERT_FRAMES = round(HCAMitigation.STEER_TIME_ALERT / (DT_CTRL * 2))

  def test_same_torque_nudge_fires(self):
    hca = HCAMitigation()
    # One frame short of the threshold: no nudge
    for _ in range(self.STUCK_TORQUE_FRAMES):
      result = hca.update(True, 100, 100)
    assert result == 100

    # One more frame tips over the threshold
    result = hca.update(True, 100, 100)
    assert result == 99

  def test_same_torque_nudge_direction_negative(self):
    hca = HCAMitigation()
    for _ in range(self.STUCK_TORQUE_FRAMES):
      hca.update(True, -100, -100)
    result = hca.update(True, -100, -100)
    assert result == -99

  def test_torque_change_resets_same_torque_counter(self):
    hca = HCAMitigation()
    # Run to one frame short of the threshold
    for _ in range(self.STUCK_TORQUE_FRAMES):
      hca.update(True, 100, 100)

    # A torque change resets the counter; the next STUCK_TORQUE_FRAMES frames should not nudge
    hca.update(True, 101, 100)
    for _ in range(self.STUCK_TORQUE_FRAMES):
      result = hca.update(True, 101, 101)
    assert result == 101

  def test_same_torque_counter_not_reset_on_inactive(self):
    # hca_frames_same_torque persists across inactive periods (matches original carcontroller behavior)
    hca = HCAMitigation()
    for _ in range(self.STUCK_TORQUE_FRAMES):
      hca.update(True, 100, 100)

    hca.update(False, 0, 100)

    result = hca.update(True, 100, 100)
    assert result == 99

  def test_active_timer_resets_on_inactive(self):
    hca = HCAMitigation()
    for _ in range(10):
      hca.update(True, 100, 50)
    assert hca.hca_frames_active == 10

    hca.update(False, 0, 100)
    assert hca.hca_frames_active == 0

  def test_active_timer_resets_on_zero_torque(self):
    hca = HCAMitigation()
    for _ in range(10):
      hca.update(True, 100, 50)
    assert hca.hca_frames_active == 10

    hca.update(True, 0, 100)
    assert hca.hca_frames_active == 0

  def test_eps_timer_alert_not_set_before_threshold(self):
    hca = HCAMitigation()
    hca.hca_frames_active = self.ALERT_FRAMES
    assert not hca.eps_timer_soft_disable_alert()

  def test_eps_timer_alert_sets_past_threshold(self):
    hca = HCAMitigation()
    hca.hca_frames_active = self.ALERT_FRAMES + 1
    assert hca.eps_timer_soft_disable_alert()


class TestVolkswagenPlatformConfigs:
  def test_spare_part_fw_pattern(self, subtests):
    # Relied on for determining if a FW is likely VW
    for platform, ecus in FW_VERSIONS.items():
      with subtests.test(platform=platform.value):
        for fws in ecus.values():
          for fw in fws:
            assert SPARE_PART_FW_PATTERN.match(fw) is not None, f"Bad FW: {fw}"

  def test_chassis_codes(self, subtests):
    for platform in CAR:
      with subtests.test(platform=platform.value):
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

  def test_custom_fuzzy_fingerprinting(self, subtests):
    all_radar_fw = list({fw for ecus in FW_VERSIONS.values() for fw in ecus[Ecu.fwdRadar, 0x757, None]})

    for platform in CAR:
      with subtests.test(platform=platform.name):
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
