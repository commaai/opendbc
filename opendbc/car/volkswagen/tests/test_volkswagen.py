import random
import re
import unittest

from opendbc.car import Bus, DT_CTRL
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.carcontroller import HCAMitigation
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR, DBC, CarControllerParams as CCP, FW_QUERY_CONFIG, WMI, VolkswagenFlags, VolkswagenSafetyFlags
from opendbc.car.volkswagen.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu

CHASSIS_CODE_PATTERN = re.compile('[A-Z0-9]{2}')
# TODO: determine the unknown groups
SPARE_PART_FW_PATTERN = re.compile(b'\xf1\x87(?P<gateway>[0-9][0-9A-Z]{2})(?P<unknown>[0-9][0-9A-Z][0-9])(?P<unknown2>[0-9A-Z]{2}[0-9])([A-Z0-9]| )')


class TestVolkswagenID3MEBGen2(unittest.TestCase):
  def test_shared_radar_fingerprint(self):
    # The original upstream ID.4 MK2 port also uses this radar firmware.
    radar_fw = b'\xf1\x871EA907567D \xf1\x890250'
    car_fw = [CarParams.CarFw(ecu=Ecu.fwdRadar, fwVersion=radar_fw, brand="volkswagen", address=0x757)]
    for vin, expected in (
      ("LSV000E9000000000", {CAR.VOLKSWAGEN_ID3_MK2}),
      ("1V2000E8000000000", {CAR.VOLKSWAGEN_ID4_MK2}),
      ("WVW000E9000000000", {CAR.VOLKSWAGEN_ID3_MK2, CAR.VOLKSWAGEN_ID4_MK2}),
      ("00000000000000000", {CAR.VOLKSWAGEN_ID3_MK2, CAR.VOLKSWAGEN_ID4_MK2}),
    ):
      with self.subTest(vin=vin):
        self.assertEqual(match_fw_to_car(car_fw, vin, log=False), (True, expected))

  def test_unique_id4_radar_without_vin(self):
    radar_fw = b'\xf1\x871EA907567C \xf1\x890099'
    car_fw = [CarParams.CarFw(ecu=Ecu.fwdRadar, fwVersion=radar_fw, brand="volkswagen", address=0x757)]
    self.assertEqual(match_fw_to_car(car_fw, "00000000000000000", log=False), (True, {CAR.VOLKSWAGEN_ID4_MK2}))

  def test_id3_vin_without_known_radar(self):
    car_fw = [CarParams.CarFw(ecu=Ecu.fwdRadar, fwVersion=b"unknown", brand="volkswagen", address=0x757)]
    self.assertEqual(match_fw_to_car(car_fw, "LSV000E9000000000", log=False), (True, set()))

  def test_gateway_parameters(self):
    platform = CAR.VOLKSWAGEN_ID3_MK2
    fingerprint = {0: {0x25D: 8, 0x3DC: 8}, 1: {0x520: 8}, 2: {}}
    for is_release in (False, True):
      with self.subTest(is_release=is_release):
        cp = CarInterface.get_params(platform, fingerprint, [], False, is_release, False)
        self.assertEqual(cp.carFingerprint, platform)
        self.assertEqual(cp.networkLocation, CarParams.NetworkLocation.gateway)
        self.assertEqual(cp.steerControlType, CarParams.SteerControlType.curvature)
        self.assertTrue(cp.flags & VolkswagenFlags.MEB_GEN2)
        self.assertTrue(cp.openpilotLongitudinalControl)
        self.assertEqual(cp.dashcamOnly, is_release)
        self.assertEqual(cp.safetyConfigs[-1].safetyModel, CarParams.SafetyModel.volkswagenMeb)
        self.assertEqual(cp.safetyConfigs[-1].safetyParam, VolkswagenSafetyFlags.MEB_ALT_CRC)
        self.assertEqual(DBC[platform][Bus.pt], "vw_meb_2024_generated")


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
