import random
import unittest

from hypothesis import settings, given, strategies as st

from opendbc.car.structs import CarParams
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.chrysler.values import FW_QUERY_CONFIG, FW_PATTERN, PLATFORM_CODE_ECUS, get_platform_codes
from opendbc.car.chrysler.fingerprints import FW_VERSIONS
from opendbc.testing import parameterized

Ecu = CarParams.Ecu


class TestChryslerFW(unittest.TestCase):
  # Validates that every platform code ECU in the database has at least some
  # parseable FW versions. Not all FW may parse (e.g. RAM radar b'22DTRHD_AA'),
  # but each ECU should have at least one standard XXXXXXXXYY format entry.
  @parameterized("car_model, fw_versions", FW_VERSIONS.items())
  def test_fw_versions(self, car_model, fw_versions):
    for (ecu, _addr, _subaddr), fws in fw_versions.items():
      if ecu not in PLATFORM_CODE_ECUS:
        continue

      assert len(get_platform_codes(fws)) > 0, \
        f"No FW versions for Ecu.{ecu} could be parsed for platform codes"

  # Fuzz test: get_platform_codes should never raise on arbitrary byte strings
  @settings(max_examples=100)
  @given(data=st.data())
  def test_platform_codes_fuzzy_fw(self, data):
    fw_strategy = st.lists(st.binary())
    fws = data.draw(fw_strategy)
    get_platform_codes(fws)

  # Spot-check that known FW versions parse to the expected part numbers
  def test_platform_codes_spot_check(self):
    results = get_platform_codes([
      b'68227902AF',   # Pacifica combinationMeter
      b'68222747AG',   # Pacifica ABS
      b'04672758AA',   # Pacifica fwdRadar (Delphi supplier prefix)
      b'68288891AE',   # Pacifica EPS
    ])
    assert results == {b'68227902', b'68222747', b'04672758', b'68288891'}

  # Engine ECUs often have a trailing space — make sure we still parse those
  def test_platform_codes_with_trailing_space(self):
    results = get_platform_codes([b'68267018AO '])
    assert results == {b'68267018'}

    results = get_platform_codes([b'68267018AO'])
    assert results == {b'68267018'}

  # Non-standard formats should return empty (no crash, no false positives)
  def test_platform_codes_unparseable(self):
    results = get_platform_codes([b'22DTRHD_AA', b'', b'short'])
    assert results == set()

  # Every supported platform needs at least 2 ECUs with parseable part numbers
  # to make fuzzy matching reliable (avoids single-ECU false positives)
  def test_platform_code_ecus_available(self):
    for car_model, fw_by_addr in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        ecus_with_codes = {ecu for ecu, fws in fw_by_addr.items()
                          if ecu[0] in PLATFORM_CODE_ECUS and len(get_platform_codes(fws)) > 0}
        assert len(ecus_with_codes) >= 2, \
          f"{car_model}: Only {len(ecus_with_codes)} ECUs have parseable platform codes"

  # Core fuzzy matching test: with exact (database) FW versions, every platform
  # should uniquely match to itself across 20 random FW selections per platform
  def test_fuzzy_match(self):
    for platform, fw_by_addr in FW_VERSIONS.items():
      for _ in range(20):
        car_fw = []
        for ecu, fw_versions in fw_by_addr.items():
          ecu_name, addr, sub_addr = ecu
          fw = random.choice(fw_versions)
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

        CP = CarParams(carFw=car_fw)
        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
        assert matches == {platform}, f"{platform}: expected {{platform}}, got {matches}"

  # The whole point of fuzzy matching: unseen revision codes (e.g. from a firmware
  # update) should still identify the correct platform via part number matching
  def test_fuzzy_match_mutated_revisions(self):
    revisions = [b'AA', b'ZZ', b'XY', b'QQ']
    for platform, fw_by_addr in FW_VERSIONS.items():
      for _ in range(10):
        car_fw = []
        for ecu, fw_versions in fw_by_addr.items():
          ecu_name, addr, sub_addr = ecu
          fw = random.choice(fw_versions)
          # Swap the revision code on platform code ECUs to simulate an update
          match = FW_PATTERN.match(fw)
          if match and ecu[0] in PLATFORM_CODE_ECUS:
            fw = match.group('part_number') + random.choice(revisions)
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

        CP = CarParams(carFw=car_fw)
        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
        assert matches == {platform}, f"{platform}: expected {{platform}}, got {matches}"

  # Fabricated part numbers should never match any platform
  def test_fuzzy_match_wrong_part_numbers(self):
    live_fw = {
      (0x742, None): {b'99999999AA'},
      (0x747, None): {b'99999998BB'},
      (0x753, None): {b'99999997CC'},
      (0x75a, None): {b'99999996DD'},
    }
    matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS)
    assert len(matches) == 0

  # No FW at all should obviously not match anything
  def test_fuzzy_match_empty_input(self):
    matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy({}, '', FW_VERSIONS)
    assert len(matches) == 0


if __name__ == "__main__":
  unittest.main()
