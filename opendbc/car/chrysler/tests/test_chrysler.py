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
  @parameterized("car_model, fw_versions", FW_VERSIONS.items())
  def test_fw_versions(self, car_model, fw_versions):
    for (_ecu, _addr, _subaddr), fws in fw_versions.items():
      for fw in fws:
        match = FW_PATTERN.match(fw)
        assert match is not None, f"Unable to parse FW: {fw!r}"

        codes = get_platform_codes([fw])
        assert 1 == len(codes), f"Unable to parse FW: {fw!r}"

  @parameterized("car_model, fw_versions", FW_VERSIONS.items())
  def test_platform_code_ecus_available(self, car_model, fw_versions):
    # Asserts ECU keys essential for fuzzy fingerprinting are available on all platforms
    present_ecus = {ecu[0] for ecu in fw_versions if ecu[0] in PLATFORM_CODE_ECUS}
    assert len(present_ecus) >= 3, "Platform has too few ECUs to fuzzy fingerprint"

  @settings(max_examples=100)
  @given(data=st.data())
  def test_platform_codes_fuzzy_fw(self, data):
    """Ensure function doesn't raise an exception"""
    fw_strategy = st.lists(st.binary())
    fws = data.draw(fw_strategy)
    get_platform_codes(fws)

  def test_platform_codes_spot_check(self):
    # Asserts basic platform code parsing behavior for a few cases
    results = get_platform_codes([
      b"68227902AF",
      b"68227902AG",
      b"68360252AC",
      b"68267018AO ",
      b"22DTRHD_AA",
      b"M2370131MB",
    ])
    assert results == {b"68227902", b"68360252", b"68267018", b"22DTRHD_", b"M2370131"}

  def test_fuzzy_match(self):
    # Ensure that unique part number combinations map to one platform
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
        assert matches == {platform}

  def test_fuzzy_match_new_revision(self):
    # Ensure fuzzy matching is robust to unseen software revisions of known part numbers
    for platform, fw_by_addr in FW_VERSIONS.items():
      for _ in range(20):
        live_fw = {}
        for (_ecu, addr, sub_addr), fw_versions in fw_by_addr.items():
          fw = random.choice(fw_versions)
          part_number = FW_PATTERN.match(fw).group('part_number')
          live_fw[(addr, sub_addr)] = {part_number + b'ZZ'}

        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS)
        assert matches == {platform}

  def test_fuzzy_match_unknown_part_number(self):
    # Ensure fuzzy matching rejects platforms on an unseen part number for a platform code ECU
    for _platform, fw_by_addr in FW_VERSIONS.items():
      live_fw = {}
      for (ecu, addr, sub_addr), fw_versions in fw_by_addr.items():
        fw = random.choice(fw_versions)
        if ecu == Ecu.combinationMeter:
          fw = b'99999999AA'
        live_fw[(addr, sub_addr)] = {fw}

      matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS)
      assert matches == set()

  def test_fuzzy_match_missing_ecu(self):
    # Ensure fuzzy matching rejects platforms when an expected platform code ECU is missing
    for _platform, fw_by_addr in FW_VERSIONS.items():
      live_fw = {}
      for (ecu, addr, sub_addr), fw_versions in fw_by_addr.items():
        if ecu == Ecu.combinationMeter:
          continue
        live_fw[(addr, sub_addr)] = {random.choice(fw_versions)}

      matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS)
      assert matches == set()

  def test_fuzzy_match_empty(self):
    matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy({}, '', FW_VERSIONS)
    assert matches == set()
