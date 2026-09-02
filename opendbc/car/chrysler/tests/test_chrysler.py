import random
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.chrysler.values import (CAR, FW_QUERY_CONFIG, FW_PATTERN, MAX_UNSEEN_PLATFORM_CODE_ECUS, PLATFORM_CODE_ECUS,
                                         get_platform_codes)
from opendbc.car.chrysler.fingerprints import FW_VERSIONS
from opendbc.testing import fuzzy_test, parameterized

Ecu = CarParams.Ecu

# EPS part number shared by the Pacifica ICE platforms, ABS part number shared by Durango and 2019+ Grand Cherokee
SHARED_EPS_PART = b'68525338'
SHARED_ABS_PART = b'68408639'


def random_live_fw(fw_by_addr, revision: bytes | None = None) -> dict:
  # Build a live FW dict from one random known version per ECU, optionally with an unseen software revision
  live_fw = {}
  for (_ecu, addr, sub_addr), fw_versions in fw_by_addr.items():
    fw = random.choice(fw_versions)
    if revision is not None:
      fw = FW_PATTERN.match(fw).group('part_number') + revision
    live_fw[(addr, sub_addr)] = {fw}
  return live_fw


def unseen_part_number() -> bytes:
  return b'99' + str(random.randint(100000, 999999)).encode() + b'AA'


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
    # Every platform needs enough platform code ECUs for one unseen part number to be tolerated
    present_ecus = {ecu[0] for ecu in fw_versions if ecu[0] in PLATFORM_CODE_ECUS}
    assert len(present_ecus) >= 3, "Platform has too few ECUs to fuzzy fingerprint"

  @fuzzy_test(max_examples=100)
  def test_platform_codes_fuzzy_fw(self, fuzzy):
    """Ensure function doesn't raise an exception"""
    get_platform_codes(fuzzy.list(fuzzy.binary))

  def test_platform_codes_spot_check(self):
    # Asserts basic part number parsing behavior for a few cases
    results = get_platform_codes([
      b"68227902AF",
      b"68227902AG",
      b"68360252AC",
      b"68267018AO ",
      b"68277370AJ",
      b"05150892AF",
      b"22DTRHD_AA",
      b"M2370131MB",
    ])
    assert results == {b"68227902", b"68360252", b"68267018", b"68277370", b"05150892", b"22DTRHD_", b"M2370131"}

  def test_platforms_distinguishable(self):
    # Every platform must differ from every other platform on more platform code ECUs than we tolerate as unseen,
    # otherwise a car with an unseen part number could be accepted as a sibling platform
    parts_by_platform = {}
    for platform, fw_by_addr in FW_VERSIONS.items():
      parts_by_platform[platform] = {ecu[1:]: get_platform_codes(fws) for ecu, fws in fw_by_addr.items()
                                     if ecu[0] in PLATFORM_CODE_ECUS}

    for platform, parts in parts_by_platform.items():
      for other, other_parts in parts_by_platform.items():
        if platform == other:
          continue
        # ECUs the other platform expects that this platform can satisfy with a shared part number
        shared_ecus = {addr for addr in other_parts if other_parts[addr] & parts.get(addr, set())}
        distinct_ecus = set(other_parts) - shared_ecus
        assert len(distinct_ecus) > MAX_UNSEEN_PLATFORM_CODE_ECUS, f"{platform} could be mistaken for {other}, only {distinct_ecus} differ"

  def test_fuzzy_match(self):
    # Known FW must fuzzy match exactly one platform
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
    # Unseen software revisions of known part numbers on every ECU must still match
    for platform, fw_by_addr in FW_VERSIONS.items():
      for _ in range(20):
        live_fw = random_live_fw(fw_by_addr, revision=b'ZZ')
        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS)
        assert matches == {platform}

  def test_fuzzy_match_one_new_part_number(self):
    # A single platform code ECU with an unseen part number (typically a new model year) must still match,
    # and must never match a different platform
    for platform, fw_by_addr in FW_VERSIONS.items():
      for ecu in fw_by_addr:
        if ecu[0] not in PLATFORM_CODE_ECUS:
          continue
        for _ in range(10):
          live_fw = random_live_fw(fw_by_addr, revision=b'ZZ')
          live_fw[ecu[1:]] = {unseen_part_number()}
          matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS)
          assert matches == {platform}, f"{platform} with unseen part number on {ecu} matched {matches}"

  def test_fuzzy_match_two_new_part_numbers(self):
    # Two unseen part numbers are not enough evidence for any platform
    for _platform, fw_by_addr in FW_VERSIONS.items():
      platform_ecus = [ecu for ecu in fw_by_addr if ecu[0] in PLATFORM_CODE_ECUS]
      for _ in range(10):
        live_fw = random_live_fw(fw_by_addr)
        for ecu in random.sample(platform_ecus, 2):
          live_fw[ecu[1:]] = {unseen_part_number()}
        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS)
        assert matches == set(), f"Matched {matches} with two unseen part numbers"

  def test_fuzzy_match_missing_ecu(self):
    # A missing platform code ECU counts the same as an unseen part number: one is tolerated, two are not
    for platform, fw_by_addr in FW_VERSIONS.items():
      platform_ecus = [ecu for ecu in fw_by_addr if ecu[0] in PLATFORM_CODE_ECUS]

      live_fw = random_live_fw(fw_by_addr)
      del live_fw[platform_ecus[0][1:]]
      assert FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS) == {platform}

      del live_fw[platform_ecus[1][1:]]
      assert FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS) == set()

  def test_fuzzy_match_sibling_platforms(self):
    # Platforms that share a part number on one ECU stay apart even when a second ECU is unseen
    pacifica_2018 = FW_VERSIONS[CAR.CHRYSLER_PACIFICA_2018]
    live_fw = random_live_fw(pacifica_2018, revision=b'ZZ')
    live_fw[(0x75a, None)] = {SHARED_EPS_PART + b'ZZ'}  # EPS shared with Pacifica 2020
    live_fw[(0x742, None)] = {unseen_part_number()}     # unseen cluster
    assert FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS) == {CAR.CHRYSLER_PACIFICA_2018}

    durango = FW_VERSIONS[CAR.DODGE_DURANGO]
    live_fw = random_live_fw(durango, revision=b'ZZ')
    live_fw[(0x747, None)] = {SHARED_ABS_PART + b'ZZ'}  # ABS shared with Grand Cherokee 2019
    live_fw[(0x742, None)] = {unseen_part_number()}     # unseen cluster
    assert FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS) == {CAR.DODGE_DURANGO}

  def test_fuzzy_match_empty(self):
    assert FW_QUERY_CONFIG.match_fw_to_car_fuzzy({}, '', FW_VERSIONS) == set()
