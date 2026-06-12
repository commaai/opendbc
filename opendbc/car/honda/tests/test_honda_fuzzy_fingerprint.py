"""
Tests for Honda fuzzy fingerprinting.

Verifies that Honda's fuzzy fingerprinting correctly identifies car models based on
platform codes in firmware strings, even for firmware versions not in the database.
"""
import random
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.honda.values import (
    CAR,
    HONDA_FW_PATTERN,
    PLATFORM_CODE_ECUS,
    get_platform_codes,
    match_fw_to_car_fuzzy,
    FW_QUERY_CONFIG,
)
from opendbc.car.honda.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu


class TestHondaFwPattern(unittest.TestCase):
  """Test that the firmware regex pattern matches all known Honda firmware versions."""

  def test_pattern_matches_all_known_fw(self):
    for car_model, ecus in FW_VERSIONS.items():
      for ecu, versions in ecus.items():
        for fw in versions:
          with self.subTest(car_model=car_model, ecu=ecu, fw=fw):
            match = HONDA_FW_PATTERN.match(fw)
            self.assertIsNotNone(match, f"Pattern failed to match: {fw!r}")

  def test_pattern_extracts_groups(self):
    match = HONDA_FW_PATTERN.match(b'39990-TVA-A150\x00\x00')
    self.assertIsNotNone(match)
    self.assertEqual(match.group('part_number'), b'39990')
    self.assertEqual(match.group('platform_code'), b'TVA')
    self.assertEqual(match.group('version'), b'A150')

  def test_pattern_alphanumeric_part_number(self):
    match = HONDA_FW_PATTERN.match(b'8S102-30A-A050\x00\x00')
    self.assertIsNotNone(match)
    self.assertEqual(match.group('part_number'), b'8S102')
    self.assertEqual(match.group('platform_code'), b'30A')

  def test_pattern_rejects_modified_fw(self):
    """Modified FW uses comma instead of second dash and should not match."""
    match = HONDA_FW_PATTERN.match(b'39990-TVA,A150\x00\x00')
    self.assertIsNone(match)


class TestGetPlatformCodes(unittest.TestCase):
  """Test platform code extraction from firmware version strings."""

  def test_extracts_platform_code(self):
    codes = get_platform_codes([b'57114-TVA-B040\x00\x00'])
    self.assertEqual(codes, {b'TVA'})

  def test_multiple_codes(self):
    codes = get_platform_codes([
      b'57114-TVA-B040\x00\x00',
      b'57114-TWA-A040\x00\x00',
    ])
    self.assertEqual(codes, {b'TVA', b'TWA'})

  def test_deduplicates(self):
    codes = get_platform_codes([
      b'57114-TVA-B040\x00\x00',
      b'57114-TVA-C050\x00\x00',
    ])
    self.assertEqual(codes, {b'TVA'})

  def test_empty_input(self):
    self.assertEqual(get_platform_codes([]), set())
    self.assertEqual(get_platform_codes(set()), set())

  def test_garbage_input(self):
    self.assertEqual(get_platform_codes([b'garbage']), set())
    self.assertEqual(get_platform_codes([b'']), set())
    self.assertEqual(get_platform_codes([b'\x00\x00\x00']), set())

  def test_all_known_fw_parseable(self):
    """Every FW version in the database for PLATFORM_CODE_ECUS should parse."""
    for car_model, ecus in FW_VERSIONS.items():
      for ecu, versions in ecus.items():
        if ecu[0] in PLATFORM_CODE_ECUS:
          codes = get_platform_codes(versions)
          with self.subTest(car_model=car_model, ecu=ecu):
            self.assertGreater(len(codes), 0, f"No platform codes parsed for {car_model} {ecu}")


class TestHondaFuzzyMatch(unittest.TestCase):
  """Test the fuzzy fingerprinting function for Honda vehicles."""

  @property
  def offline_fw(self):
    return {car.value: ecus for car, ecus in FW_VERSIONS.items()}

  def test_known_fw_matches_correct_model(self):
    """Every known FW set should fuzzy-match to its own model."""
    for car_model, fw_by_addr in FW_VERSIONS.items():
      for _ in range(5):
        car_fw = []
        for ecu, fw_versions in fw_by_addr.items():
          ecu_name, addr, sub_addr = ecu
          fw = random.choice(fw_versions)
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

        CP = CarParams(carFw=car_fw)
        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
        with self.subTest(car_model=car_model):
          self.assertIn(car_model, matches, f"{car_model} should match its own FW")

  def test_no_false_cross_platform_matches(self):
    """Cars with distinct platform codes should not cross-match.

    Checks pairs where platform codes are fully disjoint across all PLATFORM_CODE_ECUS.
    """
    # Build a map of platform codes per car (only from PLATFORM_CODE_ECUS)
    car_codes = {}
    for car_model, ecus in FW_VERSIONS.items():
      all_codes = set()
      for ecu, versions in ecus.items():
        if ecu[0] in PLATFORM_CODE_ECUS:
          all_codes |= get_platform_codes(versions)
      car_codes[car_model] = all_codes

    for car_model, fw_by_addr in FW_VERSIONS.items():
      # Pick one FW per ECU
      live = {}
      for ecu, versions in fw_by_addr.items():
        addr = ecu[1:]
        live[addr] = {versions[0]}

      matches = match_fw_to_car_fuzzy(live, '', self.offline_fw)

      # Should match own model
      self.assertIn(car_model.value, matches)

      # Should NOT match models with completely disjoint platform codes
      for other_model in FW_VERSIONS:
        if other_model == car_model:
          continue
        if not (car_codes[car_model] & car_codes[other_model]):
          with self.subTest(car=car_model, other=other_model):
            self.assertNotIn(other_model.value, matches,
                             f"{car_model} should not match {other_model} (disjoint platform codes)")

  def test_forward_compatible_unseen_versions(self):
    """Simulated future firmware versions (new version suffix) should still match."""
    test_cases = [
      # (car, ecu_addr, known_fw, simulated_future_fw)
      (CAR.HONDA_ACCORD, (0x18dab0f1, None),
       b'36802-TVA-A150\x00\x00', b'36802-TVA-Z990\x00\x00'),
      (CAR.HONDA_CIVIC_BOSCH, (0x18da53f1, None),
       b'77959-TGG-A020\x00\x00', b'77959-TGG-Z990\x00\x00'),
      (CAR.HONDA_CRV_6G, (0x18dab0f1, None),
       b'8S302-3A0-A060\x00\x00', b'8S302-3A0-Z990\x00\x00'),
      (CAR.HONDA_PILOT_4G, (0x18dab5f1, None),
       b'8S102-T90-A050\x00\x00', b'8S102-T90-Z990\x00\x00'),
    ]

    for car, addr, known_fw, future_fw in test_cases:
      with self.subTest(car=car, future_fw=future_fw):
        # Build full live FW from known FW for all ECUs, then replace one
        live = {}
        for ecu, versions in FW_VERSIONS[car].items():
          ecu_addr = ecu[1:]
          live[ecu_addr] = {versions[0]}
        live[addr] = {future_fw}

        matches = match_fw_to_car_fuzzy(live, '', self.offline_fw)
        self.assertIn(car.value, matches, f"Future FW {future_fw!r} should still match {car}")

  def test_unknown_platform_code_no_match(self):
    """FW with a platform code not in any database should not match."""
    live = {
      (0x18da53f1, None): {b'77959-ZZZ-A010\x00\x00'},
      (0x18dab0f1, None): {b'36802-ZZZ-A010\x00\x00'},
      (0x18dab5f1, None): {b'36161-ZZZ-A010\x00\x00'},
    }
    matches = match_fw_to_car_fuzzy(live, '', self.offline_fw)
    self.assertEqual(len(matches), 0)

  def test_malformed_fw_no_match(self):
    """Garbage firmware data should not produce any matches."""
    live = {
      (0x18dab0f1, None): {b'garbage'},
      (0x18da53f1, None): {b''},
      (0x18dab5f1, None): {b'\x00\x00\x00\x00'},
    }
    matches = match_fw_to_car_fuzzy(live, '', self.offline_fw)
    self.assertEqual(len(matches), 0)

  def test_empty_live_fw(self):
    matches = match_fw_to_car_fuzzy({}, '', self.offline_fw)
    self.assertEqual(len(matches), 0)

  def test_wrong_ecu_address(self):
    live = {(0x999, None): {b'36802-TVA-A150\x00\x00'}}
    matches = match_fw_to_car_fuzzy(live, '', self.offline_fw)
    self.assertEqual(len(matches), 0)

  def test_specific_no_cross_match_accord_civic(self):
    """Accord FW should not fuzzy-match Civic and vice versa."""
    # Accord live FW
    accord_live = {}
    for ecu, versions in FW_VERSIONS[CAR.HONDA_ACCORD].items():
      accord_live[ecu[1:]] = {versions[0]}

    matches = match_fw_to_car_fuzzy(accord_live, '', self.offline_fw)
    self.assertIn(CAR.HONDA_ACCORD.value, matches)
    self.assertNotIn(CAR.HONDA_CIVIC.value, matches)
    self.assertNotIn(CAR.HONDA_CIVIC_BOSCH.value, matches)
    self.assertNotIn(CAR.HONDA_CRV_5G.value, matches)

  def test_match_fw_fuzzy_changed_version(self):
    """Ensure fuzzy match works with changed version suffixes."""
    # Use Accord as test case
    offline_fw = {
      (Ecu.srs, 0x18da53f1, None): [
        b'77959-TVA-A460\x00\x00',
        b'77959-TVA-H230\x00\x00',
      ],
      (Ecu.fwdRadar, 0x18dab0f1, None): [
        b'36802-TVA-A150\x00\x00',
        b'36802-TVA-A160\x00\x00',
      ],
      (Ecu.fwdCamera, 0x18dab5f1, None): [
        b'36161-TVA-A060\x00\x00',
      ],
    }
    expected_fingerprint = CAR.HONDA_ACCORD.value

    # Changed version suffixes should still match
    live_fw = {
      (0x18da53f1, None): {b'77959-TVA-Z990\x00\x00'},
      (0x18dab0f1, None): {b'36802-TVA-Z990\x00\x00'},
      (0x18dab5f1, None): {b'36161-TVA-Z990\x00\x00'},
    }
    candidates = match_fw_to_car_fuzzy(live_fw, '', {expected_fingerprint: offline_fw})
    self.assertEqual(candidates, {expected_fingerprint})

    # Wrong platform code should not match
    live_fw[(0x18dab0f1, None)] = {b'36802-ZZZ-A010\x00\x00'}
    candidates = match_fw_to_car_fuzzy(live_fw, '', {expected_fingerprint: offline_fw})
    self.assertEqual(len(candidates), 0)


if __name__ == '__main__':
  unittest.main()
