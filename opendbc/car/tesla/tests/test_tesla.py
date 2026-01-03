import random

from opendbc.car.structs import CarParams
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.values import (
  CAR, FW_QUERY_CONFIG, FW_PATTERN, EPS_ADDR,
  get_model_code, get_platform_from_model_code, _init_model_code_mapping,
)

Ecu = CarParams.Ecu


class TestTeslaFuzzyFingerprinting:
  """Test Tesla fuzzy fingerprinting using EPS firmware model codes."""

  def test_fw_pattern_model3(self):
    """Test that Model 3 firmware patterns are correctly parsed."""
    model3_fws = list(FW_VERSIONS[CAR.TESLA_MODEL_3][(Ecu.eps, 0x730, None)])
    for fw in model3_fws:
      model_code = get_model_code(fw)
      assert model_code is not None, f"Failed to parse Model 3 firmware: {fw!r}"
      assert model_code.startswith('E'), f"Model 3 firmware should start with 'E': {fw!r} -> {model_code}"
      _init_model_code_mapping()
      platform = get_platform_from_model_code(model_code)
      assert platform == str(CAR.TESLA_MODEL_3), f"Model 3 firmware should map to Model 3: {fw!r}"

  def test_fw_pattern_model_y(self):
    """Test that Model Y firmware patterns are correctly parsed."""
    model_y_fws = list(FW_VERSIONS[CAR.TESLA_MODEL_Y][(Ecu.eps, 0x730, None)])
    for fw in model_y_fws:
      model_code = get_model_code(fw)
      assert model_code is not None, f"Failed to parse Model Y firmware: {fw!r}"
      assert model_code.startswith('Y'), f"Model Y firmware should start with 'Y': {fw!r} -> {model_code}"
      _init_model_code_mapping()
      platform = get_platform_from_model_code(model_code)
      assert platform == str(CAR.TESLA_MODEL_Y), f"Model Y firmware should map to Model Y: {fw!r}"

  def test_fw_pattern_model_x(self):
    """Test that Model X firmware patterns are correctly parsed."""
    model_x_fws = list(FW_VERSIONS[CAR.TESLA_MODEL_X][(Ecu.eps, 0x730, None)])
    for fw in model_x_fws:
      model_code = get_model_code(fw)
      assert model_code is not None, f"Failed to parse Model X firmware: {fw!r}"
      assert model_code.startswith('X'), f"Model X firmware should start with 'X': {fw!r} -> {model_code}"
      _init_model_code_mapping()
      platform = get_platform_from_model_code(model_code)
      assert platform == str(CAR.TESLA_MODEL_X), f"Model X firmware should map to Model X: {fw!r}"

  def test_get_model_code_invalid(self):
    """Test that invalid firmware strings return None or don't match platforms."""
    invalid_fws = [
      b'',
      b'invalid',
      b'no_comma_here',
      b'TeM3_E014p10_0.0.0 (16),',  # empty after comma
      b'TeM3_E014p10_0.0.0 (16),123',  # starts with number
    ]
    _init_model_code_mapping()
    for fw in invalid_fws:
      model_code = get_model_code(fw)
      if model_code:
        platform = get_platform_from_model_code(model_code)
        # Starting with a digit should not map to any platform
        if platform:
          raise AssertionError(f"Invalid firmware should not map to platform: {fw!r}")

  def test_get_platform_from_model_code_mapping(self):
    """Test model code to platform mapping."""
    _init_model_code_mapping()
    test_cases = [
      ('E014', str(CAR.TESLA_MODEL_3)),
      ('EL014', str(CAR.TESLA_MODEL_3)),
      ('ES014', str(CAR.TESLA_MODEL_3)),
      ('E4014', str(CAR.TESLA_MODEL_3)),
      ('E4H014', str(CAR.TESLA_MODEL_3)),
      ('Y002', str(CAR.TESLA_MODEL_Y)),
      ('YP002', str(CAR.TESLA_MODEL_Y)),
      ('Y4002', str(CAR.TESLA_MODEL_Y)),
      ('XPR003', str(CAR.TESLA_MODEL_X)),
    ]
    for model_code, expected_platform in test_cases:
      platform = get_platform_from_model_code(model_code)
      assert platform == expected_platform, f"Model code {model_code} should map to {expected_platform}"

  def test_get_platform_from_model_code_unknown(self):
    """Test that unknown model codes return None."""
    _init_model_code_mapping()
    unknown_codes = ['', 'Z123', 'S123', 'A', 'B']
    for code in unknown_codes:
      platform = get_platform_from_model_code(code)
      assert platform is None, f"Unknown model code {code} should return None, got {platform}"

  def test_fuzzy_match_all_platforms(self):
    """Test that fuzzy matching works for all known platforms with FW from database."""
    for platform, fw_by_addr in FW_VERSIONS.items():
      # Run multiple random samples to check consistency
      for _ in range(10):
        car_fw = []
        for ecu, fw_versions in fw_by_addr.items():
          ecu_name, addr, sub_addr = ecu
          fw = random.choice(list(fw_versions))
          car_fw.append(CarParams.CarFw(
            ecu=ecu_name, fwVersion=fw, address=addr,
            subAddress=0 if sub_addr is None else sub_addr
          ))

        # Build the live FW dict manually
        live_fw = {(fw.address, fw.subAddress if fw.subAddress != 0 else None): {bytes(fw.fwVersion)}
                   for fw in car_fw}
        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS)
        assert matches == {str(platform)}, f"Platform {platform} should fuzzy match, got {matches}"

  def test_fuzzy_match_unseen_firmware(self):
    """Test that fuzzy matching works for unseen firmware versions."""
    test_cases = [
      (b'TeMYG4_NewProject_0.0.0 (99),E4999.99.99', str(CAR.TESLA_MODEL_3)),
      (b'TeM3_Future_0.0.0 (1),E999.00.00', str(CAR.TESLA_MODEL_3)),
      (b'TeMYG4_Unknown_0.0.0 (50),Y4999.99.0', str(CAR.TESLA_MODEL_Y)),
      (b'TeM3_New_0.0.0 (1),YP999.00.00', str(CAR.TESLA_MODEL_Y)),
      (b'TeM3_SP_XP999p9_0.0.0 (99),XPR999.99.0', str(CAR.TESLA_MODEL_X)),
    ]

    for unseen_fw, expected_platform in test_cases:
      live_fw = {EPS_ADDR: {unseen_fw}}
      matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS)
      assert expected_platform in matches, f"Unseen FW {unseen_fw!r} should match {expected_platform}"

  def test_fuzzy_match_no_match_for_unknown_model(self):
    """Test that firmware with unknown model codes don't match."""
    unknown_fws = [
      b'TeMYG4_Unknown_0.0.0 (1),Z999.00.00',  # Unknown 'Z' prefix
      b'TeM3_Unknown_0.0.0 (1),S999.00.00',    # Model S not supported
      b'invalid_firmware_string',
    ]

    for unknown_fw in unknown_fws:
      live_fw = {EPS_ADDR: {unknown_fw}}
      matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS)
      assert len(matches) == 0, f"Unknown FW {unknown_fw!r} should not match, got {matches}"

  def test_fuzzy_match_empty_fw(self):
    """Test that empty firmware versions don't cause errors."""
    live_fw = {EPS_ADDR: set()}
    matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS)
    assert len(matches) == 0, "Empty firmware should not match any platform"

  def test_fuzzy_match_wrong_address(self):
    """Test that firmware on wrong address doesn't match."""
    live_fw = {(0x731, None): {b'TeM3_E014p10_0.0.0 (16),E014.17.00'}}
    matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS)
    assert len(matches) == 0, "Firmware on wrong address should not match"

  def test_fw_pattern_regex(self):
    """Test that the FW_PATTERN regex matches expected formats."""
    valid_fws = [
      b'TeM3_E014p10_0.0.0 (16),E014.17.00',
      b'TeMYG4_DCS_Update_0.0.0 (13),Y4002.27.1',
      b'TeM3_SP_XP002p2_0.0.0 (23),XPR003.6.0',
    ]
    for fw in valid_fws:
      match = FW_PATTERN.match(fw)
      assert match is not None, f"FW_PATTERN should match: {fw!r}"
      assert match.group('model_code') is not None, f"Should extract model_code from: {fw!r}"

  def test_all_fw_versions_parseable(self):
    """Test that all firmware versions in the database can be parsed correctly."""
    _init_model_code_mapping()
    for platform, fw_by_addr in FW_VERSIONS.items():
      for _ecu, versions in fw_by_addr.items():
        for fw in versions:
          model_code = get_model_code(fw)
          assert model_code is not None, f"Could not parse firmware for {platform}: {fw!r}"
          platform_from_code = get_platform_from_model_code(model_code)
          assert platform_from_code == str(platform), \
            f"Firmware {fw!r} parsed as {platform_from_code}, expected {platform}"
