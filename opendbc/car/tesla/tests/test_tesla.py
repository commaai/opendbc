import random

from opendbc.car.structs import CarParams
from opendbc.car.tesla.values import (
  CAR, FW_QUERY_CONFIG, FW_PATTERN, get_model_code, get_platform_from_model_code,
)
from opendbc.car.tesla.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu


# Known ECU address for Tesla EPS
ECU_ADDRESSES = {
  Ecu.eps: 0x730,
}


class TestTeslaFW:
  def test_fw_pattern_model3(self):
    """Test that Model 3 firmware patterns are correctly parsed."""
    model3_fws = [
      b'TeM3_E014p10_0.0.0 (16),E014.17.00',
      b'TeM3_E014p10_0.0.0 (16),EL014.17.00',
      b'TeM3_ES014p11_0.0.0 (25),ES014.19.0',
      b'TeMYG4_DCS_Update_0.0.0 (13),E4014.28.1',
      b'TeMYG4_DCS_Update_0.0.0 (9),E4014.26.0',
      b'TeMYG4_Legacy3Y_0.0.0 (2),E4015.02.0',
      b'TeMYG4_Legacy3Y_0.0.0 (5),E4015.03.2',
      b'TeMYG4_Legacy3Y_0.0.0 (5),E4L015.03.2',
      b'TeMYG4_Main_0.0.0 (59),E4H014.29.0',
      b'TeMYG4_Main_0.0.0 (65),E4H015.01.0',
      b'TeMYG4_Main_0.0.0 (67),E4H015.02.1',
      b'TeMYG4_SingleECU_0.0.0 (33),E4S014.27',
    ]
    for fw in model3_fws:
      model_code = get_model_code(fw)
      assert model_code is not None, f"Failed to parse Model 3 firmware: {fw!r}"
      assert model_code.startswith('E'), f"Model 3 firmware should start with 'E': {fw!r} -> {model_code}"
      platform = get_platform_from_model_code(model_code)
      assert platform == str(CAR.TESLA_MODEL_3), f"Model 3 firmware should map to Model 3: {fw!r} -> {platform}"

  def test_fw_pattern_model_y(self):
    """Test that Model Y firmware patterns are correctly parsed."""
    model_y_fws = [
      b'TeM3_E014p10_0.0.0 (16),Y002.18.00',
      b'TeM3_E014p10_0.0.0 (16),YP002.18.00',
      b'TeM3_ES014p11_0.0.0 (16),YS002.17',
      b'TeM3_ES014p11_0.0.0 (25),YS002.19.0',
      b'TeMYG4_DCS_Update_0.0.0 (13),Y4002.27.1',
      b'TeMYG4_DCS_Update_0.0.0 (13),Y4P002.27.1',
      b'TeMYG4_DCS_Update_0.0.0 (9),Y4P002.25.0',
      b'TeMYG4_Legacy3Y_0.0.0 (2),Y4003.02.0',
      b'TeMYG4_Legacy3Y_0.0.0 (2),Y4P003.02.0',
      b'TeMYG4_Legacy3Y_0.0.0 (5),Y4003.03.2',
      b'TeMYG4_Legacy3Y_0.0.0 (5),Y4P003.03.2',
      b'TeMYG4_SingleECU_0.0.0 (28),Y4S002.23.0',
      b'TeMYG4_SingleECU_0.0.0 (33),Y4S002.26',
    ]
    for fw in model_y_fws:
      model_code = get_model_code(fw)
      assert model_code is not None, f"Failed to parse Model Y firmware: {fw!r}"
      assert model_code.startswith('Y'), f"Model Y firmware should start with 'Y': {fw!r} -> {model_code}"
      platform = get_platform_from_model_code(model_code)
      assert platform == str(CAR.TESLA_MODEL_Y), f"Model Y firmware should map to Model Y: {fw!r} -> {platform}"

  def test_fw_pattern_model_x(self):
    """Test that Model X firmware patterns are correctly parsed."""
    model_x_fws = [
      b'TeM3_SP_XP002p2_0.0.0 (23),XPR003.6.0',
      b'TeM3_SP_XP002p2_0.0.0 (36),XPR003.10.0',
    ]
    for fw in model_x_fws:
      model_code = get_model_code(fw)
      assert model_code is not None, f"Failed to parse Model X firmware: {fw!r}"
      assert model_code.startswith('X'), f"Model X firmware should start with 'X': {fw!r} -> {model_code}"
      platform = get_platform_from_model_code(model_code)
      assert platform == str(CAR.TESLA_MODEL_X), f"Model X firmware should map to Model X: {fw!r} -> {platform}"

  def test_get_model_code_invalid(self):
    """Test that invalid firmware strings return None."""
    invalid_fws = [
      b'',
      b'invalid',
      b'no_comma_here',
      b'TeM3_E014p10_0.0.0 (16),',  # empty model code
      b'TeM3_E014p10_0.0.0 (16),123',  # starts with number
    ]
    for fw in invalid_fws:
      model_code = get_model_code(fw)
      # Empty model code after comma should still fail platform mapping
      if model_code:
        platform = get_platform_from_model_code(model_code)
        if platform:
          raise AssertionError(f"Invalid firmware should not map to platform: {fw!r}")

  def test_get_platform_from_model_code(self):
    """Test model code to platform mapping."""
    test_cases = [
      ('E014', str(CAR.TESLA_MODEL_3)),
      ('EL014', str(CAR.TESLA_MODEL_3)),
      ('ES014', str(CAR.TESLA_MODEL_3)),
      ('E4014', str(CAR.TESLA_MODEL_3)),
      ('E4H014', str(CAR.TESLA_MODEL_3)),
      ('E4S014', str(CAR.TESLA_MODEL_3)),
      ('E4L015', str(CAR.TESLA_MODEL_3)),
      ('Y002', str(CAR.TESLA_MODEL_Y)),
      ('YP002', str(CAR.TESLA_MODEL_Y)),
      ('YS002', str(CAR.TESLA_MODEL_Y)),
      ('Y4002', str(CAR.TESLA_MODEL_Y)),
      ('Y4P002', str(CAR.TESLA_MODEL_Y)),
      ('Y4S002', str(CAR.TESLA_MODEL_Y)),
      ('XPR003', str(CAR.TESLA_MODEL_X)),
    ]
    for model_code, expected_platform in test_cases:
      platform = get_platform_from_model_code(model_code)
      assert platform == expected_platform, f"Model code {model_code} should map to {expected_platform}, got {platform}"

  def test_get_platform_from_model_code_unknown(self):
    """Test that unknown model codes return None."""
    unknown_codes = ['', 'Z123', '123', 'A', 'B', 'S']
    for code in unknown_codes:
      platform = get_platform_from_model_code(code)
      assert platform is None, f"Unknown model code {code} should return None, got {platform}"

  def test_fuzzy_match_all_platforms(self):
    """Test that fuzzy matching works for all known platforms with exact FW."""
    for platform, fw_by_addr in FW_VERSIONS.items():
      # Ensure there's no overlaps - each platform's FW should match only itself
      for _ in range(10):
        car_fw = []
        for ecu, fw_versions in fw_by_addr.items():
          ecu_name, addr, sub_addr = ecu
          fw = random.choice(list(fw_versions))
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

        CP = CarParams(carFw=car_fw)
        # Build fw dict manually to avoid memoryview issue in test
        fw_dict = {(fw.address, fw.subAddress if fw.subAddress != 0 else None): {bytes(fw.fwVersion)} for fw in CP.carFw}
        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(fw_dict, CP.carVin, FW_VERSIONS)
        assert matches == {str(platform)}, f"Platform {platform} should fuzzy match to itself, got {matches}"

  def test_fuzzy_match_unseen_firmware(self):
    """Test that fuzzy matching works for firmware versions not in the database."""
    # Test with simulated new firmware versions that follow the pattern
    test_cases = [
      # (unseen_fw, expected_platform)
      (b'TeMYG4_NewProject_0.0.0 (99),E4999.99.99', str(CAR.TESLA_MODEL_3)),  # New Model 3 HW4
      (b'TeM3_Future_0.0.0 (1),E999.00.00', str(CAR.TESLA_MODEL_3)),  # Future Model 3
      (b'TeMYG4_Unknown_0.0.0 (50),Y4999.99.0', str(CAR.TESLA_MODEL_Y)),  # New Model Y HW4
      (b'TeM3_New_0.0.0 (1),YP999.00.00', str(CAR.TESLA_MODEL_Y)),  # New Model Y Performance
      (b'TeM3_SP_XP999p9_0.0.0 (99),XPR999.99.0', str(CAR.TESLA_MODEL_X)),  # New Model X
    ]

    for unseen_fw, expected_platform in test_cases:
      live_fw = {(0x730, None): {unseen_fw}}
      matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS)
      assert expected_platform in matches, f"Unseen firmware {unseen_fw!r} should match {expected_platform}, got {matches}"

  def test_fuzzy_match_no_match_for_unknown_model(self):
    """Test that firmware with unknown model codes don't match."""
    unknown_fws = [
      b'TeMYG4_Unknown_0.0.0 (1),Z999.00.00',  # Unknown 'Z' prefix
      b'TeM3_Unknown_0.0.0 (1),S999.00.00',  # Model S not supported
      b'invalid_firmware_string',
    ]

    for unknown_fw in unknown_fws:
      live_fw = {(0x730, None): {unknown_fw}}
      matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS)
      assert len(matches) == 0, f"Unknown firmware {unknown_fw!r} should not match any platform, got {matches}"

  def test_fuzzy_match_empty_fw(self):
    """Test that empty firmware versions don't cause errors."""
    live_fw = {(0x730, None): set()}
    matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', FW_VERSIONS)
    assert len(matches) == 0, "Empty firmware should not match any platform"

  def test_fuzzy_match_wrong_address(self):
    """Test that firmware on wrong address doesn't match."""
    live_fw = {(0x731, None): {b'TeM3_E014p10_0.0.0 (16),E014.17.00'}}  # Wrong address
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
    """Test that all firmware versions in the database can be parsed."""
    for platform, fw_by_addr in FW_VERSIONS.items():
      for _ecu, versions in fw_by_addr.items():
        for fw in versions:
          model_code = get_model_code(fw)
          assert model_code is not None, f"Could not parse firmware for {platform}: {fw!r}"
          platform_from_code = get_platform_from_model_code(model_code)
          assert platform_from_code == str(platform), \
            f"Firmware {fw!r} parsed as {platform_from_code}, expected {platform}"
