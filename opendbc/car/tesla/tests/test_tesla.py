from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.radar_interface import RADAR_START_ADDR
from opendbc.car.tesla.values import CAR, FSD_14_FW, get_platform_codes, match_fw_to_car_fuzzy, _platforms_match

Ecu = CarParams.Ecu


class TestTeslaFingerprint:
  def test_fsd_14_fw(self):
    for car_model, ecus in FW_VERSIONS.items():
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        _, _, version = fw.partition(b',')
        is_fsd_14 = fw in FSD_14_FW.get(car_model, [])

        if car_model == CAR.TESLA_MODEL_3:
          # Model 3: FSD 14 FW has 'P' in the Highland suffix (E4HP vs E4H)
          assert is_fsd_14 == version.startswith(b'E4HP'), f"{fw}"
        elif car_model == CAR.TESLA_MODEL_Y:
          # Model Y: FSD 14 FW is Y4x version >= 003.04
          prefix, _, ver = version.partition(b'.')
          y4_003 = prefix.startswith(b'Y4') and prefix.endswith(b'003')  # Y4=HW4, 003=version series (002 is never FSD 14)
          high_version = y4_003 and int(ver.split(b'.')[0]) >= 4
          assert is_fsd_14 == high_version, f"{fw}"

  def test_radar_detection(self):
    # Test radar availability detection for cars with radar DBC defined
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_no_radar_car(self):
    # Model X doesn't have radar DBC defined, should always be unavailable
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_X, fingerprint, [], False, False, False)
      assert CP.radarUnavailable  # Always unavailable since no radar DBC

  def test_get_platform_codes(self):
    # Test parsing of Tesla FW versions
    fw_versions = [
      b'TeMYG4_Main_0.0.0 (77),E4H015.04.5',
      b'TeMYG4_Main_0.0.0 (78),Y4003.06.0',
      b'TeM3_SP_XP002p2_0.0.0 (23),XPR003.6.0',
    ]
    codes = get_platform_codes(fw_versions)
    assert codes == {b'E4H': b'015.04.5', b'Y4': b'003.06.0', b'XPR': b'003.6.0'}

  def test_platforms_match(self):
    # Test platform code matching logic
    # Same platform should match
    assert _platforms_match(b'E4', b'E4')
    assert _platforms_match(b'Y4', b'Y4')
    assert _platforms_match(b'XPR', b'XPR')
    
    # HW4 vs HW3 should match (same physical platform)
    assert _platforms_match(b'E4H', b'E4')
    assert _platforms_match(b'E4', b'E4H')
    assert _platforms_match(b'Y4H', b'Y4')
    
    # FSD 14 variants should match
    assert _platforms_match(b'E4HP', b'E4H')
    assert _platforms_match(b'E4HP', b'E4')
    
    # Different models should not match
    assert not _platforms_match(b'E4', b'Y4')
    assert not _platforms_match(b'E4', b'XPR')
    assert not _platforms_match(b'Y4', b'XPR')
    
    # Different generations should not match
    assert not _platforms_match(b'E014', b'E4')
    assert not _platforms_match(b'Y002', b'Y4')

  def test_fuzzy_fingerprint_model3(self):
    # Test fuzzy fingerprinting for Model 3 variants
    # Simulate live FW versions from a Model 3
    live_fw_versions = {
      (0x730, None): {b'TeMYG4_Main_0.0.0 (77),E4H015.04.5'}
    }
    
    # Should match Model 3
    matches = match_fw_to_car_fuzzy(live_fw_versions, '', FW_VERSIONS)
    assert 'TESLA_MODEL_3' in matches
    assert 'TESLA_MODEL_Y' not in matches
    assert 'TESLA_MODEL_X' not in matches

  def test_fuzzy_fingerprint_model_y(self):
    # Test fuzzy fingerprinting for Model Y variants
    live_fw_versions = {
      (0x730, None): {b'TeMYG4_Main_0.0.0 (77),Y4003.05.4'}
    }
    
    # Should match Model Y
    matches = match_fw_to_car_fuzzy(live_fw_versions, '', FW_VERSIONS)
    assert 'TESLA_MODEL_Y' in matches
    assert 'TESLA_MODEL_3' not in matches
    assert 'TESLA_MODEL_X' not in matches

  def test_fuzzy_fingerprint_model_x(self):
    # Test fuzzy fingerprinting for Model X
    live_fw_versions = {
      (0x730, None): {b'TeM3_SP_XP002p2_0.0.0 (36),XPR003.10.0'}
    }
    
    # Should match Model X
    matches = match_fw_to_car_fuzzy(live_fw_versions, '', FW_VERSIONS)
    assert 'TESLA_MODEL_X' in matches
    assert 'TESLA_MODEL_3' not in matches
    assert 'TESLA_MODEL_Y' not in matches

  def test_fuzzy_fingerprint_hw3_hw4_compatibility(self):
    # Test that HW3 and HW4 variants can fuzzy match
    # HW4 car with HW3-style firmware should still match
    live_fw_versions = {
      (0x730, None): {b'TeMYG4_Main_0.0.0 (77),E4015.04.5'}  # No H marker
    }
    
    # Should still match Model 3 (HW3/HW4 compatible)
    matches = match_fw_to_car_fuzzy(live_fw_versions, '', FW_VERSIONS)
    assert 'TESLA_MODEL_3' in matches

  def test_fuzzy_fingerprint_fsd14(self):
    # Test FSD 14 firmware matching
    live_fw_versions = {
      (0x730, None): {b'TeMYG4_Main_0.0.0 (77),E4HP015.04.5'}  # FSD 14
    }
    
    # Should match Model 3
    matches = match_fw_to_car_fuzzy(live_fw_versions, '', FW_VERSIONS)
    assert 'TESLA_MODEL_3' in matches
