import random
import unittest

from hypothesis import settings, given, strategies as st

from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.radar_interface import RADAR_START_ADDR
from opendbc.car.tesla.values import CAR, FW_QUERY_CONFIG, PLATFORM_CODE_ECUS, get_platform_codes
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.testing import parameterized

Ecu = CarParams.Ecu


class TestTeslaFingerprint:
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


class TestTeslaFW(unittest.TestCase):
  @parameterized("car_model, fw_versions", FW_VERSIONS.items())
  def test_fw_versions_parseable(self, car_model, fw_versions):
    """All FW versions in the database should have parseable platform codes."""
    for (ecu, addr, subaddr), fws in fw_versions.items():
      for fw in fws:
        codes = get_platform_codes([fw])
        assert len(codes) == 1, f"Unable to parse platform code from FW: {fw!r}"

  def test_platform_codes_spot_check(self):
    """Verify platform code extraction for known FW versions."""
    # Model 3 - HW3
    assert get_platform_codes([b'TeM3_E014p10_0.0.0 (16),E014.17.00']) == {(b'E', b'014')}
    # Model 3 - HW4
    assert get_platform_codes([b'TeMYG4_Main_0.0.0 (77),E4H015.04.5']) == {(b'E', b'015')}
    # Model Y - HW3
    assert get_platform_codes([b'TeM3_E014p10_0.0.0 (16),Y002.18.00']) == {(b'Y', b'002')}
    # Model Y - HW4
    assert get_platform_codes([b'TeMYG4_Main_0.0.0 (77),Y4003.05.4']) == {(b'Y', b'003')}
    # Model X
    assert get_platform_codes([b'TeM3_SP_XP002p2_0.0.0 (23),XPR003.6.0']) == {(b'X', b'003')}

  def test_platform_codes_multiple(self):
    """Multiple FW versions should return multiple platform codes."""
    codes = get_platform_codes([
      b'TeM3_E014p10_0.0.0 (16),E014.17.00',
      b'TeMYG4_Legacy3Y_0.0.0 (2),E4015.02.0',
    ])
    assert codes == {(b'E', b'014'), (b'E', b'015')}

  def test_platform_codes_invalid(self):
    """Invalid FW should return empty set."""
    assert get_platform_codes([b'invalid']) == set()
    assert get_platform_codes([b'']) == set()
    assert get_platform_codes([]) == set()

  @settings(max_examples=100)
  @given(data=st.data())
  def test_platform_codes_fuzzy_fw(self, data):
    """Ensure get_platform_codes doesn't raise exceptions on random input."""
    fw_strategy = st.lists(st.binary())
    fws = data.draw(fw_strategy)
    get_platform_codes(fws)

  def test_fuzzy_match(self):
    """Each car should fuzzy-match to exactly itself using random FW from its database."""
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
        assert matches == {platform}, f"Expected {platform}, got {matches}"

  def test_fuzzy_match_unseen_fw(self):
    """Fuzzy matching should work for unseen FW versions with correct model identifier."""
    offline_fw = {
      (Ecu.eps, 0x730, None): [
        b'TeMYG4_Main_0.0.0 (77),Y4003.05.4',
      ],
    }
    expected = CAR.TESLA_MODEL_Y

    # New FW version for Model Y with different version numbers should still match
    live_fw = {
      (0x730, None): {b'TeMYG4_Main_0.0.0 (99),Y4099.99.9'},
    }
    candidates = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', {expected: offline_fw})
    assert candidates == {expected}

    # New variant letter should still match (model Y)
    live_fw = {
      (0x730, None): {b'TeMYG4_Main_0.0.0 (99),Y4Z003.01.0'},
    }
    candidates = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', {expected: offline_fw})
    assert candidates == {expected}

  def test_fuzzy_no_cross_match(self):
    """Model 3 FW should not match Model Y and vice versa."""
    model3_fw = {
      (Ecu.eps, 0x730, None): [
        b'TeMYG4_Main_0.0.0 (77),E4H015.04.5',
      ],
    }
    modely_fw = {
      (Ecu.eps, 0x730, None): [
        b'TeMYG4_Main_0.0.0 (77),Y4003.05.4',
      ],
    }
    offline = {
      CAR.TESLA_MODEL_3: model3_fw,
      CAR.TESLA_MODEL_Y: modely_fw,
    }

    # Model 3 live FW should only match Model 3
    live_fw = {(0x730, None): {b'TeMYG4_Main_0.0.0 (99),E4H099.99.9'}}
    candidates = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', offline)
    assert candidates == {CAR.TESLA_MODEL_3}

    # Model Y live FW should only match Model Y
    live_fw = {(0x730, None): {b'TeMYG4_Main_0.0.0 (99),Y4099.99.9'}}
    candidates = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', offline)
    assert candidates == {CAR.TESLA_MODEL_Y}

    # Model X live FW should match neither
    live_fw = {(0x730, None): {b'TeM3_SP_XP002p2_0.0.0 (99),XPR099.99.9'}}
    candidates = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', offline)
    assert len(candidates) == 0

  def test_fuzzy_no_match_invalid(self):
    """Invalid FW should not match any car."""
    offline_fw = {
      CAR.TESLA_MODEL_3: {
        (Ecu.eps, 0x730, None): [b'TeMYG4_Main_0.0.0 (77),E4H015.04.5'],
      },
    }
    live_fw = {(0x730, None): {b'invalid_fw_string'}}
    candidates = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', offline_fw)
    assert len(candidates) == 0

  def test_platform_code_ecus(self):
    """All PLATFORM_CODE_ECUS should have FW versions for all cars."""
    for platform, fw_by_addr in FW_VERSIONS.items():
      for ecu in PLATFORM_CODE_ECUS:
        found = any(e[0] == ecu for e in fw_by_addr)
        assert found, f"{platform} missing FW for {ecu}"
