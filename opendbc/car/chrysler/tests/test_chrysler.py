import pytest

from opendbc.car.chrysler.fingerprints import FW_VERSIONS
from opendbc.car.chrysler.values import PLATFORM_CODE_ECUS, get_platform_codes, match_fw_to_car_fuzzy
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu
CarFw = CarParams.CarFw


class TestGetPlatformCodes:
  def test_extracts_part_number(self):
    # Standard Chrysler FW format: 8 digit part number + 2 char revision
    codes = get_platform_codes([b'68227902AF'])
    assert codes == {b'68227902'}

  def test_handles_multiple_versions(self):
    codes = get_platform_codes([b'68227902AF', b'68227902AG', b'68227905AH'])
    assert codes == {b'68227902', b'68227905'}

  def test_strips_null_bytes(self):
    codes = get_platform_codes([b'68227902AF\x00\x00'])
    assert codes == {b'68227902'}

  def test_strips_whitespace(self):
    codes = get_platform_codes([b'68267018AO '])
    assert codes == {b'68267018'}

  def test_ignores_short_firmware(self):
    # FW too short to have part number + revision
    codes = get_platform_codes([b'AB'])
    assert codes == set()

  def test_empty_input(self):
    codes = get_platform_codes([])
    assert codes == set()


class TestPlatformCodeEcus:
  @pytest.mark.parametrize("car_model, ecus", FW_VERSIONS.items())
  def test_platform_code_ecus_available(self, car_model, ecus):
    """Each platform must have at least 2 ECUs with platform codes for reliable matching."""
    platform_ecus = [ecu for ecu in ecus if ecu[0] in PLATFORM_CODE_ECUS]
    assert len(platform_ecus) >= 2, f"{car_model}: needs at least 2 platform ECUs for fuzzy fingerprinting"


class TestMatchFwToCarFuzzy:
  def _mutate_revision(self, fw: bytes) -> bytes:
    """Change the revision suffix to simulate unseen firmware."""
    clean_fw = fw.rstrip(b'\x00').strip()
    return clean_fw[:-2] + b'ZZ'

  @pytest.mark.parametrize("car_model, ecus", FW_VERSIONS.items())
  def test_matches_with_mutated_revision(self, car_model, ecus):
    """Fuzzy matching should work when only the revision suffix changes."""
    fw = []
    for (ecu_name, addr, sub_addr), versions in ecus.items():
      if ecu_name not in PLATFORM_CODE_ECUS:
        continue

      mutated_fw = self._mutate_revision(versions[0])
      fw.append(CarFw(ecu=ecu_name, fwVersion=mutated_fw, brand='chrysler',
                      address=addr, subAddress=0 if sub_addr is None else sub_addr))

    CP = CarParams(carFw=fw)
    live_fw = build_fw_dict(CP.carFw, filter_brand='chrysler')
    matches = match_fw_to_car_fuzzy(live_fw, CP.carVin, FW_VERSIONS)

    assert car_model in matches, f"{car_model} should match with mutated revision"

  def test_no_match_with_wrong_part_number(self):
    """Should not match when part numbers are completely different."""
    # Use fake part numbers that don't exist in any platform
    fw = [
      CarFw(ecu=Ecu.combinationMeter, fwVersion=b'99999999AA', brand='chrysler', address=0x742, subAddress=0),
      CarFw(ecu=Ecu.abs, fwVersion=b'99999998AA', brand='chrysler', address=0x747, subAddress=0),
      CarFw(ecu=Ecu.fwdRadar, fwVersion=b'99999997AA', brand='chrysler', address=0x753, subAddress=0),
      CarFw(ecu=Ecu.eps, fwVersion=b'99999996AA', brand='chrysler', address=0x75a, subAddress=0),
    ]

    CP = CarParams(carFw=fw)
    live_fw = build_fw_dict(CP.carFw, filter_brand='chrysler')
    matches = match_fw_to_car_fuzzy(live_fw, CP.carVin, FW_VERSIONS)

    assert len(matches) == 0, "Should not match any platform with fake part numbers"

  def test_empty_live_fw(self):
    """Should return no matches when no live firmware is provided."""
    matches = match_fw_to_car_fuzzy({}, '', FW_VERSIONS)
    assert len(matches) == 0
