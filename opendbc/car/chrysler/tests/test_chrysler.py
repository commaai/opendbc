import unittest

from opendbc.car.chrysler.fingerprints import FW_VERSIONS
from opendbc.car.chrysler.values import CAR, PLATFORM_CODE_ECUS, get_platform_codes, match_fw_to_car_fuzzy
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu
CarFw = CarParams.CarFw


class TestGetPlatformCodes(unittest.TestCase):
  def test_extracts_part_number(self):
    # Standard Chrysler FW format: 8 digit part number + 2 char revision
    codes = get_platform_codes([b'68227902AF'])
    assert codes == {b'682279'}

  def test_handles_multiple_versions(self):
    codes = get_platform_codes([b'68227902AF', b'68227902AG', b'68227905AH'])
    assert codes == {b'682279'}

  def test_strips_null_bytes(self):
    codes = get_platform_codes([b'68227902AF\x00\x00'])
    assert codes == {b'682279'}

  def test_strips_whitespace(self):
    codes = get_platform_codes([b'68267018AO '])
    assert codes == {b'682670'}

  def test_ignores_short_firmware(self):
    # FW too short to have a platform code once the revision is dropped
    codes = get_platform_codes([b'AB'])
    assert codes == set()

  def test_empty_input(self):
    codes = get_platform_codes([])
    assert codes == set()


class TestPlatformCodeEcus(unittest.TestCase):
  def test_platform_code_ecus_available(self):
    """Each platform must have at least 2 ECUs with platform codes for reliable matching."""
    for car_model, ecus in FW_VERSIONS.items():
      platform_ecus = [ecu for ecu in ecus if ecu[0] in PLATFORM_CODE_ECUS]
      assert len(platform_ecus) >= 2, f"{car_model}: needs at least 2 platform ECUs for fuzzy fingerprinting"


class TestMatchFwToCarFuzzy(unittest.TestCase):
  def _mutate_revision(self, fw: bytes) -> bytes:
    """Simulate an unseen firmware revision (last 4 chars are the variable part)."""
    clean_fw = fw.rstrip(b'\x00').strip()
    return clean_fw[:-4] + b'ZZZZ'

  def test_matches_with_mutated_revision(self):
    """Fuzzy matching should work when only the revision suffix changes."""
    for car_model, ecus in FW_VERSIONS.items():
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

  # Real-world regression cases: actual firmware strings comma's fingerprint-collection bot
  # observed in the field that were NOT yet in the offline database at the time. These were
  # mined from opendbc's git history (Aug 2024 snapshot vs current master) to validate the
  # matcher against genuine, organically-seen updates rather than synthetic mutations alone.
  # `novel=True` ECUs are held out of the offline DB below to simulate the revision being
  # truly unseen; `novel=False` ECUs reflect that real cars don't update every ECU at once,
  # so those are left as known exact-match firmware. See PR description for methodology.
  REAL_WORLD_UNSEEN_FW_CASES = [
    (CAR.CHRYSLER_PACIFICA_2020, {
      (Ecu.combinationMeter, 1858, None): (b'68405327AC', False),
      (Ecu.abs, 1863, None): (b'68397394AA', False),
      (Ecu.fwdRadar, 1875, None): (b'68540436AB', True),
      (Ecu.eps, 1882, None): (b'68416742AA', False),
    }),
    (CAR.CHRYSLER_PACIFICA_2019_HYBRID, {
      (Ecu.combinationMeter, 1858, None): (b'68594991AB', True),
      (Ecu.fwdRadar, 1875, None): (b'04672758AB', False),
      (Ecu.eps, 1882, None): (b'68594341AC', True),
    }),
    (CAR.JEEP_GRAND_CHEROKEE, {
      (Ecu.combinationMeter, 1858, None): (b'68302214AC', True),
      (Ecu.abs, 1863, None): (b'68252642AG', False),
      (Ecu.fwdRadar, 1875, None): (b'04672627AB', False),
      (Ecu.eps, 1882, None): (b'68321650AC', True),
    }),
    (CAR.JEEP_GRAND_CHEROKEE_2019, {
      (Ecu.combinationMeter, 1858, None): (b'68402736AB', True),
      (Ecu.abs, 1863, None): (b'68408639AC', False),
      (Ecu.fwdRadar, 1875, None): (b'04672788AA', False),
      (Ecu.eps, 1882, None): (b'68501186AA', True),
    }),
  ]

  def test_matches_real_world_unseen_firmware(self):
    # Hold out only the genuinely novel ECU revisions, so this tests fuzzy generalization
    # on the ECUs that actually updated, combined with exact matches on the ones that didn't -
    # mirroring how a real car's firmware profile evolves piecemeal, not all at once.
    for car_model, live_versions in self.REAL_WORLD_UNSEEN_FW_CASES:
      offline_fw_versions = {platform: dict(ecus) for platform, ecus in FW_VERSIONS.items()}
      for (ecu, addr, sub), (version, novel) in live_versions.items():
        if not novel:
          continue
        key = (ecu, addr, sub)
        versions = [v for v in offline_fw_versions[car_model][key] if v != version]
        assert versions, f"held-out test fixture invalid: {key} would have no other known versions"
        offline_fw_versions[car_model][key] = versions

      fw = [CarFw(ecu=ecu, fwVersion=version, brand='chrysler', address=addr, subAddress=0 if sub is None else sub)
            for (ecu, addr, sub), (version, novel) in live_versions.items()]

      CP = CarParams(carFw=fw)
      live_fw = build_fw_dict(CP.carFw, filter_brand='chrysler')
      matches = match_fw_to_car_fuzzy(live_fw, CP.carVin, offline_fw_versions)

      assert matches == {car_model}, \
        f"expected unique match to {car_model} on unseen firmware, got {matches}"


if __name__ == "__main__":
  unittest.main()
