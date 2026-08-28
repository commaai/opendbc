import unittest

from opendbc.car.structs import CarParams
from opendbc.car.fw_versions import build_fw_dict, match_fw_to_car_fuzzy
from opendbc.car.chrysler.fingerprints import FW_VERSIONS
from opendbc.car.chrysler.values import PLATFORM_CODE_ECUS, get_platform_codes, match_fw_to_car_fuzzy as chrysler_fuzzy

CarFw = CarParams.CarFw

# The only FW version in the database that isn't a Mopar part number. It's on the radar, which is
# in FUZZY_EXCLUDE_ECUS and not a PLATFORM_CODE_ECU, so it can't affect fuzzy fingerprinting.
NON_PART_NUMBER_FW = {b'22DTRHD_AA'}


class TestChryslerFingerprint(unittest.TestCase):
  def test_platform_codes_parse_every_platform_code_ecu(self):
    # Every FW version on an ECU we fuzzy match on must yield a part number, otherwise that
    # ECU silently stops contributing to the match
    for car_model, ecus in FW_VERSIONS.items():
      for ecu, fw_versions in ecus.items():
        if ecu[0] not in PLATFORM_CODE_ECUS:
          continue
        for fw in fw_versions:
          with self.subTest(car_model=car_model, ecu=ecu, fw=fw):
            self.assertEqual(len(get_platform_codes([fw])), 1, f"Failed to parse a part number from {fw}")

  def test_platform_code_drops_only_the_revision(self):
    # b'68227902AF' is part 68227902 at revision AF
    self.assertEqual(get_platform_codes([b'68227902AF']), {b'68227902'})
    # trailing whitespace is present on some engine ECU versions
    self.assertEqual(get_platform_codes([b'68267018AO ']), {b'68267018'})
    # revisions of the same part collapse to one code, different parts do not
    self.assertEqual(get_platform_codes([b'68227902AF', b'68227902AG']), {b'68227902'})
    self.assertEqual(get_platform_codes([b'68227902AF', b'68227905AG']), {b'68227902', b'68227905'})
    # not a part number
    self.assertEqual(get_platform_codes(list(NON_PART_NUMBER_FW)), set())

  def test_fuzzy_matches_unseen_revision(self):
    # The point of the brand fuzzy function: a car running a revision of a known part that we have
    # never seen must still fingerprint. The generic exact-string matcher can't do this.
    for car_model, ecus in FW_VERSIONS.items():
      fw = []
      for (ecu_name, addr, sub_addr), fw_versions in ecus.items():
        unseen = fw_versions[0].strip()[:-2] + b'ZZ'
        self.assertNotIn(unseen, [f.strip() for f in fw_versions])
        fw.append(CarFw(ecu=ecu_name, fwVersion=unseen, brand='chrysler',
                        address=addr, subAddress=0 if sub_addr is None else sub_addr))

      live_fw = build_fw_dict(fw)
      with self.subTest(car_model=car_model):
        self.assertEqual(match_fw_to_car_fuzzy(live_fw, 'chrysler', log=False), set(),
                         "generic matcher shouldn't match an unseen revision, test is vacuous")
        self.assertEqual(chrysler_fuzzy(live_fw, None, FW_VERSIONS), {car_model})

  def test_no_match_on_unknown_part_number(self):
    # A platform with genuinely new hardware must fail to fingerprint rather than match the
    # closest car we know about
    for car_model, ecus in FW_VERSIONS.items():
      fw = []
      for (ecu_name, addr, sub_addr), fw_versions in ecus.items():
        unknown = b'99' + fw_versions[0].strip()[2:]
        fw.append(CarFw(ecu=ecu_name, fwVersion=unknown, brand='chrysler',
                        address=addr, subAddress=0 if sub_addr is None else sub_addr))
      with self.subTest(car_model=car_model):
        self.assertEqual(chrysler_fuzzy(build_fw_dict(fw), None, FW_VERSIONS), set())

  def test_no_cross_platform_false_match(self):
    # Each platform's own FW must fuzzy match that platform and nothing else
    for car_model, ecus in FW_VERSIONS.items():
      fw = [CarFw(ecu=ecu[0], fwVersion=fw_versions[0], brand='chrysler', address=ecu[1],
                  subAddress=0 if ecu[2] is None else ecu[2])
            for ecu, fw_versions in ecus.items()]
      with self.subTest(car_model=car_model):
        self.assertEqual(chrysler_fuzzy(build_fw_dict(fw), None, FW_VERSIONS), {car_model})

  def test_shared_part_numbers_are_resolved_by_the_other_ecus(self):
    # Three part numbers are shared between platforms. A car carrying one must still fingerprint
    # correctly, because every platform code ECU has to agree.
    codes = {}
    for car_model, ecus in FW_VERSIONS.items():
      for ecu, fw_versions in ecus.items():
        if ecu[0] not in PLATFORM_CODE_ECUS:
          continue
        for code in get_platform_codes(fw_versions):
          codes.setdefault((ecu[1], ecu[2], code), set()).add(car_model)

    shared = {k: v for k, v in codes.items() if len(v) > 1}
    self.assertEqual(len(shared), 3, f"expected 3 shared part numbers, got {sorted(shared)}")
    for car_models in shared.values():
      for car_model in car_models:
        fw = [CarFw(ecu=ecu[0], fwVersion=fw_versions[0], brand='chrysler', address=ecu[1],
                    subAddress=0 if ecu[2] is None else ecu[2])
              for ecu, fw_versions in FW_VERSIONS[car_model].items()]
        with self.subTest(car_model=car_model):
          self.assertEqual(chrysler_fuzzy(build_fw_dict(fw), None, FW_VERSIONS), {car_model})


if __name__ == "__main__":
  unittest.main()
