import itertools
import random
import unittest
from unittest.mock import patch

from opendbc.car.chrysler.fingerprints import FW_VERSIONS
from opendbc.car.chrysler.values import CAR, FW_QUERY_CONFIG, OPTIONAL_CODE_ECUS, PLATFORM_CODE_ECUS, get_platform_codes, match_fw_to_car_fuzzy
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.structs import CarParams
from opendbc.testing import fuzzy_test

Ecu = CarParams.Ecu


def unseen_revision(fw):
  codes = get_platform_codes([fw])
  return next(iter(codes)) + b'ZZ' if codes else fw


class TestChryslerFW(unittest.TestCase):
  def test_platform_codes(self):
    self.assertEqual(get_platform_codes([b'68227902AF', b'68227902AG', b'68267018AO ']), {b'68227902', b'68267018'})
    for fw in (b'', b'68227902A', b'68227902ABC', b'68227902AF\n', b'68227902AF\x00',
               b' 68227902AF', b'68227902AF  ', b'68227902af', b'22DTRHD_AA'):
      with self.subTest(fw=fw):
        self.assertEqual(get_platform_codes([fw]), set())

  @fuzzy_test(max_examples=100)
  def test_platform_codes_arbitrary_bytes(self, fuzzy):
    get_platform_codes(fuzzy.list(fuzzy.binary))

  def test_all_database_versions(self):
    unparsed = set()
    for fws in FW_VERSIONS.values():
      for ecu, versions in fws.items():
        if ecu[0] not in PLATFORM_CODE_ECUS:
          continue
        for fw in versions:
          if not get_platform_codes([fw]):
            unparsed.add((ecu[0], fw))
    self.assertEqual(unparsed, {(Ecu.fwdRadar, b'22DTRHD_AA')})

  def test_known_and_unseen_revisions(self):
    rng = random.Random(0)
    for platform, fws in FW_VERSIONS.items():
      for revise in (False, True):
        for _ in range(100):
          live = {ecu[1:]: {rng.choice(versions)} for ecu, versions in fws.items()}
          if revise:
            live = {addr: {unseen_revision(fw) for fw in versions} for addr, versions in live.items()}
          with self.subTest(platform=platform, revise=revise):
            self.assertEqual(match_fw_to_car_fuzzy(live, '', FW_VERSIONS), {platform})

  def test_each_unseen_revision(self):
    # Exercise every checked-in version, including the non-part-number radar
    # response, without relying on random draws to select rare firmware.
    for platform, fws in FW_VERSIONS.items():
      baseline = {ecu[1:]: {unseen_revision(versions[0])} for ecu, versions in fws.items()}
      for ecu, versions in fws.items():
        for fw in versions:
          with self.subTest(platform=platform, ecu=ecu, fw=fw):
            live = baseline | {ecu[1:]: {unseen_revision(fw)}}
            self.assertEqual(match_fw_to_car_fuzzy(live, '', FW_VERSIONS), {platform})

  def test_reject_changed_or_missing_required_ecu(self):
    for platform, fws in FW_VERSIONS.items():
      baseline = {ecu[1:]: {unseen_revision(versions[0])} for ecu, versions in fws.items()}
      for ecu in fws:
        if ecu[0] not in PLATFORM_CODE_ECUS or ecu[0] in OPTIONAL_CODE_ECUS:
          continue
        for replacements in (set(), {b'99999999AA'}, {b'68227902AF\n'}):
          with self.subTest(platform=platform, ecu=ecu, replacements=replacements):
            self.assertEqual(match_fw_to_car_fuzzy(baseline | {ecu[1:]: replacements}, '', FW_VERSIONS), set())

  def test_one_unseen_cluster(self):
    for platform, fws in FW_VERSIONS.items():
      baseline = {ecu[1:]: {unseen_revision(versions[0])} for ecu, versions in fws.items()}
      body_addrs = {ecu[1:] for ecu in fws if ecu[0] in OPTIONAL_CODE_ECUS}
      for addr in body_addrs:
        for versions in (set(), {b'99999999AA'}):
          with self.subTest(platform=platform, addr=addr, versions=versions):
            self.assertEqual(match_fw_to_car_fuzzy(baseline | {addr: versions}, '', FW_VERSIONS), {platform})
        self.assertEqual(match_fw_to_car_fuzzy(baseline | {addr: {b'corrupt'}}, '', FW_VERSIONS), set())
      srs_addr = next(ecu[1:] for ecu in fws if ecu[0] == Ecu.srs)
      unknown = {addr: {b'99999999AA'} for addr in body_addrs | {srs_addr}}
      self.assertEqual(match_fw_to_car_fuzzy(baseline | unknown, '', FW_VERSIONS), set())

  def test_recorded_ram_hd_cluster_parts(self):
    # Public commaCarSegments devices 64dbf4f1e713bfa8 and 77d9432d15a85e17:
    # cluster parts are absent from the database, while the control ECUs match.
    fws = FW_VERSIONS[CAR.RAM_HD_5TH_GEN]
    live = {ecu[1:]: {unseen_revision(versions[0])} for ecu, versions in fws.items()}
    for cluster_fw in (b'68525438AB', b'68620923AB'):
      self.assertEqual(match_fw_to_car_fuzzy(live | {(0x742, None): {cluster_fw}}, '', FW_VERSIONS), {CAR.RAM_HD_5TH_GEN})

  def test_known_conflicting_cluster(self):
    for platform, other in itertools.permutations(FW_VERSIONS, 2):
      fws = FW_VERSIONS[platform]
      live = {ecu[1:]: {unseen_revision(versions[0])} for ecu, versions in fws.items()}
      for ecu, versions in fws.items():
        if ecu[0] not in OPTIONAL_CODE_ECUS or ecu not in FW_VERSIONS[other]:
          continue
        foreign = get_platform_codes(FW_VERSIONS[other][ecu]) - get_platform_codes(versions)
        for code in foreign:
          with self.subTest(platform=platform, other=other, ecu=ecu):
            self.assertEqual(match_fw_to_car_fuzzy(live | {ecu[1:]: {code + b'ZZ'}}, '', FW_VERSIONS), set())

  def test_non_platform_ecus_do_not_prevent_matching(self):
    for platform, fws in FW_VERSIONS.items():
      live = {ecu[1:]: {unseen_revision(versions[0])} for ecu, versions in fws.items() if ecu[0] in PLATFORM_CODE_ECUS}
      self.assertEqual(match_fw_to_car_fuzzy(live, '', FW_VERSIONS), {platform})
      for ecu in fws:
        if ecu[0] not in PLATFORM_CODE_ECUS:
          live[ecu[1:]] = {b'99999999AA'}
      self.assertEqual(match_fw_to_car_fuzzy(live, '', FW_VERSIONS), {platform})

  def test_shared_modules_cannot_identify_sibling(self):
    for platform, other in itertools.permutations(FW_VERSIONS, 2):
      other_fws = FW_VERSIONS[other]
      live = {ecu[1:]: {unseen_revision(fw) for fw in versions} for ecu, versions in other_fws.items()}
      self.assertEqual(match_fw_to_car_fuzzy(live, '', {platform: FW_VERSIONS[platform]}), set())

  def test_empty_or_incomplete_database(self):
    eps = (Ecu.eps, 0x730, None)
    meter = (Ecu.combinationMeter, 0x742, None)
    live = {eps[1:]: {b'68421036ZZ'}, meter[1:]: {b'68227902ZZ'}}
    for offline in ({}, {'car': {}}, {'car': {eps: [b'68421036AC']}}, {'car': {meter: [b'68227902AF']}}):
      self.assertEqual(match_fw_to_car_fuzzy(live, '', offline), set())
    complete = {'car': {eps: [b'68421036AC'], meter: [b'68227902AF']}}
    self.assertEqual(match_fw_to_car_fuzzy(live, '', complete), {'car'})
    self.assertEqual(match_fw_to_car_fuzzy({}, '', complete), set())
    wrong_subaddress = {(address, 1): versions for (address, _), versions in live.items()}
    self.assertEqual(match_fw_to_car_fuzzy(wrong_subaddress, '', complete), set())

  def test_ambiguous_platforms_are_not_guessed(self):
    fws = next(iter(FW_VERSIONS.values()))
    live = {ecu[1:]: {unseen_revision(versions[0])} for ecu, versions in fws.items()}
    self.assertEqual(match_fw_to_car_fuzzy(live, '', {'first': fws, 'second': fws}), {'first', 'second'})

  def test_public_matcher_uses_chrysler_fallback(self):
    self.assertIs(FW_QUERY_CONFIG.match_fw_to_car_fuzzy, match_fw_to_car_fuzzy)
    for platform, fws in FW_VERSIONS.items():
      car_fw = [CarParams.CarFw(ecu=ecu, address=address, subAddress=subaddress or 0, brand='chrysler',
                               fwVersion=unseen_revision(versions[0])) for (ecu, address, subaddress), versions in fws.items()]
      with self.subTest(platform=platform):
        exact, matches = match_fw_to_car(car_fw, '', log=False)
        self.assertFalse(exact)
        self.assertEqual(matches, {platform})

  def test_generic_matching_cannot_bypass_control_ecus(self):
    # Keep the other firmware exact: revising every ECU would prevent the generic
    # matcher from finding the body/powertrain matches which trigger this bug.
    for platform, fws in FW_VERSIONS.items():
      for changed_ecu in {ecu[0] for ecu in fws if ecu[0] in (Ecu.eps, Ecu.abs, Ecu.fwdRadar)}:
        for missing in (False, True):
          car_fw = [CarParams.CarFw(ecu=ecu, address=address, subAddress=subaddress or 0, brand='chrysler',
                                   fwVersion=b'99999999AA' if ecu == changed_ecu else versions[0])
                    for (ecu, address, subaddress), versions in fws.items() if not (missing and ecu == changed_ecu)]
          with self.subTest(platform=platform, ecu=changed_ecu, missing=missing):
            _, matches = match_fw_to_car(car_fw, '', log=False)
            self.assertEqual(matches, set())
            # Demonstrate a reachable generic-matcher bypass; opt-out is needed
            # even when a brand hook verifies the control modules.
            with patch.object(FW_QUERY_CONFIG, 'use_generic_fuzzy', True):
              _, matches = match_fw_to_car(car_fw, '', log=False)
              self.assertEqual(matches, {platform})
