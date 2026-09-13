import itertools
import random
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.chrysler.values import CAR, FW_QUERY_CONFIG, FW_PATTERN, PLATFORM_CODE_ECUS, get_platform_codes
from opendbc.car.chrysler.fingerprints import FW_VERSIONS
from opendbc.testing import fuzzy_test, parameterized

Ecu = CarParams.Ecu

# One FW in the database isn't a Mopar part number. It's on the radar, which isn't a platform
# code ECU, so it never contributes a platform code and only has to parse without raising.
NON_PART_NUMBER_FW = {b'22DTRHD_AA'}

# How many platform code ECUs must disagree before one platform can look like another.
# match_fw_to_car_fuzzy tolerates one unseen ECU, so this has to stay above one.
MIN_DISTINGUISHING_ECUS = 2


def platform_codes_by_addr(car_model):
  return {ecu[1:]: get_platform_codes(fws) for ecu, fws in FW_VERSIONS[car_model].items()
          if ecu[0] in PLATFORM_CODE_ECUS}


def draw_fw(car_model, rng, replace=None, addrs=None):
  """Build the live FW dict for a car, optionally replacing the FW on some platform code ECUs."""
  live: dict[tuple[int, int | None], set[bytes]] = {}
  for (_ecu, addr, sub_addr), fws in FW_VERSIONS[car_model].items():
    fw = rng.choice(fws)
    if replace is not None and (addrs is None or (addr, sub_addr) in addrs):
      fw = replace(fw)
    live.setdefault((addr, sub_addr), set()).add(fw)
  return live


def unseen_revision(fw):
  """Same part number, a revision that isn't in the database. What a dealer flash looks like."""
  match = FW_PATTERN.match(fw)
  if match is None:  # not a part number, nothing to bump
    return fw
  return match.group('part_number') + b'ZZ' + (b' ' if fw.endswith(b' ') else b'')


def unseen_part_number(fw):
  """A part number that isn't in the database. What new hardware looks like."""
  match = FW_PATTERN.match(fw)
  if match is None:  # not a part number, contributes no platform code either way
    return fw
  return b'99999999' + match.group('revision') + (b' ' if fw.endswith(b' ') else b'')


class TestChryslerFW(unittest.TestCase):
  @parameterized("car_model, fw_versions", FW_VERSIONS.items())
  def test_fw_versions(self, car_model, fw_versions):
    for (ecu, _addr, _sub_addr), fws in fw_versions.items():
      for fw in fws:
        if fw in NON_PART_NUMBER_FW:
          assert ecu not in PLATFORM_CODE_ECUS, f"{fw!r} can't be parsed but is on a platform code ECU"
          assert get_platform_codes([fw]) == set()
          continue

        assert FW_PATTERN.match(fw) is not None, f"Unable to parse FW: {fw!r}"
        assert len(get_platform_codes([fw])) == 1, f"Unable to parse FW: {fw!r}"

  @fuzzy_test(max_examples=100)
  def test_platform_codes_fuzzy_fw(self, fuzzy):
    """Ensure function doesn't raise an exception"""
    get_platform_codes(fuzzy.list(fuzzy.binary))

  def test_platform_codes_spot_check(self):
    # Asserts basic platform code parsing behavior for a few cases
    results = get_platform_codes([
      b'68227902AF',   # combinationMeter, CHRYSLER_PACIFICA_2018
      b'68267018AO ',  # engine, trailing space
      b'68421036AC',   # eps, RAM_HD_5TH_GEN
      b'22DTRHD_AA',   # fwdRadar, RAM_1500_5TH_GEN, not a part number
      b'68227902AG',   # same part number as the first, newer revision
    ])
    assert results == {b'68227902', b'68267018', b'68421036'}

  def test_platform_codes_drop_only_the_revision(self):
    # The revision is the only thing dropped, so two different parts never collapse together
    for car_model, fw_versions in FW_VERSIONS.items():
      for ecu, fws in fw_versions.items():
        for fw in fws:
          if fw in NON_PART_NUMBER_FW:
            continue
          code = next(iter(get_platform_codes([fw])))
          assert len(code) == 8, f"{car_model} {ecu[0]}: {fw!r} -> {code!r}"
          assert fw.rstrip().startswith(code), f"{car_model} {ecu[0]}: {fw!r} -> {code!r}"

  def test_fuzzy_match(self):
    rng = random.Random(0)
    for platform, _fw_by_addr in FW_VERSIONS.items():
      # Ensure there's no overlaps in platform codes
      for _ in range(20):
        car_fw = []
        for ecu, fw_versions in FW_VERSIONS[platform].items():
          ecu_name, addr, sub_addr = ecu
          fw = rng.choice(fw_versions)
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

        CP = CarParams(carFw=car_fw)
        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
        assert matches == {platform}

  def test_fuzzy_match_unseen_revision(self):
    # A dealer flash bumps the revision on every ECU. The car is still the same car, and this is
    # the case the generic exact-string matcher can't handle.
    rng = random.Random(1)
    for platform in FW_VERSIONS:
      code_addrs = set(platform_codes_by_addr(platform))
      for _ in range(20):
        live = draw_fw(platform, rng, unseen_revision, code_addrs)
        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live, '', FW_VERSIONS)
        assert matches == {platform}, f"{platform}: {matches}"

  def test_fuzzy_match_one_unseen_part_number(self):
    # A new model year usually swaps one platform code ECU's part number. Tolerated, as long as
    # the platform still has two other platform code ECUs agreeing.
    rng = random.Random(2)
    for platform in FW_VERSIONS:
      code_addrs = sorted(platform_codes_by_addr(platform))
      for addr in code_addrs:
        live = draw_fw(platform, rng, unseen_part_number, {addr})
        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live, '', FW_VERSIONS)
        if len(code_addrs) - 1 >= MIN_DISTINGUISHING_ECUS:
          assert matches == {platform}, f"{platform} with unseen {addr}: {matches}"
        else:
          # Too few left to be sure which car it is. The Pacifica hybrids have no ABS entry, so
          # they only have two platform code ECUs and can't spare one.
          assert matches == set(), f"{platform} with unseen {addr} matched on one ECU: {matches}"

  def test_fuzzy_no_match_two_unseen_part_numbers(self):
    # Two unseen part numbers is new hardware, not a running change. Staying unmatched is the
    # safe outcome: openpilot asks the user rather than guessing at the nearest known car.
    rng = random.Random(3)
    for platform in FW_VERSIONS:
      code_addrs = sorted(platform_codes_by_addr(platform))
      for pair in itertools.combinations(code_addrs, 2):
        live = draw_fw(platform, rng, unseen_part_number, set(pair))
        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live, '', FW_VERSIONS)
        assert matches == set(), f"{platform} with two unseen part numbers matched {matches}"

  def test_fuzzy_no_match_unknown_car(self):
    # Every platform code ECU reporting an unknown part number matches nothing
    rng = random.Random(4)
    for platform in FW_VERSIONS:
      live = draw_fw(platform, rng, unseen_part_number)
      assert FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live, '', FW_VERSIONS) == set()

  def test_platforms_distinguishable(self):
    # match_fw_to_car_fuzzy tolerates one unseen platform code ECU, which is only safe while every
    # platform needs at least two of them replaced to look like another one. If a future
    # fingerprint submission breaks this, the tolerance has to be revisited.
    for candidate, car in itertools.permutations(FW_VERSIONS, 2):
      # how many of the candidate's platform code ECUs a car that is really `car` fails to satisfy
      car_codes = platform_codes_by_addr(car)
      unmet = sum(1 for addr, codes in platform_codes_by_addr(candidate).items()
                  if not (car_codes.get(addr, set()) & codes))
      assert unmet >= MIN_DISTINGUISHING_ECUS, \
        f"{car} is only {unmet} platform code ECU(s) away from {candidate}"

  def test_match_fw_fuzzy(self):
    # Synthetic database, so the tolerance itself is pinned rather than whatever the real database
    # happens to allow. Four platform code ECU addresses: hybrids carry ABS at an alternate address
    # (FW_QUERY_CONFIG.extra_ecus), so two ABS addresses on one platform is a shape that exists.
    offline_fw = {
      (Ecu.combinationMeter, 0x742, None): [b'68227902AF', b'68227902AG'],
      (Ecu.srs, 0x744, None): [b'68211617AF', b'68211617AG'],
      (Ecu.abs, 0x747, None): [b'68222747AG'],
      (Ecu.abs, 0x7e4, None): [b'68330876AA'],
      # not a platform code ECU, must not affect the result either way
      (Ecu.engine, 0x7e0, None): [b'68267018AO '],
    }
    expected_fingerprint = CAR.CHRYSLER_PACIFICA_2018
    offline = {expected_fingerprint: offline_fw}
    match = FW_QUERY_CONFIG.match_fw_to_car_fuzzy

    known = {
      (0x742, None): {b'68227902AF'},
      (0x744, None): {b'68211617AF'},
      (0x747, None): {b'68222747AG'},
      (0x7e4, None): {b'68330876AA'},
      (0x7e0, None): {b'68267018AO '},
    }
    assert match(known, '', offline) == {expected_fingerprint}

    # every ECU on an unseen revision of a known part still matches
    unseen_revs = {addr: {unseen_revision(fw) for fw in fws} for addr, fws in known.items()}
    assert match(unseen_revs, '', offline) == {expected_fingerprint}

    # one unseen part number is tolerated, three of four still agree
    one_unseen = dict(known) | {(0x742, None): {b'99999999AA'}}
    assert match(one_unseen, '', offline) == {expected_fingerprint}

    # two unseen part numbers is not, even though two others still agree
    two_unseen = dict(known) | {(0x742, None): {b'99999999AA'}, (0x744, None): {b'99999998AA'}}
    assert match(two_unseen, '', offline) == set()

    # a missing ECU counts the same as an unseen one
    assert match({k: v for k, v in known.items() if k != (0x742, None)}, '', offline) == {expected_fingerprint}
    assert match({k: v for k, v in known.items()
                  if k not in ((0x742, None), (0x744, None))}, '', offline) == set()

    # with only two platform code ECUs there is nothing to spare: one unseen means no match,
    # because a single agreeing ECU is never enough to name a car
    two_ecu_offline = {expected_fingerprint: {k: v for k, v in offline_fw.items()
                                              if k[0] != Ecu.abs}}
    assert match(known, '', two_ecu_offline) == {expected_fingerprint}
    assert match(one_unseen, '', two_ecu_offline) == set()

  def test_platform_code_ecus_are_present(self):
    # Every platform needs at least two platform code ECUs in the database, or it can never
    # fuzzy match at all
    for platform in FW_VERSIONS:
      found = sorted(str(ecu[0]) for ecu in FW_VERSIONS[platform] if ecu[0] in PLATFORM_CODE_ECUS)
      assert len(found) >= MIN_DISTINGUISHING_ECUS, f"{platform} only has {found}"
