import itertools
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.chrysler.values import (FW_QUERY_CONFIG, FW_PATTERN, MAX_UNSEEN_PLATFORM_CODE_ECUS,
                                          MIN_PLATFORM_CODE_MATCHES, PLATFORM_CODE_ECUS, get_platform_codes)
from opendbc.car.chrysler.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu


def live_fw(platform, ecus=None):
  """The FW a car of this platform reports, optionally only from some of its ECUs."""
  live: dict[tuple[int, int | None], set[bytes]] = {}
  for ecu, fws in FW_VERSIONS[platform].items():
    if ecu[0] not in PLATFORM_CODE_ECUS or (ecus is not None and ecu[0] not in ecus):
      continue
    live.setdefault(ecu[1:], set()).update(fws)
  return live


class TestChryslerFingerprint(unittest.TestCase):
  def test_platform_code_ecus_are_present(self):
    """Every platform must carry enough platform code ECUs to be matched."""
    for platform, fws in FW_VERSIONS.items():
      present = {ecu[0] for ecu in fws if ecu[0] in PLATFORM_CODE_ECUS}
      self.assertGreater(len(present), 1, f"{platform} has too few platform code ECUs: {present}")

  def test_fw_parses(self):
    """Every FW on a platform code ECU has to yield a part number."""
    for platform, fws in FW_VERSIONS.items():
      for ecu, versions in fws.items():
        if ecu[0] not in PLATFORM_CODE_ECUS:
          continue
        for fw in versions:
          self.assertIsNotNone(FW_PATTERN.match(fw.strip()), f"{platform} {ecu}: can't parse {fw!r}")

  def test_no_new_shared_platform_codes(self):
    """Grand Cherokee and Grand Cherokee 2019 share one srs part number. That is safe, since a
    shared code only ever makes a match ambiguous and ambiguous matches are rejected. A newly
    shared code would silently shrink what can be identified, so pin the known set."""
    known_shared = {(Ecu.srs, b"68355363")}

    owners: dict[tuple, set] = {}
    for platform, fws in FW_VERSIONS.items():
      for ecu, versions in fws.items():
        if ecu[0] not in PLATFORM_CODE_ECUS:
          continue
        for code in get_platform_codes(versions):
          owners.setdefault((ecu[0], code), set()).add(platform)

    shared = {k for k, v in owners.items() if len(v) > 1}
    self.assertEqual(shared, known_shared)

  def test_exact_match(self):
    """A car reporting its own FW matches itself and nothing else."""
    for platform in FW_VERSIONS:
      matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw(platform), "", FW_VERSIONS)
      self.assertEqual(matches, {platform})

  def test_unseen_revision(self):
    """An unseen software revision of a known part number still matches, since only the part matches."""
    for platform in FW_VERSIONS:
      live = {addr: {code + b"QQ" for code in get_platform_codes(fws)}
              for addr, fws in live_fw(platform).items()}
      matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live, "", FW_VERSIONS)
      self.assertEqual(matches, {platform}, f"{platform} lost on an unseen revision")

  def test_unseen_part_number_on_one_ecu(self):
    """A new trim can carry an unseen part number on one ECU. The rest still identify the car."""
    for platform in FW_VERSIONS:
      for spoiled in {ecu[0] for ecu in FW_VERSIONS[platform] if ecu[0] in PLATFORM_CODE_ECUS}:
        live = live_fw(platform)
        for ecu in FW_VERSIONS[platform]:
          if ecu[0] == spoiled and ecu[1:] in live:
            live[ecu[1:]] = {b"ZZZZZZZZAA"}
        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live, "", FW_VERSIONS)
        self.assertEqual(matches, {platform}, f"{platform} lost when {spoiled} was unseen")

  def test_tolerance_is_needed_and_bounded(self):
    """MAX_UNSEEN_PLATFORM_CODE_ECUS has to be exactly one: zero matches nothing when a part
    number changes, and more than one lets a car match on too little."""
    unseen_needed = 0
    for platform in FW_VERSIONS:
      for spoiled in {ecu[0] for ecu in FW_VERSIONS[platform] if ecu[0] in PLATFORM_CODE_ECUS}:
        live = live_fw(platform)
        for ecu in FW_VERSIONS[platform]:
          if ecu[0] == spoiled and ecu[1:] in live:
            live[ecu[1:]] = {b"ZZZZZZZZAA"}
        matched = {addr for addr, fws in live.items() if fws != {b"ZZZZZZZZAA"}}
        # with no tolerance this car cannot match, which is what the tolerance exists to fix
        if len(matched) >= MIN_PLATFORM_CODE_MATCHES:
          unseen_needed += 1
        self.assertEqual(FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live, "", FW_VERSIONS), {platform})

    self.assertGreater(unseen_needed, 0, "no car exercises the unseen tolerance")
    self.assertEqual(MAX_UNSEEN_PLATFORM_CODE_ECUS, 1)
    self.assertEqual(MIN_PLATFORM_CODE_MATCHES, 2)

  def test_no_wrong_match_on_partial_query(self):
    """When only some ECUs respond the match may be ambiguous, but it must never be another car."""
    for platform in FW_VERSIONS:
      present = sorted({ecu[0] for ecu in FW_VERSIONS[platform] if ecu[0] in PLATFORM_CODE_ECUS})
      for ecus in itertools.combinations(present, 2):
        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw(platform, set(ecus)), "", FW_VERSIONS)
        self.assertFalse(matches - {platform}, f"{platform} also matched {matches - {platform}} from {ecus}")


if __name__ == "__main__":
  unittest.main()
