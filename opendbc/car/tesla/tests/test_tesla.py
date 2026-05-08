import re
import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.radar_interface import RADAR_START_ADDR
from opendbc.car.tesla.values import CAR, FSD_14_FW

Ecu = CarParams.Ecu

# Fields prefixed unknown_* we observe structurally but don't know the meaning of.
# Only `platform` has evidence-backed semantic meaning (matches car_model in FW_VERSIONS).
#
# unknown_prefix is everything before the comma; we don't split it because we don't know what its
# parts mean, but observed shape is: <family>_<package>_<triplet> (<build>), e.g.
#   TeMYG4 _ Main     _ 0.0.0 (78)     or     TeM3 _ SP_XP002p2 _ 0.0.0 (23)
#   family   package    triplet build           family  package    triplet build
#
# After the comma, the version string decomposes into:
#   platform             : E/Y/X = car model (Model 3 / Y / X). The only field with known meaning.
#   variant_code         : differentiator WITHIN a platform — hardware/trim/calibration bits packed
#                          into <digit?><letters?><3-digit series>, e.g. '4HP015', '4003', 'L014',
#                          'PR003'. We don't fully know what the parts mean individually, but the
#                          whole string identifies a specific variant within the car model.
#   software_major/minor : numeric components after the first '.' — conventional release numbers.
#                          minor is optional (e.g. 'E4S014.27' has no minor).
#
# Suspected (not confirmed): for M3/MY, `TeM3_*` outer + no-leading-digit variant_code == HW3, and
# `TeMYG4_*` outer + leading-'4' variant_code == HW4 (the 'G4' in TeMYG4 likely denotes Gen 4).
#
# Example full parse of 'TeMYG4_Main_0.0.0 (78),E4HP015.05.0':
#   unknown_prefix='TeMYG4_Main_0.0.0 (78)'
#   platform=E  variant_code=4HP015  software_major=05  software_minor=0
FW_RE = re.compile(
  rb'^(?P<unknown_prefix>.+),' +
  rb'(?P<platform>[EYX])' +
  rb'(?P<variant_code>\d?[A-Z]*\d{3})' +
  rb'\.(?P<software_major>\d+)' +
  rb'(?:\.(?P<software_minor>\d+))?$'
)

PLATFORM_TO_CAR = {
  b'E': CAR.TESLA_MODEL_3,
  b'Y': CAR.TESLA_MODEL_Y,
  b'X': CAR.TESLA_MODEL_X,
}

# Hypothesized FSD 14 profile, in terms of variant_code bookends (given software_major >= 4):
#   M3: variant_code starts with '4H',  ends with '015'
#   MY: variant_code starts with '4',   ends with '003'
# Older series (M3 '014', MY '002') are never FSD 14.
# Tuple format: (variant_code_regex, software_major, software_major greater or equal)
FSD_14_FW_RULES = {
  CAR.TESLA_MODEL_3: [
    (b'^4H.*015$', 4, 1),
  ],
  CAR.TESLA_MODEL_Y: [
    (b'^4.*003$', 4, 1),
  ],
}

# Non FSD 14 HW4 and HW3 cars with recent FW (2026.8.6+) in which Tesla uses
# "Self-Driving" as name for its ADAS.
# Tuple format: (variant_code_regex, software_major, software_major greater or equal)
NON_FSD_14_SELFDRIVE_FW_RULES = {
  CAR.TESLA_MODEL_3: [
    (b'^(L|S)?014$', 20, 1),
  ],
  CAR.TESLA_MODEL_Y: [
    (b'^(P|S)?002$', 21, 1),
  ],
}

# Non FSD 14 HW4 and HW3 cars with older FW (2026.8.3-) in which Tesla uses
# "Autopilot" as name for its ADAS.
# Tuple format: (variant_code_regex, software_major, software_major greater or equal)
NON_FSD_14_AUTOPILOT_FW_RULES = {
  CAR.TESLA_MODEL_3: [
    (b'^(L|S)?014$', 19, 0),
    (b'^4.*014$', 0, 1),
    (b'^4.*015$', 3, 0),
  ],
  CAR.TESLA_MODEL_Y: [
    (b'^(P|S)?002$', 20, 0),
    (b'^4.*002$', 0, 1),
    (b'^4.*003$', 3, 0),
  ],
}

def compile_rules_dict(rules_dict):
  compiled = {}
  for car_model, rules in rules_dict.items():
    compiled[car_model] = []
    for rule in rules:
      regex_str = rule[0]
      compiled_rule = (re.compile(regex_str), rule[1], rule[2])
      compiled[car_model].append(compiled_rule)
  return compiled

def match_rules(rules, fw) -> bool:
  decoded_fw = FW_RE.match(fw)
  assert decoded_fw is not None, f"Unparsable FW: {fw}"
  any_rule_matches = False
  for rule in rules:
    variant_regex, software_major, software_major_geq = rule
    any_rule_matches |= (
      variant_regex.match(decoded_fw['variant_code']) is not None
      and ((software_major_geq and int(decoded_fw['software_major']) >= software_major)
      or (not software_major_geq and int(decoded_fw['software_major']) <= software_major))
    )
  return any_rule_matches

FSD_14_FW_RULES = compile_rules_dict(FSD_14_FW_RULES)
NON_FSD_14_SELFDRIVE_FW_RULES = compile_rules_dict(NON_FSD_14_SELFDRIVE_FW_RULES)
NON_FSD_14_AUTOPILOT_FW_RULES = compile_rules_dict(NON_FSD_14_AUTOPILOT_FW_RULES)


class TestTeslaFingerprint(unittest.TestCase):
  def test_fw_platform_code(self):
    # Every EPS FW must parse and its platform letter must match the car it's filed under.
    for car_model, ecus in FW_VERSIONS.items():
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        m = FW_RE.match(fw)

        assert m is not None, f"Unparsable FW: {fw}"
        assert PLATFORM_TO_CAR[m['platform']] == car_model, f"Platform letter {m['platform']!r} != {car_model.value}: {fw}"

  def test_fsd_14_fw(self):
    for car_model, ecus in FW_VERSIONS.items():
      if car_model not in FSD_14_FW_RULES:
        continue

      for rule in FSD_14_FW_RULES[car_model]:
        variant_regex, software_major, software_major_geq = rule
        for fw in ecus.get((Ecu.eps, 0x730, None), []):
          is_fsd_14 = fw in FSD_14_FW.get(car_model, [])
          expected = match_rules(FSD_14_FW_RULES[car_model], fw)
          assert is_fsd_14 == expected, f"{fw}"

  def test_rules_uniqueness(self):
    # check if rules lead to exactly one match
    for car_model, ecus in FW_VERSIONS.items():
      if car_model not in FSD_14_FW_RULES:
        continue

      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        matches = [
          match_rules(FSD_14_FW_RULES[car_model], fw),
          match_rules(NON_FSD_14_SELFDRIVE_FW_RULES[car_model], fw),
          match_rules(NON_FSD_14_AUTOPILOT_FW_RULES[car_model], fw),
        ]
        assert sum(matches) == 1, f"{fw}: matched {sum(matches)} rule sets"

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
