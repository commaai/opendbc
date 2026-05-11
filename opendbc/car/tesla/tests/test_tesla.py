import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.radar_interface import RADAR_START_ADDR
from opendbc.car.tesla.values import CAR, FSD_14_FW_RULES, NON_FSD_14_SELFDRIVE_FW_RULES, Ecu, FW_RE, match_rules, compile_rules_dict

PLATFORM_TO_CAR = {
  b'E': CAR.TESLA_MODEL_3,
  b'Y': CAR.TESLA_MODEL_Y,
  b'X': CAR.TESLA_MODEL_X,
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

NON_FSD_14_AUTOPILOT_FW_RULES = compile_rules_dict(NON_FSD_14_AUTOPILOT_FW_RULES)

# Cars with this EPS FW have FSD 14 and use TeslaFlags.FSD_14
# For these cars we need to send a different value on DAS_steeringControlType.
FSD_14_FW = {
  CAR.TESLA_MODEL_3: [
    b'TeMYG4_Main_0.0.0 (77),E4HP015.04.5',
    b'TeMYG4_Main_0.0.0 (78),E4HP015.05.0',
    b'TeMYG4_Main_0.0.0 (77),E4H015.04.5',
    b'TeMYG4_Main_0.0.0 (78),E4H015.05.0',
  ],
  CAR.TESLA_MODEL_Y: [
    b'TeMYG4_Legacy3Y_0.0.0 (6),Y4003.04.0',
    b'TeMYG4_Main_0.0.0 (77),Y4003.05.4',
    b'TeMYG4_Main_0.0.0 (78),Y4003.06.0',
  ]
}

# HW3 cars with FW 2026.8.6+ can trigger a false FSD14 detection.
NON_FSD_14_SELFDRIVE_FW = {
  CAR.TESLA_MODEL_3: [
    b'TeM3_E014p10_0.0.0 (24),E014.20.2',
  ],
  CAR.TESLA_MODEL_Y: [
    b'TeM3_E014p10_0.0.0 (24),YP002.21.2',
  ]
}
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

      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        is_fsd_14 = fw in FSD_14_FW.get(car_model, [])
        expected = match_rules(FSD_14_FW_RULES[car_model], fw)
        assert is_fsd_14 == expected, f"{fw}"

  def test_non_fsd_14_selfdrive_fw(self):
    for car_model, ecus in FW_VERSIONS.items():
      if car_model not in NON_FSD_14_SELFDRIVE_FW_RULES:
        continue

      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        is_non_fsd_14_selfdrive = fw in NON_FSD_14_SELFDRIVE_FW.get(car_model, [])
        expected = match_rules(NON_FSD_14_SELFDRIVE_FW_RULES[car_model], fw)
        assert is_non_fsd_14_selfdrive == expected, f"{fw}"

  def test_fingerprints_parsable(self):
    # check if all fingerprints are parsable
    for _car_model, ecus in FW_VERSIONS.items():
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        decoded_fw = FW_RE.match(fw)
        assert decoded_fw is not None, f"Unparsable FW: {fw}"

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
