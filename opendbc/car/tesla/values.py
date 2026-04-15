import re
from dataclasses import dataclass, field
from enum import Enum, IntFlag
from typing import Set
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimits, ISO_LATERAL_ACCEL
from opendbc.car.structs import CarParams, CarState
from opendbc.car.docs_definitions import CarDocs, CarFootnote, CarHarness, CarParts, Column
from opendbc.car.fw_query_definitions import FwQueryConfig, LiveFwVersions, OfflineFwVersions, Request, StdQueries

Ecu = CarParams.Ecu


class Footnote(Enum):
  HW_TYPE = CarFootnote(
    "Some 2023 model years have HW4. To check which hardware type your vehicle has, look for " +
    "<b>Autopilot computer</b> under <b>Software -> Additional Vehicle Information</b> on your vehicle's touchscreen. </br></br>" +
    "See <a href=\"https://www.notateslaapp.com/news/2173/how-to-check-if-your-tesla-has-hardware-4-ai4-or-hardware-3\">this page</a> for more information.",
    Column.MODEL)

  SETUP = CarFootnote(
    "See more setup details for <a href=\"https://github.com/commaai/openpilot/wiki/tesla\" target=\"_blank\">Tesla</a>.",
    Column.MAKE, setup_note=True)


@dataclass
class TeslaCarDocsHW3(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.tesla_a]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.HW_TYPE, Footnote.SETUP])


@dataclass
class TeslaCarDocsHW4(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.tesla_b]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.HW_TYPE, Footnote.SETUP])


@dataclass
class TeslaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.party: 'tesla_model3_party'})


class CAR(Platforms):
  TESLA_MODEL_3 = TeslaPlatformConfig(
    [
      # TODO: do we support 2017? It's HW3
      TeslaCarDocsHW3("Tesla Model 3 (with HW3) 2019-23"),
      TeslaCarDocsHW4("Tesla Model 3 (with HW4) 2024-25"),
    ],
    CarSpecs(mass=1899., wheelbase=2.875, steerRatio=12.0),
    {Bus.party: 'tesla_model3_party', Bus.radar: 'tesla_radar_continental_generated'},
  )
  TESLA_MODEL_Y = TeslaPlatformConfig(
    [
      TeslaCarDocsHW3("Tesla Model Y (with HW3) 2020-23"),
      TeslaCarDocsHW4("Tesla Model Y (with HW4) 2024-25"),
    ],
    CarSpecs(mass=2072., wheelbase=2.890, steerRatio=12.0),
    {Bus.party: 'tesla_model3_party', Bus.radar: 'tesla_radar_continental_generated'},
  )
  TESLA_MODEL_X = TeslaPlatformConfig(
    [TeslaCarDocsHW4("Tesla Model X (with HW4) 2024")],
    CarSpecs(mass=2495., wheelbase=2.960, steerRatio=12.0),
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    )
  ],
  # Custom fuzzy fingerprinting function using Tesla platform codes
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)

# Regex pattern for parsing Tesla FW versions
# Format: TeM3_...,... or TeMYG4_...,...
# After comma: E/Y/XP + platform code + version
# Examples:
#   Model 3: E014.17.00, E4014.28.1, E4H015.04.5, E4HP015.04.5
#   Model Y: Y002.18.00, Y4002.27.1, Y4003.05.4
#   Model X: XPR003.6.0, XPR003.10.0
TESLA_FW_PATTERN = re.compile(b',(?P<platform>[EY][A-Z0-9]*|XP[A-Z0-9]*)(?P<version>[0-9.]+)')


def get_platform_codes(fw_versions: list[bytes]) -> dict[bytes, bytes]:
  """
  Parse platform codes from Tesla FW versions.
  
  Returns dict mapping platform code to version string.
  Example: {b'E4H': b'015.04.5', b'Y4': b'003.05.4'}
  """
  codes = {}
  for fw in fw_versions:
    match = TESLA_FW_PATTERN.search(fw)
    if match:
      platform = match.group('platform')
      version = match.group('version')
      codes[platform] = version
  return codes


def match_fw_to_car_fuzzy(live_fw_versions: LiveFwVersions, vin: str, offline_fw_versions: OfflineFwVersions) -> Set[str]:
  """
  Tesla fuzzy fingerprinting based on EPS firmware platform codes.
  
  Tesla firmware versions contain platform identifiers that can be used to:
  - Distinguish Model 3 vs Model Y vs Model X (E vs Y vs XP prefix)
  - Identify hardware generation (HW3 vs HW4)
  - Detect FSD 14 compatibility
  
  The fuzzy match allows minor version differences while requiring:
  - Matching platform prefix (E/Y/XP)
  - Compatible hardware generation markers
  """
  candidates: Set[str] = set()
  
  for candidate, fws in offline_fw_versions.items():
    # Keep track of ECUs which pass all checks
    valid_found_ecus = set()
    valid_expected_ecus = {ecu[1:] for ecu in fws if ecu[0] in PLATFORM_CODE_ECUS}
    
    for ecu, expected_versions in fws.items():
      addr = ecu[1:]
      # Only check ECUs expected to have platform codes
      if ecu[0] not in PLATFORM_CODE_ECUS:
        continue
      
      # Expected platform codes
      expected_codes = get_platform_codes(expected_versions)
      expected_platforms = set(expected_codes.keys())
      
      # Found platform codes
      found_versions = live_fw_versions.get(addr, set())
      found_codes = get_platform_codes(list(found_versions))
      found_platforms = set(found_codes.keys())
      
      # Check if any found platform code matches expected
      # Platform prefix determines the model family:
      # - E* = Model 3
      # - Y* = Model Y  
      # - XP* = Model X
      if not any(_platforms_match(exp, found) for exp in expected_platforms for found in found_platforms):
        break
      
      valid_found_ecus.add(addr)
    
    # If all live ECUs pass all checks for candidate, add it as a match
    if valid_expected_ecus.issubset(valid_found_ecus):
      candidates.add(str(candidate))
  
  return candidates


def _platforms_match(expected: bytes, found: bytes) -> bool:
  """
  Check if two platform codes are compatible.
  
  Rules:
  - Same prefix letter (E/Y/XP) = same model family
  - HW4 versions (with H) are backward compatible with HW3
  - FSD 14 versions (with P) are a superset
  """
  # Extract base platform (first 1-2 chars)
  exp_base = expected[:2] if expected.startswith(b'XP') else expected[:1]
  found_base = found[:2] if found.startswith(b'XP') else found[:1]
  
  # Must have same base platform
  if exp_base != found_base:
    return False
  
  # Check HW4 compatibility (H suffix in middle of code)
  # E4H is compatible with E4, Y4H compatible with Y4
  exp_has_hw4 = b'H' in expected
  found_has_hw4 = b'H' in found
  
  # HW4 cars can match HW3 firmware and vice versa (same physical platform)
  # This allows fuzzy matching across HW generations
  
  # Check FSD 14 compatibility (P suffix)
  exp_has_fsd14 = b'P' in expected
  found_has_fsd14 = b'P' in found
  
  # FSD 14 is software feature, hardware compatible
  # Allow matching regardless of FSD status
  
  return True


# ECU types expected to have platform codes for Tesla
# Currently only EPS is queried for Tesla
PLATFORM_CODE_ECUS = [Ecu.eps]

# Cars with this EPS FW have FSD 14 and use TeslaFlags.FSD_14
FSD_14_FW = {
  CAR.TESLA_MODEL_3: [
    b'TeMYG4_Main_0.0.0 (77),E4HP015.04.5',
    b'TeMYG4_Main_0.0.0 (78),E4HP015.05.0',
    b'TeMYG4_Main_0.0.0 (77),E4H015.04.5',
  ],
  CAR.TESLA_MODEL_Y: [
    b'TeMYG4_Legacy3Y_0.0.0 (6),Y4003.04.0',
    b'TeMYG4_Main_0.0.0 (77),Y4003.05.4',
    b'TeMYG4_Main_0.0.0 (78),Y4003.06.0',
  ]
}


class CANBUS:
  party = 0
  vehicle = 1
  autopilot_party = 2


GEAR_MAP = {
  "DI_GEAR_INVALID": CarState.GearShifter.unknown,
  "DI_GEAR_P": CarState.GearShifter.park,
  "DI_GEAR_R": CarState.GearShifter.reverse,
  "DI_GEAR_N": CarState.GearShifter.neutral,
  "DI_GEAR_D": CarState.GearShifter.drive,
  "DI_GEAR_SNA": CarState.GearShifter.unknown,
}


# Add extra tolerance for average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll lowers lateral acceleration


class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    # EPAS faults above this angle
    360,  # deg
    # Tesla uses a vehicle model instead, check carcontroller.py for details
    ([], []),
    ([], []),

    # Vehicle model angle limits
    # Add extra tolerance for average banked road since safety doesn't have the roll
    MAX_LATERAL_ACCEL=ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),  # ~3.6 m/s^2
    MAX_LATERAL_JERK=3.0 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),  # ~3.6 m/s^3

    # limit angle rate to both prevent a fault and for low speed comfort (~12 mph rate down to 0 mph)
    MAX_ANGLE_RATE=5,  # deg/20ms frame, EPS faults at 12 at a standstill
  )

  STEER_STEP = 2  # Angle command is sent at 50 Hz
  ACCEL_MAX = 2.0    # m/s^2
  ACCEL_MIN = -3.48  # m/s^2
  JERK_LIMIT_MAX = 4.9  # m/s^3, ACC faults at 5.0
  JERK_LIMIT_MIN = -4.9  # m/s^3, ACC faults at 5.0


class TeslaSafetyFlags(IntFlag):
  LONG_CONTROL = 1
  FSD_14 = 2


class TeslaFlags(IntFlag):
  LONG_CONTROL = 1
  FSD_14 = 2
  MISSING_DAS_SETTINGS = 4


DBC = CAR.create_dbc_map()

STEER_THRESHOLD = 1
