import re
from dataclasses import dataclass, field
from enum import Enum, IntFlag
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


# Tesla firmware version code pattern (after comma in FW string)
# Examples: E014.17.00, Y4003.05.4, XPR003.6.0
# Structure: <model><variant><platform_number>.<version>
# - model: E (Model 3), Y (Model Y), X (Model X)
# - variant: optional letters/digits for generation and hardware variant (e.g. 4, L, S, H, P, PR)
# - platform_number: 3 digits
# - version: dot-separated numbers (ignored for fuzzy matching)
FW_VERSIONS_PATTERN = re.compile(rb',[EYX]')
PLATFORM_CODE_PATTERN = re.compile(rb',(?P<model>[EYX])(?P<variant>[A-Z0-9]*)(?P<platform>\d{3})\.')

PLATFORM_CODE_ECUS = (Ecu.eps,)


def get_platform_codes(fw_versions: list[bytes] | set[bytes]) -> set[tuple[bytes, bytes]]:
  """Extract (model, platform_number) tuples from firmware versions."""
  codes = set()
  for fw in fw_versions:
    match = PLATFORM_CODE_PATTERN.search(fw)
    if match is not None:
      codes.add((match.group('model'), match.group('platform')))
  return codes


def match_fw_to_car_fuzzy(live_fw_versions: LiveFwVersions, vin: str, offline_fw_versions: OfflineFwVersions) -> set[str]:
  candidates: set[str] = set()

  for candidate, fws in offline_fw_versions.items():
    valid_found_ecus = set()
    valid_expected_ecus = {ecu[1:] for ecu in fws if ecu[0] in PLATFORM_CODE_ECUS}
    for ecu, expected_versions in fws.items():
      addr = ecu[1:]
      if ecu[0] not in PLATFORM_CODE_ECUS:
        continue

      # Expected platform codes (model letter identifies the car)
      codes = get_platform_codes(expected_versions)
      expected_models = {model for model, _ in codes}

      # Found platform codes from live FW
      codes = get_platform_codes(live_fw_versions.get(addr, set()))
      found_models = {model for model, _ in codes}

      # Check model identifier matches (E=Model 3, Y=Model Y, X=Model X)
      if not found_models & expected_models:
        break

      valid_found_ecus.add(addr)

    if valid_expected_ecus.issubset(valid_found_ecus):
      candidates.add(candidate)

  return candidates


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    )
  ],
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)

# Cars with this EPS FW have FSD 14 and use TeslaFlags.FSD_14
FSD_14_FW = {
  CAR.TESLA_MODEL_Y: [
    b'TeMYG4_Legacy3Y_0.0.0 (6),Y4003.04.0',
    b'TeMYG4_Main_0.0.0 (77),Y4003.05.4',
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
