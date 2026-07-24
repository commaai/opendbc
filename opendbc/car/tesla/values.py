from dataclasses import dataclass, field
from enum import Enum, IntFlag
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimitsVM
from opendbc.car.structs import CarParams, CarState
from opendbc.car.docs_definitions import CarDocs, CarFootnote, CarHarness, CarParts, Column
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

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
  ]
)

# Cars with this EPS FW have a 2-bit DAS_steeringControlType and use TeslaFlags.LEGACY_DAS_STEERING
LEGACY_DAS_STEERING_FW = {
  CAR.TESLA_MODEL_3: [
    b'TeM3_E014p10_0.0.0 (16),E014.17.00',
    b'TeM3_E014p10_0.0.0 (16),EL014.17.00',
    b'TeM3_ES014p11_0.0.0 (25),ES014.19.0',
    b'TeMYG4_DCS_Update_0.0.0 (13),E4014.28.1',
    b'TeMYG4_DCS_Update_0.0.0 (9),E4014.26.0',
    b'TeMYG4_Legacy3Y_0.0.0 (2),E4015.02.0',
    b'TeMYG4_Legacy3Y_0.0.0 (5),E4015.03.2',
    b'TeMYG4_Legacy3Y_0.0.0 (5),E4L015.03.2',
    b'TeMYG4_Main_0.0.0 (59),E4H014.29.0',
    b'TeMYG4_Main_0.0.0 (65),E4H015.01.0',
    b'TeMYG4_Main_0.0.0 (67),E4H015.02.1',
    b'TeMYG4_SingleECU_0.0.0 (33),E4S014.27',
  ],
  CAR.TESLA_MODEL_Y: [
    b'TeM3_E014p10_0.0.0 (16),Y002.18.00',
    b'TeM3_E014p10_0.0.0 (16),YP002.18.00',
    b'TeM3_ES014p11_0.0.0 (16),YS002.17',
    b'TeM3_ES014p11_0.0.0 (25),YS002.19.0',
    b'TeMYG4_DCS_Update_0.0.0 (13),Y4002.27.1',
    b'TeMYG4_DCS_Update_0.0.0 (13),Y4P002.27.1',
    b'TeMYG4_DCS_Update_0.0.0 (9),Y4P002.25.0',
    b'TeMYG4_Legacy3Y_0.0.0 (2),Y4003.02.0',
    b'TeMYG4_Legacy3Y_0.0.0 (2),Y4P003.02.0',
    b'TeMYG4_Legacy3Y_0.0.0 (5),Y4003.03.2',
    b'TeMYG4_Legacy3Y_0.0.0 (5),Y4P003.03.2',
    b'TeMYG4_SingleECU_0.0.0 (28),Y4S002.23.0',
    b'TeMYG4_SingleECU_0.0.0 (33),Y4S002.26',
  ],
  CAR.TESLA_MODEL_X: [
    b'TeM3_SP_XP002p2_0.0.0 (23),XPR003.6.0',
    b'TeM3_SP_XP002p2_0.0.0 (36),XPR003.10.0',
  ],
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


class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimitsVM = AngleSteeringLimitsVM(
    # EPAS faults above this angle
    360,  # deg
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
  LEGACY_DAS_STEERING = 2


class TeslaFlags(IntFlag):
  LONG_CONTROL = 1
  LEGACY_DAS_STEERING = 2
  MISSING_DAS_SETTINGS = 4


DBC = CAR.create_dbc_map()

STEER_THRESHOLD = 1
