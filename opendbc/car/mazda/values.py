from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, DT_CTRL, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


# Steer torque limits

class CarControllerParams:
  STEER_MAX = 800                # theoretical max_steer 2047
  STEER_DELTA_UP = 10             # torque increase per refresh
  STEER_DELTA_DOWN = 25           # torque decrease per refresh
  STEER_DRIVER_ALLOWANCE = 15     # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 1     # weight driver torque
  STEER_DRIVER_FACTOR = 1         # from dbc
  STEER_STEP = 1  # 100 Hz

  ACCEL_MAX = 2.0   # m/s²
  ACCEL_MIN = -3.5  # m/s²

  LONG_STEP = 2        # CRZ_INFO and CRZ_CTRL at 50 Hz
  RADAR_STEP = 10      # radar static and track frames at 10 Hz
  RADAR_UDS_STEP = 50  # session control and tester present at 2 Hz

  FSC_SETTLE_T = 10.0
  STOCK_RADAR_ALIVE_T = 0.05
  PANDA_RADAR_SILENT_T = 1.0
  STOCK_RADAR_GUARD_MARGIN_T = 0.2
  STOCK_RADAR_GUARD_T = STOCK_RADAR_ALIVE_T + LONG_STEP * DT_CTRL + PANDA_RADAR_SILENT_T + STOCK_RADAR_GUARD_MARGIN_T
  RADAR_SESSION_LIMIT_T = 10.0
  CAM_LANEINFO_FRESH_T = 1.5
  CANCEL_CONTEXT_T = 0.5

  RELEASE_DEBOUNCE_T = 0.2
  RESUME_UNLATCH_LATCHED_T = 0.18
  RESUME_REPULSE_T = 1.0

  ACCEL_HOLD_LATCHED = -0.001
  ACCEL_RESUME_PULSE_MAX = 0.25
  ACCEL_RELEASE_BAND = -0.26
  ACCEL_RELEASE_RAMP = 1.25
  ACCEL_BREAKAWAY_MAX = 1.45
  ACCEL_BREAKAWAY_T = 3.0
  ACCEL_BREAKAWAY_OVERSHOOT = 0.75
  ACCEL_WINDUP_LIMIT = 4.0 * DT_CTRL
  ACCEL_WINDDOWN_LIMIT = -10.0 * DT_CTRL

  ACCEL_CEILING_BP = [0., 4., 9., 14., 18., 25.]
  ACCEL_CEILING_V = [1.5, 1.75, 1.45, 1.05, 0.85, 0.65]
  ACCEL_BUILD_BP = [3., 6.]
  ACCEL_BUILD_V = [1.25, 0.8]
  ACCEL_LIFT_LIMIT = -2.0

  def __init__(self, CP):
    pass


@dataclass
class MazdaCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.mazda]))


@dataclass(frozen=True, kw_only=True)
class MazdaCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.7  # not optimized yet


class MazdaFlags(IntFlag):
  # Static flags
  # Gen 1 hardware: same CAN messages and same camera
  GEN1 = 1


class MazdaSafetyFlags(IntFlag):
  LONG = 1


@dataclass
class MazdaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'mazda_2017'})
  flags: int = MazdaFlags.GEN1


class CAR(Platforms):
  MAZDA_CX5 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2017-21")],
    MazdaCarSpecs(mass=3655 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=15.5)
  )
  MAZDA_CX9 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2016-20")],
    MazdaCarSpecs(mass=4217 * CV.LB_TO_KG, wheelbase=3.1, steerRatio=17.6)
  )
  MAZDA_3 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2017-18")],
    MazdaCarSpecs(mass=2875 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=14.0)
  )
  MAZDA_6 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 6 2017-20")],
    MazdaCarSpecs(mass=3443 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=15.5)
  )
  MAZDA_CX9_2021 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2021-23", video="https://youtu.be/dA3duO4a0O4")],
    MAZDA_CX9.specs
  )
  MAZDA_CX5_2022 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2022-25")],
    MAZDA_CX5.specs,
  )


class LKAS_LIMITS:
  STEER_THRESHOLD = 15
  DISABLE_SPEED = 45    # kph
  ENABLE_SPEED = 52     # kph


class Buttons:
  NONE = 0
  SET_PLUS = 1
  SET_MINUS = 2
  RESUME = 3
  CANCEL = 4


FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"[A-Z0-9-]{11,16}\x00{8,13}",
  requests=[
    # TODO: check data to ensure ABS does not skip ISO-TP frames on bus 0
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
