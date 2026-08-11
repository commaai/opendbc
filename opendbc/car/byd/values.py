from collections import defaultdict
from dataclasses import dataclass, field
from enum import Enum, IntEnum, IntFlag

from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from opendbc.car.byd.angle_rate_limit import AngleRateLimit
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.docs_definitions import CarDocs, CarParts, CUSTOM_CAR_PARTS, CarFootnote, Column


@dataclass
class CamLkaPlatformConfig(PlatformConfig):
  """482 STEERING_MODULE_ADAS + 790 LKAS_HUD_ADAS (byd_general_pt)."""
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("byd_general_pt", "byd_radar_fd"))


@dataclass
class MpcLkaPlatformConfig(PlatformConfig):
  """790 ACC_MPC_STATE + 792 ACC_EPS_STATE (byd_han_dmev_2020)."""
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("byd_han_dmev_2020", None))


@dataclass
class BYDCarDocs(CarDocs):
  car_parts: CarParts = field(default_factory=CUSTOM_CAR_PARTS)


class CANBUS:
  main_bus = 0
  radar_bus = 1
  cam_bus = 2


class BydFlags(IntFlag):
  MPC_LKA = 1
  # Flipped harness: ACC_HUD_ADAS + ACC_CMD on ESC bus 0; ACC_MPC_STATE alone on MPC bus 2.
  ACC_ON_ESC = 2


class LKASConfig(IntEnum):
  DISABLE = 0
  ALARM = 1
  LKA = 2
  ALARM_AND_LKA = 3


BYD_SUPPORT_COMMON_FIELDS = {
  "acc_low_speed": True,
  "acc_speed_range": "0 - 150",
  "acc_stop_and_go": True,
  "lkc_torque": "Very High",
  "lkc_speed_range": "0 - 150",
  "max_steering_angle": "120°",
}

BYD_LKC_ACC_INTELLIGENT_NOTE = CarFootnote(
  "Support: Lane Keep Assist + Adaptive Cruise Control. (Intelligent Cruise: yes)",
  Column.LONGITUDINAL,
)
BYD_LKC_ACC_STOCK_NOTE = CarFootnote(
  "Support: Lane Keep Assist + Adaptive Cruise Control. (Longitudinal: stock ACC)",
  Column.LONGITUDINAL,
)


class Footnote(Enum):
  LKC_ACC_INTELLIGENT = BYD_LKC_ACC_INTELLIGENT_NOTE
  LKC_ACC_STOCK = BYD_LKC_ACC_STOCK_NOTE


class CAR(Platforms):
  BYD_ATTO3 = CamLkaPlatformConfig(
    [BYDCarDocs(
      "BYD Atto 3 2023-26",
      "ALL",
      footnotes=[Footnote.LKC_ACC_INTELLIGENT],
      variant="All",
      **BYD_SUPPORT_COMMON_FIELDS,
    )],
    CarSpecs(mass=2090.0, wheelbase=2.72, steerRatio=16.0),
  )
  BYD_M6 = CamLkaPlatformConfig(
    [BYDCarDocs(
      "BYD M6 2024-26",
      "ALL",
      footnotes=[Footnote.LKC_ACC_INTELLIGENT],
      variant="Extended",
      **BYD_SUPPORT_COMMON_FIELDS,
    )],
    CarSpecs(mass=2374.0, wheelbase=2.80, steerRatio=16.0),
  )
  BYD_SONG_PLUS_DMI_21 = MpcLkaPlatformConfig(
    [BYDCarDocs(
      "BYD Song Plus DMI 2021",
      "ALL",
      footnotes=[Footnote.LKC_ACC_STOCK],
      variant="All",
      **BYD_SUPPORT_COMMON_FIELDS,
    )],
    CarSpecs(mass=1785.0, wheelbase=2.765, steerRatio=15.0),
    flags=BydFlags.MPC_LKA | BydFlags.ACC_ON_ESC,
  )
  BYD_SEAL = CamLkaPlatformConfig(
    [
      BYDCarDocs(
        "BYD Seal 2024-26",
        "ALL",
        footnotes=[Footnote.LKC_ACC_STOCK],
        variant="All",
        **BYD_SUPPORT_COMMON_FIELDS,
      ),
      BYDCarDocs(
        "BYD Dolphin 2023-26",
        "ALL",
        footnotes=[Footnote.LKC_ACC_STOCK],
        variant="All",
        **BYD_SUPPORT_COMMON_FIELDS,
      ),
    ],
    CarSpecs(mass=2180.0, wheelbase=2.92, steerRatio=16.0),
  )
  BYD_SEALION7 = CamLkaPlatformConfig(
    [BYDCarDocs(
      "BYD Sealion 7 2024-26",
      "ALL",
      footnotes=[Footnote.LKC_ACC_STOCK],
      variant="All",
      **BYD_SUPPORT_COMMON_FIELDS,
    )],
    CarSpecs(mass=2340.0, wheelbase=2.93, steerRatio=16.0),
  )
  BYD_SEAL6 = CamLkaPlatformConfig(
    [BYDCarDocs(
      "BYD Seal 6 2025-26",
      "ALL",
      footnotes=[Footnote.LKC_ACC_INTELLIGENT],
      variant="All",
      **BYD_SUPPORT_COMMON_FIELDS,
    )],
    CarSpecs(mass=1665.0, wheelbase=2.79, steerRatio=16.0),
    dbc_dict=dbc_dict("byd_general_pt", "byd_radar_seal6_fd"),
  )
  BYD_SHARK = CamLkaPlatformConfig(
    [BYDCarDocs(
      "BYD Shark 2024-26",
      "ALL",
      footnotes=[Footnote.LKC_ACC_STOCK],
      variant="All",
      **BYD_SUPPORT_COMMON_FIELDS,
    )],
    CarSpecs(mass=2710.0, wheelbase=3.26, steerRatio=16.0),
  )

PLATFORM_CAM_LKA = (
  CAR.BYD_ATTO3,
  CAR.BYD_M6,
  CAR.BYD_SEAL,
  CAR.BYD_SEALION7,
  CAR.BYD_SEAL6,
  CAR.BYD_SHARK,
)

PLATFORM_MPC_LKA = (
  CAR.BYD_SONG_PLUS_DMI_21,
)

# Atto 3 / M6: cam_lka PT+cam layout, Atto-style LKA latch on cam bus.
BYD_ATTO_STYLE_PLATFORMS = (
  CAR.BYD_ATTO3,
  CAR.BYD_M6,
  CAR.BYD_SEAL6,
)

BYD_OP_LONG_PLATFORMS = (
  CAR.BYD_ATTO3,
  CAR.BYD_M6,
  CAR.BYD_SEAL6,
)

DBC = CAR.create_dbc_map()
ACCEL_MULT = defaultdict(
  lambda: 1,
  {
    CAR.BYD_ATTO3: 26,
    CAR.BYD_M6: 26,
    CAR.BYD_SEAL: 1,
    CAR.BYD_SEALION7: 1,
    CAR.BYD_SEAL6: 26,
    CAR.BYD_SHARK: 1,
  },
)
HUD_MULTIPLIER = 1.12


class CarControllerParams:
  STEER_ANGLE_MAX = 120.0  # deg
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[6., 4., 3.])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[8., 6., 4.])
  ANGLE_LIMITS = AngleSteeringLimits(STEER_ANGLE_MAX, ANGLE_RATE_LIMIT_UP, ANGLE_RATE_LIMIT_DOWN)

  def __new__(cls, CP):
    # MPC_LKA platforms are torque-controlled (MpcLkaCarControllerParams below);
    # generic test infra (test_lateral_limits.py) always looks up brand.values.CarControllerParams
    # by name, so dispatch here rather than duplicating a brand-level lookup convention.
    if CP.carFingerprint in PLATFORM_MPC_LKA:
      return MpcLkaCarControllerParams(CP)
    return super().__new__(cls)

  def __init__(self, CP):
    pass


class MpcLkaCarControllerParams:
  """Torque LKAS limits for mpc_lka platforms (790 ACC_MPC_STATE path)."""
  STEER_MAX = 300
  STEER_DELTA_UP = 7
  STEER_DELTA_DOWN = 10
  STEER_DRIVER_ALLOWANCE = 68
  STEER_DRIVER_MULTIPLIER = 3
  STEER_DRIVER_FACTOR = 1
  STEER_ERROR_MAX = 50
  STEER_STEP = 2
  STEER_SOFTSTART_STEP = 6
  USE_STEERING_SPEED_LIMITER = False

  def __init__(self, CP):
    del CP
