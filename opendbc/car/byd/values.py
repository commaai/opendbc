from dataclasses import dataclass, field
from enum import IntFlag, StrEnum

from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, structs
from opendbc.car.lateral import AngleSteeringLimits, ISO_LATERAL_ACCEL
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig
from opendbc.car.vin import Vin

Ecu = structs.CarParams.Ecu


# Add extra tolerance for average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll lowers lateral acceleration


class CarControllerParams:
  STEER_STEP = 2  # Angle command is sent at 50 Hz

  # STEERING_TORQUE.MAIN_TORQUE is saturated at +-300 (probably 3.0nm)
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    390,  # deg
    # BYD uses a vehicle model instead, check carcontroller.py for details
    ([], []),
    ([], []),

    # Vehicle model angle limits
    # Add extra tolerance for average banked road since safety doesn't have the roll
    MAX_LATERAL_ACCEL=ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),  # ~3.6 m/s^2
    MAX_LATERAL_JERK=3.0 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),  # ~3.6 m/s^3

    # limit angle rate to both prevent a fault and for low speed comfort
    MAX_ANGLE_RATE=5,  # deg/20ms frame
  )

  STEER_DRIVER_OVERRIDE = 10   # EPS torque threshold for soft override
  STEER_DRIVER_DISENGAGE = 30  # EPS torque threshold for hard disengage


class BydSafetyFlags(IntFlag):
  LONG_CONTROL = 1


class WMI(StrEnum):
  BYD_AUTO = "LGX"  # BYD Auto Co., Ltd. (Shenzhen)


class ModelYear(StrEnum):
  N_2022 = "N"
  P_2023 = "P"
  R_2024 = "R"
  S_2025 = "S"


@dataclass
class BydCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class BydPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'byd_atto3',
  })
  wmis: set[WMI] = field(default_factory=set)
  years: set[ModelYear] = field(default_factory=set)


class CAR(Platforms):
  BYD_ATTO_3 = BydPlatformConfig(
    [BydCarDocs("BYD Atto 3 2022-25")],
    CarSpecs(mass=1750, wheelbase=2.72, steerRatio=14.8),
    wmis={WMI.BYD_AUTO},
    years={ModelYear.N_2022, ModelYear.P_2023, ModelYear.R_2024, ModelYear.S_2025},
  )


def match_fw_to_car_fuzzy(live_fw_versions, vin, offline_fw_versions) -> set[str]:
  # BYD Atto 3 VIN: LGX (WMI) + <VDS> + <year><plant><seq> (VIS).
  # TODO: currently we only match on WMI + model year
  vin_obj = Vin(vin)
  year = vin_obj.vis[:1]

  candidates = set()
  for platform in CAR:
    if vin_obj.wmi in platform.config.wmis and year in platform.config.years:
      candidates.add(platform)

  return {str(c) for c in candidates}


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[],
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)


DBC = CAR.create_dbc_map()
