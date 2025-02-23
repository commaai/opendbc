from dataclasses import dataclass, field
from enum import StrEnum

from opendbc.car.structs import CarParams, CarState
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig

Ecu = CarParams.Ecu


class WMI(StrEnum):
  RIVIAN_TRUCK = "7FC"
  RIVIAN_MPV = "7PD"


class ModelYear(StrEnum):
  N_2022 = "N"
  P_2023 = "P"
  R_2024 = "R"
  S_2025 = "S"


class ModelLine(StrEnum):
  R1T = "T"  # R1T 4-door Pickup Truck
  R1S = "S"  # R1S 4-door MPV


@dataclass
class RivianCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.rivian]))

@dataclass
class RivianPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'tesla_model3_party',
    Bus.radar: 'rivian_mando_front_radar_generated'
  })
  wmis: set[WMI] = field(default_factory=set)
  lines: set[ModelLine] = field(default_factory=set)
  years: set[ModelYear] = field(default_factory=set)

class CAR(Platforms):
  RIVIAN_R1_GEN1 = RivianPlatformConfig(
    # TODO: verify this
    [
      RivianCarDocs("Rivian R1S 2022-24"),
      RivianCarDocs("Rivian R1T 2022-24"),
    ],
    CarSpecs(mass=3206., wheelbase=3.08, steerRatio=15.2),
    {Bus.pt: 'rivian_can'},
    wmis={WMI.RIVIAN_TRUCK, WMI.RIVIAN_MPV},
    lines={ModelLine.R1T, ModelLine.R1T},
    years={ModelYear.N_2022, ModelYear.P_2023, ModelYear.R_2024},
  )


def match_fw_to_car_fuzzy(live_fw_versions, vin, offline_fw_versions) -> set[str]:
  candidates = set()

  wmi = vin[:3]
  line = vin[3]
  year = vin[9]

  for platform in CAR:
    if wmi in platform.config.wmis and line in platform.config.lines and  year in platform.config.years:
      candidates.add(platform)

  return {str(c) for c in candidates}


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
  ],
  match_fw_to_car_fuzzy = match_fw_to_car_fuzzy,
)

GEAR_MAP = [
  CarState.GearShifter.unknown,
  CarState.GearShifter.park,
  CarState.GearShifter.reverse,
  CarState.GearShifter.neutral,
  CarState.GearShifter.drive,
]


class CarControllerParams:
  STEER_MAX = 350
  STEER_STEP = 1
  STEER_DELTA_UP = 3  # torque increase per refresh
  STEER_DELTA_DOWN = 5  # torque decrease per refresh
  STEER_DRIVER_ALLOWANCE = 15  # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 1  # weight driver torque
  STEER_DRIVER_FACTOR = 1

  ACCEL_MIN = -3.48  # m/s^2
  ACCEL_MAX = 2.0  # m/s^2

  def __init__(self, CP):
    pass


DBC = CAR.create_dbc_map()
