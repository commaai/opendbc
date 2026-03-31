from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, structs
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries


@dataclass
class MgCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.vw_a]))


@dataclass
class MgPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'mg'})


class CAR(Platforms):
  MG_5_EV = MgPlatformConfig(
    [
      MgCarDocs("MG 5 EV 2021"),
    ],
    CarSpecs(mass=1640., wheelbase=2.66, steerRatio=15.8),
  )

  MG_ZS_EV = MgPlatformConfig(
    [
      MgCarDocs("MG ZS EV 2022"),
    ],
    CarSpecs(mass=1590., wheelbase=2.58, steerRatio=15.8),
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_ECU_HARDWARE_NUMBER_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_ECU_HARDWARE_NUMBER_RESPONSE],
      bus=1,
    ),
  ],
)

GEAR_MAP = {
  0: structs.CarState.GearShifter.unknown,
  15: structs.CarState.GearShifter.park,
  14: structs.CarState.GearShifter.reverse,
  13: structs.CarState.GearShifter.neutral,
  **{i: structs.CarState.GearShifter.drive for i in range(1, 9)},
}


class CarControllerParams:
  STEER_STEP = 2  # FVCM_HSC2_FrP03 message frequency 50Hz
  STEER_MAX = 300
  STEER_DELTA_UP = 6       # Max torque reached in 1.00s (STEER_MAX / (50Hz * 1.00))
  STEER_DELTA_DOWN = 10    # Min torque reached in 0.60s (STEER_MAX / (50Hz * 0.60))
  STEER_DRIVER_ALLOWANCE = 100  # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 2  # weight driver torque
  STEER_DRIVER_FACTOR = 100

  def __init__(self, CP):
    pass


class MgSafetyFlags(IntFlag):
  ALT_BRAKE = 1


DBC = CAR.create_dbc_map()
