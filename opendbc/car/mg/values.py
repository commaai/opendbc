from dataclasses import dataclass, field
from enum import StrEnum, IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, structs, uds
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts, Device
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries, p16
from opendbc.car.vin import Vin

@dataclass
class MgCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.mg_a]))

@dataclass
class MgPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'mg', Bus.radar: 'mg'})

class CAR(Platforms):
  MG_5_EV = MgPlatformConfig(
    # TODO: verify this
    [
      MgCarDocs("MG ZS EV 2021-24"),
    ],
    CarSpecs(mass=1992., wheelbase=2.66, steerRatio=15.8),
  )


MG_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf1a0)
MG_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40])

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

  STEER_MAX = 250
  STEER_DELTA_UP = 15    # torque increase per refresh
  STEER_DELTA_DOWN = 10  # torque decrease per refresh
  STEER_DRIVER_ALLOWANCE = 100  # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 2  # weight driver torque
  STEER_DRIVER_FACTOR = 100

  ACCEL_MIN = -3.5  # m/s^2
  ACCEL_MAX = 2.0  # m/s^2

  def __init__(self, CP):
    pass


class MgSafetyFlags(IntFlag):
  LONG_CONTROL = 1


DBC = CAR.create_dbc_map()
