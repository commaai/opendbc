from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, structs
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries


@dataclass
class RenaultCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class RenaultPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'renault_5_etech'})


class CAR(Platforms):
  RENAULT_5_ETECH = RenaultPlatformConfig(
    [RenaultCarDocs("Renault 5 E-Tech 2024-25")],
    # Renault 5 E-Tech 150hp: kerb mass 1524 kg, wheelbase 2.54 m, steering ratio 15.0
    CarSpecs(mass=1524., wheelbase=2.54, steerRatio=15.0),
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[structs.CarParams.Ecu.fwdCamera],
    ),
  ],
)


# 0x12F.GEAR_STATE (see VAL table in renault_5_etech.dbc); PARK is a separate bit
GEAR_MAP = {
  0: structs.CarState.GearShifter.unknown,
  1: structs.CarState.GearShifter.reverse,
  2: structs.CarState.GearShifter.neutral,
  3: structs.CarState.GearShifter.brake,  # B = one-pedal regen
  8: structs.CarState.GearShifter.drive,
}


class CarControllerParams:
  # Tuned against torque_data/override.toml ISO 11270 jerk limits; retune on-vehicle.
  STEER_STEP = 2               # 0x134 is 100 Hz; tx cadence 50 Hz
  STEER_MAX = 100              # 0x134 TORQUE_CMD is an 8-bit field
  STEER_DELTA_UP = 2
  STEER_DELTA_DOWN = 4
  STEER_DRIVER_ALLOWANCE = 150
  STEER_DRIVER_MULTIPLIER = 2
  STEER_DRIVER_FACTOR = 1

  def __init__(self, CP):
    pass


class RenaultSafetyFlags(IntFlag):
  pass


DBC = CAR.create_dbc_map()
