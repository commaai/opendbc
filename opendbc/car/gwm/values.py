from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 2  # STEER_CMD is sent at 50Hz
  STEER_MAX = 253  # max torque the stock camera commands
  STEER_DELTA_UP = 4
  STEER_DELTA_DOWN = 6
  STEER_ERROR_MAX = 80

  STEER_THRESHOLD = 50  # driver torque considered steering pressed
  STEER_DRIVER_ALLOWANCE = 100  # stop steering above this driver torque

  ACCEL_MAX = 2.0  # m/s^2
  ACCEL_MIN = -3.5  # m/s^2

  # ACC_CMD gas is a powertrain torque request, negative values reduce it to coasting like the stock ACC does.
  # Brakes start acting around 41. Both gains fit from engaged drives
  GAS_PER_ACCEL = 1900
  GAS_MIN = -192
  BRAKE_ZERO = 41
  BRAKE_PER_ACCEL = (107 - 41) / 3.5
  # switching between powertrain and brakes releases and reapplies brake pressure, add hysteresis
  BRAKE_ENTER_ACCEL = GAS_MIN / GAS_PER_ACCEL  # once gas is at its minimum
  BRAKE_EXIT_ACCEL = 0.1
  # like the stock ACC, apply moderate braking slowly enough for regen to take it instead of the friction brakes
  BRAKE_RAMP_RATE = 1.0  # m/s^3
  BRAKE_RAMP_MIN_ACCEL = -1.0  # harder braking isn't limited

  def __init__(self, CP):
    pass


class GwmSafetyFlags(IntFlag):
  LONG_CONTROL = 1


@dataclass
class GwmCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class GwmPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'gwm_haval_h6_mk3'})


class CAR(Platforms):
  GWM_HAVAL_H6 = GwmPlatformConfig(
    [GwmCarDocs("Haval H6 2024-25")],
    CarSpecs(mass=2040, wheelbase=2.738, steerRatio=17.416),
  )


GWM_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_SPARE_PART_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_ECU_SOFTWARE_VERSION_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_DATA_IDENTIFICATION)
GWM_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40])

# Only the engine responds, and only on the OBD port
FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"\xf1\x87[0-9A-Z]{15}\xf1\x89[0-9A-Z]{15}",
  requests=[
    Request(
      [GWM_VERSION_REQUEST],
      [GWM_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
      bus=1,
    ),
  ],
)

DBC = CAR.create_dbc_map()
