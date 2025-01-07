from enum import IntFlag
from opendbc.car.structs import CarParams
from opendbc.car import Bus, structs
from opendbc.car import CarSpecs, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class CAR(Platforms):
  RIVIAN_R1S = PlatformConfig(
    [CarDocs("Rivian R1S", "All")],
    CarSpecs(mass=3206., wheelbase=3.08, steerRatio=15.2),
    {Bus.pt: 'rivian_can'}
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.eps],
      rx_offset=0x08,
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
      rx_offset=0x10,
      bus=1,
      obd_multiplexing=False,
    ),
  ]
)

GEAR_MAP = [
  structs.CarState.GearShifter.unknown,
  structs.CarState.GearShifter.park,
  structs.CarState.GearShifter.reverse,
  structs.CarState.GearShifter.neutral,
  structs.CarState.GearShifter.drive,
]


class CarControllerParams:
  STEER_MAX = 350
  STEER_DELTA_UP = 8  # torque increase per refresh
  STEER_DELTA_DOWN = 8  # torque decrease per refresh
  STEER_DRIVER_ALLOWANCE = 15  # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 1  # weight driver torque
  STEER_DRIVER_FACTOR = 1

  ACCEL_MIN = -3.48  # m/s^2
  ACCEL_MAX = 2.0  # m/s^2

class RivianFlags(IntFlag):
  FLAG_RIVIAN_LONG_CONTROL = 1


DBC = CAR.create_dbc_map()
