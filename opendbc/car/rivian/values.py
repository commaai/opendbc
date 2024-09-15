from collections import namedtuple
from enum import IntFlag
from opendbc.car.structs import CarParams
from opendbc.car import structs
from opendbc.car import AngleRateLimit, CarSpecs, PlatformConfig, Platforms, dbc_dict
from opendbc.car.docs_definitions import CarDocs
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

class CAR(Platforms):
  RIVIAN_R1S = PlatformConfig(
    [CarDocs("Rivian R1S", "All")],
    CarSpecs(mass=3206., wheelbase=3.08, steerRatio=15.0),
    dbc_dict('rivian_can', None)
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
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 15.], angle_v=[.4, .1])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 15.], angle_v=[.4, .1])
  ACCEL_MIN = -3.48  # m/s^2
  ACCEL_MAX = 2.0    # m/s^2
  JERK_LIMIT_MAX = 5
  JERK_LIMIT_MIN = -5
  ACCEL_TO_SPEED_MULTIPLIER = 3

  def __init__(self, CP):
    pass


class RivianFlags(IntFlag):
  FLAG_RIVIAN_LONG_CONTROL = 1


DBC = CAR.create_dbc_map()
