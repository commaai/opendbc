from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16

Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 2
  STEER_MAX = 253
  ACCEL_MAX = 2
  ACCEL_MIN = -3.5

  def __init__(self, CP: CarParams):
    self.STEER_DELTA_UP = 4
    self.STEER_DELTA_DOWN = 6
    self.STEER_ERROR_MAX = 80


class GwmSafetyFlags(IntFlag):
  LONG_CONTROL = 1


@dataclass
class GWMCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.gwm]))


@dataclass
class GWMPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'gwm_haval_h6_mk3_generated',
  })


class CAR(Platforms):
  GWM_HAVAL_H6 = GWMPlatformConfig(
    [GWMCarDocs("Haval H6 2019-26")],
    CarSpecs(mass=2040, wheelbase=2.738, steerRatio=17.416),
  )


GREATWALLMOTORS_VERSION_REQUEST_MULTI = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_SPARE_PART_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_ECU_SOFTWARE_VERSION_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_DATA_IDENTIFICATION)
GREATWALLMOTORS_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40])

GREATWALLMOTORS_RX_OFFSET = 0x6a

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[request for bus, obd_multiplexing in [(1, True), (1, False), (0, False)] for request in [
    Request(
      [GREATWALLMOTORS_VERSION_REQUEST_MULTI],
      [GREATWALLMOTORS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
      rx_offset=GREATWALLMOTORS_RX_OFFSET,
      bus=bus,
      obd_multiplexing=obd_multiplexing,
    ),
    Request(
      [GREATWALLMOTORS_VERSION_REQUEST_MULTI],
      [GREATWALLMOTORS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
      bus=bus,
      obd_multiplexing=obd_multiplexing,
    ),
  ]],
)

DBC = CAR.create_dbc_map()
