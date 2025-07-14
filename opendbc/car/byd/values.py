from enum import IntFlag
from dataclasses import dataclass, field
from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, Bus
from opendbc.car.docs_definitions import CarParts, CarDocs, CarHarness
from opendbc.car.structs import CarParams
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

class BydSteerStates(IntFlag):
  IDLE = 0
  FAULTED = 1
  STEER_ENABLED = 2

class CANBUS:
  main_bus = 0
  radar_bus = 1
  cam_bus = 2

def dbc_dict(pt, radar):
  return {Bus.pt: pt, Bus.radar: radar}

class BydSafetyFlags(IntFlag):
  LONG_CONTROL = 1

@dataclass
class BydCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))

@dataclass
class BYDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('byd_general_pt', None))

class CAR(Platforms):
  BYD_ATTO3 = BYDPlatformConfig(
    [BydCarDocs("BYD ATTO 3 2022-24")],
    CarSpecs(mass=2090., wheelbase=2.72, steerRatio=16.0, centerToFrontRatio=0.44)
  )

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.UDS_VERSION_RESPONSE],
      bus=0,
    )],
)

DBC = CAR.create_dbc_map()
