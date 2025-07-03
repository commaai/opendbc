from dataclasses import dataclass, field
from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, Bus
from opendbc.car.docs_definitions import CarParts, CarDocs, CarHarness
from opendbc.car.structs import CarParams
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

HUD_MULTIPLIER = 1.068

class CANBUS:
  main_bus = 0
  radar_bus = 1
  cam_bus = 2

def dbc_dict(pt, radar):
  return {Bus.pt: pt, Bus.radar: radar}

@dataclass
class BydCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))

@dataclass
class BYDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('byd_general_pt', None))

class CAR(Platforms):
  BYD_ATTO3 = BYDPlatformConfig(
    [BydCarDocs("Byd Atto 3")],
    CarSpecs(mass=2090., wheelbase=2.72, steerRatio=16.0)
  )

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    )
  ]
)

DBC = CAR.create_dbc_map()
