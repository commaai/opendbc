from dataclasses import dataclass, field
from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, Bus
from opendbc.car.docs_definitions import CarParts, CarDocs, CarHarness

HUD_MULTIPLIER = 1.068

def dbc_dict(pt, radar):
  return {Bus.pt: pt, Bus.radar: radar}

@dataclass
class BydCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))

@dataclass
class BYDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('byd_general_pt', None))

class CANBUS:
  main_bus = 0
  radar_bus = 1
  cam_bus = 2

class CAR(Platforms):
  ATTO3 = BYDPlatformConfig(
    BydCarDocs("BYD ATTO 3"),
    CarSpecs(mass=2090., wheelbase=2.72, steerRatio=16.0)
  )

DBC = CAR.create_dbc_map()
