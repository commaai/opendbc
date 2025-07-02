from dataclasses import dataclass, field
from openpilot.selfdrive.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarInfo

HUD_MULTIPLIER = 1.068

@dataclass
class BYDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('byd_general_pt', None))

class CANBUS:
  main_bus = 0
  radar_bus = 1
  cam_bus = 2

class CAR(Platforms):
  ATTO3 = BYDPlatformConfig(
    "BYD ATTO 3",
    CarInfo("BYD Atto 3", "ALL"),
    specs=CarSpecs(mass=2090., wheelbase=2.72, steerRatio=16.0)
  )

CAR_INFO = CAR.create_carinfo_map()
DBC = CAR.create_dbc_map()
