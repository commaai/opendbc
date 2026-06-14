from enum import Enum
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu


class CAR(Enum):
  FORD_F150_2026 = "FORD F-150 2026"


class DBC:
  ford_f150_2026 = {
    "pt": "ford_f150_2026_pt",
    "radar": "ford_f150_2026_radar",
    "camera": "ford_f150_2026_camera",
  }