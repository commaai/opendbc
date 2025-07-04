from dataclasses import dataclass, field
from enum import Enum, StrEnum, IntFlag
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, AngleSteeringLimits
from opendbc.car.structs import CarParams
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column
from opendbc.car.fw_query_definitions import FwQueryConfig , Request, StdQueries
from opendbc.car.vin import Vin

Ecu = CarParams.Ecu

class Buttons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  GAP_DIST = 3
  CANCEL = 4  # on newer models, this is a pause/resume button


class CarControllerParams:
  ACCEL_MAX = 2.0 # m/s
  ACCEL_MIN = -3.5 # m/s
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    90,  # deg
    ([0., 5., 25.], [2.5, 1.5, 0.2]),
    ([0., 5., 25.], [5., 2.0, 0.3]),
  )

  def __init__(self, CP):
    self.STEER_DELTA_UP = 3
    self.STEER_DELTA_DOWN = 5
    self.STEER_DRIVER_ALLOWANCE = 50
    self.STEER_DRIVER_MULTIPLIER = 2
    self.STEER_DRIVER_FACTOR = 1
    self.STEER_THRESHOLD = 150
    self.STEER_STEP = 1  # 100 Hz

    if CP.carFingerprint in (CAR.LANDROVER_DEFENDER_2023):
      self.STEER_DRIVER_ALLOWANCE = 200
      self.STEER_DRIVER_MULTIPLIER = 2
      self.STEER_THRESHOLD = 20
      self.STEER_STEP = 2  # 50 Hz

class CanBus:
  UNDERBODY = 0
  CAN2FLEXRAY = 1
  CAM = 2


class LandroverFlags(IntFlag):
  FLEXRAY_HARNESS = 1



class Footnote(Enum):
  FLEXRAY = CarFootnote(
    "Requires a " +
    "<a href=\"https://blog.comma.ai/hacking-an-audi-performing-a-man-in-the-middle-attack-on-flexray/\" target=\"_blank\">FlexRay Car Harness</a> " +
    "flexray <a href=\"https://en.wikipedia.org/wiki/FlexRay\" target=\"_blank\">FexRay car</a>.",
    Column.MODEL)

@dataclass
class LandroverCarDocsDefender(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.FLEXRAY])





"""
 LANDROVER VIN Code

 01234567890123456
 SALEA7AX8L2XXXXXX  : L663 2020 Defender 110
 SALEA8BW6P2XXXXXX  : L663 2023 Defender 130

 0~2 : WMI
 3~8 : VDS
 9~16: VIS

 https://twinwoods4x4.co.uk/how-to-decode-your-land-rover-vin-vehicle-identification-number/?srsltid=AfmBOoozLKVg5YTwYBJStW9m-2yq5J6CSmVEQGH4GPi5uNvPZf--ZdSE

 0~2 SAL: Land Rover   , Word Manufactureer Identifier
 3~4 EA : Defneder L663, Model Type
 5   7  : 110          , Wheelbase
     8  : 130
 6~8    : ??
 9   L  : 2020         , Year
     P  : 2023
 10  2  : Solihull     , Factory
 11~16  : Serial Number
"""

class WMI(StrEnum):
  LANDROVER = "SAL"


class ModelLine(StrEnum):
  L663 = "EA"  # Defender L663


class ModelYear(StrEnum):
  L_2020 = "L"
  P_2023 = "P"

@dataclass
class LandroverPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: "landrover_defender_2023" })
  wmis: set[WMI] = field(default_factory=set)
  lines: set[ModelLine] = field(default_factory=set)
  years: set[ModelYear] = field(default_factory=set)


class CAR(Platforms):
  LANDROVER_DEFENDER_2023 = LandroverPlatformConfig(
    [
      LandroverCarDocsDefender("LANDROVER DEFENDER 2023"),
    ],
    CarSpecs(mass=2550, wheelbase=3.022, steerRatio=19.0, minSteerSpeed=50*CV.KPH_TO_MS),
    wmis=(WMI.LANDROVER),
    lines={ModelLine.L663},
    years={ModelYear.L_2020, ModelYear.P_2023},
  )

def match_fw_to_car_fuzzy(live_fw_versions, vin, offline_fw_versions) -> set[str]:
  vin_obj = Vin(vin)
  line = vin_obj.vds[:2]
  year = vin_obj.vis[:1]

  candidates = set()
  for platform in CAR:
    if vin_obj.wmi in platform.config.wmis and line in platform.config.lines and year in platform.config.years:
      candidates.add(platform)

  return {str(c) for c in candidates}


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      obd_multiplexing=True,
    ),
  ],
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)


DBC = CAR.create_dbc_map()

if __name__ == "__main__":
  cars = []
  for platform in CAR:
    for doc in platform.config.car_docs:
      cars.append(doc.name)
  cars.sort()
  for c in cars:
    print(c)
