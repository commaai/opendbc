from dataclasses import dataclass, field
from enum import Enum, StrEnum, IntFlag
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.structs import CarParams
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
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
    self.STEER_MAX = 500

    if CP.carFingerprint in (CAR.LANDROVER_DEFENDER_2023):
      self.STEER_DRIVER_ALLOWANCE = 200
      self.STEER_DRIVER_MULTIPLIER = 2
      self.STEER_THRESHOLD = 50
      self.STEER_STEP = 2  # 50 Hz

    """
    elif CP.carFingerprint in (CAR.RANGEROVER_VOGUE_2017):
      self.STEER_MAX = 500
      self.STEER_DELTA_UP = 2
      self.STEER_DELTA_DOWN = 3
      self.STEER_DRIVER_ALLOWANCE = 20
      self.STEER_DRIVER_MULTIPLIER = 2
      self.STEER_DRIVER_FACTOR = 1
      self.STEER_THRESHOLD = 150
      self.STEER_STEP = 4  # 25 Hz
    """


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
class LandroverCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))

  def init_make(self, CP: CarParams):
    if CP.flags & LandroverFlags.FLEXRAY_HARNESS:
      self.footnotes.insert(0, Footnote.FLEXRAY)


"""
 LANDROVER VIN Code

 01234567890123456
 SALEA7AX8L2XXXXXX  : L663 2020 Defender 110
 SALEA8BW6P2XXXXXX  : L663 2023 Defender 130
 SALGA2FE0HAXXXXXX  : L405 2017 RangeRover Vogue

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
  L405 = "GA"  # RangeRover Vogue L405


class ModelYear(StrEnum):
  H_2017 = "H"
  L_2020 = "L"
  P_2023 = "P"


@dataclass
class LandroverPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: "landrover_rangerover_2017"})
  wmis: set[WMI] = field(default_factory=set)
  lines: set[ModelLine] = field(default_factory=set)
  years: set[ModelYear] = field(default_factory=set)


@dataclass
class LandroverFlexrayPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: "landrover_defender_2023"})
  wmis: set[WMI] = field(default_factory=set)
  lines: set[ModelLine] = field(default_factory=set)
  years: set[ModelYear] = field(default_factory=set)

  def init(self):
    self.flags |= LandroverFlags.FLEXRAY_HARNESS


class CAR(Platforms):
  LANDROVER_DEFENDER_2023 = LandroverFlexrayPlatformConfig(
    [
      LandroverCarDocs("LANDROVER DEFENDER 2023"),
    ],
    CarSpecs(mass=2550, wheelbase=3.022, steerRatio=19.0, minSteerSpeed=50*CV.KPH_TO_MS),
    wmis=(WMI.LANDROVER),
    lines={ModelLine.L663},
    years={ModelYear.L_2020, ModelYear.P_2023},
  )
  """
  RANGEROVER_VOGUE_2017 = LandroverPlatformConfig(
    [
      LandroverCarDocs("RANGEROVER VOGUE 2017"),
    ],
    CarSpecs(mass=2500, wheelbase=2.922, steerRatio=16.5),
    wmis=(WMI.LANDROVER),
    lines={ModelLine.L405},
    years={ModelYear.H_2017},
  )
  """


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


# addr: (bus, 1/freq*100, vl)
# 1D8 4694050000000582
# 1D8#46950500000008BC
# 3D4 800d0141e73ded00
STATIC_MSGS = [
 (0x1D8, 0, 10, b'\x46\x95\x05\x05\x00\x00\x08\xbc'),
]


FLEXRAY_CAR = CAR.with_flags(LandroverFlags.FLEXRAY_HARNESS)
EVA2_CARS = {CAR.LANDROVER_DEFENDER_2023, }

DBC = CAR.create_dbc_map()

if __name__ == "__main__":
  cars = []
  for platform in CAR:
    for doc in platform.config.car_docs:
      cars.append(doc.name)
  cars.sort()
  for c in cars:
    print(c)
