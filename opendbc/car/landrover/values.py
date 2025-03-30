from dataclasses import dataclass, field
from enum import Enum, IntFlag
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, AngleSteeringLimits
from opendbc.car.structs import CarParams
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class CarControllerParams:
  ACCEL_MAX = 2.0 # m/s
  ACCEL_MIN = -3.5 # m/s
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    360,  # deg
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
    Column.MODEL, shop_footnote=False)

@dataclass
class LandroverCarDocsDefender(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.FLEXRAY])



@dataclass
class LandroverPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: "landrover_defender_2023" }
  )


class CAR(Platforms):
  LANDROVER_DEFENDER_2023 = LandroverPlatformConfig(
    [
      LandroverCarDocsDefender("LANDROVER DEFENDER 2023"),
    ],
    CarSpecs(mass=2550, wheelbase=3.0, steerRatio=19.0, minSteerSpeed=51*CV.KPH_TO_MS),
  )



class Buttons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  GAP_DIST = 3
  CANCEL = 4  # on newer models, this is a pause/resume button



FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
      obd_multiplexing=True,
    )
  ]
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
