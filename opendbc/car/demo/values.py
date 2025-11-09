from dataclasses import dataclass, field

from opendbc.car import Bus, CanBusBase, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)

  @property
  def pt(self) -> int:
    # ADAS / Extended CAN, gateway side of the relay
    return self.offset

  @property
  def aux(self) -> int:
    return self.offset + 1

  @property
  def cam(self) -> int:
    return self.offset + 2


class CarControllerParams:
  STEER_STEP = 2                           # 50Hz
  LKA_HUD_STEP = 10                        # 10Hz

  STEER_MAX = 300                          # Placeholder to represent 3.00 Nm
  STEER_DELTA_UP = 4                       # Max reached in 1.50s (STEER_MAX / (50Hz * 1.50))
  STEER_DELTA_DOWN = 10                    # Min reached in 0.60s (STEER_MAX / (50Hz * 0.60))

  STEER_DRIVER_ALLOWANCE = 60              # Driver intervention threshold 0.6 Nm
  STEER_DRIVER_MULTIPLIER = 3              # weight driver torque heavily
  STEER_DRIVER_FACTOR = 1                  # from dbc

  def __init__(self, CP):
    pass


@dataclass
class DemoPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'demo'})


@dataclass(frozen=True, kw_only=True)
class DemoCarSpecs(CarSpecs):
  centerToFrontRatio: float = 0.45
  steerRatio: float = 15.6


@dataclass
class DemoCarDocs(CarDocs):
  package: str = "ACC & LKA"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


class CAR(Platforms):
  config: DemoPlatformConfig

  BATMOBILE = DemoPlatformConfig(
    [
      # DemoCarDocs("Wayne Enterprises Tumbler 2005", video="https://youtu.be/uLx6el5dCwA"),
    ],
    DemoCarSpecs(mass=1600, wheelbase=2.8),
  )

DBC = CAR.create_dbc_map()
