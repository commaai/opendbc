from dataclasses import dataclass, field

from opendbc.car import Bus, CanBusBase, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

class CarControllerParams:
  STEER_STEP = 1
  HUD_2_STEP = 25
  HUD_3_STEP = 100

  STEER_MAX = 300
  STEER_THRESHOLD = 140
  STEER_DRIVER_ALLOWANCE = 80
  STEER_DRIVER_MULTIPLIER = 2
  STEER_DRIVER_FACTOR = 1  # from dbc
  STEER_DELTA_UP = 3  # stock LKA ramps at up to 3/frame
  STEER_DELTA_DOWN = 3
  STEER_RELEASE_HOLD_FRAMES = 3  # frames to keep LKA_ACTIVE at zero torque before releasing

  def __init__(self, CP):
    pass


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)

  @property
  def pt(self) -> int:
    return self.offset

  @property
  def cam(self) -> int:
    return self.offset + 2


@dataclass
class FcaGiorgioPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'fca_giorgio'})


@dataclass(frozen=True, kw_only=True)
class FcaGiorgioCarSpecs(CarSpecs):
  centerToFrontRatio: float = 0.45
  # paramsd learned 19.7 at wheelbase 3.45 on a 159" (4.04 m) Thor Tellaro 20K. It fits steerRatio * wheelbase,
  # so that van's ratio is 19.7 * 3.45 / 4.04 = 16.8, which also applies at 136" if the steering rack is shared.
  steerRatio: float = 16.8
  # minSteerSpeed: float = 13.8 # m/s


@dataclass
class FcaGiorgioCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.vw_a]))


class CAR(Platforms):
  config: FcaGiorgioPlatformConfig

  # ALFA_ROMEO_STELVIO_1ST_GEN = FcaGiorgioPlatformConfig(
  #   [FcaGiorgioCarDocs("Alfa Romeo Stelvio 2017-24")],
  #   FcaGiorgioCarSpecs(wheelbase=2.82),
  # )

  RAM_PROMASTER = FcaGiorgioPlatformConfig(
    [FcaGiorgioCarDocs("Ram ProMaster 2022-26")],
    # 136" wheelbase. Other wheelbases are left to paramsd, which should learn about
    # steerRatio * wheelbase / 136 if the steering rack is shared (~19.7 for 159").
    FcaGiorgioCarSpecs(wheelbase=3.45),
  )

FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"\x01\x01[A-Z0-9-]{10}\x00\x0b",
  requests=[
    Request(
      [StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.UDS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.eps],
      bus=0,
    ),
  ],
)


DBC = CAR.create_dbc_map()
