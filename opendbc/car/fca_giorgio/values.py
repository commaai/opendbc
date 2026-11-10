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
  # paramsd at the configured 136" wheelbase: 159" vans learn 20.2-20.5 on highway and ~21.2 in city driving, and a 136"
  # van learned ~18.4 in city driving, about 136/159 of the 159" value (the wheelbases share a steering rack).
  # 17.5 sits between the 136" estimates for highway (20.3 * 136/159 = 17.4) and city (~18.2).
  steerRatio: float = 17.5
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
    # steerRatio * 159 / 136 for a 159" (~20.5).
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
    # Data collection: the ADAS camera and ACC radar, at their Fiat node addresses (0x31, 0x2A).
    # Logging them from vans with and without ACC should show whether they can gate fingerprinting.
    Request(
      [StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.UDS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.fwdCamera, Ecu.fwdRadar],
      bus=0,
      logging=True,
    ),
  ],
  extra_ecus=[
    (Ecu.fwdCamera, 0x18da31f1, None),
    (Ecu.fwdRadar, 0x18da2af1, None),
  ],
)


DBC = CAR.create_dbc_map()
