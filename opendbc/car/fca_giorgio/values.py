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
  STEER_DRIVER_ALLOWANCE = 80
  STEER_DRIVER_MULTIPLIER = 2
  STEER_DRIVER_FACTOR = 1  # from dbc
  STEER_DELTA_UP = 2
  STEER_DELTA_DOWN = 2

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
  steerRatio: float = 16
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
    [FcaGiorgioCarDocs("RAM ProMaster 2024-26")],
    # rough specs for Promaster 2500 with medium wheelbase
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
