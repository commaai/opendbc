import re
from dataclasses import dataclass, field

from opendbc.car import Bus, CanBusBase, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, LiveFwVersions, OfflineFwVersions, Request, StdQueries

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

# 0xF181 responses share one layout: 01 01, a 10-byte field, 00, then one byte.
# EPS:   01 01 'S2FI04EB00' 00 0b
# ABS:   01 01 18 06 01 00 '      ' 00 01
# meter: 01 01 02 92 '        ' 00 05
# camera: 01 01 06 00 06 20 00 03 20 00 00 01 00 26
FW_VERSION_REGEX = br"\x01\x01[\x00-\xff]{10}\x00[\x00-\xff]"

EPS_ADDR = (0x18da30f1, None)
ABS_ADDR = (0x18da28f1, None)
CAMERA_ADDR = (0x18da31f1, None)


def match_fw_to_car_fuzzy(live_fw_versions: LiveFwVersions, vin: str, offline_fw_versions: OfflineFwVersions) -> set[str]:
  """Allows an unseen ABS version, as long as the EPS and camera versions are known and the ABS response is well-formed.

  EPS and camera versions must always be known: EPS is the ECU we command, and the camera tells vans with ADAS
  from vans without it. ABS is likely shared with vans that lack ADAS, so it never stands in for either.
  """
  live_abs = live_fw_versions.get(ABS_ADDR, set())
  if any(re.fullmatch(FW_VERSION_REGEX, v) is None for v in live_abs):
    return set()

  candidates = set()
  for candidate, fws in offline_fw_versions.items():
    known = {(addr, sub_addr): set(versions) for (_, addr, sub_addr), versions in fws.items()}
    if all(live_fw_versions.get(addr, set()) & known.get(addr, set()) for addr in (EPS_ADDR, CAMERA_ADDR)):
      candidates.add(candidate)
  return candidates


FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=FW_VERSION_REGEX,
  requests=[
    Request(
      [StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.UDS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.eps, Ecu.abs, Ecu.fwdCamera],
      bus=0,
    ),
  ],
  # ABS was added after EPS; older logs and occasional missed responses should still match exactly.
  # The camera stays essential, so a van without the ADAS camera doesn't fingerprint.
  non_essential_ecus={Ecu.abs: [CAR.RAM_PROMASTER]},
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)


DBC = CAR.create_dbc_map()
