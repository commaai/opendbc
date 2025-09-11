from dataclasses import dataclass, field
from enum import Enum, IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, CarFootnote, CarHarness, CarParts, Column
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    # When output steering Angle not within range -1311 and 1310,
    #   CANPacker packs wrong angle output to be decoded by panda
    600,  # deg, reasonable limit
    ([0., 5., 15.], [5., .8, .15]),
    ([0., 5., 15.], [5., 3.5, 0.4]),
  )

  LKAS_MAX_TORQUE = 1               # A value of 1 is easy to overpower
  STEER_THRESHOLD = 1.0

  def __init__(self, CP):
    pass


class NissanSafetyFlags(IntFlag):
  ALT_EPS_BUS = 1


class Footnote(Enum):
  SETUP = CarFootnote(
    "See more setup details for <a href=\"https://github.com/commaai/openpilot/wiki/nissan\" target=\"_blank\">Nissan</a>.",
    Column.MAKE, setup_note=True)


@dataclass
class NissanCarDocs(CarDocs):
  package: str = "ProPILOT Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.nissan_a]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.SETUP])


@dataclass(frozen=True)
class NissanCarSpecs(CarSpecs):
  centerToFrontRatio: float = 0.44
  steerRatio: float = 17.


@dataclass
class NissanPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'nissan_x_trail_2017_generated'})


class CAR(Platforms):
  # Wheelbase validated based off owner's manual, 
  # Mass based off Nissan spec spreadsheet, averaging MY 2017 4WD curb weight ranges
  # https://uk.nissannews.com/en-GB/releases/release-576be476b0162b157afef7b4ac01f1be/download
  NISSAN_XTRAIL = NissanPlatformConfig(
    [NissanCarDocs("Nissan X-Trail 2017")],
    NissanCarSpecs(mass=1675, wheelbase=2.705)
  )

  # Wheelbase validated based off owner's manual, mass based off spec sheet for MY 2018
  # https://www.nissanusa.com/content/dam/Nissan/us/vehicle-brochures/2018/2018-leaf-brochure-en.pdf
  NISSAN_LEAF = NissanPlatformConfig(
    [NissanCarDocs("Nissan Leaf 2018-23", video="https://youtu.be/vaMbtAh_0cY")],
    NissanCarSpecs(mass=1594, wheelbase=2.700, steer_ratio=17.68, centerToFrontRatio=0.51),
    {Bus.pt: 'nissan_leaf_2018_generated'},
  )

  # Leaf with ADAS ECU found behind instrument cluster instead of glovebox
  # Currently the only known difference between them is the inverted seatbelt signal.
  NISSAN_LEAF_IC = NISSAN_LEAF.override(car_docs=[])

  # Wheelbase validated based off owner's manual
  # Mass based off Nissan spec sheet, averaging MY 2018 SL FWD and AWD curb weights
  # centerToFrontRatio calculated from averaged MY 2018 SL FWD and AWD curb front and rear axle weights
  # https://usa.nissannews.com/en-US/releases/us-2018-nissan-rogue-press-kit
  NISSAN_ROGUE = NissanPlatformConfig(
    [NissanCarDocs("Nissan Rogue 2018-20")],
    NissanCarSpecs(mass=1632, wheelbase=2.706, centerToFrontRatio=0.57)
  )

  # Wheelbase validated based off owner's manual
  # Mass based off spec sheet for MY 2019
  # centerToFrontRatio calculated from curb front and rear axle weights
  # https://usa.nissannews.com/en-US/releases/2019-nissan-altima-specifications
  NISSAN_ALTIMA = NissanPlatformConfig(
    [NissanCarDocs("Nissan Altima 2019-20", car_parts=CarParts.common([CarHarness.nissan_b]))],
    NissanCarSpecs(mass=1558, wheelbase=2.825, centerToFrontRatio=0.61)
  )


DBC = CAR.create_dbc_map()

# Default diagnostic session
NISSAN_DIAGNOSTIC_REQUEST_KWP = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, 0x81])
NISSAN_DIAGNOSTIC_RESPONSE_KWP = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, 0x81])

# Manufacturer specific
NISSAN_DIAGNOSTIC_REQUEST_KWP_2 = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, 0xda])
NISSAN_DIAGNOSTIC_RESPONSE_KWP_2 = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, 0xda])

NISSAN_VERSION_REQUEST_KWP = b'\x21\x83'
NISSAN_VERSION_RESPONSE_KWP = b'\x61\x83'

NISSAN_RX_OFFSET = 0x20

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[request for bus, logging in ((0, False), (1, True)) for request in [
    Request(
      [NISSAN_DIAGNOSTIC_REQUEST_KWP, NISSAN_VERSION_REQUEST_KWP],
      [NISSAN_DIAGNOSTIC_RESPONSE_KWP, NISSAN_VERSION_RESPONSE_KWP],
      bus=bus,
      logging=logging,
    ),
    Request(
      [NISSAN_DIAGNOSTIC_REQUEST_KWP, NISSAN_VERSION_REQUEST_KWP],
      [NISSAN_DIAGNOSTIC_RESPONSE_KWP, NISSAN_VERSION_RESPONSE_KWP],
      rx_offset=NISSAN_RX_OFFSET,
      bus=bus,
      logging=logging,
    ),
    # Rogue's engine solely responds to this
    Request(
      [NISSAN_DIAGNOSTIC_REQUEST_KWP_2, NISSAN_VERSION_REQUEST_KWP],
      [NISSAN_DIAGNOSTIC_RESPONSE_KWP_2, NISSAN_VERSION_RESPONSE_KWP],
      bus=bus,
      logging=logging,
    ),
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      rx_offset=NISSAN_RX_OFFSET,
      bus=bus,
      logging=logging,
    ),
  ]],
)
