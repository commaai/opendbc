from dataclasses import dataclass, field
from enum import Enum, IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, CarFootnote, CarHarness, CarParts, Column, WIKI, source, apply_remote_setup
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

  LKAS_MIN_TORQUE = 0.2             # Adequate torque when overriding without faulting
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

  def init_setup(self, guide):
    refs = [source(f"{self.make} installation reference", WIKI + 'nissan')]
    if self.setup_video:
      refs.append(source("Vehicle installation video", self.setup_video))
    guide["sources"][1:] = refs
    harness = next(p for p in self.car_parts.all_parts() if isinstance(p, CarHarness))
    remote = None
    if harness in (CarHarness.nissan_a, CarHarness.nissan_b):
      refs = [source("Nissan ADAS installation locations", WIKI + "nissan")]
      if self.model.startswith("Leaf"):
        remote = ("ADAS module behind the glovebox", [
          "Remove the glovebox to look for the ADAS module above it.",
          "Some Leaf variants place the module behind the instrument cluster instead. " +
          "Check the reference photos: other nearby connectors also fit the harness.",
        ], "ADAS module", "glovebox")
      elif self.model.startswith("Altima"):
        remote = ("ADAS module behind the glovebox", [
          "Look for the ADAS module behind the glovebox on the right side.",
          "Use the Nissan reference photos to identify the ADAS module before disconnecting its wiring.",
        ], "ADAS module", "glovebox")
      elif self.model.startswith("Rogue"):
        remote = ("ADAS module in the cargo area", [
          "Lift the luggage floor cover to expose the spare-tire area and locate the ADAS module.",
          "Check that your cable reaches the windshield before routing it behind trim.",
        ], "ADAS module", "cargo")
      else:
        remote = ("ADAS module", ["Locate the ADAS module using the Nissan installation reference for your model and market.",
                                    "The Nissan harness connects to this separate module, not the windshield camera."], "ADAS module", "module")
    if remote:
      apply_remote_setup(guide, harness, *remote, refs)


@dataclass(frozen=True)
class NissanCarSpecs(CarSpecs):
  centerToFrontRatio: float = 0.44
  steerRatio: float = 17.


@dataclass
class NissanPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'nissan_x_trail_2017_generated'})


class CAR(Platforms):
  NISSAN_XTRAIL = NissanPlatformConfig(
    [NissanCarDocs("Nissan X-Trail 2017")],
    NissanCarSpecs(wheelbase=2.705)
  )
  NISSAN_LEAF = NissanPlatformConfig(
    [NissanCarDocs("Nissan Leaf 2018-23", video="https://youtu.be/vaMbtAh_0cY")],
    NissanCarSpecs(wheelbase=2.705),
    {Bus.pt: 'nissan_leaf_2018_generated'},
  )
  # Leaf with ADAS ECU found behind instrument cluster instead of glovebox
  # Currently the only known difference between them is the inverted seatbelt signal.
  NISSAN_LEAF_IC = NISSAN_LEAF.override(car_docs=[])
  NISSAN_ROGUE = NissanPlatformConfig(
    [NissanCarDocs("Nissan Rogue 2018-20")],
    NissanCarSpecs(wheelbase=2.705)
  )
  NISSAN_ALTIMA = NissanPlatformConfig(
    [NissanCarDocs("Nissan Altima 2019-24", car_parts=CarParts.common([CarHarness.nissan_b]))],
    NissanCarSpecs(wheelbase=2.824)
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

# TODO: once we gather enough Altima data on PT bus (1), we can remove OBD queries to speed up fingerprinting
FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"(?:[A-Z0-9]{10}|[\x00-\xff]{24})",
  requests=[request for bus, obd_multiplexing in ((0, False), (1, False), (1, True)) for request in [
    Request(
      [NISSAN_DIAGNOSTIC_REQUEST_KWP, NISSAN_VERSION_REQUEST_KWP],
      [NISSAN_DIAGNOSTIC_RESPONSE_KWP, NISSAN_VERSION_RESPONSE_KWP],
      bus=bus,
      logging=obd_multiplexing,
      obd_multiplexing=obd_multiplexing,
    ),
    Request(
      [NISSAN_DIAGNOSTIC_REQUEST_KWP, NISSAN_VERSION_REQUEST_KWP],
      [NISSAN_DIAGNOSTIC_RESPONSE_KWP, NISSAN_VERSION_RESPONSE_KWP],
      rx_offset=NISSAN_RX_OFFSET,
      bus=bus,
      logging=obd_multiplexing,
      obd_multiplexing=obd_multiplexing,
    ),
    # Rogue's engine solely responds to this
    Request(
      [NISSAN_DIAGNOSTIC_REQUEST_KWP_2, NISSAN_VERSION_REQUEST_KWP],
      [NISSAN_DIAGNOSTIC_RESPONSE_KWP_2, NISSAN_VERSION_RESPONSE_KWP],
      bus=bus,
      logging=obd_multiplexing,
      obd_multiplexing=obd_multiplexing,
    ),
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      rx_offset=NISSAN_RX_OFFSET,
      bus=bus,
      logging=obd_multiplexing,
      obd_multiplexing=obd_multiplexing,
    ),
    # Some newer Altima engines respond at normal rx offset
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=bus,
      logging=obd_multiplexing,
      obd_multiplexing=obd_multiplexing,
    ),
  ]],
)
