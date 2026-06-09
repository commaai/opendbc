from dataclasses import dataclass, field
from enum import IntEnum

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds, DT_CTRL
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16
from opendbc.car.structs import CarParams

from opendbc.can import CANDefine

Ecu = CarParams.Ecu

# Volvo EUCD/C1MCA CAN buses: pt (BCM/FSM/PSCM/SAS), chassis (DIM/PAM via CEM), ext (SODL/SODR via CEM)


class SteerDirection(IntEnum):
  """LKASteerDirection: EUCD needs 8-frame wait on direction change; C1MCA can use BOTH."""
  NONE = 0
  RIGHT = 1
  LEFT = 2
  BOTH = 3


class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(90,
    ([0., 5., 15.], [5., .8, .15]),
    ([0., 5., 15.], [5., 3.5, 0.4]),
  )

  STEER_TIMEOUT = 30 / DT_CTRL  # frames before steer fault on sustained 0-torque from EPS
  BLOCK_LEN = 8   # EUCD: frames to block steering on direction change (servo ignores otherwise)
  DEADZONE = 0.2  # deg: hold previous direction inside deadzone to avoid unwind
  BUTTON_BURST = 15  # CAN messages per button press to increase ECU acceptance probability

  ACCEL_MIN = -4.0  # m/s^2
  ACCEL_MAX = 2.0   # m/s^2

  def __init__(self, CP):
    pass

class CANBUS:
  pt = 0
  body = 1  # aux bus — carries Delphi ESR 2.5 radar (0x500..0x53F) and CEM dynamics
  cam = 2


# Delphi ESR 2.5 on aux bus (0x500–0x53F, 20Hz, 64 tracks); read directly so planner sees leads independently of stock FSM
RADAR_ESR = "ESR"

@dataclass
class VolvoEUCDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'volvo_v60_2015_pt',
    Bus.radar: RADAR_ESR,
  })


@dataclass
class VolvoCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control & Lane Keeping Aid"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass(frozen=True)
class VolvoCarSpecs(CarSpecs):
  steerRatio: float = 15.0
  centerToFrontRatio: float = 0.44
  minSteerSpeed: float = 1.0


class CAR(Platforms):
  VOLVO_V60 = VolvoEUCDPlatformConfig(
    [VolvoCarDocs("Volvo V60")],
    VolvoCarSpecs(mass=1750, wheelbase=2.776),
  )


VOLVO_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf1a2)
VOLVO_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(0xf1a2)

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [VOLVO_VERSION_REQUEST],
      [VOLVO_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
