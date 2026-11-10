from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig


class LkaState(IntEnum):
  OFF = 0
  STANDBY = 1
  READY = 2
  ACTIVE = 19


STEER_1_DT = 0.0096
CLUSTER_SPEED_FACTOR = 1.081

BUTTON_TAP_GAP_FRAMES = 3
ENGAGE_TAP_WINDOW_FRAMES = 70
ENGAGE_TAPS = 2


class CarControllerParams:
  STEER_STEP = 2
  STEER_MAX = 650
  STEER_MAX_LOOKUP = ([35. / 3.6, 37.5 / 3.6, 40. / 3.6, 45. / 3.6, 55. / 3.6, 60. / 3.6, 65. / 3.6, 70. / 3.6, 75. / 3.6,
                       77.5 / 3.6],
                      [650, 593, 555, 530, 530, 503, 480, 460, 443, 433])
  STEER_DELTA_UP = 15
  STEER_DELTA_DOWN = 60
  STEER_DRIVER_ALLOWANCE = 80
  STEER_DRIVER_MULTIPLIER = 2
  STEER_DRIVER_FACTOR = 1

  STEER_THRESHOLD = 100
  STEER_THRESHOLD_FAST = 140
  STEER_DRIVER_TORQUE_ZERO = 10

  def __init__(self, CP):
    pass

  def steer_max(self, v_ego_raw: float) -> int:
    return int(np.interp(v_ego_raw, *self.STEER_MAX_LOOKUP))


@dataclass
class LeapmotorCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class LeapmotorPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'leapmotor_t03_pt',
  })


class CAR(Platforms):
  LEAPMOTOR_T03 = LeapmotorPlatformConfig(
    [LeapmotorCarDocs("Leapmotor T03 2022")],
    CarSpecs(wheelbase=2.40, steerRatio=19.2, tireStiffnessFactor=0.87),
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[],
  fw_version_regex=b"",
)


DBC = CAR.create_dbc_map()
