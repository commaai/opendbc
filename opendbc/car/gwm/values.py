from collections import namedtuple
from dataclasses import dataclass, field

from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, structs
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts

Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])

class CarControllerParams:
  def __init__(self, CP):
    self.BUTTONS = [
      Button(structs.CarState.ButtonEvent.Type.setCruise, "STEER_AND_AP_STALK", "AP_ENABLE_COMMAND", [1]),
      # Button(structs.CarState.ButtonEvent.Type.resumeCruise, "STEER_AND_AP_STALK", "AP_ENABLE_COMMAND", [1]),
      Button(structs.CarState.ButtonEvent.Type.accelCruise, "STEER_AND_AP_STALK", "AP_INCREASE_SPEED_COMMAND", [1]),
      Button(structs.CarState.ButtonEvent.Type.decelCruise, "STEER_AND_AP_STALK", "AP_DECREASE_SPEED_COMMAND", [1]),
      Button(structs.CarState.ButtonEvent.Type.cancel, "STEER_AND_AP_STALK", "AP_CANCEL_COMMAND", [1]),
      Button(structs.CarState.ButtonEvent.Type.gapAdjustCruise, "STEER_AND_AP_STALK", "AP_INCREASE_DISTANCE_COMMAND", [1]),
    ]


class CANBUS:
  pt = 0
  cam = 2


@dataclass
class GwmPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {CANBUS.pt: 'gwm_haval_h6_mk3_generated'})


@dataclass(frozen=True, kw_only=True)
class GwmCarSpecs(CarSpecs):
  centerToFrontRatio: float = 0.45
  steerRatio: float = 15.6


@dataclass
class GwmCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


class CAR(Platforms):
  config: GwmPlatformConfig

  GWM_HAVAL_H6_PHEV_3RD_GEN = GwmPlatformConfig(
    [
      GwmCarDocs("GWM Haval H6 hybrid plug-in 2020-24"),
    ],
    GwmCarSpecs(mass=2050, wheelbase=2.74),
  )


"""
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # TODO:
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)
"""

DBC = CAR.create_dbc_map()