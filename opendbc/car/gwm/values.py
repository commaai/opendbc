from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 1
  STEER_MAX = 200

  # TO-DO Cleanup this later
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    390, # deg
    ([0., 5., 25.], [2.5, 1.5, .2]),
    ([0., 5., 25.], [5., 2., .3]),
  )
  STEER_DRIVER_ALLOWANCE = 5  # Driver intervention threshold, 0.5 Nm
  # TO-DO Cleanup this later

  def __init__(self, CP: CarParams):
    self.ACCEL_MAX = 1
    self.ACCEL_MIN = -3.5
    self.STEER_DELTA_UP = 3.25
    self.STEER_DELTA_DOWN = 5.0


@dataclass
class GWMCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.psa_a]))


@dataclass
class GWMPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'gwm_haval_h6_mk3_generated',
    # Bus.pt: 'psa_aee2010_r3',
  })


class CAR(Platforms):
  GWM_HAVAL_H6 = GWMPlatformConfig(
    [GWMCarDocs("Peugeot 208 2019-25")],
    CarSpecs(mass=1530, wheelbase=2.54, steerRatio=17.6),
  )


# Placeholder, FW Query will be added in separate PR
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
