from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 1

  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    390, # deg
    ([0., 5., 25.], [2.5, 1.5, .2]),
    ([0., 5., 25.], [5., 2., .3]),
  )
  STEER_DRIVER_ALLOWANCE = 5  # Driver intervention threshold, 0.5 Nm

  # T9 driver effort is only calibrated in raw units.
  T9_STEER_DRIVER_THRESHOLD_RAW = 5

  def __init__(self, CP):
    if CP.carFingerprint == CAR.PSA_PEUGEOT_308_T9:
      # No permitted actuation in the dashcam port. These normalized limits
      # do not describe the EPS torque scale or a vehicle calibration.
      self.STEER_MAX = 1
      self.STEER_DELTA_UP = 0
      self.STEER_DELTA_DOWN = 0


@dataclass
class PSACarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.psa_a]))


@dataclass
class PSAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'psa_aee2010_r3',
  })


class CAR(Platforms):
  PSA_PEUGEOT_208 = PSAPlatformConfig(
    [PSACarDocs("Peugeot 208 2019-25")],
    CarSpecs(mass=1530, wheelbase=2.54, steerRatio=17.6),
  )
  PSA_PEUGEOT_308_T9 = PSAPlatformConfig(
    [PSACarDocs("Peugeot 308 2018", package="Conventional cruise control", car_parts=CarParts())],
    # Preliminary geometry inherited from the local T9 research profile.
    # Mass and steering ratio require variant-specific validation before control.
    CarSpecs(mass=1300, wheelbase=2.62, steerRatio=15.0),
    dbc_dict={Bus.pt: 'psa_308_t9_2018'},
  )


# Placeholder, FW Query will be added in separate PR
FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"\d{9}",
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
