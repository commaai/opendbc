from opendbc.car import Bus, CarSpecs, PlatformConfig, Platforms
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

class CAR(Platforms):
  COMMA_BODY_V1 = PlatformConfig(
    [CarDocs("comma body", package="All", video="https://youtu.be/VT-i3yRsX2s?t=2736")],
    CarSpecs(mass=9, wheelbase=0.406, steerRatio=0.5, centerToFrontRatio=0.44),
    {Bus.main: 'comma_body'},
  )
  COMMA_BODY_V2 = PlatformConfig(
    [CarDocs("comma body", package="All")],
    CarSpecs(mass=9, wheelbase=0.406, steerRatio=0.5, centerToFrontRatio=0.44),
    {Bus.main: 'comma_body'},
  )


class CarControllerParams:
  def __init__(self, CP):
    # speed = RPM * (pi * diameter (~6.5 inches) / 60)
    self.SPEED_FROM_RPM = 0.008644

    self.MAX_POS_INTEGRATOR = 1

    # body v1 is torque control, body v2 is speed control
    if CP.carFingerprint in CAR.COMMA_BODY_V1:
      self.SPEED_FROM_RPM = self.SPEED_FROM_RPM / 16 # v1 firmware RPM is unscaled
      self.CONTROL_BUS = 0
      self.MAX_TORQUE = 700
      self.MAX_TORQUE_RATE = 70
      self.FLIP_Y = True # flip sign of differential wheel speed
      self.v_pid_settings = self.w_pid_settings = {
        "k_p": 83,
        "k_i": 73,
        "k_d": 12,
      }
    elif CP.carFingerprint in CAR.COMMA_BODY_V2:
      self.CONTROL_BUS = 2
      self.MAX_TORQUE = 1000
      self.MAX_TORQUE_RATE = 250
      self.FLIP_Y = False
      self.v_pid_settings = self.w_pid_settings = {
        "k_p": 1.27 * self.MAX_TORQUE_RATE,
        "k_i": 1.26 * self.MAX_TORQUE_RATE,
        "k_d": 0.12 * self.MAX_TORQUE_RATE,
      }


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
