#!/usr/bin/env python3
from opendbc.car import structs, get_safety_config
from opendbc.car.tesla.values import CAR
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.values import TeslaFlags

SteerControlType = structs.CarParams.SteerControlType

class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.carName = "tesla"

    # There is no safe way to do steer blending with user torque,
    # so the steering behaves like autopilot. This is not
    # how openpilot should be, hence dashcamOnly
    ret.dashcamOnly = False

    ret.steerControlType = SteerControlType.angle

    ret.longitudinalActuatorDelay = 0.5 # s
    ret.radarUnavailable = True

    if candidate in [CAR.TESLA_AP3_MODEL3, CAR.TESLA_AP3_MODELY]:
      flags = TeslaFlags.FLAG_TESLA_MODEL3_Y
      flags |= TeslaFlags.FLAG_TESLA_LONG_CONTROL
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla, flags)]

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.25
    return ret
