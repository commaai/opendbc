#!/usr/bin/env python3
from panda import Panda
from opendbc.car import structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.values import CAR, PLATFORM_3Y

class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.brand = "tesla"

    # Needs safety validation and final testing before pulling out of dashcam
    # ret.dashcamOnly = True

    flags = Panda.FLAG_TESLA_LONG_CONTROL
    if candidate in PLATFORM_3Y:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla, flags)]
    else:
      flags |= Panda.FLAG_TESLA_RAVEN
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.tesla, flags),
        get_safety_config(structs.CarParams.SafetyModel.tesla, flags | Panda.FLAG_TESLA_POWERTRAIN),
      ]

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.25
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = candidate in PLATFORM_3Y

    ret.experimentalLongitudinalAvailable = True
    if experimental_long or (candidate == CAR.TESLA_MODEL_S_RAVEN):
      ret.openpilotLongitudinalControl = True
      # ret.safetyConfigs[0].safetyParam |= Panda.FLAG_TESLA_LONG_CONTROL

    return ret
