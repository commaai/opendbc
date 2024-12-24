#!/usr/bin/env python3
from panda import Panda
from opendbc.car import structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.values import PLATFORM_3Y

class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.carName = "tesla"

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

    # TODO: remove after adding back the radar parser
    ret.radarUnavailable = True

    ret.openpilotLongitudinalControl = True

    return ret
