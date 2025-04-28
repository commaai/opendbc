from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import CAR, TeslaSafetyFlags, PLATFORM_3Y


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, docs) -> structs.CarParams:
    ret.brand = "tesla"

    # Needs safety validation and final testing before pulling out of dashcam
    # ret.dashcamOnly = True

    flags = 0
    if candidate in PLATFORM_3Y:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla, flags)]
    else:
      flags |= TeslaSafetyFlags.RAVEN.value
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.tesla, flags),
        get_safety_config(structs.CarParams.SafetyModel.tesla, flags | TeslaSafetyFlags.POWERTRAIN.value),
      ]

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = candidate in PLATFORM_3Y

    ret.alphaLongitudinalAvailable = True
    if alpha_long or (candidate == CAR.TESLA_MODEL_S_RAVEN):
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

      ret.vEgoStopping = 0.1
      ret.vEgoStarting = 0.1
      ret.stoppingDecelRate = 0.3

    return ret
