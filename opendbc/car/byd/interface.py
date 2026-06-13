from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.values import CAR


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "byd"
    ret.dashcamOnly = True
    ret.radarUnavailable = True
    ret.alphaLongitudinalAvailable = False

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd)]
    ret.steerControlType = structs.CarParams.SteerControlType.angle

    if candidate == CAR.BYD_ATTO_3:
      ret.steerActuatorDelay = 0.2
      ret.steerLimitTimer = 0.4

    elif candidate == CAR.BYD_SEALION_7:
      ret.steerActuatorDelay = 0.1
      ret.steerLimitTimer = 0.4

    return ret
