import math
from opendbc.car import get_safety_config, structs
from opendbc.car.pinball.carcontroller import CarController
from opendbc.car.pinball.carstate import CarState
from opendbc.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.notCar = True
    ret.brand = "pinball"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.allOutput)]

    ret.minSteerSpeed = -math.inf
    ret.maxLateralAccel = math.inf
    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.

    ret.radarUnavailable = True
    ret.openpilotLongitudinalControl = True
    ret.steerControlType = structs.CarParams.SteerControlType.angle

    return ret
