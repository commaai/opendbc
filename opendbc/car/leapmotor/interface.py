from opendbc.car import structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.leapmotor.carcontroller import CarController
from opendbc.car.leapmotor.carstate import CarState


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'leapmotor'

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.leapmotor)]
    # The camera's angle request (ADAS_LKA.STEER_ANGLE_REQ) leads the measured angle by 70-100 ms during LCC
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.15
    ret.steerLimitTimer = 0.4
    ret.steerAtStandstill = False

    ret.radarUnavailable = True
    ret.alphaLongitudinalAvailable = False

    return ret
