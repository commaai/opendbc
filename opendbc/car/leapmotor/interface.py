from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.leapmotor.carcontroller import CarController
from opendbc.car.leapmotor.carstate import CarState


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "leapmotor"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.leapmotor)]

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.22
    ret.steerLimitTimer = 0.8
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.radarUnavailable = True
    ret.pcmCruise = False
    ret.alphaLongitudinalAvailable = False
    ret.openpilotLongitudinalControl = False

    return ret
