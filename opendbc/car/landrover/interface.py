from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.landrover.carcontroller import CarController
from opendbc.car.landrover.carstate import CarState


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "landrover"

    #ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.allOutput)]
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.landrover)]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1  # Default delay

    #CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True


    ret.alphaLongitudinalAvailable = False
    if alpha_long:
      ret.openpilotLongitudinalControl = True


    ret.enableBsm = True


    ret.pcmCruise = True # managed by cruise state manager

    if ret.centerToFront == 0:
      ret.centerToFront = ret.wheelbase * 0.4

    return ret

