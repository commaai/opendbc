from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.landrover.values import LandroverFlags, CAR, DBC


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.brand = "landrover"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.landrover, 0)]

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.12  # Default delay

    #CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True


    ret.experimentalLongitudinalAvailable = False

    ret.enableBsm = True


    ret.pcmCruise = True # managed by cruise state manager

    if ret.centerToFront == 0:
      ret.centerToFront = ret.wheelbase * 0.4

    return ret

