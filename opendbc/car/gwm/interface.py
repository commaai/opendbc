from opendbc.car import structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.gwm.values import CAR

class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "gwm"
    ret.radarUnavailable = True

    #ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.allOutput, 1)]

    ret.steerActuatorDelay = 0.2
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.pcmCruise = False

    return ret