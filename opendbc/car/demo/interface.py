from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.demo.carcontroller import CarController
from opendbc.car.demo.carstate import CarState
from opendbc.car.demo.values import CanBus, CAR


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate: CAR, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    CAN = CanBus(fingerprint=fingerprint)
    ret.brand = "demo"
    ret.radarUnavailable = True
    # ret.dashcamOnly = True

    # FIXME: The allOutput safety mode is for very early development only
    safety_configs = [get_safety_config(structs.CarParams.SafetyModel.allOutput, 1)]
    # TODO: just always use the demo mode and set it up for initial bidirectional forwarding
    # safety_configs = [get_safety_config(structs.CarParams.SafetyModel.demo)]

    # NOTE: Identify the BSM CAN message ID, if applicable, and set it here
    #  ret.enableBsm = 0x30F in fingerprint[0]

    ret.steerActuatorDelay = 0.1
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if CAN.pt >= 4:
      safety_configs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))
    ret.safetyConfigs = safety_configs

    return ret
