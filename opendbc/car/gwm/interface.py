from opendbc.car import structs, get_safety_config, CanBusBase
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.gwm.carcontroller import CarController
from opendbc.car.gwm.carstate import CarState

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'gwm'

    cfgs = [get_safety_config(structs.CarParams.SafetyModel.gwm)]

    # If multipanda mapping is detected (offset >= 4), keep the first safety slot
    # as `noOutput` so an internal panda remains silent and the vehicle safety config
    # stays as the last entry (`-1`). This enables external panda to control the vehicle.
    CAN = CanBusBase(None, fingerprint)
    if CAN.offset >= 4:
      cfgs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))

    ret.safetyConfigs = cfgs

    ret.dashcamOnly = True
    ret.dashcamOnly = False

    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = False

    return ret