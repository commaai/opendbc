from opendbc.car import structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.carstate import CarState

TransmissionType = structs.CarParams.TransmissionType

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.brand = 'psa'
    ret.dashcamOnly = False
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.4

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    if not docs:
      ret.transmissionType = TransmissionType.automatic
      ret.minEnableSpeed = 0
    ret.minSteerSpeed = 0.

    ret.centerToFront = ret.wheelbase * 0.44
    ret.wheelSpeedFactor = 1.04

    return ret