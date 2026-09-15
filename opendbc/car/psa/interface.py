from opendbc.car import structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.carstate import CarState
from opendbc.car.psa.values import CAR

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'psa'

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    ret.dashcamOnly = True

    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = False

    if candidate == CAR.PSA_PEUGEOT_308_T9:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.noOutput)]
      # Factory captures suggest a torque API; no EPS calibration is supplied.
      # Keep an inert PID configuration for this observation-only interface.
      ret.steerControlType = structs.CarParams.SteerControlType.torque
      ret.lateralTuning.pid.kpBP = [0.]
      ret.lateralTuning.pid.kpV = [0.]
      ret.lateralTuning.pid.kiBP = [0.]
      ret.lateralTuning.pid.kiV = [0.]
      ret.steerAtStandstill = False
      ret.openpilotLongitudinalControl = False
      ret.autoResumeSng = False

    return ret
