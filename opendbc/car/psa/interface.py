from opendbc.car import structs
from opendbc.car.common.conversions  import Conversions as CV
from opendbc.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.values import CAR

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret: structs.CarParams, candidate: CAR, fingerprint, car_fw, experimental_long, docs):
    ret.carName = 'psa'
    ret.dashcamOnly = False

    ret.radarUnavailable = True
    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.2  # TODO: measure

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    if not docs:
      ret.transmissionType = TransmissionType.automatic # TODO: implement manual
      ret.minEnableSpeed = 30 * CV.KPH_TO_MS  # 19 mph, TODO: 40kph/25mph for non-ACC variants
    ret.minSteerSpeed = 0.  # TODO: verify

    ret.autoResumeSng = ret.minEnableSpeed == -1 # TODO: check
    ret.centerToFront = ret.wheelbase * 0.44  # TODO: verify
    ret.wheelSpeedFactor = 1.04

    if False: # TODO: check: False: CC torque based, True: ACC angle based?
      ret.steerControlType = structs.CarParams.SteerControlType.angle
    else:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate == CAR.PSA_OPEL_CORSA_F:
      ret.lateralTuning.init('pid') # TODO: tune
      ret.lateralTuning.pid.kf = 0.00003
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0., 20.], [0., 20.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.0025, 0.1], [0.00025, 0.01]]
    else:
      raise ValueError(f"unknown car: {candidate}")

    return ret