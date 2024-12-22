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
    ret.steerControlType = structs.CarParams.SteerControlType.angle # TODO: check angle or torque controlled
    ret.steerActuatorDelay = 0.2  # TODO: not measured
    ret.steerLimitTimer = 1.0

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    if not docs:
      ret.transmissionType = TransmissionType.automatic # TODO: implement manual
      ret.minEnableSpeed = 30 * CV.KPH_TO_MS  # 19 mph, TODO: 40kph/25mph for non-ACC variants
    ret.minSteerSpeed = 0.  # TODO: verify

    ret.autoResumeSng = ret.minEnableSpeed == -1 # TODO: check
    ret.centerToFront = ret.wheelbase * 0.44  # TODO: verify
    ret.wheelSpeedFactor = 1.04
    return ret