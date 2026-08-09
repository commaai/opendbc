from opendbc.car import structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.carstate import CarState
from opendbc.car.psa.fingerprints import FW_VERSIONS
from opendbc.car.psa.values import FW_QUERY_CONFIG, CAR

TransmissionType = structs.CarParams.TransmissionType


class PSAInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  CAR = CAR
  BRAND = "psa"
  FW_QUERY_CONFIG = FW_QUERY_CONFIG
  FW_VERSIONS = FW_VERSIONS

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    ret.dashcamOnly = True

    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = False

    return ret