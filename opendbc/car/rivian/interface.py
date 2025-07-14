from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.rivian.carcontroller import CarController
from opendbc.car.rivian.carstate import CarState
from opendbc.car.rivian.radar_interface import RadarInterface
from opendbc.car.rivian.values import CAR, RivianSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "rivian"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.rivian)]

    # Enhanced lateral tuning for Gen2 Rivian vehicles
    if candidate == CAR.RIVIAN_R1_GEN2:
      ret.steerActuatorDelay = 0.12  # Improved latency for Gen2
      ret.steerLimitTimer = 0.35     # Reduced timer for better responsiveness
    else:
      ret.steerActuatorDelay = 0.15
      ret.steerLimitTimer = 0.4

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.radarUnavailable = True

    # Enhanced longitudinal support for Gen2
    if candidate == CAR.RIVIAN_R1_GEN2:
      ret.alphaLongitudinalAvailable = True  # Gen2 has improved longitudinal
      ret.longitudinalActuatorDelay = 0.25   # Better response time
    else:
      ret.alphaLongitudinalAvailable = False
      ret.longitudinalActuatorDelay = 0.35

    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= RivianSafetyFlags.LONG_CONTROL.value

    ret.vEgoStopping = 0.25
    ret.stopAccel = 0

    return ret
