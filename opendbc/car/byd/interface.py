from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.radar_interface import RadarInterface
from opendbc.car.byd.values import BydSafetyFlags, SAFETY_PLATFORM_PARAMS


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "byd"

    # LKS_ON matches first-boot persist. card.py clears/sets it from BydLksEnabled.
    # The platform bit selects this platform's steering/long limits in byd_init.
    safety_param = int(BydSafetyFlags.LKS_ON) | SAFETY_PLATFORM_PARAMS[candidate].safety_param
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd, safety_param)]

    ret.dashcamOnly = False
    ret.hudLaneFromModel = True
    ret.hudLeadShading = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.25
    ret.steerLimitTimer = 0.4
    ret.longitudinalActuatorDelay = 0.5

    ret.radarUnavailable = False

    ret.alphaLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = alpha_long and ret.alphaLongitudinalAvailable
    ret.pcmCruise = not ret.openpilotLongitudinalControl
    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= BydSafetyFlags.LONG_CONTROL.value

    return ret