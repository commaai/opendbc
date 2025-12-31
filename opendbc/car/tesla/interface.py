from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, CAR


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "tesla"

    """
    Comparing:
    - FSD 13: 2c912ca5de3b1ee9/000002f7--e9a4e26504
    - FSD 14: 0f79c454f812791a/000000b6--8c7866d3bf
    Differences between FSD 13 and 14:
      - 0x51: 13 is 3 bytes, 14 is 8 bytes
      - 0x20E: doesn't exist on 13
      - 0x248: doesn't exist on 13
      - 0x3E9: doesn't exist on 13
      - 0x40C: doesn't exist on 13
      - 0x489: doesn't exist on 13
      - EPAS3S_sysStatus: bit 52 is 0 on 13, 1 on 14
    """

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

      ret.vEgoStopping = 0.1
      ret.vEgoStarting = 0.1
      ret.stoppingDecelRate = 0.3

    ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X) # dashcam only, pending find invalidLkasSetting signal

    return ret
