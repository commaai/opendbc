import math
from opendbc.car import get_safety_config, structs
from opendbc.car.body.carcontroller import CarController
from opendbc.car.body.carstate import CarState
from opendbc.car.body.values import SPEED_FROM_RPM, BodySafetyFlags, CAR
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.can_definitions import CanData

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.notCar = True
    ret.brand = "body"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.body)]
    if candidate == CAR.COMMA_BODY_V2:
      ret.safetyConfigs[0].safetyParam |= BodySafetyFlags.BODY_V2.value

    ret.minSteerSpeed = -math.inf
    ret.maxLateralAccel = math.inf  # TODO: set to a reasonable value
    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.

    ret.wheelSpeedFactor = SPEED_FROM_RPM

    ret.radarUnavailable = True
    ret.openpilotLongitudinalControl = True
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    return ret

  def update(self, can_packets: list[tuple[int, list[CanData]]]) -> structs.CarState:
    ret = super().update(can_packets)
    ret.canValid = True
    ret.canTimeout = False
    return ret
