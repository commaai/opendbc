#!/usr/bin/env python3
from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.values import BydSafetyFlags

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "byd"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd)]

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerLimitTimer = 0.4             # time before steerLimitAlert is issued
    ret.steerActuatorDelay = 0.1          # Steering wheel actuator delay in seconds

    ret.wheelSpeedFactor = 0.695
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= BydSafetyFlags.LONG_CONTROL.value
      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kiBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [1.2, 1.1, 1.0]
      ret.longitudinalTuning.kiV = [0.22, 0.2, 0.15]
      ret.stoppingDecelRate = 0.1 # reach stopping target smoothly

    return ret
