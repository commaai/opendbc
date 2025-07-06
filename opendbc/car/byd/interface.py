#!/usr/bin/env python3
from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "byd"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd)]

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerLimitTimer = 0.1              # time before steerLimitAlert is issued
    ret.steerActuatorDelay = 0.1          # Steering wheel actuator delay in seconds

    ret.steerAtStandstill = False
    ret.radarUnavailable = True
    ret.centerToFront = ret.wheelbase * 0.44

    ret.openpilotLongitudinalControl = True
    ret.longitudinalTuning.kpBP = [0., 5., 20.]
    ret.longitudinalTuning.kiBP = [0., 5., 20.]
    ret.longitudinalTuning.kpV = [2.2, 2.0, 1.8]
    ret.longitudinalTuning.kiV = [0.45, 0.40, 0.32]

    ret.wheelSpeedFactor = 0.695

    ret.stoppingDecelRate = 0.1 # reach stopping target smoothly

    return ret
