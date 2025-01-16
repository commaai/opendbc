#!/usr/bin/env python3
from panda import Panda
from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.values import CAR

class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.carName = "tesla"

    # Needs safety validation and final testing before pulling out of dashcam
    ret.dashcamOnly = False

    # Not merged yet
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]
    ret.safetyConfigs[0].safetyParam |= Panda.FLAG_TESLA_LONG_CONTROL

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.25

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True
    ret.startAccel = 0.16
    ret.stopAccel = -0.52

    ret.openpilotLongitudinalControl = True

    return ret
