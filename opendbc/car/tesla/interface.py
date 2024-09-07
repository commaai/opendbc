#!/usr/bin/env python3
from panda import Panda
from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.values import CAR


class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.carName = "tesla"
    # There is no safe way to do steer blending with user torque,
    # so the steering behaves like autopilot. This is not
    # how openpilot should be, hence dashcamOnly
    ret.dashcamOnly = False

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.25

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    if candidate in [CAR.TESLA_AP3_MODEL3, CAR.TESLA_AP3_MODELY]:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_TESLA_LONG_CONTROL
      ret.openpilotLongitudinalControl = True

    return ret
