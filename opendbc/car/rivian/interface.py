#!/usr/bin/env python3
from panda import Panda
from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.rivian.values import CAR


class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.carName = "rivian"
    # There is no safe way to do steer blending with user torque,
    # so the steering behaves like autopilot. This is not
    # how openpilot should be, hence dashcamOnly
    ret.dashcamOnly = False

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.rivian)]

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.25

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    # ret.safetyConfigs[0].safetyParam |= Panda.FLAG_RIVIAN_LONG_CONTROL
    ret.openpilotLongitudinalControl = False

    return ret
