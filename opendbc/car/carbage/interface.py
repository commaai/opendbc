#!/usr/bin/env python3
from opendbc.car import structs
from opendbc.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.carName = "carbage"

    # Needs safety validation and final testing before pulling out of dashcam
    ret.dashcamOnly = True

    # Not merged yet
    #ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]
    #ret.safetyConfigs[0].safetyParam |= Panda.FLAG_TESLA_LONG_CONTROL

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.12

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.openpilotLongitudinalControl = True

    return ret
