#!/usr/bin/env python3
from opendbc.car import structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.carName = "carbage"

    # it's model 29 in panda, it's the wrong name here
    # don't want have another custom submodule
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.volkswagenMqbEvo)]

    #CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15, 0.30], [0.03, 0.05]]
    ret.lateralTuning.pid.kf = 0.00006

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.12

    # TODO: fix radar interface
    ret.radarUnavailable = True

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.openpilotLongitudinalControl = True

    return ret
