#!/usr/bin/env python3
from opendbc.car import structs
from opendbc.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.perodua.values import CAR


class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.carName = "perodua"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.perodua)]
    ret.safetyConfigs[0].safetyParam = 1
    ret.transmissionType = structs.CarParams.TransmissionType.automatic
    ret.radarOffCan = True
    ret.enableApgs = False                 # advanced parking guidance system
    ret.enableDsu = False                  # driving support unit

    ret.steerRateCost = 0.7                # Lateral MPC cost on steering rate, higher value = sharper turn
    ret.steerLimitTimer = 0.1              # time before steerLimitAlert is issued
    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.steerActuatorDelay = 0.48          # Steering wheel actuator delay in seconds

    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.longitudinalTuning.kpV = [0.9, 0.8, 0.8]

    ret.enableGasInterceptor = 0x201 in fingerprint[0] or 0x401 in fingerprint[0]
    ret.openpilotLongitudinalControl = True

    if candidate == CAR.MYVI_PSD:
      ret.wheelbase = 2.5
      ret.steerRatio = 17.44
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.9871
      ret.mass = 1025. + STD_CARGO_KG
      ret.wheelSpeedFactor = 1.31 # if it's still braking too much, can lower to 1.283

      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.12], [0.20]]
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [255]]
      ret.lateralTuning.pid.kf = 0.00012

      ret.longitudinalTuning.kpBP = [0., 5., 20., 30.]
      ret.longitudinalTuning.kpV = [0.5, 0.5, 0.4, 0.3]
      ret.longitudinalTuning.kiBP = [5, 7, 20, 30]
      ret.longitudinalTuning.kiV = [0.11, 0.1, 0.08, 0.07]
      ret.longitudinalActuatorDelayLowerBound = 0.32
      ret.longitudinalActuatorDelayUpperBound = 0.40
      ret.speedControlled = True

    else:
      ret.dashcamOnly = True
      ret.safetyModel = structs.CarParams.SafetyModel.noOutput

    ret.minEnableSpeed = -1
    ret.steerActuatorDelay = 0.30           # Steering wheel actuator delay in seconds
    ret.enableBsm = True
    ret.stoppingDecelRate = 0.25 # reach stopping target smoothly

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(
      ret.mass, ret.wheelbase, ret.centerToFront, tire_stiffness_factor=tire_stiffness_factor)

    return ret
