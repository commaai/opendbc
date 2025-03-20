import numpy as np
from opendbc.car import get_safety_config, structs, get_friction
from opendbc.car.interfaces import CarInterfaceBase, TorqueFromLateralAccelCallbackType, FRICTION_THRESHOLD, LatControlInputs, NanoFFModel
from opendbc.car.rivian.carcontroller import CarController
from opendbc.car.rivian.carstate import CarState
from opendbc.car.rivian.radar_interface import RadarInterface
from opendbc.car.rivian.values import CarControllerParams


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  def get_steer_max(self, v_ego: float) -> int:
    return round(np.interp(v_ego, CarControllerParams.STEER_MAXES[0], CarControllerParams.STEER_MAXES[1]))

  # def torque_from_lateral_accel_rivian(self, latcontrol_inputs: LatControlInputs, torque_params: structs.CarParams.LateralTorqueTuning,
  #                                      lateral_accel_error: float, lateral_accel_deadzone: float, friction_compensation: bool, gravity_adjusted: bool) -> float:
  #   # The default is a linear relationship between torque and lateral acceleration (accounting for road roll and steering friction)
  #   friction = get_friction(lateral_accel_error, lateral_accel_deadzone, FRICTION_THRESHOLD, torque_params, friction_compensation)
  #   return (latcontrol_inputs.lateral_acceleration / float(torque_params.latAccelFactor)) + friction
  #
  # def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
  #   return self.torque_from_lateral_accel_rivian

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.brand = "rivian"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.rivian)]

    ret.steerActuatorDelay = 0.15
    ret.steerLimitTimer = 0.4
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.radarUnavailable = True

    # TODO: pending finding/handling missing set speed and fixing up radar parser
    ret.experimentalLongitudinalAvailable = False
    if experimental_long:
      ret.openpilotLongitudinalControl = True
      #ret.safetyConfigs[0].safetyParam |= Panda.FLAG_RIVIAN_LONG_CONTROL

    ret.longitudinalActuatorDelay = 0.35
    ret.vEgoStopping = 0.25
    ret.stopAccel = 0

    return ret
