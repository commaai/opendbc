from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.fca_giorgio.carcontroller import CarController
from opendbc.car.fca_giorgio.carstate import CarState
from opendbc.car.fca_giorgio.values import CAR, CarControllerParams


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate: CAR, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "fca_giorgio"
    ret.radarUnavailable = True

    # Set global parameters

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.fcaGiorgio)]

    # Global lateral tuning defaults, can be overridden per-vehicle

    ret.steerLimitTimer = 1.0
    # lagd learned a total lateral delay of 0.48 s (11 blocks, route 57a1cc884d1e5a8b|0000000b--19a8aadecc), minus the
    # 0.2 s that openpilot adds
    ret.steerActuatorDelay = 0.28

    # PID on steering angle rather than the torque controller: the steady torque this van needs tracks steering angle
    # times speed, not lateral accel, so no single lateral-accel-to-torque factor fits all speeds (about 6 below
    # 40 mph, 17-18 above 50 mph). The feedforward and kp both follow the measured steering stiffness of
    # 0.1 * v + 0.35 counts per degree (v in m/s). The RAM_PROMASTER entry in torque_data/override.toml is kept for
    # MAX_LAT_ACCEL_MEASURED.
    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kf = 1.
    ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kpV = [0., 30.], [0.35 / CarControllerParams.STEER_MAX, 3.35 / CarControllerParams.STEER_MAX]
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kiV = [0.], [0.003]

    # Global longitudinal tuning defaults, can be overridden per-vehicle

    ret.pcmCruise = not ret.openpilotLongitudinalControl

    return ret

  @staticmethod
  def get_steer_feedforward_promaster(desired_angle, v_ego):
    # measured steady LKA torque per degree of steering angle, rising with speed; normalized to STEER_MAX
    return desired_angle * (0.1 * v_ego + 0.35) / CarControllerParams.STEER_MAX

  def get_steer_feedforward_function(self):
    return self.get_steer_feedforward_promaster
