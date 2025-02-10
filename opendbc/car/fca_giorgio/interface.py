from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.fca_giorgio.values import CAR


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate: CAR, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "fca_giorgio"
    ret.radarUnavailable = True

    # Set global parameters

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.fcaGiorgio)]

    # Global lateral tuning defaults, can be overridden per-vehicle

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.1
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    # Global longitudinal tuning defaults, can be overridden per-vehicle

    ret.pcmCruise = not ret.openpilotLongitudinalControl

    return ret
