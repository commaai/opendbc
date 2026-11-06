from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.fca_giorgio.carcontroller import CarController
from opendbc.car.fca_giorgio.carstate import CarState
from opendbc.car.fca_giorgio.values import CAR


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
    ret.steerActuatorDelay = 0.18  # learned total lateral delay ~0.385s (minus 0.2 added by lagd); 1 valid block only
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    # Global longitudinal tuning defaults, can be overridden per-vehicle

    ret.pcmCruise = not ret.openpilotLongitudinalControl

    return ret
