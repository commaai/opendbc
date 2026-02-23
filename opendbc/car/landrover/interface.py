from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.landrover.carcontroller import CarController
from opendbc.car.landrover.carstate import CarState
from opendbc.car.landrover.values import CAR


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "landrover"

    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 0.5

    ret.radarUnavailable = True
    ret.alphaLongitudinalAvailable = ret.radarUnavailable

    if alpha_long:
      ret.openpilotLongitudinalControl = True

    ret.steerControlType = structs.CarParams.SteerControlType.torque

    if ret.centerToFront == 0:
      ret.centerToFront = ret.wheelbase * 0.4

    if candidate in (CAR.RANGEROVER_VOGUE_2017):
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.steerActuatorDelay = 0.11
      ret.enableBsm = True

    elif candidate in (CAR.LANDROVER_DEFENDER_2023):
      ret.steerControlType = structs.CarParams.SteerControlType.angle
      ret.enableBsm = True

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.landrover, ret.flags)]

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:

    stock_cp.enableBsm = True
    stock_cp.radarUnavailable = False

    return ret
