from cereal import car
from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.carstate import CarState
from opendbc.car.volvo.radar_interface import RadarInterface
from opendbc.car.volvo.values import DBC

#ButtonType = car.CarState.ButtonEvent.Type
#EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  # Without this binding, CarInterfaceBase uses the default RadarInterfaceBase stub
  # which returns empty RadarData — liveTracks.points would stay at 0.
  RadarInterface = RadarInterface

  @staticmethod
  #def _get_params(ret, candidate: CAR, fingerprint, car_fw, experimental_long, docs):
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "volvo"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.volvo)]
    # ret.dashcamOnly = True

    # When Bus.radar is in DBC, RadarInterface decodes the Delphi ESR 2.5 directly
    # (64 tracks at 20Hz) instead of relying on stock FSM's single-lead ACC_Distance.
    ret.radarUnavailable = Bus.radar not in DBC[candidate]

    ret.steerControlType = car.CarParams.SteerControlType.angle

    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 0.8

    ret.openpilotLongitudinalControl = False

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
 car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    ret.intelligentCruiseButtonManagementAvailable = True
    return ret
