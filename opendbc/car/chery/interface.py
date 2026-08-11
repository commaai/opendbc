from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.chery.carcontroller import CarController
from opendbc.car.chery.carstate import CarState
from opendbc.car.chery.radar_interface import RadarInterface
from opendbc.car.chery.values import (
  CAR,
  CHERY_ICAUR_SAFETY_PARAM,
  CHERY_OMODA_NO_TORQUE_SPOOF_PARAM,
  CHERY_OMODA_SAFETY_PARAM,
  ICAUR_DISABLE_TORQUE_SPOOF,
  OMODA_DISABLE_TORQUE_SPOOF,
)


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    del fingerprint, car_fw, alpha_long, is_release, docs
    ret.brand = "chery"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.chery)]
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerLimitTimer = 0.6
    ret.steerActuatorDelay = 0.01
    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.9871
    ret.wheelSpeedFactor = 0.832
    ret.openpilotLongitudinalControl = False
    ret.radarUnavailable = True
    ret.enableBsm = True
    if candidate == CAR.CHERY_OMODA_5:
      ret.safetyConfigs[0].safetyParam = CHERY_OMODA_SAFETY_PARAM
      if OMODA_DISABLE_TORQUE_SPOOF:
        ret.safetyConfigs[0].safetyParam |= CHERY_OMODA_NO_TORQUE_SPOOF_PARAM
    elif candidate == CAR.CHERY_ICAUR_03:
      ret.safetyConfigs[0].safetyParam = CHERY_ICAUR_SAFETY_PARAM
      if ICAUR_DISABLE_TORQUE_SPOOF:
        ret.safetyConfigs[0].safetyParam |= CHERY_OMODA_NO_TORQUE_SPOOF_PARAM
      # GPS origin-fit on 2026-07-13--04-13-59: 13-bit DBC raw (scale 1) × ~0.01756 ≈ gpsLocation.speed
      # (supersedes 8-bit×0.55; 0.55/32=0.0171875 was a close prior guess).
      ret.wheelSpeedFactor = 0.01756
      # No BSM signals reverse-engineered yet.
      ret.enableBsm = False
    return ret
