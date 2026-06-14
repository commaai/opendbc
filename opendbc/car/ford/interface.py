from opendbc.car import structs, get_safety_config
from opendbc.car.ford.values import CAR, DBC
from opendbc.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_params(candidate, fingerprint=None, car_fw=None):
    ret = structs.CarParams()
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.ford)]
    
    if candidate == CAR.FORD_F150_2026:
      ret.safetyConfigs[0].safetyParam = 1  # TRON platform specific parameter
      ret.steerActuatorDelay = 0.1
      ret.steerLimitTimer = 0.8
      
    return ret