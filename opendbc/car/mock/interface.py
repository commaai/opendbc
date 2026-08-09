#!/usr/bin/env python3
from opendbc.car import structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mock.carcontroller import CarController
from opendbc.car.mock.carstate import CarState
from opendbc.car.mock.values import CAR


# mocked car interface for dashcam mode
class MockInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  CAR = CAR
  BRAND = "mock"

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.lateralTuning.pid.kpBP = [0.]
    ret.lateralTuning.pid.kpV = [0.]
    ret.lateralTuning.pid.kiBP = [0.]
    ret.lateralTuning.pid.kiV = [0.]
    ret.dashcamOnly = True
    return ret
