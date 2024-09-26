#!/usr/bin/env python3
from opendbc.car import car
from opendbc.car.interfaces import CarInterfaceBase


# mocked car interface for dashcam mode
class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret: car.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> car.CarParams:
    ret.carName = "mock"
    ret.mass = 1700.
    ret.wheelbase = 2.70
    ret.centerToFront = ret.wheelbase * 0.5
    ret.steerRatio = 13.
    ret.dashcamOnly = True
    return ret
