from opendbc.car import car
from opendbc.car.interfaces import CarStateBase


class CarState(CarStateBase):
  def update(self, *_) -> car.CarState:
    return car.CarState.new_message()
