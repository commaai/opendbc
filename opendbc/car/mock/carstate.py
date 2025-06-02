from opendbc.car import structs
from opendbc.car.interfaces import CarStateBase


class CarState(CarStateBase):
  def update(self, *_) -> tuple[structs.CarState, structs.CarStateSP]:
    return structs.CarState(), structs.CarStateSP()
