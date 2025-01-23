from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
  def update(self, CC, CC_SP, CS, now_nanos):
    return CC.actuators.as_builder(), []
