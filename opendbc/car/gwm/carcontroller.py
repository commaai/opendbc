from opendbc.car.interfaces import CarControllerBase

class CarController(CarControllerBase):
  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    new_actuators = actuators
    can_sends = []

    return new_actuators, can_sends