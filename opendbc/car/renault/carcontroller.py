from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
  def update(self, CC, CS, now_nanos):
    # Dashcam-only port: no control messages are transmitted.
    return CC.actuators, []
