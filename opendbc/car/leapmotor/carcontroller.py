from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.leapmotor import leapmotorcan
from opendbc.car.leapmotor.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel


def get_safety_CP():
  # the safety model's vehicle-model limits are computed for this platform
  from opendbc.car.leapmotor.interface import CarInterface
  return CarInterface.get_non_essential_params("LEAPMOTOR_B10")


class CarController(CarControllerBase):
  """Replaces the camera's ADAS_LKA (0x83) with a copy carrying openpilot's angle request, and cancels the
  stock ACC through the camera's copy of POWERTRAIN_STATE when openpilot disengages."""

  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.apply_angle_last = 0.
    self.VM = VehicleModel(get_safety_CP())
    self.adas_lka_idle = leapmotorcan.decode_adas_lka_idle()

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    if self.frame % CarControllerParams.STEER_STEP == 0:
      camera_lka = CS.adas_lka if CS.adas_lka is not None else self.adas_lka_idle
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                          CS.out.steeringAngleDeg, CC.latActive, CarControllerParams, self.VM)
      can_sends.append(leapmotorcan.create_steering_control(self.packer, camera_lka, self.apply_angle_last, CC.latActive))

    # cancel stock ACC when openpilot asks (it is engaged with ACC, so it cancels ACC whenever it disengages)
    if CC.cruiseControl.cancel and self.frame % CarControllerParams.STEER_STEP == 0 and CS.powertrain_state is not None:
      can_sends.append(leapmotorcan.create_cancel(self.packer, CS.powertrain_state))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
