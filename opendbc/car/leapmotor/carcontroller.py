import math

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_steer_angle_limits_vm, get_max_angle_vm
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
    self.lcc_ramp = leapmotorcan.LCC_RAMP_IDLE
    self.lat_active_last = False
    self.VM = VehicleModel(get_safety_CP())
    self.adas_lka_idle = leapmotorcan.decode_adas_lka_idle()

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []
    # like the stock LCC, let go while the driver steers (the EPS flags it), then ramp back in
    lat_active = CC.latActive and not CS.driver_override
    # take the wheel back only once it is near the angle openpilot wants: after the driver let go mid-turn at walking
    # pace with the wheel at +308 deg and the model at +24, the EPS swung it to -203 deg in 3 s
    if lat_active and not self.lat_active_last:
      lat_active = abs(actuators.steeringAngleDeg - CS.out.steeringAngleDeg) < CarControllerParams.TAKE_BACK_MAX_ANGLE_ERROR
      # and within the angle openpilot may command at this speed: otherwise the first frame jumps to that limit, the
      # panda blocks it, and the missing frame cancels ACC
      if CS.out.vEgoRaw > 1.:
        max_angle = get_max_angle_vm(CS.out.vEgoRaw, self.VM, CarControllerParams)
        lat_active = lat_active and abs(CS.out.steeringAngleDeg) < max_angle - 1.0

    if self.frame % CarControllerParams.STEER_STEP == 0:
      camera_lka = CS.adas_lka if CS.adas_lka is not None else self.adas_lka_idle
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                          CS.out.steeringAngleDeg, lat_active, CarControllerParams, self.VM)
      lat_accel = self.VM.calc_curvature(math.radians(self.apply_angle_last), CS.out.vEgoRaw, 0.) * CS.out.vEgoRaw ** 2
      self.lcc_ramp = leapmotorcan.lcc_ramp_next(self.lcc_ramp, lat_active,
                                                 leapmotorcan.lcc_ramp_target(lat_accel, CS.out.vEgoRaw * 3.6))
      can_sends.append(leapmotorcan.create_steering_control(self.packer, camera_lka, self.apply_angle_last, lat_active,
                                                             self.lcc_ramp))

    # cancel stock ACC when openpilot asks (it is engaged with ACC, so it cancels ACC whenever it disengages)
    if CC.cruiseControl.cancel and self.frame % CarControllerParams.STEER_STEP == 0 and CS.powertrain_state is not None:
      can_sends.append(leapmotorcan.create_cancel(self.packer, CS.powertrain_state))

    self.lat_active_last = lat_active
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
