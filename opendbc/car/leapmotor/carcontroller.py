import math
import numpy as np

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
    self.ramp_boost = 0

  def update_ramp_boost(self, lat_active: bool, CS) -> int:
    """Extra LCC_RAMP steps: one more per frame while the wheel lags the command at low speed, one fewer once it is
    back (see CarControllerParams.RAMP_BOOST_LAG), and none past the EPS torque at which the EPS lets go by itself
    (see CarControllerParams.RAMP_BOOST_EPS_TORQUE_HOLD)."""
    if not lat_active or CS.out.vEgoRaw * 3.6 > CarControllerParams.RAMP_BOOST_MAX_SPEED:
      return 0
    behind = abs(self.apply_angle_last - CS.out.steeringAngleDeg)
    eps_torque = abs(CS.out.steeringTorqueEps)
    boost = self.ramp_boost
    if eps_torque >= CarControllerParams.RAMP_BOOST_EPS_TORQUE_BACK or behind < CarControllerParams.RAMP_BOOST_CAUGHT_UP:
      boost -= 1
    elif behind > CarControllerParams.RAMP_BOOST_LAG and eps_torque < CarControllerParams.RAMP_BOOST_EPS_TORQUE_HOLD:
      boost += 1
    return min(max(boost, 0), leapmotorcan.LCC_RAMP_STEPS_MAX - leapmotorcan.LCC_RAMP_STEPS_MIN)

  def long_accel(self, CC, camera_lka: dict) -> float | None:
    """openpilot's acceleration request for ADAS_LKA, or None to send the camera's.

    Only while the stock ACC is engaged (ACC_STATE 3): the camera keeps engagement, the standstill hold and the
    accelerator override (ACC_STATE 1), and its request goes through unchanged then. When the camera asks for hard
    braking, openpilot never asks for less.
    """
    if not (self.CP.openpilotLongitudinalControl and CC.longActive and int(camera_lka["ACC_STATE"]) == 3):
      return None
    accel = float(np.clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    camera_accel = float(camera_lka["ACC_ACCEL_REQ"])
    if camera_accel <= CarControllerParams.STOCK_BRAKE_PASSTHROUGH:
      accel = min(accel, camera_accel)
    return accel

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []
    # like the stock LCC, let go while the driver steers (the EPS flags it), then ramp back in
    # (while the wheel follows openpilot's angle, the stricter rule: see CarControllerParams.OVERRIDE_TRACKING_ERROR)
    tracking = (self.lat_active_last and
                abs(self.apply_angle_last - CS.out.steeringAngleDeg) < CarControllerParams.OVERRIDE_TRACKING_ERROR)
    lat_active = CC.latActive and not (CS.driver_override_tracking if tracking else CS.driver_override)
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
      self.ramp_boost = self.update_ramp_boost(lat_active, CS)
      self.lcc_ramp = leapmotorcan.lcc_ramp_next(self.lcc_ramp, lat_active,
                                                 leapmotorcan.lcc_ramp_target(lat_accel, CS.out.vEgoRaw * 3.6, self.ramp_boost))
      can_sends.append(leapmotorcan.create_steering_control(self.packer, camera_lka, self.apply_angle_last, lat_active,
                                                             self.lcc_ramp, self.long_accel(CC, camera_lka)))

    # cancel stock ACC when openpilot asks (it is engaged with ACC, so it cancels ACC whenever it disengages)
    if CC.cruiseControl.cancel and self.frame % CarControllerParams.STEER_STEP == 0 and CS.powertrain_state is not None:
      can_sends.append(leapmotorcan.create_cancel(self.packer, CS.powertrain_state))

    self.lat_active_last = lat_active
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
