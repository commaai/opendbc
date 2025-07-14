from opendbc.can.packer import CANPacker
from opendbc.car import Bus, apply_std_steer_angle_limits, AngleSteeringLimits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd.bydcan import create_can_steer_command, create_lkas_hud, create_accel_command

class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    220, # deg ECU_FAULT_MAX_ANGLES([0., 1., 8.], [360, 290, 220])
    ([0., 5., 15.], [4., 3., 2.]),
    ([0., 5., 15.], [6., 4., 3.]),
  )

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.apply_angle = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    if (self.frame % 2) == 0:
      self.apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle, \
      CS.out.vEgo, CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)
      can_sends.append(create_can_steer_command(self.packer, self.apply_angle, CC.latActive, CS.out.standstill, CS.steer_state))
      can_sends.append(create_lkas_hud(self.packer, CS.hud_passthrough, CS.adas_settings_pt, CC.enabled, CS.lka_on))

      if self.CP.openpilotLongitudinalControl:
        long_active = CC.longActive and not CS.out.gasPressed
        brake_hold = CS.out.standstill and actuators.accel < 0
        can_sends.append(create_accel_command(self.packer, actuators.accel, long_active, brake_hold))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle

    self.frame += 1
    return new_actuators, can_sends
