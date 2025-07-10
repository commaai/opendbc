from opendbc.can.packer import CANPacker
from opendbc.car import Bus, apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.landrover.defendercan import create_lkas_command_defender, create_hud_command_defender
from opendbc.car.landrover.values import CarControllerParams


def process_hud(enabled, active, leftBs, rightBs, hud_control):
  # 0: off, 1:green, 2:red, 3:white
  left_lane_warning = 0
  right_lane_warning = 0

  if enabled:
    if hud_control.leftLaneVisible:
      left_lane_warning = 1
    else:
      left_lane_warning = 3

    if hud_control.rightLaneVisible:
      right_lane_warning = 1
    else:
      right_lane_warning = 3

    if hud_control.leftLaneDepart:
      left_lane_warning = 0

    if hud_control.rightLaneDepart:
      right_lane_warning = 0

  if leftBs:
    left_lane_warning = 4 + 2

  if rightBs:
    right_lane_warning = 4 + 2

  return left_lane_warning, right_lane_warning




class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0

    self.packer = CANPacker(dbc_names[Bus.pt])

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl


    can_sends = []


    if self.frame % 2 == 0:
      # Angular rate limit based on speed
      self.apply_angle_last = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgo,
                                                           CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)

      # LKAS msg
      can_sends.append(
        create_lkas_command_defender(
             self.packer,
             CC.enabled, CC.latActive,
             self.apply_angle_last,
             self.frame % 255,
             ))

    if self.frame % 4 == 0:
      # HUD control
      left_lane, right_lane = process_hud(CC.enabled, CC.latActive, CS.out.leftBlindspot, CS.out.rightBlindspot, hud_control)
      # HUD msg
      can_sends.append(
        create_hud_command_defender(
             self.packer,
             CC.enabled, CC.latActive,
             self.frame % 255,
             left_lane, right_lane))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1

    return new_actuators, can_sends

