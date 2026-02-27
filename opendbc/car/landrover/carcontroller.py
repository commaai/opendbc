from opendbc.can.packer import CANPacker
from opendbc.car.can_definitions import CanData
from opendbc.car import Bus
from opendbc.car.lateral import apply_std_steer_angle_limits, apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.landrover.landrovercan import create_lkas_command_defender, create_hud_command_defender, create_lkas_command, create_lkas_hud
from opendbc.car.landrover.values import CarControllerParams, LandroverFlags, STATIC_MSGS


def process_hud_alert_rr(enabled, active, leftBs, rightBs, hud_control, counter):
  #sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))
  sys_warning =0

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  # 0: off, 1:green, 2:red, 3:white
  left_lane_warning = 0
  right_lane_warning = 0

  if active:
    left_lane_warning = 1
    right_lane_warning = 1
  else:
    left_lane_warning = 3
    right_lane_warning = 3

  if leftBs:
    left_lane_warning = 2

  if rightBs:
    right_lane_warning = 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


def process_hud(enabled, active, leftBs, rightBs, hud_control):
  # 0: off, 1:green, 2:red, 3:white
  left_lane_warning = 0
  right_lane_warning = 0

  if enabled:
    left_lane_warning = 3
    right_lane_warning = 3

    if active:
      left_lane_warning = 1
      right_lane_warning = 1

    if hud_control.leftLaneDepart:
      left_lane_warning = 3

    if hud_control.rightLaneDepart:
      right_lane_warning = 3

  if leftBs:
    left_lane_warning = 4 + 2

  if rightBs:
    right_lane_warning = 4 + 2

  return left_lane_warning, right_lane_warning


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)

    self.params = CarControllerParams(CP)
    self.apply_torque_last = 0
    self.apply_angle_last = 0

    self.packer = CANPacker(dbc_names[Bus.pt])
    self.lkascnt = 0
    self.lrflag = 0
    self.main_on_last = False

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available

    # Steering Torque
    new_torque = int(round(actuators.torque * self.params.STEER_MAX))
    apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params)

    if not CC.latActive:
      apply_torque = 0

    self.apply_torque_last = apply_torque

    can_sends = []

    if self.CP.flags & LandroverFlags.FLEXRAY_HARNESS == 0:

      if CS.out.leftBlinker or CS.out.rightBlinker:
        apply_torque = 0
        self.apply_torque_last = 0

      for (addr, bus, _fr_step, vl) in STATIC_MSGS:
         if (self.frame % 2 == 0):  # 50Hz
           can_sends.append(CanData(addr, vl, bus))

      if self.frame % 4 == 0:
        can_sends.append(
          create_lkas_command(
             self.packer,
             CC.latActive,
             self.lkascnt,
             int(apply_torque),
             ))
        self.lkascnt += 1

      # LaneInfo
      if (self.frame % 8 == 0):  # 8hz
        sys_warning, sys_state, left_lane, right_lane = process_hud_alert_rr(
          main_on, CC.latActive, CS.out.leftBlindspot, CS.out.rightBlindspot, hud_control, self.frame
          )

        if left_lane == 2 and right_lane == 2:
          if self.lrflag ==0:
            left_lane = 0
            self.lrflag = 1
          else:
            right_lane = 0
            self.lrflag = 0

        can_sends.append(
          create_lkas_hud(
             self.packer,
             left_lane,
             right_lane
             ))

    else:
      # FLEXRAY_HARNESS
      if self.frame % 2 == 0:
        # Angular rate limit based on speed
        self.apply_angle_last = \
             apply_std_steer_angle_limits(actuators.steeringAngleDeg,
                 self.apply_angle_last, CS.out.vEgo,
                 CS.out.steeringAngleDeg, CC.latActive,
                 CarControllerParams.ANGLE_LIMITS)

        # LKAS msg
        can_sends.append(
          create_lkas_command_defender(
             self.packer,
             main_on, CC.latActive,
             self.apply_angle_last,
             self.lkascnt,
             ))
        self.lkascnt += 1

      if self.frame % 4 == 0:
        # HUD control
        left_lane, right_lane = process_hud(main_on, CC.latActive, CS.out.leftBlindspot, CS.out.rightBlindspot, hud_control)
        # HUD msg
        can_sends.append(
            create_hud_command_defender(
            self.packer,
            main_on, CC.latActive,
            self.frame % 255,
            left_lane, right_lane))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1

    return new_actuators, can_sends
