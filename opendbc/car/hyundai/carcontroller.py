import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL, apply_driver_steer_torque_limits, common_fault_avoidance, make_tester_present_msg, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai import hyundaicanfd, hyundaican
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CAR
from opendbc.car.interfaces import CarControllerBase

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second, all slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.angle_limit_counter = 0
    self.accel_last = 0
    self.apply_torque_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0


  def update(self, CC, CS, now_nanos):
    if self.CP.flags & HyundaiFlags.CANFD:
      return self.update_canfd(CC, CS, now_nanos)

    (actuators, hud_control, apply_torque, apply_steer_req, torque_fault, accel, stopping, set_speed_in_units, sys_warning, sys_state, left_lane_warning,
     right_lane_warning, tester_present_msgs) = self.compute_common_controls(CC, CS)

    can_sends = tester_present_msgs.copy()
    can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.CP, apply_torque, apply_steer_req, torque_fault, CS.lkas11, sys_warning,
      sys_state, CC.enabled, hud_control.leftLaneVisible, hud_control.rightLaneVisible, left_lane_warning, right_lane_warning))

    if self.CP.openpilotLongitudinalControl:
      if self.frame % 2 == 0:
        jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0  # TODO: unclear if this is needed
        can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2), hud_control, set_speed_in_units, stopping,
                                                          CC.cruiseControl.override, self.CP.flags & HyundaiFlags.USE_FCA.value, self.CP))
      if self.frame % 20 == 0:
        can_sends.extend(hyundaican.create_acc_opt(self.packer, self.CP))
      if self.frame % 50 == 0:
        can_sends.append(hyundaican.create_frt_radar_opt(self.packer))
    else:
      can_sends.extend(self.create_button_messages(CC, CS, use_clu11=True))

    if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
      can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled))

    return self.build_actuators(actuators, apply_torque, accel, can_sends)


  def update_canfd(self, CC, CS, now_nanos):
    (actuators, hud_control, apply_torque, apply_steer_req, torque_fault, accel, stopping, set_speed_in_units, sys_warning, sys_state, left_lane_warning,
     right_lane_warning, tester_present_msgs) = self.compute_common_controls(CC, CS)

    can_sends = tester_present_msgs.copy()
    can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_torque))

    lka_steering = self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING

    if self.frame % 5 == 0:
      if lka_steering:
        can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.lfa_block_msg, self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT))
      if not lka_steering or self.CP.openpilotLongitudinalControl:
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled))

    if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS and lka_steering:
      can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, self.frame, CC.leftBlinker, CC.rightBlinker))

    if self.CP.openpilotLongitudinalControl:
      if lka_steering:
        can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
      else:
        can_sends.extend(hyundaicanfd.create_fca_warning_light(self.packer, self.CAN, self.frame))
      if self.frame % 2 == 0:
        can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                         set_speed_in_units, hud_control))
        self.accel_last = accel
    else:
      can_sends.extend(self.create_button_messages(CC, CS, use_clu11=False))

    return self.build_actuators(actuators, apply_torque, accel, can_sends)


  def build_actuators(self, actuators, apply_torque, accel, can_sends):
    new_actuators = actuators.as_builder()
    new_actuators.torque, new_actuators.torqueOutputCan, new_actuators.accel = apply_torque / self.params.STEER_MAX, apply_torque, accel
    self.frame += 1
    return new_actuators, can_sends


  def compute_common_controls(self, CC, CS):
    actuators, hud_control = CC.actuators, CC.hudControl

    new_torque = int(round(actuators.torque * self.params.STEER_MAX))
    apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params)

    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive, self.angle_limit_counter,
      MAX_ANGLE_FRAMES, MAX_ANGLE_CONSECUTIVE_FRAMES)

    if not CC.latActive:
      apply_torque = 0

    torque_fault = CC.latActive and not apply_steer_req
    self.apply_torque_last = apply_torque

    accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))
    sys_state = 1  # TODO: this is not accurate for all cars
    if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
      sys_state = 3 if CC.enabled or sys_warning else 4
    elif hud_control.leftLaneVisible:
      sys_state = 5
    elif hud_control.rightLaneVisible:
      sys_state = 6
    left_lane_warning = (1 if self.car_fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2) if hud_control.leftLaneDepart else 0
    right_lane_warning = (1 if self.car_fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2) if hud_control.rightLaneDepart else 0

    tester_present_msgs = []
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC) and self.CP.openpilotLongitudinalControl:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, self.CAN.ECAN if self.CP.flags & HyundaiFlags.CANFD else 0
      if self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING.value:
        addr, bus = 0x730, self.CAN.ECAN
      tester_present_msgs.append(make_tester_present_msg(addr, bus, suppress_response=True))

      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        tester_present_msgs.append(make_tester_present_msg(0x7b1, self.CAN.ECAN, suppress_response=True))

    return (actuators, hud_control, apply_torque, apply_steer_req, torque_fault, accel, stopping, set_speed_in_units, sys_warning, sys_state, left_lane_warning,
       right_lane_warning, tester_present_msgs)


  def create_button_messages(self, CC: structs.CarControl, CS: CarState, use_clu11: bool):
    can_sends = []
    if use_clu11:
      if CC.cruiseControl.cancel:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP))
      elif CC.cruiseControl.resume:
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP)] * 25)
          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
            self.last_button_frame = self.frame
    else:
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
        if CC.cruiseControl.cancel:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, self.CAN, CS.cruise_info))
            self.last_button_frame = self.frame
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.CANCEL))
            self.last_button_frame = self.frame
        elif CC.cruiseControl.resume:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            pass
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.RES_ACCEL))
            self.last_button_frame = self.frame

    return can_sends
