from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL, apply_driver_steer_torque_limits, common_fault_avoidance, make_tester_present_msg, structs, apply_std_steer_angle_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.numpy_fast import clip, interp
from opendbc.car.hyundai import hyundaicanfd, hyundaican
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CAR, ANGLE_CONTROL_CAR
from opendbc.car.interfaces import CarControllerBase
from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN


GearShifter = structs.CarState.GearShifter
VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

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
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.angle_limit_counter = 0

    self.accel_last = 0
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0

    self.apply_angle_now = 0
    self.apply_angle_last = 0
    self.lkas_max_torque = 0

    self.driver_steering_torque_above_timer = 150
    self.driver_steering_angle_above_timer = 150

    self.to_avoid_lkas_fault_enabled = True
    self.to_avoid_lkas_fault_max_angle = 85
    self.to_avoid_lkas_fault_max_frame = 89
    self.enable_steer_more = False
    self.no_mdps_mods = False

    self.steer_timer_apply_torque = 1.0
    self.DT_STEER = 0.005             # 0.01 1sec, 0.005  2sec

    self.lkas_onoff_counter = 0
    self.lkas_temp_disabled = False
    self.lkas_temp_disabled_timer = 0
    
    self.stock_lfa_counter = 0


  def smooth_steer( self, apply_torque, CS ):
    if self.CP.smoothSteer.maxSteeringAngle and abs(CS.out.steeringAngleDeg) > self.CP.smoothSteer.maxSteeringAngle:
      if self.CP.smoothSteer.maxDriverAngleWait and CS.out.steeringPressed:
        self.steer_timer_apply_torque -= self.CP.smoothSteer.maxDriverAngleWait # 0.002 #self.DT_STEER   # 0.01 1sec, 0.005  2sec   0.002  5sec
      elif self.CP.smoothSteer.maxSteerAngleWait:
        self.steer_timer_apply_torque -= self.CP.smoothSteer.maxSteerAngleWait # 0.001  # 10 sec
    elif self.CP.smoothSteer.driverAngleWait and CS.out.steeringPressed:
      self.steer_timer_apply_torque -= self.CP.smoothSteer.driverAngleWait #0.001
    else:
      if self.steer_timer_apply_torque >= 1:
          return int(round(float(apply_torque)))
      self.steer_timer_apply_torque += self.DT_STEER

    if self.steer_timer_apply_torque < 0:
      self.steer_timer_apply_torque = 0
    elif self.steer_timer_apply_torque > 1:
      self.steer_timer_apply_torque = 1

    apply_torque *= self.steer_timer_apply_torque

    return  int(round(float(apply_torque)))


  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    if abs(CS.out.steeringTorque) > 170 and CS.out.vEgo < LANE_CHANGE_SPEED_MIN and not (self.CP.flags & HyundaiFlags.CANFD):
      self.driver_steering_torque_above_timer -= 1
      if self.driver_steering_torque_above_timer <= 0:
        self.driver_steering_torque_above_timer = 0
    else:
      self.driver_steering_torque_above_timer += 10
      if self.driver_steering_torque_above_timer >= 150:
        self.driver_steering_torque_above_timer = 150

    # steering torque
    if self.CP.smoothSteer.method == 1:
      new_steer = int(round(actuators.steer * self.params.STEER_MAX))
      new_steer = self.smooth_steer( new_steer, CS )
    elif 0 <= self.driver_steering_torque_above_timer < 150 and not self.user_specific_feature == 60:
      new_steer = int(round(actuators.steer * self.params.STEER_MAX * (self.driver_steering_torque_above_timer / 150)))
    else:
      new_steer = int(round(actuators.steer * self.params.STEER_MAX))

    apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)

    if self.to_avoid_lkas_fault_max_angle == 90:
      lat_active = CC.latActive and abs(CS.out.steeringAngleDeg) < self.to_avoid_lkas_fault_max_angle and \
      (CS.out.gearShifter == GearShifter.drive or self.user_specific_feature == 11)
    elif self.to_avoid_lkas_fault_max_angle > 90:
      str_angle_limit = interp(CS.out.vEgo * CV.MS_TO_KPH, [0, 20], [self.to_avoid_lkas_fault_max_angle+60, self.to_avoid_lkas_fault_max_angle])
      lat_active = CC.latActive and abs(CS.out.steeringAngleDeg) < str_angle_limit and \
      (CS.out.gearShifter == GearShifter.drive or self.user_specific_feature == 11)
    else:
      lat_active = CC.latActive and (CS.out.gearShifter == GearShifter.drive or self.user_specific_feature == 11)

    if self.no_mdps_mods and CS.out.vEgo < self.CP.minSteerSpeed:
      lat_active = False

    # >90 degree steering fault prevention
    if self.to_avoid_lkas_fault_enabled or self.CP.flags & HyundaiFlags.CANFD:
      self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= self.to_avoid_lkas_fault_max_angle, lat_active,
                                                                         self.angle_limit_counter, self.to_avoid_lkas_fault_max_frame,
                                                                         MAX_ANGLE_CONSECUTIVE_FRAMES)
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, self.params)
      self.apply_angle_now = apply_angle

      # Figure out torque value.  On Stock when LKAS is active, this is variable,
      # but 0 when LKAS is not actively steering, so because we're "tricking" ADAS
      # into thinking LKAS is always active, we need to make sure we're applying
      # torque when the driver is not actively steering. The default value chosen
      # here is based on observations of the stock LKAS system when it's engaged
      # CS.out.steeringPressed and steeringTorque are based on the
      # STEERING_COL_TORQUE value

      lkas_max_torque = 180
      if abs(CS.out.steeringTorque) > 200:
        self.driver_steering_angle_above_timer -= 1
        if self.driver_steering_angle_above_timer <= 30:
          self.driver_steering_angle_above_timer = 30
      else:
        self.driver_steering_angle_above_timer += 1
        if self.driver_steering_angle_above_timer >= 150:
          self.driver_steering_angle_above_timer = 150

      ego_weight = interp(CS.out.vEgo, [0, 5, 10, 20], [0.2, 0.3, 0.5, 1.0])

      if 0 <= self.driver_steering_angle_above_timer < 150:
        self.lkas_max_torque = int(round(lkas_max_torque * (self.driver_steering_angle_above_timer / 150) * ego_weight))
      else:
        self.lkas_max_torque = lkas_max_torque * ego_weight

      # Hold torque with induced temporary fault when cutting the actuation bit
      torque_fault = lat_active and not apply_steer_req
    else:
      torque_fault = False

    if not lat_active:
      apply_angle = CS.out.steeringAngleDeg
      apply_steer = 0
      self.lkas_max_torque = 0

    self.apply_angle_last = apply_angle

    self.apply_steer_last = apply_steer

    # accel + longitudinal
    accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    can_sends = []

    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and self.CP.openpilotLongitudinalControl:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, 0
      if self.CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append(make_tester_present_msg(addr, bus, suppress_response=True))

      # for blinkers
      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.append(make_tester_present_msg(0x7b1, self.CAN.ECAN, suppress_response=True))

    # CAN-FD platforms
    if self.CP.flags & HyundaiFlags.CANFD:
      hda2 = self.CP.flags & HyundaiFlags.CANFD_HDA2

      # steering control
      angle_control = self.CP.carFingerprint in ANGLE_CONTROL_CAR
      can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled,
                                                             apply_steer_req, CS.out.steeringPressed,
                                                             apply_steer, apply_angle, self.lkas_max_torque, angle_control))

      # prevent LFA from activating on HDA2 by sending "no lane lines detected" to ADAS ECU
      if self.frame % 5 == 0 and hda2:
        if not CC.enabled:
          self.stock_lfa_counter += 1 if self.stock_lfa_counter < 100 else 0
        elif self.stock_lfa_counter:
          self.stock_lfa_counter -= 1
        can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.hda2_lfa_block_msg,
                                                          self.CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING, CC.enabled, bool(self.stock_lfa_counter)))

      # LFA and HDA icons
      updateLfaHdaIcons = (not hda2) or angle_control
      if self.frame % 5 == 0 and updateLfaHdaIcons:
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled))

      # blinkers
      if hda2 and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, self.frame, CC.leftBlinker, CC.rightBlinker))

      if self.CP.openpilotLongitudinalControl:
        if hda2:
          can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
        if self.frame % 2 == 0:
          can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                           set_speed_in_units, hud_control))
          self.accel_last = accel
      else:
        # button presses
        can_sends.extend(self.create_button_messages(CC, CS, use_clu11=False))
    else:
      can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.CP, apply_steer, apply_steer_req,
                                                torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                                hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                left_lane_warning, right_lane_warning))

      if not self.CP.openpilotLongitudinalControl:
        can_sends.extend(self.create_button_messages(CC, CS, use_clu11=True))

      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
        # TODO: unclear if this is needed
        jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
        use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
        can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                        hud_control, set_speed_in_units, stopping,
                                                        CC.cruiseControl.override, use_fca))

      # 20 Hz LFA MFA message
      if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
        can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled))

      # 5 Hz ACC options
      if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl:
        can_sends.extend(hyundaican.create_acc_opt(self.packer))

      # 2 Hz front radar options
      if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl:
        can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    new_actuators = actuators.as_builder()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.steeringAngleDeg = apply_angle
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends

  def create_button_messages(self, CC: structs.CarControl, CS: CarState, use_clu11: bool):
    can_sends = []
    if use_clu11:
      if CC.cruiseControl.cancel:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP))
      elif CC.cruiseControl.resume:
        # send resume at a max freq of 10Hz
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP)] * 25)
          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
            self.last_button_frame = self.frame
    else:
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
        # cruise cancel
        if CC.cruiseControl.cancel:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, self.CAN, CS.cruise_info))
            self.last_button_frame = self.frame
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, Buttons.CANCEL))
            self.last_button_frame = self.frame

        # cruise standstill resume
        elif CC.cruiseControl.resume:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            pass
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, Buttons.RES_ACCEL))
            self.last_button_frame = self.frame

    return can_sends
