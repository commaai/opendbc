import copy
from opendbc.can.packer import CANPacker
from opendbc.car import DT_CTRL, apply_driver_steer_torque_limits, apply_std_steer_angle_limits, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.numpy_fast import clip, interp
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mqbcan, pqcan, mebcan
from opendbc.car.volkswagen.values import CANBUS, CarControllerParams, VolkswagenFlags
#from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

#def apply_meb_curvature_limits(apply_curvature, apply_curvature_last, current_curvature, v_ego_raw, CCP):
#  if v_ego_raw > 1: # we don't enforce checks for this in panda at the moment, but keep it near measured current curv to project user input
#    apply_curvature = clip(apply_curvature, current_curvature - CCP.CURVATURE_ERROR, current_curvature + CCP.CURVATURE_ERROR)
#  apply_curvature = apply_std_steer_angle_limits(apply_curvature, apply_curvature_last, v_ego_raw, CCP)
#
#  return clip(apply_curvature, -CCP.CURVATURE_MAX, CCP.CURVATURE_MAX)


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    super().__init__(dbc_name, CP)
    self.CCP = CarControllerParams(CP)

    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MEB:
      self.CCS = mebcan
    else:
      self.CCS = mqbcan

    self.packer_pt = CANPacker(dbc_name)
    self.ext_bus = CANBUS.pt if CP.networkLocation == structs.CarParams.NetworkLocation.fwdCamera else CANBUS.cam

    self.apply_steer_last = 0
    #self.apply_curvature_last = 0
    self.apply_angle_last = 0
    self.gra_acc_counter_last = None
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0
    self.lat_active_prev = False
    self.steering_power = 0
    self.long_heartbeat = 0
    self.accel_last = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      if self.CP.flags & VolkswagenFlags.MEB:
        # Logic to avoid HCA refused state
        #   * steering power as counter near zero before standstill OP lane assist deactivation
        # MEB rack can be used continously without found time limits yet
        # Steering power counter is used to:
        #   * prevent sudden fluctuations at low speeds
        #   * avoid HCA refused
        #   * easy user intervention
        #   * keep it near maximum regarding speed to get full steering power in shortest time

        if CC.latActive:
          hca_enabled          = True
          self.lat_active_prev = True
          #current_curvature    = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1) # TODO verify sign (clockwise is negative)
          #apply_curvature      = apply_meb_curvature_limits(actuators.curvature, self.apply_curvature_last, current_curvature, CS.out.vEgoRaw, self.CCP)
          apply_angle          = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, self.CCP)
          apply_angle          = clip(apply_angle, CS.out.steeringAngleDeg - self.CCP.ANGLE_ERROR, CS.out.steeringAngleDeg + self.CCP.ANGLE_ERROR)

          # steering power as lazy counter
          steering_power_min_by_speed       = interp(CS.out.vEgoRaw, [0, self.CCP.STEERING_POWER_MAX_BY_SPEED], [self.CCP.STEERING_POWER_MIN, self.CCP.STEERING_POWER_MAX])
          steering_angle_diff               = abs(apply_angle - CS.out.steeringAngleDeg)
          steering_power_target_angle       = steering_power_min_by_speed + self.CCP.ANGLE_POWER_FACTOR * steering_angle_diff + abs(apply_angle)
          steering_power_target             = clip(steering_power_target_angle, self.CCP.STEERING_POWER_MIN, self.CCP.STEERING_POWER_MAX)

          if self.steering_power < self.CCP.STEERING_POWER_MIN:  # OP lane assist just activated
            self.steering_power = min(self.steering_power + self.CCP.STEERING_POWER_STEPS, self.CCP.STEERING_POWER_MIN)

          elif CS.out.steeringPressed and self.steering_power > self.CCP.STEERING_POWER_USER: # user action results in decreasing the steering power
            self.steering_power = max(self.steering_power - self.CCP.STEERING_POWER_STEPS, self.CCP.STEERING_POWER_USER)

          elif self.steering_power < self.CCP.STEERING_POWER_MAX: # following desired target
            if self.steering_power < steering_power_target:
              self.steering_power = min(self.steering_power + self.CCP.STEERING_POWER_STEPS, steering_power_target)
            elif self.steering_power > steering_power_target:
              self.steering_power = max(self.steering_power - self.CCP.STEERING_POWER_STEPS, steering_power_target)

        else:
          if self.lat_active_prev and self.steering_power > 0: # monotonously decrement power to zero before disabling lane assist to prevent EPS fault
            hca_enabled            = True
            #current_curvature      = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
            #apply_curvature        = current_curvature
            apply_angle            = CS.out.steeringAngleDeg
            self.steering_power    = max(self.steering_power - self.CCP.STEERING_POWER_STEPS, 0)
          else:
            hca_enabled           = False
            self.lat_active_prev  = False
            self.steering_power   = 0
            #apply_curvature       = 0.
            apply_angle           = 0

        #self.apply_curvature_last = apply_curvature
        self.apply_angle_last = clip(apply_angle, -self.CCP.ANGLE_MAX, self.CCP.ANGLE_MAX)
        can_sends.append(self.CCS.create_steering_control_curvature(self.packer_pt, CANBUS.pt, apply_angle, hca_enabled, self.steering_power))

      else:
        # Logic to avoid HCA state 4 "refused":
        #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
        #   * Don't steer at standstill
        #   * Don't send > 3.00 Newton-meters torque
        #   * Don't send the same torque for > 6 seconds
        #   * Don't send uninterrupted steering for > 360 seconds
        # MQB racks reset the uninterrupted steering timer after a single frame
        # of HCA disabled; this is done whenever output happens to be zero.

        if CC.latActive:
          new_steer = int(round(actuators.steer * self.CCP.STEER_MAX))
          apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.CCP)
          self.hca_frame_timer_running += self.CCP.STEER_STEP
          if self.apply_steer_last == apply_steer:
            self.hca_frame_same_torque += self.CCP.STEER_STEP
            if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
              apply_steer -= (1, -1)[apply_steer < 0]
              self.hca_frame_same_torque = 0
          else:
            self.hca_frame_same_torque = 0
          hca_enabled = abs(apply_steer) > 0
        else:
          hca_enabled = False
          apply_steer = 0

        if not hca_enabled:
          self.hca_frame_timer_running = 0

        self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
        self.apply_steer_last = apply_steer
        can_sends.append(self.CCS.create_steering_control(self.packer_pt, CANBUS.pt, apply_steer, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = clip(apply_steer * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX)
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, CANBUS.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #

    if self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl:
      stopping = actuators.longControlState == LongCtrlState.stopping
      starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)

      if self.CP.flags & VolkswagenFlags.MEB:
        accel = clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.enabled and CS.out.cruiseState.enabled else 0
        self.accel_last = accel
        current_speed = CS.out.vEgo * CV.MS_TO_KPH
        reversing = CS.out.gearShifter in [structs.CarState.GearShifter.reverse]
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled and CS.out.cruiseState.enabled,
                                                 CS.esp_hold_confirmation, CC.cruiseControl.override)
        acc_hold_type = self.CCS.acc_hold_type(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled and CS.out.cruiseState.enabled,
                                               starting, stopping, CS.esp_hold_confirmation, CC.cruiseControl.override)
        required_jerk = min(3, abs(accel - CS.out.aEgo) * 50) ## pfeiferj:openpilot:pfeifer-hkg-long-control-tune
        lower_jerk = required_jerk
        upper_jerk = required_jerk

        if CS.out.aEgo < accel:
          lower_jerk = 0
        else:
          upper_jerk = 0
          
        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, CS.acc_type, CC.enabled and CS.out.cruiseState.enabled,
                                                           accel, acc_control, acc_hold_type, stopping, starting, lower_jerk, upper_jerk,
                                                           CS.esp_hold_confirmation, CC.cruiseControl.override, current_speed, reversing))

      else:
        accel = clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0
        self.accel_last = accel
        
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, CS.acc_type, CC.longActive, accel,
                                                           acc_control, stopping, starting, CS.esp_hold_confirmation))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      sound_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOverUrgent"]
        sound_alert = 1
      if self.CP.flags & VolkswagenFlags.MEB:
        can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.pt, CS.ldw_stock_values, CC.latActive,
                                                         CS.out.steeringPressed, hud_alert, hud_control, sound_alert))
      else:
        can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.pt, CS.ldw_stock_values, CC.latActive,
                                                         CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % 100 == 0 and self.lead_distance_bar_timer <= 3:
      self.lead_distance_bar_timer += 1

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      if self.CP.flags & VolkswagenFlags.MEB:
        self.long_heartbeat = generate_vw_meb_hud_heartbeat()
        desired_gap = max(1, CS.out.vEgo * 1) #get_T_FOLLOW(hud_control.leadDistanceBars))

        distance = 50 # TODO get distance from model
        desired_gap = min(CS.out.vEgo, 100) # TODO get desired gap from OP
        distance = 50 #min(self.lead_distance, 100)

        change_distance_bar = False
        if hud_control.leadDistanceBars != self.lead_distance_bars_last:
          self.lead_distance_bar_timer = 0
        self.lead_distance_bars_last = hud_control.leadDistanceBars
        change_distance_bar = True if self.lead_distance_bar_timer <= 3 else False

        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled and CS.out.cruiseState.enabled,
                                                       CS.esp_hold_confirmation, CC.cruiseControl.override)
        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.pt, acc_hud_status, hud_control.setSpeed * CV.MS_TO_KPH,
                                                         hud_control.leadVisible, hud_control.leadDistanceBars, change_distance_bar,
                                                         desired_gap, distance, self.long_heartbeat, CS.esp_hold_confirmation))

      else:
        lead_distance = 0
        if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
          lead_distance = 512 if CS.upscale_lead_car_signal else 8
        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        # FIXME: follow the recent displayed-speed updates, also use mph_kmh toggle to fix display rounding problem?
        set_speed = hud_control.setSpeed * CV.MS_TO_KPH
        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.pt, acc_hud_status, set_speed,
                                                         lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.ext_bus, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = copy.copy(actuators)
    new_actuators.steer = self.apply_steer_last / self.CCP.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last
    #new_actuators.curvature = self.apply_curvature_last
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.accel = self.accel_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends

  def generate_vw_meb_hud_heartbeat(self):
    if self.long_heartbeat != 221:
      return 221
    elif self.long_heartbeat == 221:
      return 360
