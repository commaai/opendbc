import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL, apply_driver_steer_torque_limits, apply_std_steer_angle_limits, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mqbcan, pqcan, mebcan
from opendbc.car.volkswagen.values import CANBUS, CarControllerParams, VolkswagenFlags
#from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CCP = CarControllerParams(CP)
    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MEB:
      self.CCS = mebcan
    else:
      self.CCS = mqbcan
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.ext_bus = CANBUS.pt if CP.networkLocation == structs.CarParams.NetworkLocation.fwdCamera else CANBUS.cam
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ

    self.apply_steer_last = 0
    self.apply_curvature_last = 0
    self.steering_power_last = 0
    self.accel_last = 0
    self.long_override_counter = 0
    self.long_disabled_counter = 0
    self.gra_acc_counter_last = None
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      if self.CP.flags & VolkswagenFlags.MEB:
        # Logic to avoid HCA refused state:
        #   * steering power as counter and near zero before OP lane assist deactivation
        # MEB rack can be used continously without time limits
        # maximum real steering angle change ~ 120-130 deg/s

        if CC.latActive:
          hca_enabled = True
          # Adjust our curvature command by the offset between openpilot's current curvature and the QFK's current curvature
          actuator_curvature_with_offset = actuators.curvature + (CS.curvature - CC.currentCurvature)
          apply_curvature = apply_std_steer_angle_limits(actuator_curvature_with_offset, self.apply_curvature_last, CS.out.vEgoRaw, self.CCP)
          apply_curvature = np.clip(apply_curvature, -self.CCP.CURVATURE_MAX, self.CCP.CURVATURE_MAX)

          # FIXME: implement power control, backoff on driver input torque
          steering_power = 125

        else:
          # TODO: see if we can do without this extra ramp-down-on-disengage logic, it makes safety more complex and MQBevo didn't need it
          if self.steering_power_last > 0: # keep HCA alive until steering power has reduced to zero
            hca_enabled = True
            current_curvature = CS.curvature #-CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
            apply_curvature = np.clip(current_curvature, -self.CCP.CURVATURE_MAX, self.CCP.CURVATURE_MAX) # synchronize with current curvature
            steering_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEPS, 0)
          else:
            hca_enabled = False
            apply_curvature = 0. # inactive curvature
            steering_power = 0

        can_sends.append(self.CCS.create_steering_control(self.packer_pt, CANBUS.pt, apply_curvature, hca_enabled, steering_power, False))
        self.apply_curvature_last = apply_curvature
        self.steering_power_last = steering_power

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
        ea_simulated_torque = float(np.clip(apply_steer * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, CANBUS.cam, CS.eps_stock_values, ea_simulated_torque))

    # TODO: refactor a bit
    if self.CP.flags & VolkswagenFlags.MEB:
      if self.frame % 2 == 0:
        can_sends.append(mebcan.create_ea_control(self.packer_pt, CANBUS.pt))
      if self.frame % 50 == 0:
        can_sends.append(mebcan.create_ea_hud(self.packer_pt, CANBUS.pt))


    # **** Acceleration Controls ******************************************** #

    if self.CP.openpilotLongitudinalControl:
      if self.frame % self.CCP.ACC_CONTROL_STEP == 0:
        stopping = actuators.longControlState == LongCtrlState.stopping
        starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)

        if self.CP.flags & VolkswagenFlags.MEB:
          # Logic to prevent car error with EPB:
          #   * send a few frames of HMS RAMP RELEASE command at the very begin of long overrideand and at the end of active long control
          accel = np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.enabled else 0
          self.accel_last = accel

          # 1 frame of long_override_begin is enough, but lower the possibility of panda safety blocking it for now until we adapt panda safety correctly
          long_override = CC.cruiseControl.override or CS.out.gasPressed
          self.long_override_counter = min(self.long_override_counter + 1, 5) if long_override else 0
          long_override_begin = long_override and self.long_override_counter < 5

          # 1 frame of long_disabling is enough, but lower the possibility of panda safety blocking it for now until we adapt panda safety correctly
          self.long_disabled_counter = min(self.long_disabled_counter + 1, 5) if not CC.enabled else 0
          long_disabling = not CC.enabled and self.long_disabled_counter < 5

          acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled,
                                                   CS.esp_hold_confirmation, long_override)
          acc_hold_type = self.CCS.acc_hold_type(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled, starting, stopping,
                                                 CS.esp_hold_confirmation, long_override, long_override_begin, long_disabling)
          can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, CS.acc_type, CC.enabled,
                                                             accel, acc_control, acc_hold_type, stopping, starting, CS.esp_hold_confirmation,
                                                             long_override, CS.travel_assist_available))

        else:
          accel = np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0
          self.accel_last = accel

          acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
          can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, CS.acc_type, CC.longActive, accel,
                                                             acc_control, stopping, starting, CS.esp_hold_confirmation))

        #if self.aeb_available:
        #  if self.frame % self.CCP.AEB_CONTROL_STEP == 0:
        #    can_sends.append(self.CCS.create_aeb_control(self.packer_pt, False, False, 0.0))
        #  if self.frame % self.CCP.AEB_HUD_STEP == 0:
        #    can_sends.append(self.CCS.create_aeb_hud(self.packer_pt, False, False))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0

      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.pt, CS.ldw_stock_values, CC.latActive,
                       CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      if self.CP.flags & VolkswagenFlags.MEB:
        desired_gap = max(1, CS.out.vEgo * 1.5) # TODO gap from OP, get_T_FOLLOW(hud_control.leadDistanceBars))
        distance = 30 if hud_control.leadVisible else 0 # TODO lead distance from model

        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled,
                                                       CS.esp_hold_confirmation, CC.cruiseControl.override or CS.out.gasPressed)
        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.pt, acc_hud_status, hud_control.setSpeed * CV.MS_TO_KPH,
                                                         hud_control.leadVisible, hud_control.leadDistanceBars, CS.esp_hold_confirmation,
                                                         distance, desired_gap))

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

    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.CCP.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last
    new_actuators.curvature = float(self.apply_curvature_last)
    new_actuators.accel = float(self.accel_last)

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
