import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL, apply_driver_steer_torque_limits, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mqbcan, pqcan, mebcan
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)
    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MEB:
      self.CCS = mebcan
    else:
      self.CCS = mqbcan
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.ext_bus = self.CAN.main if CP.networkLocation == structs.CarParams.NetworkLocation.fwdCamera else self.CAN.camera
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ
    self.openpilot_longitudinal = self.CP.openpilotLongitudinalControl and not self.CP.flags & VolkswagenFlags.MEB

    self.apply_torque_last = 0
    self.apply_curvature_last = 0
    self.apply_steer_power_last = 0
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
        # The QFK (lateral control coordinator) control loop compares actuation curvature to its own current curvature
        # Calibrate our actuator command by the offset between openpilot's vehicle model and QFK perceived curvatures
        apply_curvature = actuators.curvature + (CS.qfk_curvature - CC.currentCurvature)

        # Progressive QFK power control: smooth engage and disengage, reduce power when the driver is overriding
        qfk_enable = True
        if CC.latActive:
          min_power = max(self.apply_steer_power_last - self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MIN)
          max_power = max(self.apply_steer_power_last + self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MAX)
          target_power = int(np.interp(CS.out.steeringTorque, [self.CCP.STEER_DRIVER_ALLOWANCE, self.CCP.STEER_DRIVER_MAX],
                                                              [self.CCP.STEERING_POWER_MAX, self.CCP.STEERING_POWER_MIN]))
          apply_steer_power = min(max(target_power, min_power), max_power)
        elif self.apply_steer_power_last > 0:
          apply_steer_power = max(self.apply_steer_power_last - self.CCP.STEERING_POWER_STEP, 0)
        else:
          qfk_enable = False
          apply_curvature = 0
          apply_steer_power = 0

        can_sends.append(mebcan.create_steering_control(self.packer_pt, self.CAN.main, apply_curvature, qfk_enable, apply_steer_power))

        self.apply_curvature_last = apply_curvature
        self.apply_steer_power_last = apply_steer_power

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
          new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
          apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)
          self.hca_frame_timer_running += self.CCP.STEER_STEP
          if self.apply_torque_last == apply_torque:
            self.hca_frame_same_torque += self.CCP.STEER_STEP
            if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
              apply_torque -= (1, -1)[apply_torque < 0]
              self.hca_frame_same_torque = 0
          else:
            self.hca_frame_same_torque = 0
          hca_enabled = abs(apply_torque) > 0
        else:
          hca_enabled = False
          apply_torque = 0

        if not hca_enabled:
          self.hca_frame_timer_running = 0

        self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
        self.apply_torque_last = apply_torque
        can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.main, apply_torque, hca_enabled))

        if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
          # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
          # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
          # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
          ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
          if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
            ea_simulated_torque = CS.out.steeringTorque
          can_sends.append(self.CCS.create_eps_update(self.packer_pt, self.CAN.camera, CS.eps_stock_values, ea_simulated_torque))

    # TODO: refactor a bit
    if self.CP.flags & VolkswagenFlags.MEB:
      if self.frame % 2 == 0:
        can_sends.append(mebcan.create_ea_control(self.packer_pt, self.CAN.main))
      if self.frame % 50 == 0:
        can_sends.append(mebcan.create_ea_hud(self.packer_pt, self.CAN.main))

    # **** Acceleration Controls ******************************************** #

    if self.openpilot_longitudinal:
      if self.frame % self.CCP.ACC_CONTROL_STEP == 0:
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0)
        stopping = actuators.longControlState == LongCtrlState.stopping
        starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.main, CS.acc_type, CC.longActive, accel,
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
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.main, CS.ldw_stock_values, CC.latActive,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.openpilot_longitudinal:
      lead_distance = 0
      if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
        lead_distance = 512 if CS.upscale_lead_car_signal else 8
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
      # FIXME: follow the recent displayed-speed updates, also use mph_kmh toggle to fix display rounding problem?
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.main, acc_hud_status, set_speed,
                                                       lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.ext_bus, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.curvature = float(self.apply_curvature_last)

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
