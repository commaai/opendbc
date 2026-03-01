import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mlbcan, mqbcan, pqcan
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

HOLD_TORQUE_DEADBAND_NM = 20   # stop integrating when this close to ESP_Haltemoment (Nm at wheel)
HOLD_ACCEL_KI = 0.00002        # I-controller gain: m/s² per Nm of torque error per ACC_CONTROL_STEP


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ

    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MLB:
      self.CCS = mlbcan
    else:
      self.CCS = mqbcan

    self.apply_torque_last = 0
    self.gra_acc_counter_last = None
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0

    self.hold_counter = 0
    self.previous_resettable = False
    self.previous_impulse_count = 0
    self.distance_button_was_stopped = None
    self.hold_accel = 0.0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
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
      can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_torque, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, self.CAN.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #

    if self.CP.openpilotLongitudinalControl:
      if self.frame % self.CCP.ACC_CONTROL_STEP == 0:
        esp_starting_override = None
        esp_stopping_override = None
        allow_indefinite_hold = not CS.esp_hold_uphill
        long_active = (
          # acc type 1 is sensitive to control signals when brake pressed (i.e. preEnabled)
          False if CS.acc_type == 1 and CS.out.brakePressed else
          # stop regulating if needed to prevent a cruise fault
          False if CS.acc_type == 1 and self.hold_counter > self.CCP.MAX_HOLD_FRAMES and not allow_indefinite_hold else
          CC.longActive
        )
        if CS.esp_hold_confirmation:
          self.hold_counter += 1
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, long_active)
        accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if long_active else 0)
        stopping = actuators.longControlState == LongCtrlState.stopping
        starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)

        # distance button debug helper, force stop or start when distance button is pressed
        if CS.distance_button_pressed:
          if self.distance_button_was_stopped is None:
            self.distance_button_was_stopped = CS.esp_standstill_confirmation
          if long_active:
            if self.distance_button_was_stopped:
              accel = max(1.5, accel)
              stopping = False
              starting = CS.out.vEgo < self.CP.vEgoStopping if long_active else False
            else:
              accel = min(-1.5, accel)
              stopping = CS.out.vEgo < self.CP.vEgoStopping if long_active else False
              starting = False
        else:
          self.distance_button_was_stopped = None

        # Standstill handling for MQB w/ ACC type 1.
        # When hold is confirmed, TSK stops sending brake requests — the ESP manages braking
        # autonomously, responding only to starting/stopping signals on ACC_07.
        # On flat/downhill the ESP can hold indefinitely; we just drop the hold and prevent
        # re-engagement. On uphill it can't, so we use ACC_06 to build engine torque as a rollback
        # prevention measure so that the ESP will willingly cycle its own hold timer when we ask
        # while keeping ACC_07 in stopping mode so ESP holds the brake.
        if self.CCS == mqbcan and CS.acc_type == 1 and long_active:
          # flat/downhill: drop hold, prevent re-engagement
          if allow_indefinite_hold and CS.esp_standstill_confirmation:
            esp_starting_override = not CS.esp_hold_confirmation
            esp_stopping_override = False

          # uphill: build engine torque via ACC_06, ESP braking held via ACC_07
          elif not allow_indefinite_hold and (CS.esp_hold_confirmation or CS.esp_standstill_confirmation):
            # esp_hold_torque_nm is 0 when invalid; error goes negative and the integrator backs off.
            error_nm = CS.esp_hold_torque_nm - CS.actual_torque_nm
            if abs(error_nm) > HOLD_TORQUE_DEADBAND_NM:
              self.hold_accel = float(np.clip(self.hold_accel + HOLD_ACCEL_KI * error_nm, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX))
            accel = max(accel, self.hold_accel)
            starting = True
            stopping = False
            # near counter limit: attempt hold release to cycle the ESP hold;
            # if torque management worked, hold_confirmation drops and the counter resets
            near_limit = self.hold_counter >= self.CCP.MAX_HOLD_FRAMES - 10
            esp_starting_override = near_limit
            esp_stopping_override = not near_limit
          else:
            self.hold_accel = 0.0

        # our standstill timer resets when either:
        # - wheels move while a hold is not confirmed
        # wheel impulses update earlier than vEgo, so we can reset the timer faster by watching them directly
        # - we drop a hold confirmation after a frame where we were actively starting with hold confirmed
        # (previous_resettable captures is_starting AND hold_confirmation from the prior frame, which is sufficient)
        is_starting = long_active and (esp_starting_override if esp_starting_override is not None else starting)
        if CS.wheel_impulse_count != self.previous_impulse_count and not CS.esp_hold_confirmation:
          self.hold_counter = 0
        elif self.previous_resettable and not CS.esp_hold_confirmation:
          self.hold_counter = 0
        self.previous_resettable = is_starting and CS.esp_hold_confirmation
        self.previous_impulse_count = CS.wheel_impulse_count

        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, long_active, accel,
                                                           acc_control, stopping, starting, CS.esp_hold_confirmation, esp_starting_override, esp_stopping_override))

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
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      lead_distance = 0
      if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
        lead_distance = 512 if CS.upscale_lead_car_signal else 8
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
      # FIXME: PQ may need to use the on-the-wire mph/kmh toggle to fix rounding errors
      # FIXME: Detect clusters with vEgoCluster offsets and apply an identical vCruiseCluster offset
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed,
                                                       lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.CAN.ext, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
