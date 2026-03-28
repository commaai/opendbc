import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mlbcan, mqbcan, pqcan
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags

VisualAlert = structs.CarControl.HUDControl.VisualAlert


class HCAMitigation:
  """
  Manages HCA fault mitigations for VW/Audi EPS racks:
    * Reduces torque by 1 for a single frame after commanding the same torque value for too long
  """

  def __init__(self, CCP):
    self._max_same_torque_frames = CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP)
    self._same_torque_frames = 0

  def update(self, apply_torque, apply_torque_last):
    if apply_torque != 0 and apply_torque_last == apply_torque:
      self._same_torque_frames += 1
      if self._same_torque_frames > self._max_same_torque_frames:
        apply_torque -= (1, -1)[apply_torque < 0]
        self._same_torque_frames = 0
    else:
      self._same_torque_frames = 0

    return apply_torque


class MQBStandstillManager:
  """
  Extended standstill for MQB w/ ACC type 1

  Normally brake is commanded by the TSK. During a stopping procedure the ESP handles brake autonomously.
  If we exit the stopping procedure at the perfect moment, the ESP will hold indefinitely without complaining.

  One side effect of exiting the stopping procedure early is that we are prone to rollback on very steep hills.
  To avoid rolling back, we use ACC_06 to build engine torque while simultaneously using ACC_07 to hold the brake.
  If the engine is producing enough torque to prevent rollback the ESP will happily cycle its timer when we ask it to.
  """

  HOLD_MAX_FRAMES = 50             # frames to hold before disabling long control to avoid a fault

  def __init__(self):
    self.esp_hold_frames = 0
    self.prev_starting_hold = False
    self.can_stop_forever = False

  def update(self, CS, long_active: bool, accel: float, stopping: bool, starting: bool
             ) -> tuple[bool, float, bool, bool, bool | None, bool | None]:
    esp_starting_override: bool | None = None
    esp_stopping_override: bool | None = None

    if CS.esp_hold_confirmation:
      self.esp_hold_frames += 1

    # acc type 1 is sensitive to control signals when brake is pressed (when preEnabled)
    if CS.out.brakePressed:
      long_active = False

    # avoid a cruise fault if a hold is confirmed for too long
    if self.esp_hold_frames > self.HOLD_MAX_FRAMES:
      long_active = False

    # uphill launch: TSK rarely commands enough torque to move from a hill hold
    desired_launch_accel = 0.2 * CS.grade - 1
    if long_active and accel > 0 and desired_launch_accel > 0 and (CS.out.standstill or CS.rolling_backward):
      accel = max(accel, desired_launch_accel)

    # rollback prevention:
    if long_active and accel <= 0 and CS.rolling_backward:
      accel = -3.5

    # end the stopping procedure right after it starts, before any hold has been confirmed
    # if a hold is confirmed before we end the stopping procedure we won't be able to hold indefinitely
    if long_active:
      if CS.esp_stopping:
        self.can_stop_forever = True
      if CS.esp_hold_confirmation:
        self.can_stop_forever = False
      if not (stopping or starting):
        self.can_stop_forever = False
      # if CS.grade >= 12:
      #   self.can_stop_forever = False
      if self.can_stop_forever:
        esp_starting_override = True
        esp_stopping_override = False
    else:
      self.can_stop_forever = False

    # steep uphill: build engine torque via ACC_06 as rollback prevention, ESP braking held via ACC_07
    if long_active and accel <= 0 and not self.can_stop_forever and (CS.esp_hold_confirmation or CS.out.standstill):
      # skip torque management for one frame each cycle to avoid check engine light
      # if self.esp_hold_frames > 1:
      #   # too much torque and the car moves, too little and the ESP won't cycle its timer
      #   # targets 80% of torque needed to hold the car at stop, derived from ESP_15 and some experimentation
      #   hill_accel = 0.045 * CS.grade + 0.0625
      #   accel = max(accel, hill_accel)
      #   starting = True
      #   stopping = False
      # near counter limit: send starting request to cycle the ESP hold and reset the timer
      near_limit = self.esp_hold_frames >= self.HOLD_MAX_FRAMES - 10
      esp_starting_override = near_limit
      esp_stopping_override = not near_limit

    # standstill timer resets when:
    # - wheels move while hold is not confirmed
    # - we drop a hold confirmation after a frame where we were actively starting with a confirmed hold
    is_starting = long_active and (esp_starting_override if esp_starting_override is not None else starting)
    if not CS.out.standstill and not CS.esp_hold_confirmation:
      self.esp_hold_frames = 0
    elif long_active and self.prev_starting_hold and not CS.esp_hold_confirmation:
      self.esp_hold_frames = 0
    self.prev_starting_hold = is_starting and CS.esp_hold_confirmation

    return long_active, accel, stopping, starting, esp_starting_override, esp_stopping_override


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
    self.hca_mitigation = HCAMitigation(self.CCP)
    self.standstill_manager = MQBStandstillManager()
    self.distance_button_was_stopped = None

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      apply_torque = 0
      if CC.latActive:
        new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)

      apply_torque = self.hca_mitigation.update(apply_torque, self.apply_torque_last)
      hca_enabled = apply_torque != 0
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
        long_active = CC.longActive
        accel = actuators.accel
        esp_starting_override = None
        esp_stopping_override = None
        starting = CS.out.vEgo < self.CP.vEgoStopping and accel >= 0
        stopping = CS.out.vEgo < self.CP.vEgoStopping and not starting

        # distance button debug helper, force stop or start when distance button is pressed
        if self.CCS == mqbcan and CS.distance_button_pressed:
          if self.distance_button_was_stopped is None:
            self.distance_button_was_stopped = CS.out.standstill
          if long_active:
            if self.distance_button_was_stopped:
              accel = max(1.5, accel)
              stopping = False
              starting = CS.out.vEgo < self.CP.vEgoStopping
            else:
              accel = min(-1.5, accel)
              stopping = CS.out.vEgo < self.CP.vEgoStopping
              starting = False
        else:
          self.distance_button_was_stopped = None

        if self.CCS == mqbcan and CS.acc_type == 1:
          long_active, accel, stopping, starting, esp_starting_override, esp_stopping_override = \
            self.standstill_manager.update(CS, long_active, accel, stopping, starting)

        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, long_active)
        accel = float(np.clip(accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if long_active else 0)

        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, long_active, accel,
                                                           acc_control, stopping, starting, CS.esp_hold_confirmation,
                                                           esp_starting_override, esp_stopping_override))

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
