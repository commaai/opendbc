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
  Standstill handling for MQB w/ ACC type 1.

  When hold is confirmed, TSK stops sending brake requests and the ESP manages braking
  autonomously, responding only to starting/stopping signals on ACC_07.
  On flat/downhill the ESP can hold indefinitely; we just force starting state.
  On uphill it can't, so we use ACC_06 to build engine torque as a rollback
  prevention measure so that the ESP will willingly cycle its own hold timer when we ask
  it to while keeping ACC_07 in stopping mode so ESP holds the brake.
  """

  HOLD_MAX_FRAMES = 50             # frames to hold before disabling long control to avoid a fault
  HOLD_RELEASE_TOTAL_FRAMES = 20  # reserve the last hold-confirmed frames for progressive hill-release pulses
  HOLD_RELEASE_WINDOW_FRAMES = 3  # allow a few frames for ESP to acknowledge a recent hill release request

  def __init__(self, CCP):
    self._CCP = CCP
    self.esp_hold_frames = 0
    self._recent_release_request_frames = 0
    self._can_stop_forever = False

  def update(self, CS, long_active: bool, accel: float, stopping: bool, starting: bool
             ) -> tuple[bool, float, bool, bool, bool | None, bool | None]:
    esp_starting_override: bool | None = None
    esp_stopping_override: bool | None = None

    if CS.esp_hold_confirmation:
      self.esp_hold_frames += 1

    # acc type 1 is sensitive to control signals when brake is pressed (preEnabled)
    if CS.out.brakePressed:
      long_active = False
    # stop regulating if hold has been held too long on a hill to avoid a cruise fault
    elif self.esp_hold_frames > self.HOLD_MAX_FRAMES:
      long_active = False

    # uphill launch: TSK rarely commands enough torque to move from a hill hold, so keep accel > 1 m/s²
    if long_active and accel > 0 and CS.grade > 4 and CS.out.standstill:
      accel = max(accel, 1.0)
    if long_active and CS.rolling_backward and accel > 0:
      accel = max(accel, 1.0)

    # stopping procedure
    if long_active:
      if CS.esp_stopping:
        self._can_stop_forever = True
      if CS.esp_hold_confirmation:
        self._can_stop_forever = False
      if not (stopping or starting):
        self._can_stop_forever = False
      if CS.grade > 12:
        self._can_stop_forever = False
      if self._can_stop_forever:
        esp_starting_override = True
        esp_stopping_override = False
    else:
      self._can_stop_forever = False

    # steep uphill: build engine torque via ACC_06 as rollback prevention, ESP braking held via ACC_07
    if long_active and accel <= 0 and not self._can_stop_forever and (CS.esp_hold_confirmation or CS.out.standstill):
      # skip torque management for one frame each cycle to avoid check engine light
      if self.esp_hold_frames > 1:
        hill_accel = 0.045 * CS.grade + 0.0625
        accel = max(accel, hill_accel)
        starting = True
        stopping = False
      # Near the counter limit, send progressively longer starting pulses:
      # 1 frame, wait 3, 2 frames, wait 3, 3 frames, wait 3, then hold starting until cutoff.
      release_phase = self.esp_hold_frames - (self.HOLD_MAX_FRAMES - self.HOLD_RELEASE_TOTAL_FRAMES + 1)
      is_release_attempt = release_phase >= 0 and release_phase not in (1, 2, 3, 6, 7, 8, 12, 13, 14)
      esp_starting_override = is_release_attempt
      esp_stopping_override = not is_release_attempt

    # standstill timer resets when:
    # - wheels move while hold is not confirmed
    # - we drop a hold confirmation shortly after actively starting with hold confirmed
    is_starting = long_active and (esp_starting_override if esp_starting_override is not None else starting)
    if CS.out.vEgo > 0 and not CS.esp_hold_confirmation:
      self.esp_hold_frames = 0
      self._recent_release_request_frames = 0
    elif self._recent_release_request_frames > 0 and not CS.esp_hold_confirmation:
      self.esp_hold_frames = 0
      self._recent_release_request_frames = 0

    if not long_active:
      self._recent_release_request_frames = 0
    elif is_starting and CS.esp_hold_confirmation:
      self._recent_release_request_frames = self.HOLD_RELEASE_WINDOW_FRAMES
    elif self._recent_release_request_frames > 0:
      self._recent_release_request_frames -= 1

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
    self.standstill_manager = MQBStandstillManager(self.CCP)
    self.distance_button_was_stopped = None  # DEBUG HELPER

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
        accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if long_active else 0)
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
              starting = CS.out.vEgo < self.CP.vEgoStopping if long_active else False
            else:
              accel = min(-1.5, accel)
              stopping = CS.out.vEgo < self.CP.vEgoStopping if long_active else False
              starting = False
        else:
          self.distance_button_was_stopped = None

        esp_starting_override = None
        esp_stopping_override = None
        if self.CCS == mqbcan and CS.acc_type == 1:
          long_active, accel, stopping, starting, esp_starting_override, esp_stopping_override = \
            self.standstill_manager.update(CS, long_active, accel, stopping, starting)

        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, long_active)

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
