import math
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
  Extended standstill for MQB w/ ACC type 1.

  Normally brake is commanded by the TSK. During a stopping procedure the ESP handles brake autonomously.
  If we exit the stopping procedure at the perfect moment, the ESP will hold indefinitely without complaining.

  It does get slightly more complicated than that because we must manually prevent rollback.
  """

  BRAKE_TORQUE_RAMP_RATE = 2000.0     # Nm/s
  ASSUMED_WHEEL_RADIUS = 0.328        # m, typical MQB tire rolling radius
  GRAVITY = 9.81                      # m/s^2
  WEGIMPULSE_STILLNESS_FRAMES = 5     # frames of no wheel tick change before assuming standstill
  ESP_OVERRIDE_SPEED = 9.5 * CV.KPH_TO_MS

  def __init__(self, vehicle_mass: float, accel_min: float):
    self.vehicle_mass = vehicle_mass
    self.accel_min = accel_min
    self.can_stop_forever = False
    self.rollback_detected = False
    self.start_commit_active = False
    self.frames_since_last_wheel_pulse = 0
    self.prev_sum_wegimpulse: int | None = None

  def get_hill_hold_decel_deficit(self, grade_pct: float, brake_torque: float) -> float:
    """
    Estimate how much more braking deceleration is needed to hold the car on an uphill slope
    """
    if self.vehicle_mass <= 0:
      return 0.0

    hill_hold_decel = self.GRAVITY * np.clip(grade_pct, 0, 30) / 100
    brake_decel = max(brake_torque, 0.0) / (self.vehicle_mass * self.ASSUMED_WHEEL_RADIUS)
    return max(hill_hold_decel - brake_decel, 0.0)

  def get_safe_speed_for_brake_torque(self, grade_pct: float, brake_torque: float) -> float:
    """
    Brake deceleration is slow to build.

    Above this speed we can stop without rolling back thanks to forward momentum.
    Below this speed there isn't enough time to build the missing brake decel before we roll backward.
    """
    missing_brake_decel = self.get_hill_hold_decel_deficit(grade_pct, brake_torque)
    if missing_brake_decel <= 0 or self.vehicle_mass <= 0:
      return 0.0

    brake_decel_build_rate = self.BRAKE_TORQUE_RAMP_RATE / (self.vehicle_mass * self.ASSUMED_WHEEL_RADIUS)
    forward_speed_needed_while_brake_builds = 1.5 * missing_brake_decel ** 2 / brake_decel_build_rate

    return forward_speed_needed_while_brake_builds

  def get_blended_brake_accel(self, raw_accel: float, v_ego: float, grade_pct: float, brake_torque: float) -> float:
    """
    Bias raw openpilot accel toward hard braking as rollback risk rises.
    """
    zero_brake_decel_deficit = self.get_hill_hold_decel_deficit(grade_pct, 0.0)
    current_brake_decel_deficit = self.get_hill_hold_decel_deficit(grade_pct, brake_torque)
    zero_brake_safe_speed = self.get_safe_speed_for_brake_torque(grade_pct, 0.0)
    if zero_brake_decel_deficit <= 0 or zero_brake_safe_speed <= 0:
      return raw_accel

    # risk ≈ how much should we care on a scale from 0 to 1
    brake_deficit_risk = current_brake_decel_deficit / zero_brake_decel_deficit
    speed_risk = max(zero_brake_safe_speed - v_ego, 0.0) / zero_brake_safe_speed
    rollback_risk = float(np.clip(speed_risk * brake_deficit_risk, 0.0, 1.0))
    blended_accel = raw_accel + rollback_risk * (self.accel_min - raw_accel)
    return min(raw_accel, blended_accel)

  def update(self, CS, long_active: bool, accel: float, stopping: bool, starting: bool,
             max_planned_speed: float, grade_pct: float,
             tsk_brake_torque: float) -> tuple[bool, float, bool, bool, "mqbcan.ESPOverride | None"]:

    safe_stopping_speed = self.get_safe_speed_for_brake_torque(grade_pct, 0.0)
    below_safe_stop_speed = CS.out.vEgo < safe_stopping_speed
    can_accelerate = max_planned_speed > safe_stopping_speed
    takeoff_acceleration = max(0.2, 0.1 * grade_pct)
    esp_override = mqbcan.ESPOverride.START if CS.out.vEgo < self.ESP_OVERRIDE_SPEED else None

    if CS.rolling_backward:
      self.rollback_detected = True
    elif CS.rolling_forward:
      self.rollback_detected = False

    wheel_did_pulse = CS.sum_wegimpulse != self.prev_sum_wegimpulse
    self.prev_sum_wegimpulse = CS.sum_wegimpulse
    if wheel_did_pulse:
      self.frames_since_last_wheel_pulse = 0
    else:
      self.frames_since_last_wheel_pulse += 1
    # this is far more sensitive than CS.out.standstill
    # we want to trigger this *right before* the car itself thinks it's at a standstill
    # vEgo lags behind the actual impulse signals and is too delayed for us to use here
    near_standstill = self.frames_since_last_wheel_pulse >= self.WEGIMPULSE_STILLNESS_FRAMES

    # acc type 1 is sensitive to control signals when brake is pressed (when preEnabled)
    if CS.out.brakePressed:
      long_active = False

    # manage takeoff behavior
    if long_active and not CS.out.gasPressed:
      # this only happens if the user preEnables, we must handle it to prevent faulting
      if CS.esp_hold_confirmation:
        self.start_commit_active = True
      # trigger a start commit when openpilot wants to drive
      if can_accelerate and below_safe_stop_speed and accel > 0:
        self.start_commit_active = True
      # start commit ends when we exceed safe stop speed
      elif self.start_commit_active:
        if CS.out.vEgo > safe_stopping_speed:
          self.start_commit_active = False
    else:
      self.start_commit_active = False

    # apply acceleration adjustments based on our current rollback prevention state
    if long_active:
      raw_accel = accel
      if self.start_commit_active:
        accel = max(accel, takeoff_acceleration)
        stopping = False
        starting = True
      elif self.rollback_detected:
        accel = self.accel_min
        stopping = True
        starting = False
      elif below_safe_stop_speed:
        accel = self.get_blended_brake_accel(accel, CS.out.vEgo, grade_pct, tsk_brake_torque)
        if accel < raw_accel:
          stopping = True
          starting = False
      # during our (short) stopping procedure, there's a short moment where we have no brake at all
      # prime the TSK so that it resumes at a reasonable level when the procedure ends
      if near_standstill and accel < 0 and tsk_brake_torque == 0:
        accel = self.accel_min
        stopping = True
        starting = False

    # the magic sauce for infinite standstill
    # begin a stopping procedure, then exit to starting state before the car reaches standstill
    if long_active:
      # reset when moving fast
      if CS.out.vEgo > self.ESP_OVERRIDE_SPEED:
        self.can_stop_forever = False
      # reset if hold is confirmed
      if CS.esp_hold_confirmation:
        self.can_stop_forever = False

      # force ESP into starting state during a start commit
      if self.start_commit_active:
        esp_override = mqbcan.ESPOverride.START
      # latch into starting state when a hold procedure is detected
      elif CS.esp_stopping:
        self.can_stop_forever = True
        esp_override = mqbcan.ESPOverride.START
      elif self.can_stop_forever:
        esp_override = mqbcan.ESPOverride.START
      # trigger a stopping procedure
      elif near_standstill:
        esp_override = mqbcan.ESPOverride.STOP
    else:
      self.can_stop_forever = False

    return long_active, accel, stopping, starting, esp_override


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
    self.standstill_manager = MQBStandstillManager(CP.mass, self.CCP.ACCEL_MIN)

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
        esp_override = None
        stopping = CS.out.vEgo < self.CP.vEgoStopping and actuators.longControlState == LongCtrlState.stopping
        starting = CS.out.vEgo < self.CP.vEgoStopping and not stopping

        if self.CCS == mqbcan and CS.acc_type == 1:
          grade_pct = math.tan(CC.orientationNED[1]) * 100.0 if len(CC.orientationNED) == 3 else 0.0
          long_active, accel, stopping, starting, esp_override = \
            self.standstill_manager.update(CS, long_active, accel, stopping, starting, actuators.maxPlannedSpeed,
                                           grade_pct, CS.tsk_brake_torque)

        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, long_active)
        accel = float(np.clip(accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if long_active else 0)

        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, long_active, accel,
                                                           acc_control, stopping, starting, CS.esp_hold_confirmation,
                                                           esp_override))

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
