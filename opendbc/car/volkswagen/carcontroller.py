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
  WEGIMPULSE_STILLNESS_FRAMES = 10    # frames of no wheel tick change before assuming standstill

  def __init__(self, vehicle_mass: float = 1540.0):
    self.vehicle_mass = vehicle_mass
    self.can_stop_forever = False
    self.rollback_detected = False
    self.stop_commit_active = False
    self.start_commit_active = False
    self.frames_since_wegimpulse_change = 0
    self.prev_sum_wegimpulse: int | None = None

  def get_theoretical_safe_speed(self, grade_pct: float, v_ego: float) -> float:
    # Because brake torque is based off a jerk-limited speed target even at standstill, the TSK may
    # not be able to build torque fast enough to prevent rollback when the car is moving slowly. If
    # the car is moving fast enough, we can rely on momentum to prevent rollback while the TSK is
    # building brake torque. Below this speed we lose our momentum buffer and risk rollback, so we
    # must force the car to stop prematurely. Higher grades require a higher minimum safe speed.
    if grade_pct <= 0 or self.vehicle_mass <= 0:
      return 0.0

    sin_theta = grade_pct / math.sqrt(grade_pct ** 2 + 10000.0)
    grade_accel = self.GRAVITY * sin_theta
    brake_decel_build_rate = self.BRAKE_TORQUE_RAMP_RATE / (self.vehicle_mass * self.ASSUMED_WHEEL_RADIUS)

    return 1.5 * grade_accel ** 2 / brake_decel_build_rate

  def update(self, CS, long_active: bool, accel: float, stopping: bool, starting: bool,
             max_planned_speed: float) -> tuple[bool, float, bool, bool, "mqbcan.ESPOverride | None"]:
    esp_override: mqbcan.ESPOverride | None = None
    theoretical_safe_stop_speed = self.get_theoretical_safe_speed(CS.grade, CS.out.vEgo)
    can_leave_stop = max_planned_speed > theoretical_safe_stop_speed
    hill_launch_accel = 0.2 * CS.grade - 1

    if CS.rolling_backward:
      self.rollback_detected = True
    elif CS.rolling_forward:
      self.rollback_detected = False

    if CS.sum_wegimpulse != self.prev_sum_wegimpulse:
      self.frames_since_wegimpulse_change = 0
    else:
      self.frames_since_wegimpulse_change += 1
    self.prev_sum_wegimpulse = CS.sum_wegimpulse
    # this is more sensitive than CS.out.standstill so we can exit before the hold confirmation fires
    # vEgo lags behind the actual impulse signals and is too delayed for us to use reliably here
    at_standstill = self.frames_since_wegimpulse_change >= self.WEGIMPULSE_STILLNESS_FRAMES

    # acc type 1 is sensitive to control signals when brake is pressed (when preEnabled)
    if CS.out.brakePressed:
      long_active = False

    # rollback prevention!
    # If we drop below our safe speed, we must force the car to stop. We remain stopped until the
    # vehicle has strong intent to drive away to prevent a scenario where we want to stop but cannot build
    # brake torque fast enough to prevent rollback.
    if long_active:
      # this only happens if the user preEnables, we must handle it
      if CS.esp_hold_confirmation:
        self.start_commit_active = True
        self.stop_commit_active = False
      # start commit ends when we exceed safe stop speed
      elif self.start_commit_active:
        if CS.out.vEgo > theoretical_safe_stop_speed and not at_standstill:
          self.start_commit_active = False
          self.stop_commit_active = False
      # stop commit ends if we want to drive away
      elif self.stop_commit_active:
        if can_leave_stop and accel > 0:
          self.start_commit_active = True
          self.stop_commit_active = False
      # trigger stop commit when necessary
      elif CS.out.vEgo < theoretical_safe_stop_speed:
        self.start_commit_active = False
        self.stop_commit_active = True
    else:
      self.start_commit_active = False
      self.stop_commit_active = False

    # apply acceleration adjustments based on our current rollback prevention state
    if long_active:
      if self.stop_commit_active or (self.rollback_detected and accel <= 0):
        accel = -3.5
        stopping = True
        starting = False
      elif self.start_commit_active or (self.rollback_detected and accel > 0):
        accel = max(accel, hill_launch_accel, 0.2)
        stopping = False
        starting = True

    # the magic sauce for infinite standstill
    # begin a stopping procedure, then exit to starting state before the car reaches standstill
    if long_active:
      # reset if hold is confirmed
      if CS.esp_hold_confirmation:
        self.can_stop_forever = False
      # reset when moving
      elif CS.out.vEgo > 1 and not CS.esp_stopping:
        self.can_stop_forever = False
      # latch into holding state when detected
      elif CS.esp_stopping or self.can_stop_forever:
        self.can_stop_forever = True
        esp_override = mqbcan.ESPOverride.START
      # trigger stopping state when almost stopped or during a stop commit
      elif at_standstill or self.stop_commit_active:
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
    self.standstill_manager = MQBStandstillManager(CP.mass)

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
          long_active, accel, stopping, starting, esp_override = \
            self.standstill_manager.update(CS, long_active, accel, stopping, starting, actuators.maxPlannedSpeed)

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
