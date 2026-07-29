import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mebcan, mlbcan, mqbcan, pqcan
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


class MebBrakeOnlyRepro:
  """
  REPRO BRANCH ONLY. Reproduces the TSK_Status temp fault from
  f73c01590368ee5b/00000088--a80293d8f7 seg8 (route t=493): openpilot requested ANFAHREN with
  positive accel while the drivetrain was still in TSK_Status 5 (brake_only), and TSK went to 6
  90 ms later. In that log a gas press landed in the same 120 ms window, so it can't be told apart
  from the ANFAHREN request. This runs the sequence with no pedal input so the two separate.

  Alternates on every engagement, for as long as that engagement lasts:
    1st engage: brake
    2nd engage: request accel (ANFAHREN once stopped)
    3rd engage: brake
    ...

  So the run is: engage and let it brake, brake pedal to disengage (drivetrain enters brake_only),
  re-engage and it requests. Distinctive accel values make each attempt easy to find in the logs.
  """
  STOP_ACCEL = -1.5
  REQUEST_ACCEL = 0.55
  HOLD_SPEED = 0.3          # below this, hold with HALTEN instead of braking in pid

  # Both TSK faults so far came 0.08-0.09 s after HMS 4->0 and accel->0.0 landed on the same frame,
  # which is what the gas pedal was producing via the override. Doing it in code removes the pedal
  # from the loop. Gas landed 0.41 s and 1.05 s into the request on the two faults, so sweep that
  # range across successive request engagements instead of guessing one value.
  WITHDRAW_DELAY = 0.55     # how long ANFAHREN runs before the simulated gas press lands
  WITHDRAW_HOLD = 1.0       # how long the simulated gas press lasts before the request resumes

  def __init__(self):
    self.engage_count = 0
    self.prev_enabled = False
    self.request_frames = 0

  def update(self, CC, CS, accel, long_control_state):
    # count on enabled, not longActive, so a gas override doesn't flip the parity mid-attempt
    if CC.enabled and not self.prev_enabled:
      self.engage_count += 1
      self.request_frames = 0
    self.prev_enabled = CC.enabled

    # but never command actuation while overriding
    if not CC.longActive:
      return accel, long_control_state

    if self.engage_count % 2:
      # odd engagement: brake. fault 0 was braking in pid, not stopping, when the driver braked at
      # 2.2 m/s, so HALTEN was never sent and the re-engage went KEINE_ANFORDERUNG -> ANFAHREN
      # directly. Below HOLD_SPEED we fall back to HALTEN so the car is held if it does stop.
      return self.STOP_ACCEL, LongCtrlState.pid if CS.out.vEgo > self.HOLD_SPEED else LongCtrlState.stopping

    # even engagement: request drive-off, then withdraw the way the gas override does.
    # ANFAHREN only goes out once ESP confirms standstill, so time the withdrawal from there.
    if CS.esp_hold_confirmation:
      self.request_frames += 1
    elapsed = self.request_frames * DT_CTRL

    if self.WITHDRAW_DELAY <= elapsed < self.WITHDRAW_DELAY + self.WITHDRAW_HOLD:
      # simulate the gas override: HMS 4 -> 0 and accel -> 0.0 on the same frame, exactly as fault 0
      return 0.0, LongCtrlState.off

    # request before and after, so the withdrawal is the only event. never hand back to the policy
    # here -- at standstill it flip-flops pid/stopping and oscillates HMS, which is its own fault mode
    return self.REQUEST_ACCEL, LongCtrlState.pid


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ

    if CP.flags & VolkswagenFlags.MEB:
      self.meb_long_state = mebcan.MebLongStateMachine(self.CP, self.CCP)
      self.brake_only_repro = MebBrakeOnlyRepro()

    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MLB:
      self.CCS = mlbcan
    else:
      self.CCS = mqbcan

    self.apply_torque_last = 0
    self.apply_curvature_last = 0.
    self.steering_power_last = 0
    self.accel_last = 0.
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0
    self.gra_acc_counter_last = None
    self.hca_mitigation = HCAMitigation(self.CCP)

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    accel_req, long_control_state = actuators.accel, actuators.longControlState
    if self.CP.flags & VolkswagenFlags.MEB and self.CP.openpilotLongitudinalControl:
      accel_req, long_control_state = self.brake_only_repro.update(CC, CS, accel_req, long_control_state)

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      apply_torque = 0
      if self.CP.flags & VolkswagenFlags.MEB:
        # Logic to avoid HCA refused state:
        #   * steering power as counter and near zero before OP lane assist deactivation
        # MEB rack can be used continuously without time limits
        # maximum real steering angle change ~ 120-130 deg/s

        if CC.latActive:
          hca_enabled = True
          # compensate the gap between measured and current curvature
          apply_curvature = actuators.curvature + (CS.curvature_meas - CC.currentCurvature)
          apply_curvature = self.CCP.CURVATURE_LIMITS.apply_limits(apply_curvature, self.apply_curvature_last, CS.out.vEgoRaw, CS.curvature_meas,
                                                                   CC.latActive, self.CCP.STEER_STEP)

          min_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MIN)
          max_power = min(self.steering_power_last + self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MAX)
          target_power_driver = int(np.interp(CS.out.steeringTorque, [self.CCP.STEER_DRIVER_ALLOWANCE, self.CCP.STEER_DRIVER_MAX],
                                                                     [self.CCP.STEERING_POWER_MAX, self.CCP.STEERING_POWER_MIN]))
          target_power = int(np.interp(CS.out.vEgo, [0., 0.5], [self.CCP.STEERING_POWER_MIN, target_power_driver]))
          steering_power = min(max(target_power, min_power), max_power)

        else:
          if self.steering_power_last > 0:  # keep HCA alive until steering power has reduced to zero
            hca_enabled = True
            apply_curvature = float(np.clip(CS.curvature_meas, -self.CCP.CURVATURE_MAX, self.CCP.CURVATURE_MAX))
            steering_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, 0)
          else:
            hca_enabled = False
            apply_curvature = 0.  # inactive curvature
            steering_power = 0

        can_sends.append(mebcan.create_steering_control(self.packer_pt, self.CAN.pt, apply_curvature, hca_enabled, steering_power))
        self.apply_curvature_last = apply_curvature
        self.steering_power_last = steering_power

      else:
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

    # Emergency Assist intervention
    if self.CP.flags & VolkswagenFlags.MEB and self.CP.flags & VolkswagenFlags.STOCK_KLR_PRESENT:
      # send capacitive steering wheel hands-on message to keep ACC resume active and control Emergency Assist
      # MEB Emergency Assist brake jerks after 30s of continued hands-off time.
      # We send the stock wheeltouch message to start the stock DM timer when openpilot latches the critical driver monitoring alert
      if self.frame % self.CCP.KLR_01_STEP == 0:
        lat_active = CC.latActive and not CC.driverMonitoringEscalation
        can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.cam, lat_active, CS.klr_stock_values))
        can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.pt, lat_active, CS.klr_stock_values))

    # **** Acceleration Controls ******************************************** #

    if self.CP.openpilotLongitudinalControl:
      if self.frame % self.CCP.ACC_CONTROL_STEP == 0:
        if self.CP.flags & VolkswagenFlags.MEB:
          accel = float(np.clip(accel_req, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX))
          accel, acc_status, acc_hold_type, braking_to_stop = self.meb_long_state.update(CS, CC, accel, long_control_state)
          can_sends.extend(mebcan.create_acc_accel_control(self.packer_pt, self.CAN.pt, self.CCP, CS.acc_type, CC.enabled,
                                                           accel, acc_status, acc_hold_type, braking_to_stop,
                                                           CS.out.vEgoRaw * CV.MS_TO_KPH, CS.travel_assist_available))

        else:
          stopping = actuators.longControlState == LongCtrlState.stopping
          acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
          accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0)
          starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < 0.25)
          can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, CC.longActive, accel,
                                                             acc_control, stopping, starting, CS.esp_hold_confirmation))

        self.accel_last = accel

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

    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      self.distance_bar_frame = self.frame

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      if self.CP.flags & VolkswagenFlags.MEB:
        fcw_alert = hud_control.visualAlert == VisualAlert.fcw
        show_distance_bars = self.frame - self.distance_bar_frame < 400
        lead_distance = 0
        if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:
          lead_distance = 8
        can_sends.append(mebcan.create_acc_hud_control(self.packer_pt, self.CAN.pt, self.meb_long_state.acc_status, hud_control.setSpeed * CV.MS_TO_KPH,
                                                       hud_control.leadVisible, hud_control.leadDistanceBars, show_distance_bars,
                                                       lead_distance, fcw_alert))

      else:
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
    new_actuators.curvature = self.apply_curvature_last
    new_actuators.accel = self.accel_last

    self.lead_distance_bars_last = hud_control.leadDistanceBars
    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
