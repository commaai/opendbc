import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, rate_limit, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.longitudinal import BREAKAWAY_FRAMES, StandstillHold
from opendbc.car.mazda.radar_session import RadarSessionManager
from opendbc.car.mazda.values import CarControllerParams, Buttons

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState
LONG_BUSES = (0, 2)


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.stop_and_go = StandstillHold()
    self.radar_session = RadarSessionManager()
    self.long_counter = 0
    self.radar_counter = 0
    self.accel_last = 0.
    self.release_ramp = None
    self.breakaway_frames = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    apply_torque = 0

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_torque = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, CarControllerParams)

    stock_mrcc_owns_cruise = self.CP.openpilotLongitudinalControl and not CS.radar_was_silenced
    if CC.cruiseControl.cancel and not stock_mrcc_owns_cruise:
      # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
      # a race condition with the stock system, where the second cancel from openpilot
      # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
      # read 3 messages and most likely sync state before we attempt cancel.
      self.brake_counter = self.brake_counter + 1
      if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
        # Cancel Stock ACC if it's enabled while OP is disengaged
        # Send at a rate of 10hz until we sync with stock ACC state
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
    else:
      self.brake_counter = 0
      if not self.CP.openpilotLongitudinalControl and CC.cruiseControl.resume and self.frame % 5 == 0:
        # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
        # Send Resume button when planner wants car to move
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))

    self.apply_torque_last = apply_torque

    if self.CP.openpilotLongitudinalControl:
      can_sends.extend(self.update_longitudinal(CC, CS))

    # send HUD alerts
    if self.frame % 50 == 0:
      ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      # TODO: find a way to silence audible warnings so we can add more hud alerts
      steer_required = steer_required and CS.lkas_allowed_speed
      can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

    # send steering command
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_torque, CS.cam_lkas))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    if self.CP.openpilotLongitudinalControl:
      new_actuators.accel = self.accel_last

    self.frame += 1
    return new_actuators, can_sends

  def update_longitudinal(self, CC, CS):
    can_sends = []

    setup_ok = CS.out.canValid and CS.fsc_settled and not (CS.stock_radar_alive and CS.cruise_enabled)
    self.radar_session.update(setup_ok, CS.stock_radar_alive, CS.stock_radar_gone,
                              CS.out.standstill, self.frame)
    radar_master = self.radar_session.replacement_active
    CS.radar_control_active = radar_master and not (self.radar_session.restoring or self.radar_session.failed)
    CS.radar_takeover_failed = self.radar_session.failed

    if CS.out.canValid and self.radar_session.diagnostic_message is not None:
      can_sends.append(self.radar_session.diagnostic_message)

    control_ready = CS.radar_control_active and CS.stock_radar_gone and CS.out.canValid
    long_engaged = CC.enabled and control_ready
    long_active = CC.longActive and control_ready
    stopping = CC.actuators.longControlState == LongCtrlState.stopping
    sm = self.stop_and_go
    sm.update(long_engaged, stopping, CS.out.standstill, CC.actuators.accel,
              CS.brake_hold, CS.out.gasPressed)

    if sm.just_released:
      self.release_ramp = CarControllerParams.ACCEL_HOLD_LATCHED if sm.latched_release else CarControllerParams.ACCEL_RELEASE_BAND
    elif sm.holding or not long_active:
      self.release_ramp = None

    accel = 0.
    if long_active:
      plan_accel = float(np.clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      if self.release_ramp is None or not CS.out.standstill:
        self.breakaway_frames = 0
      else:
        self.breakaway_frames += 1
      breakaway = CS.out.standstill and self.breakaway_frames <= BREAKAWAY_FRAMES
      ramp_ceiling = max(plan_accel, min(CarControllerParams.ACCEL_BREAKAWAY_MAX,
                                         plan_accel + CarControllerParams.ACCEL_BREAKAWAY_OVERSHOOT))
      if self.release_ramp is not None and (self.release_ramp < plan_accel or breakaway):
        accel = self.release_ramp
        if not (sm.latched_release and CS.brake_hold):
          self.release_ramp = max(min(self.release_ramp + CarControllerParams.ACCEL_RELEASE_RAMP * DT_CTRL, ramp_ceiling),
                                  self.release_ramp + CarControllerParams.ACCEL_WINDDOWN_LIMIT)
      else:
        self.release_ramp = None
        accel = rate_limit(plan_accel, self.accel_last, CarControllerParams.ACCEL_WINDDOWN_LIMIT,
                           CarControllerParams.ACCEL_WINDUP_LIMIT)
        if accel > 0.:
          ceiling = float(np.interp(CS.out.vEgoRaw, CarControllerParams.ACCEL_CEILING_BP,
                                    CarControllerParams.ACCEL_CEILING_V))
          build = float(np.interp(CS.out.vEgoRaw, CarControllerParams.ACCEL_BUILD_BP,
                                  CarControllerParams.ACCEL_BUILD_V)) * DT_CTRL
          accel = min(accel, ceiling, max(self.accel_last, 0.) + build)
        if self.accel_last > 0. and CC.actuators.accel >= 0.:
          accel = max(accel, self.accel_last + CarControllerParams.ACCEL_LIFT_LIMIT * DT_CTRL)

      if sm.car_has_hold:
        accel = CarControllerParams.ACCEL_HOLD_LATCHED
      elif sm.holding:
        accel = min(accel, 0.) if plan_accel <= 0. else min(self.accel_last, 0.)
      if sm.resume_unlatching:
        accel = min(max(accel, CarControllerParams.ACCEL_HOLD_LATCHED),
                    CarControllerParams.ACCEL_RESUME_PULSE_MAX)
    self.accel_last = accel

    if radar_master and self.frame % CarControllerParams.RADAR_STEP == 0:
      for bus in LONG_BUSES:
        can_sends.extend(mazdacan.create_radar_frames(bus, self.radar_counter))
      self.radar_counter += 1

    if radar_master and self.frame % CarControllerParams.LONG_STEP == 0:
      acc_available = CS.cruise_available and control_ready
      gap = (int(CC.hudControl.leadDistanceBars) or 2) if (long_engaged or acc_available) else 0
      for bus in LONG_BUSES:
        can_sends.append(mazdacan.create_acc_command(self.packer, bus, self.long_counter, accel,
                                                     long_active=long_engaged, acc_available=acc_available,
                                                     brake_pressed=CS.out.brakePressed,
                                                     stopping=sm.stop_bits,
                                                     resume_unlatching=sm.resume_unlatching))
        can_sends.append(mazdacan.create_crz_ctrl(self.packer, bus, long_engaged, acc_available, gap,
                                                  sm.acc_active_2 if long_engaged else False))
      self.long_counter += 1

    return can_sends
