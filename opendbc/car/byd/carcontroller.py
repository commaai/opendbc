import numpy as np

from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import BydFlags, CarControllerParams
from opendbc.car.vehicle_model import VehicleModel


DT_STEER = CarControllerParams.STEER_STEP * DT_CTRL
UNSET_SET_SPEED_KPH = 150.0


def get_safety_CP(car_fingerprint="BYD_ATTO_3"):
  # Geometry matches safetyParam platform bit / byd_tx_hook.
  from opendbc.car.byd.interface import CarInterface
  return CarInterface.get_non_essential_params(car_fingerprint)


class LksCamera:
  def __init__(self, CCP, CP, packer):
    self.CCP = CCP
    self.CP = CP
    self.packer = packer
    self.enabled_last = False
    self.camera_on = None
    self.cam_raw_last = None
    self.cam_stable_frames = 0
    self.quiet_frames = 0
    self.lks_lockout = 0
    self.lks_pulse = 0
    self.lks_pulse_wait = 0
    self.lks_confirm = 0
    self.lks_retries = 0
    self.lks_want = None
    self.lks_give_up_want = None
    self.lks_recover_frames = 0
    self.pcm_buttons_ts_last = 0
    self.lks_relay_tick = False
    self.hold_steer = 0
    self.eps_idle_frames = 0

  def _forget_camera(self):
    self.camera_on = None
    self.cam_raw_last = None
    self.cam_stable_frames = 0

  def _abort_lks_pulse(self):
    if self.lks_pulse > 0 or self.lks_confirm > 0:
      self._forget_camera()
    self.lks_pulse = 0
    self.lks_confirm = 0
    self.lks_retries = 0
    self.lks_want = None

  def _hold_for(self, frames):
    self.hold_steer = max(self.hold_steer, frames)

  def _update_camera_on(self, state):
    raw = state != 0
    if raw == self.cam_raw_last:
      self.cam_stable_frames += 1
    else:
      self.cam_stable_frames = 0
      self.cam_raw_last = raw
    if self.cam_stable_frames >= self.CCP.LKS_CAM_DEBOUNCE_FRAMES:
      self.camera_on = raw

  def _want_camera(self, enabled, lks_enabled):
    # None=leave alone; False while we own EPS; True after HUD quiet if latch on.
    if enabled:
      return False
    if not lks_enabled:
      return False
    if self.quiet_frames >= self.CCP.LKS_HUD_QUIET_FRAMES:
      return True
    return None

  def _pulse_hold(self):
    return (self.CCP.LKS_PULSE_TICKS * self.CCP.LKS_PULSE_PERIOD +
            self.CCP.LKS_CONFIRM_FRAMES)

  def _pulse_blocked(self, CS, want):
    # No pulse in EPS standby (faults ACC). Block restore in LKAS 4 / until EPS idle.
    if CS.eps_standby:
      return True
    if want and CS.camera_lkas_state == 4:
      return True
    if want and self.eps_idle_frames < self.CCP.LKS_EPS_IDLE_FRAMES:
      return True
    return False

  def _start_pulse(self, want):
    self.lks_pulse = self.CCP.LKS_PULSE_TICKS
    self.lks_pulse_wait = 0
    self.lks_want = want
    self.lks_retries = 0
    self.lks_confirm = 0
    if not want:
      self._hold_for(self._pulse_hold())

  def update(self, CC, CS, can_sends):
    self._update_camera_on(CS.camera_lkas_state)
    fresh_buttons = CS.pcm_buttons_ts != self.pcm_buttons_ts_last
    self.pcm_buttons_ts_last = CS.pcm_buttons_ts

    if CC.enabled:
      self.quiet_frames = 0
      if not self.enabled_last and self.lks_give_up_want is False:
        self.lks_give_up_want = None
        self.lks_recover_frames = 0
    else:
      self.quiet_frames += 1

    if CS.lks_btn_rising:
      camera_was_off = self.camera_on is False or self.cam_raw_last is False
      self._abort_lks_pulse()
      self.lks_lockout = self.CCP.LKS_LOCKOUT_FRAMES
      self.lks_give_up_want = None
      self._forget_camera()
      # Hold 0x1E2 while the camera toggles so stock LKS cannot grab EPS.
      if (not CS.lks_enabled) or CC.enabled or self.enabled_last or camera_was_off:
        self._hold_for(self.CCP.LKS_LOCKOUT_FRAMES + self.CCP.LKS_CONFIRM_FRAMES)

    if getattr(CS, "eps_idle", False):
      self.eps_idle_frames += 1
    else:
      self.eps_idle_frames = 0

    want = self._want_camera(CC.enabled, CS.lks_enabled)
    if want is not None and self.camera_on is not None and self.camera_on == want:
      self.lks_give_up_want = None
      self.lks_recover_frames = 0
    if want is not None and want != self.lks_give_up_want:
      self.lks_give_up_want = None
    if want is not None and self.lks_want is not None and want != self.lks_want:
      if self.lks_pulse > 0 or self.lks_confirm > 0:
        self._abort_lks_pulse()

    # Retry failed camera-off; leave failed restore until want changes.
    if self.lks_give_up_want is False and want is False:
      if self.lks_lockout == 0 and self.lks_pulse == 0 and self.lks_confirm == 0:
        self.lks_recover_frames += 1
        if self.lks_recover_frames >= self.CCP.LKS_RECOVER_FRAMES:
          self.lks_give_up_want = None
          self.lks_recover_frames = 0
    else:
      self.lks_recover_frames = 0

    self.enabled_last = CC.enabled

    if self.lks_lockout > 0:
      self.lks_lockout -= 1
    elif self.lks_pulse == 0 and self.lks_confirm == 0:
      if want is not None and self.camera_on is not None and self.camera_on != want:
        if want != self.lks_give_up_want and not self._pulse_blocked(CS, want):
          self._start_pulse(want)

    # Pulse on fresh 0x3B0 (counter match); long mode uses the relayed frame.
    if self.lks_pulse > 0 and not self._pulse_blocked(CS, self.lks_want):
      self.lks_pulse_wait += 1
      if fresh_buttons or self.lks_pulse_wait > self.CCP.LKS_PULSE_PERIOD:
        if self.CP.openpilotLongitudinalControl and CS.pcm_buttons_new:
          self.lks_relay_tick = True
        else:
          can_sends.append(bydcan.create_buttons(self.packer, CS.pcm_buttons_stock, lkas=True, bus=2))
        self.lks_pulse -= 1
        self.lks_pulse_wait = 0
        if self.lks_pulse == 0:
          self.lks_confirm = self.CCP.LKS_CONFIRM_FRAMES
          if not self.lks_want:
            self._hold_for(self.CCP.LKS_CONFIRM_FRAMES + 1)

    if self.lks_pulse == 0 and self.lks_confirm > 0:
      self.lks_confirm -= 1
      if self.lks_confirm == 0:
        if self.camera_on is not None and self.lks_want is not None and self.camera_on != self.lks_want:
          if self._pulse_blocked(CS, self.lks_want):
            self.lks_confirm = 1
          elif self.lks_retries < 1:
            self.lks_retries += 1
            self.lks_pulse = self.CCP.LKS_PULSE_TICKS
            self.lks_pulse_wait = 0
            if not self.lks_want:
              self._hold_for(self._pulse_hold())
          else:
            self.lks_give_up_want = self.lks_want


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CCP = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.lks = LksCamera(self.CCP, self.CP, self.packer)
    self.apply_angle_last = 0.0
    self.angle_filt = 0.0
    self.accel = 0.0
    self.not_accepted_frames = 0
    self.send_op_last = False
    self.lat_send_last = False
    self.warm_up_left = 0
    self.standby_slots = 0
    self.ack_slots = 0
    self.ack_cooldown = 0
    self.ack_attempts = 0
    self.steer_fault_latched = False
    self.yield_hold = 0
    self.standstill_hold = False
    self.standstill_resume = False
    self.gas_override = False
    self.gas_resume_wait = False

    self.VM = VehicleModel(get_safety_CP(CP.carFingerprint))

  def _steer_tx(self, CC, CS, own_steer):
    if not CC.enabled:
      self.standby_slots = 0
      self.ack_slots = 0
      self.ack_cooldown = 0
      self.ack_attempts = 0
      self.steer_fault_latched = False
    elif self.steer_fault_latched and CS.eps_engaged:
      self.steer_fault_latched = False
      self.ack_attempts = 0

    # Warm-up: REQ=0 at measured angle before first REQ=1 (panda rate-limit ref).
    if own_steer and not self.send_op_last:
      self.warm_up_left = self.CCP.STEER_WARM_UP_FRAMES

    # Keep 0x1E2; drop STEER_REQ while driver torque is high.
    if CS.out.steeringPressed:
      self.yield_hold = self.CCP.STEER_YIELD_HOLD_SLOTS
    elif self.yield_hold > 0:
      self.yield_hold -= 1
    driver_yield = self.yield_hold > 0

    if driver_yield:
      self.standby_slots = 0
      self.ack_slots = 0
      self.ack_cooldown = 0
      self.ack_attempts = 0
    elif not (self.CP.flags & BydFlags.EPS_ACK):
      self.standby_slots = 0
      self.ack_slots = 0
      self.ack_cooldown = 0
      self.ack_attempts = 0
    else:
      # Standby: camera-style REQ=0 / ACTIVE_LOW=0 ack before STEER_REQ.
      self.standby_slots = self.standby_slots + 1 if (own_steer and CS.eps_standby) else 0
      self.ack_cooldown = max(self.ack_cooldown - 1, 0)
      if own_steer and self.warm_up_left == 0 and self.ack_slots == 0 and self.ack_cooldown == 0 and self.standby_slots >= 2:
        if self.ack_attempts < self.CCP.STEER_ACK_ATTEMPTS:
          self.ack_slots = 2
          self.ack_cooldown = self.CCP.STEER_ACK_PERIOD
          self.ack_attempts += 1
        elif not self.steer_fault_latched:
          self.steer_fault_latched = True
          self.ack_cooldown = self.CCP.STEER_ACK_RETRY_PERIOD
        else:
          self.ack_slots = 2
          self.ack_cooldown = self.CCP.STEER_ACK_RETRY_PERIOD

    ack = False
    if not own_steer:
      lat_send = False
    elif self.warm_up_left > 0:
      lat_send = False
      self.warm_up_left -= 1
    elif self.ack_slots > 0:
      lat_send = False
      ack = self.ack_slots == 1
      self.ack_slots -= 1
    else:
      lat_send = CC.latActive and not driver_yield

    return lat_send, ack, driver_yield

  def _update_steering(self, CC, CS, cntr, can_sends, own_steer, actuators, hud_control):
    lat_send, ack, driver_yield = self._steer_tx(CC, CS, own_steer)

    tau = np.interp(CS.out.vEgoRaw, self.CCP.LOW_SPEED_TAU_BP, self.CCP.LOW_SPEED_TAU_V)
    if not lat_send:
      self.angle_filt = CS.out.steeringAngleDeg
    elif tau > 0:
      self.angle_filt += (actuators.steeringAngleDeg - self.angle_filt) * (DT_STEER / (tau + DT_STEER))
    else:
      self.angle_filt = actuators.steeringAngleDeg
    rate = np.interp(CS.out.vEgoRaw, self.CCP.LOW_SPEED_RATE_BP, self.CCP.LOW_SPEED_RATE_V)
    desired_angle = float(np.clip(self.angle_filt, self.apply_angle_last - rate, self.apply_angle_last + rate))

    # First REQ=1 repeats the angle of the prior REQ=0 frame.
    first_req = lat_send and not self.lat_send_last
    if not first_req:
      self.apply_angle_last = apply_steer_angle_limits_vm(desired_angle, self.apply_angle_last, CS.out.vEgoRaw,
                                                          CS.out.steeringAngleDeg, lat_send, self.CCP, self.VM)

    if own_steer:
      can_sends.append(bydcan.create_steering_control(
        self.packer, self.apply_angle_last, lat_send, cntr, ack))
      can_sends.append(bydcan.create_lkas_hud(
        self.packer, cntr, CS.lkas_hud, hud_control, CC.latActive, CS.lks_enabled))

    if lat_send and own_steer:
      self.not_accepted_frames = self.not_accepted_frames + 1 if not CS.eps_engaged else 0
    elif not CC.latActive or driver_yield:
      self.not_accepted_frames = 0
    CS.steer_not_accepted = self.steer_fault_latched or self.not_accepted_frames >= self.CCP.STEER_NOT_ACCEPTED_FRAMES

    self.send_op_last = own_steer
    self.lat_send_last = lat_send if own_steer else False

  def _apply_accel(self, target, standstill):
    if self.gas_override and not standstill:
      if self.gas_resume_wait:
        self.accel = 0.0
        self.gas_resume_wait = False
        return
      step = self.CCP.GAS_RESUME_STEP_HARD if target <= self.CCP.GAS_RESUME_HARD else self.CCP.GAS_RESUME_STEP
      gap = target - self.accel
      if abs(gap) <= step:
        self.accel = target
        self.gas_override = False
      else:
        self.accel += float(np.copysign(step, gap))
      return
    self.accel = target
    self.gas_override = False
    self.gas_resume_wait = False

  def _update_long(self, CC, CS, cntr, can_sends, actuators, hud_control):
    if CS.out.stockAeb:
      can_sends.append(bydcan.create_acc_cmd_passthrough(self.packer, CS.acc_cmd_stock, cntr))
    else:
      if CC.longActive:
        target = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        self._apply_accel(target, CS.out.standstill)
        # Standstill hold bits; RESUME clears once rolling.
        hold = CS.out.standstill and self.accel <= 0.0
        if hold or CS.out.vEgoRaw >= self.CCP.STANDSTILL_RESUME_CLEAR_SPEED:
          self.standstill_resume = False
        elif self.standstill_hold:
          self.standstill_resume = True
        self.standstill_hold = hold
      else:
        self.accel = 0.0
        self.standstill_hold = False
        self.standstill_resume = False
      can_sends.append(bydcan.create_acc_cmd(self.packer, self.accel, CC.longActive, self.standstill_hold,
                                             self.standstill_resume, cntr,
                                             gas_override=self.gas_override and not CC.longActive))
    acc_state = 0 if CS.camera_acc_state == 0 else (5 if CC.enabled and CS.out.gasPressed and not CC.longActive else
                                                   (3 if CC.longActive else 2))
    # vCruise is 255 until first engage; do not paint that.
    set_kph = hud_control.setSpeed * CV.MS_TO_KPH
    if set_kph > UNSET_SET_SPEED_KPH:
      set_kph = float(CS.acc_hud_stock.get("SET_SPEED", 0.0))
    lead_follow_status = hud_control.leadFollowStatus if hud_control.leadVisible else 0
    can_sends.append(bydcan.create_acc_hud(
      self.packer, CS.acc_hud_stock, cntr, acc_state,
      set_kph, hud_control.leadDistanceBars, hud_control.leadVisible,
      acc_state == 5, lead_follow_status,
    ))

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    hud_control = CC.hudControl

    # 0x32E is 50 Hz.
    if self.CP.openpilotLongitudinalControl:
      override = CC.enabled and (not CC.longActive) and CS.out.gasPressed and CS.camera_acc_state != 0
      if not CC.enabled or CS.out.stockAeb or (not CC.longActive and not override):
        self.gas_override = False
        self.gas_resume_wait = False
      elif override:
        self.gas_override = True
        self.gas_resume_wait = True

    self.lks.lks_relay_tick = False
    if self.CP.flags & BydFlags.LKS_HUD_CONTROL:
      self.lks.update(CC, CS, can_sends)
    if self.CP.openpilotLongitudinalControl:
      for frame in CS.pcm_buttons_new:
        can_sends.append(bydcan.create_buttons_relay(self.packer, frame, lkas=self.lks.lks_relay_tick))

    own_steer = CC.enabled or self.lks.hold_steer > 0

    if self.frame % self.CCP.STEER_STEP:
      cntr = (self.frame // self.CCP.STEER_STEP) % 16
      self._update_steering(CC, CS, cntr, can_sends, own_steer, actuators, hud_control)
      if self.CP.openpilotLongitudinalControl:
        self._update_long(CC, CS, cntr, can_sends, actuators, hud_control)

    if CC.cruiseControl.cancel and self.frame % 10 == 0 and not self.CP.openpilotLongitudinalControl:
      can_sends.append(bydcan.create_buttons(self.packer, CS.pcm_buttons_stock, cancel=True))

    if self.lks.hold_steer > 0:
      self.lks.hold_steer -= 1

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = float(self.apply_angle_last)
    if self.CP.openpilotLongitudinalControl:
      new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends
