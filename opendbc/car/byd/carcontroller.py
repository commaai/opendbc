from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel


def get_safety_CP():
  from opendbc.car.byd.interface import CarInterface
  return CarInterface.get_non_essential_params("BYD_ATTO_3")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.apply_angle_last = 0.0
    self.not_accepted_frames = 0
    self.sending_last = False
    self.lat_send_last = False
    self.warmup_left = 0
    self.standby_slots = 0
    self.ack_slots = 0
    self.ack_cooldown = 0
    self.ack_attempts = 0
    self.steer_fault_latched = False

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
    self.hold_steer = 0

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

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
    if self.cam_stable_frames >= CarControllerParams.LKS_CAM_DEBOUNCE_FRAMES:
      self.camera_on = raw

  def _want_camera(self, enabled, lks_enabled):
    # None = do not touch. False while we own the EPS. True after the HUD
    # hold if the latch is still on. Latch off is always camera off.
    if enabled:
      return False
    if not lks_enabled:
      return False
    if self.quiet_frames >= CarControllerParams.LKS_HUD_QUIET_FRAMES:
      return True
    return None

  def _pulse_hold(self):
    return (CarControllerParams.LKS_PULSE_TICKS * CarControllerParams.LKS_PULSE_PERIOD +
            CarControllerParams.LKS_CONFIRM_FRAMES)

  def _start_pulse(self, want):
    self.lks_pulse = CarControllerParams.LKS_PULSE_TICKS
    self.lks_pulse_wait = 0
    self.lks_want = want
    self.lks_retries = 0
    self.lks_confirm = 0
    if not want:
      self._hold_for(self._pulse_hold())

  def _update_lks_camera(self, CC, CS, can_sends):
    # Latch on bus 0 is the count of real presses. Camera is a toggle we may
    # have desynced. After lockout + debounce, snap camera to want. Never TX LKS
    # on bus 0. Never tick while the EPS is in standby: LKS coming on then faults
    # the camera.
    self._update_camera_on(CS.camera_lkas_state)
    fresh_buttons = CS.pcm_buttons_ts != self.pcm_buttons_ts_last
    self.pcm_buttons_ts_last = CS.pcm_buttons_ts

    if CC.enabled:
      self.quiet_frames = 0
    else:
      self.quiet_frames += 1

    if CS.lks_btn_rising:
      camera_was_off = self.camera_on is False or self.cam_raw_last is False
      self._abort_lks_pulse()
      self.lks_lockout = CarControllerParams.LKS_LOCKOUT_FRAMES
      self.lks_give_up_want = None
      self._forget_camera()
      # Press turns the camera. Keep 0x1E2 up if OP was on or the camera was off
      # (it is about to come on) so stock LKS cannot grab the wheel.
      if (not CS.lks_enabled) or CC.enabled or self.enabled_last or camera_was_off:
        self._hold_for(CarControllerParams.LKS_LOCKOUT_FRAMES + CarControllerParams.LKS_CONFIRM_FRAMES)

    want = self._want_camera(CC.enabled, CS.lks_enabled)
    if want is not None and self.camera_on is not None and self.camera_on == want:
      self.lks_give_up_want = None
      self.lks_recover_frames = 0
    if want is not None and want != self.lks_give_up_want:
      self.lks_give_up_want = None
    if want is not None and self.lks_want is not None and want != self.lks_want:
      if self.lks_pulse > 0 or self.lks_confirm > 0:
        self._abort_lks_pulse()

    # Failed camera-off is unsafe. Retry every 2 s from live CAN. Failed
    # restore stays given up until want changes so we do not spam LDW on.
    if self.lks_give_up_want is False and want is False:
      if self.lks_lockout == 0 and self.lks_pulse == 0 and self.lks_confirm == 0:
        self.lks_recover_frames += 1
        if self.lks_recover_frames >= CarControllerParams.LKS_RECOVER_FRAMES:
          self.lks_give_up_want = None
          self.lks_recover_frames = 0
    else:
      self.lks_recover_frames = 0

    self.enabled_last = CC.enabled

    if self.lks_lockout > 0:
      self.lks_lockout -= 1
    elif self.lks_pulse == 0 and self.lks_confirm == 0 and not CS.eps_standby:
      if want is not None and self.camera_on is not None and self.camera_on != want:
        if want != self.lks_give_up_want:
          self._start_pulse(want)

    # Send right after the car's 0x3B0 so ours carries the same counter while it is current
    if self.lks_pulse > 0 and not CS.eps_standby:
      self.lks_pulse_wait += 1
      if fresh_buttons or self.lks_pulse_wait > CarControllerParams.LKS_PULSE_PERIOD:
        can_sends.append(bydcan.create_buttons(self.packer, CS.pcm_buttons_stock, lkas=True, bus=2))
        self.lks_pulse -= 1
        self.lks_pulse_wait = 0
        if self.lks_pulse == 0:
          self.lks_confirm = CarControllerParams.LKS_CONFIRM_FRAMES
          if not self.lks_want:
            self._hold_for(CarControllerParams.LKS_CONFIRM_FRAMES + 1)

    if self.lks_pulse == 0 and self.lks_confirm > 0:
      self.lks_confirm -= 1
      if self.lks_confirm == 0:
        if self.camera_on is not None and self.lks_want is not None and self.camera_on != self.lks_want:
          if CS.eps_standby:
            self.lks_confirm = 1
          elif self.lks_retries < 1:
            self.lks_retries += 1
            self.lks_pulse = CarControllerParams.LKS_PULSE_TICKS
            self.lks_pulse_wait = 0
            if not self.lks_want:
              self._hold_for(self._pulse_hold())
          else:
            self.lks_give_up_want = self.lks_want

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    hud_control = CC.hudControl

    self._update_lks_camera(CC, CS, can_sends)
    send_op = CC.enabled or self.hold_steer > 0

    if self.frame % 2:
      cntr = (self.frame // 2) % 16

      if not CC.enabled:
        self.standby_slots = 0
        self.ack_slots = 0
        self.ack_cooldown = 0
        self.ack_attempts = 0
        self.steer_fault_latched = False

      # 0x1E2 runs for the whole engagement, not just while steering. STEER_REQ=0 carries
      # the measured angle, which keeps the EPS fed and panda's angle reference synced.
      # panda rate limits STEER_REQ=1 against our last frame, however old, so after a TX
      # gap the first frames are REQ=0 and the first REQ=1 repeats the angle.
      # hold_steer keeps that block for a few hundred ms after a real LKS press so the
      # camera cannot grab the wheel before we snap it back off.
      if send_op and not self.sending_last:
        self.warmup_left = CarControllerParams.STEER_WARMUP_FRAMES

      # EPS standby ignores STEER_REQ until it sees the camera's ack. Replay what
      # precedes every logged exit: one idle frame, then REQ=0 with ACTIVE_LOW=0.
      # panda keeps the camera blocked meanwhile. Give up on it after a few tries.
      self.standby_slots = self.standby_slots + 1 if (send_op and CS.eps_standby) else 0
      self.ack_cooldown = max(self.ack_cooldown - 1, 0)
      if CC.latActive and self.warmup_left == 0 and self.ack_slots == 0 and self.ack_cooldown == 0 and self.standby_slots >= 2:
        if self.ack_attempts < CarControllerParams.STEER_ACK_ATTEMPTS:
          self.ack_slots = 2
          self.ack_cooldown = CarControllerParams.STEER_ACK_PERIOD
          self.ack_attempts += 1
        else:
          self.steer_fault_latched = True

      ack = False
      if self.warmup_left > 0:
        lat_send = False
        self.warmup_left -= 1
      elif self.ack_slots > 0:
        lat_send = False
        ack = self.ack_slots == 1
        self.ack_slots -= 1
      else:
        lat_send = CC.latActive

      # The first REQ=1 repeats the angle of the REQ=0 frame panda just took as reference
      first_req = lat_send and not self.lat_send_last
      if not first_req:
        self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                            CS.out.steeringAngleDeg, lat_send, CarControllerParams, self.VM)

      if send_op:
        can_sends.append(bydcan.create_steering_control(self.packer, self.apply_angle_last, lat_send, cntr, ack))
        # HUD for the whole engagement so TAKE CONTROL can paint the cluster after
        # latActive drops. Panda keeps camera 0x316 off while we still send 0x1E2.
        can_sends.append(bydcan.create_lkas_hud(self.packer, cntr, CS.lkas_hud, hud_control, CC.latActive))

      # STEER_REQ=1 while EPS reports idle (LKS_PREPARED=1) for 200 ms => not accepted.
      # Only frames we actually sent with STEER_REQ=1 count.
      if lat_send:
        self.not_accepted_frames = self.not_accepted_frames + 1 if not CS.eps_engaged else 0
      elif not CC.latActive:
        self.not_accepted_frames = 0
      CS.steer_not_accepted = self.steer_fault_latched or self.not_accepted_frames >= CarControllerParams.STEER_NOT_ACCEPTED_FRAMES

      self.sending_last = send_op
      self.lat_send_last = lat_send

    if CC.cruiseControl.cancel and self.frame % 10 == 0:
      can_sends.append(bydcan.create_buttons(self.packer, CS.pcm_buttons_stock, cancel=True))

    if self.hold_steer > 0:
      self.hold_steer -= 1

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = float(self.apply_angle_last)

    self.frame += 1
    return new_actuators, can_sends
