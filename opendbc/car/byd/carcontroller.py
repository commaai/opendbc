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

    self.enabled_last = False
    self.camera_on = None
    self.cam_raw_last = None
    self.cam_stable_frames = 0
    self.quiet_frames = 0
    self.lks_lockout = 0
    self.lks_pending = False
    self.lks_pulse = 0
    self.lks_confirm = 0
    self.lks_retries = 0
    self.lks_want = None
    self.lks_give_up_want = None
    self.hold_steer = 0

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

  def _abort_lks_pulse(self):
    self.lks_pulse = 0
    self.lks_confirm = 0
    self.lks_retries = 0

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

  def _update_lks_camera(self, CC, CS, can_sends):
    # Meaning A: one bus-2 toggle toward want, then confirm. A real bus-0
    # press is lockout then snap. Never TX LKS on bus 0.
    self._update_camera_on(CS.camera_lkas_state)

    if CC.enabled:
      self.quiet_frames = 0
    else:
      self.quiet_frames += 1

    if CS.lks_btn_rising:
      self._abort_lks_pulse()
      self.lks_lockout = CarControllerParams.LKS_LOCKOUT_FRAMES
      self.lks_pending = True
      self.lks_give_up_want = None
      if not CS.lks_enabled:
        self.hold_steer = CarControllerParams.LKS_LOCKOUT_FRAMES + CarControllerParams.LKS_CONFIRM_FRAMES

    if CC.enabled and not self.enabled_last:
      self.lks_pending = True
      self.lks_give_up_want = None
    if (not CC.enabled) and CS.lks_enabled and self.quiet_frames == CarControllerParams.LKS_HUD_QUIET_FRAMES:
      self.lks_pending = True
      self.lks_give_up_want = None

    self.enabled_last = CC.enabled
    want = self._want_camera(CC.enabled, CS.lks_enabled)
    if want is not None and want != self.lks_give_up_want:
      self.lks_give_up_want = None

    if self.lks_lockout > 0:
      self.lks_lockout -= 1
    elif self.lks_pending and self.lks_pulse == 0 and self.lks_confirm == 0:
      if want is None or self.camera_on is None:
        pass
      elif self.camera_on != want:
        self.lks_pulse = CarControllerParams.LKS_PULSE_TICKS
        self.lks_want = want
        self.lks_retries = 0
        self.lks_pending = False
        if not want:
          self.hold_steer = max(self.hold_steer, CarControllerParams.LKS_PULSE_TICKS * CarControllerParams.LKS_PULSE_PERIOD +
                                CarControllerParams.LKS_CONFIRM_FRAMES)
      else:
        self.lks_pending = False

    if self.lks_pulse > 0 and self.frame % CarControllerParams.LKS_PULSE_PERIOD == 0:
      can_sends.append(bydcan.create_buttons(self.packer, lkas=True, bus=2))
      self.lks_pulse -= 1
      if self.lks_pulse == 0:
        self.lks_confirm = CarControllerParams.LKS_CONFIRM_FRAMES

    if self.lks_pulse == 0 and self.lks_confirm > 0:
      self.lks_confirm -= 1
      if self.lks_confirm == 0:
        if self.camera_on is not None and self.lks_want is not None and self.camera_on != self.lks_want:
          if self.lks_retries < 1:
            self.lks_pulse = CarControllerParams.LKS_PULSE_TICKS
            self.lks_retries += 1
            if not self.lks_want:
              self.hold_steer = max(self.hold_steer, CarControllerParams.LKS_PULSE_TICKS * CarControllerParams.LKS_PULSE_PERIOD +
                                    CarControllerParams.LKS_CONFIRM_FRAMES)
          else:
            self.lks_give_up_want = self.lks_want

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    hud_control = CC.hudControl

    self._update_lks_camera(CC, CS, can_sends)
    send_op = CC.enabled or self.hold_steer > 0

    if self.frame % 2:
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                          CS.out.steeringAngleDeg, CC.latActive, CarControllerParams, self.VM)

      cntr = (self.frame // 2) % 16

      # 0x1E2 runs for the whole engagement, not just while steering. STEER_REQ=0 carries
      # the measured angle, which keeps the EPS fed and panda's angle reference synced, so
      # the first frame after lateral comes back is not rate limited against a stale angle.
      # hold_steer keeps that block for a few hundred ms after a real LKS press so the
      # camera cannot grab the wheel before we snap it back off.
      if send_op:
        can_sends.append(bydcan.create_steering_control(self.packer, self.apply_angle_last, CC.latActive, cntr))
        # HUD for the whole engagement so TAKE CONTROL can paint the cluster after
        # latActive drops. Panda keeps camera 0x316 off while we still send 0x1E2.
        can_sends.append(bydcan.create_lkas_hud(self.packer, cntr, CS.lkas_hud, hud_control, CC.latActive))

      if CC.latActive:
        # STEER_REQ=1 while EPS reports idle (LKS_PREPARED=1) for 200 ms => not accepted
        not_accepted = not CS.eps_engaged
        self.not_accepted_frames = self.not_accepted_frames + 1 if not_accepted else 0
        CS.steer_not_accepted = self.not_accepted_frames >= CarControllerParams.STEER_NOT_ACCEPTED_FRAMES
      else:
        self.not_accepted_frames = 0
        CS.steer_not_accepted = False

    if CC.cruiseControl.cancel and self.frame % 10 == 0:
      can_sends.append(bydcan.create_buttons(self.packer, cancel=True))

    if self.hold_steer > 0:
      self.hold_steer -= 1

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = float(self.apply_angle_last)

    self.frame += 1
    return new_actuators, can_sends
