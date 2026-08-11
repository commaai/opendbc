from opendbc.can.packer import CANPacker

from opendbc.car import DT_CTRL, structs
from opendbc.car.chery import cherycan
from opendbc.car.chery.values import (
  AUTORESUME_BURST_FRAMES,
  AUTORESUME_CYCLE_S,
  CAR,
  CarControllerParams,
  DBC,
  EPS_SPOOF_STEP,
  EPS_TAP_FRAMES,
  EPS_TAP_PERIOD_FRAMES,
  EPS_TAP_TORQUE,
  HUD_STEP,
  ICAUR_DISABLE_TORQUE_SPOOF,
  ICAUR_DRIVER_OVERRIDE_ENABLED,
  ICAUR_LKAS_CMD_DEADBAND_DEG,
  ICAUR_OVERRIDE_EXIT,
  ICAUR_OVERRIDE_EXIT_FRAMES,
  ICAUR_OVERRIDE_INSTANT_TORQUE,
  ICAUR_OVERRIDE_OPPOSING_TORQUE,
  LANE_KEEP_STEP,
  LKAS_INFO_STEP,
  OMODA_DISABLE_HUD_OVERRIDE,
  OMODA_DISABLE_TORQUE_SPOOF,
  OMODA_PCM_DISABLE_RES_CYCLE_S,
  ICAUR_DISABLE_HUD_OVERRIDE,
  lowpass_steer_cmd,
)
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

ButtonType = structs.CarState.ButtonEvent.Type


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])
    self.last_apply_angle: float | None = None
    self.prev_brake_pressed = False
    self.prev_cruise_enabled = False
    self.prev_cruise_state = 3
    self.omoda_pcm_disable_pending = False
    self.omoda_pcm_disable_recovery = False  # Omoda: pcmDisable at standstill → RES until cruise on
    self.omoda_pcm_disable_idle_seen = False  # saw CRUISE_STATE=IDLE before first RES
    self.omoda_pcm_disable_burst_left = 0
    self.omoda_pcm_disable_last_burst_frame = -10_000

    # ACC arming + auto-resume from standstill state.
    self.acc_armed = False
    self.autoresume_burst_left = 0
    self.autoresume_last_burst_frame = -10_000
    self.autoresume_burst_idx = 0  # alternates RES (even) / SET (odd)
    self.hud_counter = 0
    self.eps_spoof_counter = 0
    self.eps_spoof_armed = False  # latched on first cam-spoof frame to sync with PT counter
    self.eps_tap_active_for = 0   # frames remaining on the current HOW-suppression tap
    # iCaur steer override latch (pressed / firm yank / opposing torque)
    self.icaur_steer_override = False
    self.icaur_override_clear = 0

  @staticmethod
  def _driver_opposes_lkas(CS, cmd_angle_deg: float) -> bool:
    """True when |T| is high and driver torque opposes the LKAS angle command."""
    cmd_delta = cmd_angle_deg - CS.out.steeringAngleDeg
    if abs(cmd_delta) <= ICAUR_LKAS_CMD_DEADBAND_DEG:
      return False
    cmd_dir = 1 if cmd_delta > 0 else -1

    driver_torque = abs(CS.out.steeringTorque)
    if driver_torque < ICAUR_OVERRIDE_OPPOSING_TORQUE:
      return False

    driver_dir = CS.out.steeringTorqueEps
    if driver_dir == 0:
      return False
    driver_dir = 1 if driver_dir > 0 else -1
    return driver_dir != cmd_dir

  def _update_icaur_steer_override(self, CS, lat_active: bool, cmd_angle_deg: float) -> bool:
    """Latch on pressed, firm yank, or opposing LKAS; exit on low torque hysteresis."""
    if not ICAUR_DRIVER_OVERRIDE_ENABLED:
      self.icaur_steer_override = False
      self.icaur_override_clear = 0
      return False

    driver_torque = abs(CS.out.steeringTorque)
    if not lat_active:
      self.icaur_steer_override = False
      self.icaur_override_clear = 0
      return False

    # pressed: CarState 1-frame |T|>=10. instant: firm yank |T|>=20. opposing: curve-safe.
    wants_override = (
      CS.out.steeringPressed
      or driver_torque >= ICAUR_OVERRIDE_INSTANT_TORQUE
      or self._driver_opposes_lkas(CS, cmd_angle_deg)
    )

    if self.icaur_steer_override:
      if driver_torque <= ICAUR_OVERRIDE_EXIT:
        self.icaur_override_clear += 1
        if self.icaur_override_clear >= ICAUR_OVERRIDE_EXIT_FRAMES:
          self.icaur_steer_override = False
          self.icaur_override_clear = 0
          self.last_apply_angle = CS.out.steeringAngleDeg
      else:
        self.icaur_override_clear = 0
    elif wants_override:
      self.icaur_steer_override = True
      self.icaur_override_clear = 0
    return self.icaur_steer_override

  def _compute_apply_angle(self, CS, actuators, steer_req):
    if not steer_req:
      return CS.out.steeringAngleDeg
    meas = CS.out.steeringAngleDeg
    prev = self.last_apply_angle if self.last_apply_angle is not None else meas
    filtered = lowpass_steer_cmd(actuators.steeringAngleDeg, self.last_apply_angle)
    self.last_apply_angle = apply_std_steer_angle_limits(
      filtered, prev, CS.out.vEgo, meas, True, CarControllerParams.ANGLE_LIMITS,
    )
    return self.last_apply_angle

  def _update_acc_armed(self, CS):
    """Arm when stock ACC is on; disarm on explicit driver intent."""
    omoda = self.CP.carFingerprint == CAR.CHERY_OMODA_5

    # Omoda arms even at standstill (pcmDisable recovery relies on it). Jaecoo/Tiggo keep
    # the original "arm only while moving" behavior to avoid standstill button injection.
    if CS.out.cruiseState.enabled and (omoda or not CS.out.standstill):
      self.acc_armed = True

    if omoda:
      if self.prev_cruise_state == 3 and CS.cruise_state == 1 and CS.out.standstill:
        self.omoda_pcm_disable_recovery = True
        self.omoda_pcm_disable_pending = False
        self._omoda_pcm_disable_recovery_started()
      if self.prev_cruise_enabled and not CS.out.cruiseState.enabled:
        self.omoda_pcm_disable_pending = True
        if CS.out.standstill:
          self.omoda_pcm_disable_recovery = True
          self.omoda_pcm_disable_pending = False
          self._omoda_pcm_disable_recovery_started()
      if CS.out.cruiseState.enabled:
        self.omoda_pcm_disable_pending = False
        self.omoda_pcm_disable_recovery = False
        self.omoda_pcm_disable_idle_seen = False
        self.omoda_pcm_disable_burst_left = 0
      elif self.omoda_pcm_disable_pending and CS.out.standstill:
        self.omoda_pcm_disable_recovery = True
        self.omoda_pcm_disable_pending = False
        self._omoda_pcm_disable_recovery_started()

    brake_edge = CS.out.brakePressed and not self.prev_brake_pressed
    icc_press = any(be.pressed and be.type == ButtonType.altButton2 for be in CS.out.buttonEvents)
    if brake_edge or icc_press:
      self.acc_armed = False
      if omoda:
        self.omoda_pcm_disable_pending = False
        self.omoda_pcm_disable_recovery = False
        self.omoda_pcm_disable_idle_seen = False
        self.omoda_pcm_disable_burst_left = 0
    self.prev_brake_pressed = CS.out.brakePressed
    self.prev_cruise_enabled = CS.out.cruiseState.enabled
    self.prev_cruise_state = CS.cruise_state

  def _omoda_pcm_disable_recovery_started(self):
    # Allow first RES as soon as stock PCM reports CRUISE_STATE=IDLE at standstill.
    self.omoda_pcm_disable_idle_seen = False
    self.omoda_pcm_disable_last_burst_frame = self.frame - int(OMODA_PCM_DISABLE_RES_CYCLE_S / DT_CTRL)
    self.omoda_pcm_disable_burst_left = 0

  def _auto_resume(self, CS, can_sends):
    """Periodic RES/SET at standstill. Omoda: RES-only repeat after pcmDisable until cruise on."""
    omoda = self.CP.carFingerprint == CAR.CHERY_OMODA_5
    omoda_recovery = (
      omoda and self.omoda_pcm_disable_recovery
      and not CS.out.cruiseState.enabled and CS.out.standstill
    )

    if omoda_recovery:
      if CS.out.brakePressed:
        return
      # Wait for stock PCM to drop to IDLE (~1s after stop); then keep RES until ENABLE (3).
      if CS.cruise_state == 1:
        self.omoda_pcm_disable_idle_seen = True
      if not self.omoda_pcm_disable_idle_seen or CS.cruise_state == 3:
        return
      elapsed = (self.frame - self.omoda_pcm_disable_last_burst_frame) * DT_CTRL
      if self.omoda_pcm_disable_burst_left == 0 and elapsed >= OMODA_PCM_DISABLE_RES_CYCLE_S:
        self.omoda_pcm_disable_burst_left = AUTORESUME_BURST_FRAMES
        self.omoda_pcm_disable_last_burst_frame = self.frame
      if self.omoda_pcm_disable_burst_left > 0 and self.frame % 2 == 0:
        ctr = (CS.pcm_button_counter + 1) % 16
        can_sends.append(cherycan.create_pcm_button(self.packer, ctr, 0, "RES_BUTTON"))
        can_sends.append(cherycan.create_pcm_button(self.packer, ctr, 2, "RES_BUTTON"))
        self.omoda_pcm_disable_burst_left -= 1
      return

    # Omoda never uses the Jaecoo RES/SET standstill alternation — only pcmDisable recovery above.
    # iCaur has no PCM_BUTTONS (0x360) on the bus — do not inject Jaecoo button frames.
    if omoda or self.CP.carFingerprint == CAR.CHERY_ICAUR_03:
      return

    hard_disarm = self.CP.openpilotLongitudinalControl or not self.acc_armed
    if hard_disarm or not CS.out.standstill or CS.out.brakePressed:
      self.autoresume_burst_left = 0
      if hard_disarm:
        self.autoresume_burst_idx = 0
      return

    elapsed = (self.frame - self.autoresume_last_burst_frame) * DT_CTRL
    if self.autoresume_burst_left == 0 and elapsed >= AUTORESUME_CYCLE_S:
      self.autoresume_burst_left = AUTORESUME_BURST_FRAMES
      self.autoresume_last_burst_frame = self.frame

    if self.autoresume_burst_left > 0 and self.frame % 2 == 0:
      button = "CRUISE_BUTTON" if self.autoresume_burst_idx % 2 else "RES_BUTTON"
      ctr = (CS.pcm_button_counter + 1) % 16
      can_sends.append(cherycan.create_pcm_button(self.packer, ctr, 0, button))
      can_sends.append(cherycan.create_pcm_button(self.packer, ctr, 2, button))
      self.autoresume_burst_left -= 1
      if self.autoresume_burst_left == 0:
        self.autoresume_burst_idx += 1

  def _torque_spoof_disabled(self) -> bool:
    fp = self.CP.carFingerprint
    return (
      (fp == CAR.CHERY_OMODA_5 and OMODA_DISABLE_TORQUE_SPOOF) or
      (fp == CAR.CHERY_ICAUR_03 and ICAUR_DISABLE_TORQUE_SPOOF)
    )

  def _cam_torque_spoof_active(self, CS) -> bool:
    """Feed bus-2 torque spoofs while parked or while cruise is on.

    Pre-arms the camera path at standstill so you can engage ACC/LKAS in the
    driveway and see HOW / LKAS-fault on the meter without another drive.
    """
    if self._torque_spoof_disabled():
      return False
    return CS.out.standstill or CS.out.cruiseState.enabled

  def update(self, CC, CS, now_nanos):
    del now_nanos
    can_sends = []

    cam_spoof = self._cam_torque_spoof_active(CS)

    lat_active = CC.latActive and not CS.out.standstill
    icaur = self.CP.carFingerprint == CAR.CHERY_ICAUR_03
    if icaur:
      driver_over = self._update_icaur_steer_override(CS, lat_active, CC.actuators.steeringAngleDeg)
    else:
      driver_over = CS.out.steeringPressed or CS.steer_related_intervention
    steer_req = lat_active and not driver_over

    apply_angle = CS.out.steeringAngleDeg
    # LANE_KEEP normally at 50 Hz; while iCaur is overriding, TX STEER_REQ=0 every frame
    # so EPS never sees a gap that looks like "still requesting" between even frames.
    send_lane_keep = (self.frame % LANE_KEEP_STEP == 0) or (icaur and self.icaur_steer_override)
    if send_lane_keep:
      if self.frame % LANE_KEEP_STEP == 0:
        apply_angle = self._compute_apply_angle(CS, CC.actuators, steer_req)
      if not steer_req:
        apply_angle = CS.out.steeringAngleDeg
        self.last_apply_angle = apply_angle
      can_sends.append(cherycan.create_lane_keep_command(
        self.packer, apply_angle, steer_req, CS.out.steeringAngleDeg,
      ))

    if self.frame % LKAS_INFO_STEP == 0:
      # Bus-2 mirror only when cruise is engaged. At standstill (cruise off) we let
      # panda forward the native LKAS_INFO PT->cam — injecting a stale spoof there
      # makes the cam see MAIN_TORQUE>0 with LKAS inactive, which the meter flags.
      if not self._torque_spoof_disabled():
        can_sends.extend(cherycan.create_lkas_info_torque_spoof(
          self.packer, steer_req, steer_req,
          steer_related=CS.lkas_info_steer_related,
          apply_spoof_offset=not driver_over,
          inject_on_cam=CS.out.cruiseState.enabled,
        ))

    if self.frame % HUD_STEP == 0 and not (
        (self.CP.carFingerprint == CAR.CHERY_OMODA_5 and OMODA_DISABLE_HUD_OVERRIDE) or
        (self.CP.carFingerprint == CAR.CHERY_ICAUR_03 and ICAUR_DISABLE_HUD_OVERRIDE)
    ):
      can_sends.append(cherycan.create_hud_override(self.packer, CS.cam_hud, self.hud_counter))
      self.hud_counter = (self.hud_counter + 1) % 16

    # While cam_spoof is active, panda blocks native EPS PT->cam (see chery_fwd_hook).
    # Re-emit EPS on bus 2 byte-identical to stock: real STEERING_ANGLE + real
    # DRIVER_TORQUE + self-incrementing counter (synced once to PT). To suppress the
    # hands-on-wheel warning we overlay a brief "tap" override every few seconds while
    # LKAS is actively asking — mimics the natural light hand touch (T~12) the camera
    # uses to reset its HOW timer.
    if cam_spoof:
      if not self.eps_spoof_armed:
        self.eps_spoof_counter = CS.eps_counter
        self.eps_spoof_armed = True
        self.eps_tap_active_for = 0
      if self.frame % EPS_SPOOF_STEP == 0:
        self.eps_spoof_counter = (self.eps_spoof_counter + 1) % 16

        driver_torque = CS.eps_driver_torque
        if steer_req and not driver_over:
          if self.eps_tap_active_for > 0:
            driver_torque = EPS_TAP_TORQUE
            self.eps_tap_active_for -= 1
          elif self.frame % EPS_TAP_PERIOD_FRAMES == 0:
            self.eps_tap_active_for = EPS_TAP_FRAMES - 1
            driver_torque = EPS_TAP_TORQUE
        else:
          self.eps_tap_active_for = 0

        can_sends.append(cherycan.create_eps_passthrough(
          self.packer,
          steering_angle_deg=CS.eps_steering_angle,
          driver_torque=driver_torque,
          counter=self.eps_spoof_counter,
        ))
    else:
      self.eps_spoof_armed = False
      self.eps_tap_active_for = 0

    self._update_acc_armed(CS)
    self._auto_resume(CS, can_sends)

    new_actuators = CC.actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle
    self.frame += 1
    return new_actuators, can_sends
