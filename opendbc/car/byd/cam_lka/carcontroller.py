import math
import numpy as np

from opendbc.can.packer import CANPacker

from opendbc.car.byd.cam_lka.bydcan import (
  create_accel_command,
  create_can_steer_command,
  create_lkas_hud,
  create_steering_torque_spoof_camera,
  send_buttons,
)
from opendbc.car.byd.values import DBC, CAR, ACCEL_MULT, CANBUS, BYD_ATTO_STYLE_PLATFORMS, BYD_OP_LONG_PLATFORMS
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

STEER_LOWPASS_HZ = 2
STEER_DT = 0.02
MAX_STEER_ANGLE_OFFSET_DEG = 10
# Degrees: warn on HUD when command hits angle safety envelope (meas offset or global max).
STEER_ANGLE_LIMIT_WARN_EPS_DEG = 0.08
# Seal 6 angle path: hard-drop STEER_REQ on driver override (MAIN_TORQUE only).
# Thresholds from Seal6 logs: straight LKA (no driver) p95=7.6 Nm, p50=0.7 Nm.
SEAL6_DRIVER_OVERRIDE_ENABLED = True
SEAL6_OVERRIDE_ENTER_NM = 25
SEAL6_OVERRIDE_EXIT_NM = 4
SEAL6_OVERRIDE_ENTER_FRAMES = 5   # 50ms at 100Hz CC; drop LKA torque glitches
SEAL6_OVERRIDE_EXIT_FRAMES = 20   # 0.2s at 100Hz CC
LKA_COOLDOWN_MIN_FRAMES = 30


BUTTON_KEEPALIVE_FRAMES = 100
SPOOF_DURATION_FRAMES = 50
SPOOF_CYCLE_FRAMES = 150


def lowpass_1pole(x, y_prev):
  if y_prev is None:
    return x
  alpha = math.exp(-2.0 * math.pi * STEER_LOWPASS_HZ * STEER_DT)
  return alpha * y_prev + (1.0 - alpha) * x


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])

    self.lka_active = False
    self.last_apply_angle = 0
    self.accel_mult = ACCEL_MULT[CP.carFingerprint]
    self.lka_cooldown = 0
    self.prev_press = False
    self.prev_res_press = False
    self.lka_latched = False
    self.button_send_bus = CANBUS.cam_bus if CP.carFingerprint in BYD_ATTO_STYLE_PLATFORMS else CANBUS.main_bus
    self.seal6_steer_override = False
    self.seal6_override_clear = 0
    self.seal6_override_enter = 0

  def _update_lka_latch_state(self, CS):
    if self.CP.carFingerprint == CAR.BYD_SEAL6:
      # Seal 6: ICC/RES engage ACC via CRUISE_STATE; LKA follows cruise enabled.
      if CS.out.cruiseState.enabled:
        self.lka_cooldown += 1
        self.lka_active = True
      else:
        self.lka_active = False
        self.lka_cooldown = 0
    elif self.CP.carFingerprint in (CAR.BYD_M6, CAR.BYD_SEAL, CAR.BYD_SHARK, CAR.BYD_SEALION7):
      lkas_rising = CS.lkas_rdy_btn and not self.prev_press
      if lkas_rising:
        if not self.lka_latched:
          self.lka_latched = True
        else:
          self.lka_latched = False
          self.lka_cooldown = 0
          self.lka_active = False
      elif (
        self.CP.carFingerprint == CAR.BYD_SHARK
        and CS.res_btn
        and not self.prev_res_press
        and not self.lka_latched
      ):
        # Shark: stock RES (resume) also arms LKA latch; disengage still via LKAS btn or brake.
        self.lka_latched = True
      self.prev_press = CS.lkas_rdy_btn
      self.prev_res_press = CS.res_btn

      if CS.out.brakePressed:
        self.lka_latched = False
        self.lka_cooldown = 0
        self.lka_active = False
      elif self.lka_latched:
        self.lka_active = True
        self.lka_cooldown += 1
    else:
      # Atto 3: STEER_ACTIVE arms lateral; LKAS button disarms when HUD LKA is off.
      if CS.lka_on:
        self.lka_cooldown += 1
        self.lka_active = True
      if not CS.lka_on and CS.lkas_rdy_btn:
        self.lka_active = False
        self.lka_cooldown = 0

  def _compute_apply_angle(self, CS, actuators, lat_active):
    meas_deg = CS.out.steeringAngleDeg
    if not lat_active:
      return meas_deg, False

    limits = CarControllerParams.ANGLE_LIMITS
    after_lowpass = lowpass_1pole(actuators.steeringAngleDeg, self.last_apply_angle)
    after_std = apply_std_steer_angle_limits(
      after_lowpass,
      self.last_apply_angle,
      CS.out.vEgo,
      meas_deg,
      lat_active,
      limits,
    )
    lo = meas_deg - MAX_STEER_ANGLE_OFFSET_DEG
    hi = meas_deg + MAX_STEER_ANGLE_OFFSET_DEG
    apply_angle = float(np.clip(after_std, lo, hi))
    self.last_apply_angle = apply_angle

    eps = STEER_ANGLE_LIMIT_WARN_EPS_DEG
    meas_clip_limited = abs(after_std - apply_angle) > eps
    angle_max_limited = abs(apply_angle) >= limits.STEER_ANGLE_MAX - eps
    steer_angle_limited = meas_clip_limited or angle_max_limited
    return apply_angle, steer_angle_limited

  def update(self, CC, CS, now_nanos):
    del now_nanos
    can_sends = []

    enabled = CC.latActive
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

    self._update_lka_latch_state(CS)

    lat_active = (self.lka_cooldown > LKA_COOLDOWN_MIN_FRAMES) and enabled and self.lka_active and not CS.out.standstill
    steer_req = lat_active
    if self.CP.carFingerprint == CAR.BYD_SEAL6 and SEAL6_DRIVER_OVERRIDE_ENABLED:
      driver_torque = abs(CS.out.steeringTorque)
      if not lat_active:
        self.seal6_steer_override = False
        self.seal6_override_clear = 0
        self.seal6_override_enter = 0
      elif self.seal6_steer_override:
        if driver_torque <= SEAL6_OVERRIDE_EXIT_NM:
          self.seal6_override_clear += 1
          if self.seal6_override_clear >= SEAL6_OVERRIDE_EXIT_FRAMES:
            self.seal6_steer_override = False
            self.seal6_override_clear = 0
            self.seal6_override_enter = 0
            self.last_apply_angle = CS.out.steeringAngleDeg
        else:
          self.seal6_override_clear = 0
      elif driver_torque >= SEAL6_OVERRIDE_ENTER_NM:
        self.seal6_override_enter += 1
        if self.seal6_override_enter >= SEAL6_OVERRIDE_ENTER_FRAMES:
          self.seal6_steer_override = True
          self.seal6_override_clear = 0
          self.seal6_override_enter = 0
      else:
        self.seal6_override_enter = 0
      if self.seal6_steer_override:
        steer_req = False
    elif self.CP.carFingerprint == CAR.BYD_SEAL6:
      self.seal6_steer_override = False
      self.seal6_override_clear = 0
      self.seal6_override_enter = 0

    apply_angle = CS.out.steeringAngleDeg
    hand_on_wheel_warning = False
    # Steer TX is normally 50Hz; while Seal6 is overriding, TX STEER_REQ=0 every frame so the
    # EPS never sees a gap that looks like "still requesting" between even frames.
    send_steer = (self.frame % 2) == 0 or (
      self.CP.carFingerprint == CAR.BYD_SEAL6 and self.seal6_steer_override
    )
    if send_steer:
      if (self.frame % 2) == 0:
        apply_angle, steer_angle_limited = self._compute_apply_angle(CS, actuators, lat_active)
      else:
        steer_angle_limited = False
      if not steer_req:
        apply_angle = CS.out.steeringAngleDeg
        self.last_apply_angle = apply_angle
        steer_angle_limited = False
      hand_on_wheel_warning = bool(lat_active and steer_angle_limited)
      can_sends.append(
        create_can_steer_command(
          self.packer,
          apply_angle,
          steer_req if self.CP.carFingerprint == CAR.BYD_SEAL6 else lat_active,
          CS.out.standstill,
          CS.lkas_healthy,
          CS.lkas_rdy_btn or CS.out.brakePressed,
        )
      )
      if (self.frame % 2) == 0:
        can_sends.append(
          create_lkas_hud(
            self.packer,
            lat_active,
            CS.lss_state,
            CS.lss_alert,
            CS.tsr,
            CS.HMA,
            CS.pt2,
            CS.pt3,
            CS.pt4,
            CS.pt5,
            CS.lkas_hud_status_passthrough,
            CS.lka_on,
            hand_on_wheel_warning,
          )
        )

        if self.CP.carFingerprint in BYD_OP_LONG_PLATFORMS and self.CP.openpilotLongitudinalControl:
          long_active = CC.enabled and not CS.out.gasPressed
          brake_hold = CS.out.standstill and actuators.accel < 0
          can_sends.append(create_accel_command(self.packer, actuators.accel, long_active, self.accel_mult, brake_hold))
        else:
          if CS.out.standstill and CC.enabled and (self.frame % BUTTON_KEEPALIVE_FRAMES == 0):
            can_sends.append(send_buttons(self.packer, 1, 0, self.button_send_bus))

    if self.CP.carFingerprint in (CAR.BYD_ATTO3, CAR.BYD_M6, CAR.BYD_SEAL, CAR.BYD_SEAL6, CAR.BYD_SEALION7, CAR.BYD_SHARK):
      # Atto: always keep fake hands-on torque (no 50/150 off-cycle) and TX at ~50Hz.
      # Stock Atto nags HOW immediately if MAIN_TORQUE drops while 0x1FC is blocked.
      if self.CP.carFingerprint == CAR.BYD_ATTO3:
        spoof_active = True
        send_spoof = (self.frame % 2) == 0
      else:
        cycle_position = self.frame % SPOOF_CYCLE_FRAMES
        spoof_active = cycle_position < SPOOF_DURATION_FRAMES
        if self.CP.carFingerprint == CAR.BYD_SEAL6 and not steer_req:
          spoof_active = False
        send_spoof = (self.frame % 5) == 0

      if send_spoof:
        can_sends.append(
          create_steering_torque_spoof_camera(self.packer, lat_active, CS.out.steeringTorque, spoof_active)
        )

    if pcm_cancel_cmd:
      can_sends.append(send_buttons(self.packer, 0, 1, self.button_send_bus))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle

    self.frame += 1
    return new_actuators, can_sends
