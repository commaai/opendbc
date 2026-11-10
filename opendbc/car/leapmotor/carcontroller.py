import math

from opendbc.can import CANPacker

from opendbc.car import DT_CTRL, Bus
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.leapmotor import leapmotorcan
from opendbc.car.leapmotor.angle_servo import AngleServo
from opendbc.car.leapmotor.values import CarControllerParams, LkaState

LKA_SIDE_LEFT_LINE = 1
LKA_SIDE_RIGHT_LINE = 2

READY_FRAMES = 5
EPS_ACK_FRAMES = 5
EPS_DROP_FRAMES = 2

REHANDSHAKE_READY_FRAMES = 2
EPS_ATTEMPTS = 3
EPS_FAULT_FRAMES = 50
EXIT_READY_FRAMES = 5

YIELD_START, YIELD_FULL = 40., 100.
YIELD_DOWN, YIELD_UP = 15., 3.
HELP_TORQUE = 60.
HELP_SIGN_TORQUE = 20.
RELEASE_TORQUE = 40.
RELEASE_HOLD_FRAMES = 30
RELEASE_HOLD_MAX_FRAMES = 900
RELEASE_PULL_ANGLE = 2.

STEER_1_MISSING_FRAMES = 3


class EpsHandshake:
  def __init__(self):
    self.fault_frames = 0
    self.exit_frames = 0
    self.attempts = 0
    self.ready_frames = 0
    self.ready_needed = READY_FRAMES
    self.active_frames = 0
    self.acked = False
    self.released = False

  @property
  def fault(self) -> bool:
    return self.fault_frames > 0

  def _start(self, ready_frames: int):
    self.ready_frames = 0
    self.ready_needed = ready_frames
    self.active_frames = 0
    self.acked = False
    self.released = False

  def _failed(self):
    self.attempts += 1
    if self.attempts >= EPS_ATTEMPTS:
      self.attempts = 0
      self.fault_frames = EPS_FAULT_FRAMES
    self._start(REHANDSHAKE_READY_FRAMES if self.attempts == 1 else READY_FRAMES)

  def update(self, lat_active: bool, eps_inactive_frames: int, rejected: bool) -> tuple[LkaState | None, bool]:
    was_active = self.active_frames > 0
    eps_active = eps_inactive_frames == 0
    if was_active:
      if not self.acked and eps_active and self.released:
        self.acked = True
        self.attempts = 0
      if rejected:
        self._failed()
      elif self.acked and eps_inactive_frames >= EPS_DROP_FRAMES:
        self._failed()
      elif not self.acked and self.active_frames >= EPS_ACK_FRAMES:
        self._failed()
    elif not eps_active:
      self.released = True

    if self.fault_frames > 0:
      self.fault_frames -= 1
      lat_active = False
    if not lat_active:
      self.attempts = 0
      self._start(READY_FRAMES)
      if was_active:
        self.exit_frames = EXIT_READY_FRAMES
      if self.exit_frames > 0:
        self.exit_frames -= 1
        return LkaState.READY, False
      return None, False

    self.exit_frames = 0
    if self.ready_frames < self.ready_needed:
      self.ready_frames += 1
      return LkaState.READY, False
    self.active_frames += 1
    return LkaState.ACTIVE, self.acked


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = CarControllerParams(CP)

    self.apply_torque_last = 0
    self.apply_angle_last = 0.
    self.lka_side_last = LKA_SIDE_LEFT_LINE
    self.eps = EpsHandshake()
    self.lka_active_rejected = 0
    self.servo = AngleServo(CP, self.params)
    self.servo_frame = -self.params.STEER_STEP - 1
    self.angle_des_last = 0.
    self.steer_angles: list[float] = []
    self.driver_frames = 0
    self.release_frames = 0
    self.steer_1_missing = 0
    self.helping = False
    self.yield_k = 1.

  def update(self, CC, CS, now_nanos):
    can_sends = []
    CS.op_enabled = CC.enabled
    self.steer_angles += CS.steer_angles

    driver_torque = CS.driver_torque.avg
    if abs(driver_torque) < HELP_SIGN_TORQUE or driver_torque * self.apply_torque_last <= 0.:
      self.helping = False
    elif abs(self.apply_torque_last) >= HELP_TORQUE:
      self.helping = True
    pressed = CS.out.steeringPressed and not self.helping
    if pressed:
      self.driver_frames = RELEASE_HOLD_FRAMES
      self.release_frames = 0
    else:
      self.release_frames += 1
      if abs(driver_torque) < RELEASE_TORQUE or self.helping or self.release_frames > RELEASE_HOLD_MAX_FRAMES:
        self.driver_frames = max(self.driver_frames - 1, 0)
    if abs(driver_torque) >= RELEASE_TORQUE and (pressed or self.driver_frames > 0):
      self.servo.push_dir = 1 if driver_torque > 0 else -1
    pulls_back = self.servo.push_dir * (CC.actuators.steeringAngleDeg - CS.out.steeringAngleDeg) < RELEASE_PULL_ANGLE
    driver_steering = pressed or (self.driver_frames > 0 and pulls_back)

    if self.frame % self.params.STEER_STEP == 0:

      lat_active = CC.latActive and not CS.out.brakePressed

      rejected = CS.lka_active_rejected != self.lka_active_rejected
      self.lka_active_rejected = CS.lka_active_rejected
      if rejected:
        self.apply_torque_last = 0

      lka_state, steer = self.eps.update(lat_active, CS.eps_inactive_frames, rejected)
      CS.eps_fault = self.eps.fault

      apply_torque = 0
      ldw = bool(CS.stock_lka_cmd.get("LDW_ACTIVE", 0))
      if lka_state is None:

        stock_lka_state = int(CS.stock_lka_cmd.get("LKA_STATE", LkaState.OFF))
        if stock_lka_state == LkaState.ACTIVE:
          lka_state = LkaState.READY
        else:
          lka_state = stock_lka_state if stock_lka_state in (LkaState.OFF, LkaState.STANDBY, LkaState.READY) else LkaState.OFF

      for angle in self.steer_angles or ([CS.out.steeringAngleDeg] if steer else []):
        self.servo.observe(angle)
      self.steer_1_missing = 0 if self.steer_angles else self.steer_1_missing + 1
      if steer and self.steer_1_missing >= STEER_1_MISSING_FRAMES:
        self.servo_frame = -self.params.STEER_STEP - 1
        steer_max = self.params.steer_max(CS.out.vEgoRaw)
        apply_torque = apply_driver_steer_torque_limits(0, self.apply_torque_last, CS.out.steeringTorque, self.params, steer_max)
      elif steer:
        steer_max = self.params.steer_max(CS.out.vEgoRaw)
        new_torque = self.update_servo(CC, CS, steer_max, driver_steering, driver_torque)
        if self.steer_1_missing and abs(new_torque) > abs(self.apply_torque_last):
          new_torque = self.apply_torque_last
        new_torque = int(round(new_torque * self.update_yield(new_torque, driver_torque)))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params,
                                                        steer_max)
      if not steer:
        self.yield_k = 1.
      self.steer_angles = []

      if apply_torque != 0:
        self.lka_side_last = LKA_SIDE_LEFT_LINE if apply_torque < 0 else LKA_SIDE_RIGHT_LINE
      lka_side = self.lka_side_last if lka_state == LkaState.ACTIVE else 0

      counter = (self.frame // self.params.STEER_STEP) % 16
      can_sends.append(leapmotorcan.create_lka_command(self.packer, lka_state, apply_torque, lka_side, ldw, counter))
      self.apply_torque_last = apply_torque

    angle_des = CC.actuators.steeringAngleDeg
    if self.frame - self.servo_frame < self.params.STEER_STEP and not self.servo.saturated:
      self.apply_angle_last = angle_des
    else:
      self.apply_angle_last = CS.out.steeringAngleDeg
    self.angle_des_last = angle_des

    new_actuators = CC.actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends

  def update_yield(self, torque: float, driver_torque: float) -> float:
    """Share of the servo torque that stays: falls with the driver torque that pushes AGAINST it (rate-limited)."""
    opp = max(-driver_torque if torque > 0 else (driver_torque if torque < 0 else 0.), 0.)
    target = min(max((YIELD_FULL - opp) / (YIELD_FULL - YIELD_START), 0.), 1.)
    dt = DT_CTRL * self.params.STEER_STEP
    if target < self.yield_k:
      self.yield_k = max(target, self.yield_k - YIELD_DOWN * dt)
    else:
      self.yield_k = min(target, self.yield_k + YIELD_UP * dt)
    return self.yield_k

  def update_servo(self, CC, CS, steer_max: int, driver_steering: bool, driver_torque: float) -> int:
    if self.frame - self.servo_frame > self.params.STEER_STEP:
      self.servo.reset(CS.out.steeringAngleDeg)
    self.servo_frame = self.frame
    self.servo.steer_max = steer_max
    torque = self.servo.update(CS.out.vEgo, CS.out.steeringAngleDeg, CC.actuators.steeringAngleDeg, CS.out.steeringRateDeg,
                               driver_steering, CS.eps_inactive_frames == 0, self.apply_torque_last, driver_torque)
    if not math.isfinite(torque):
      self.servo.reset(CS.out.steeringAngleDeg)
      return 0
    return int(round(torque))
