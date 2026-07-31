"""On-car experiment for reproducing the MEB near-standstill TSK permanent fault."""

from opendbc.car import structs

LongCtrlState = structs.CarControl.Actuators.LongControlState


class _ReproCarControl:
  """Stand-in for the immutable CarControl reader with only longControlState replaced."""

  def __init__(self, CC, long_control_state):
    self.enabled = CC.enabled
    self.longActive = CC.longActive
    self.cruiseControl = CC.cruiseControl
    self.actuators = structs.CarControl.Actuators(longControlState=long_control_state)


class MebShouldStopChurnRepro:
  """REPRO ONLY, do not merge.

  Recreates the low-speed pid/stopping uncertainty preceding TSK permanent faults in
  000000b6--48c9d2f02a/23 and 000000b8--ab902978ef/9. The experiment automatically approaches the
  recorded creep band, chatters through the production MebLongStateMachine while moving, repeats the
  chatter after the ESP hold grabs, then remains quietly in HALTEN for the measured post-churn delay.

  No ACC payload or fault status is replayed. In particular, negative acceleration is requested only
  with longControlState=stopping, so the production state machine pairs it with HALTEN. Brake, gas,
  excessive speed, a parking brake, or an ACC fault blocks the experiment until the next engagement.
  """

  APPROACH = "approach"
  LAUNCH = "launch"
  CREEP_CHURN = "creep_churn"
  HELD_CHURN = "held_churn"
  SETTLE = "settle"

  ARM_SPEED = 10.0
  CHURN_ENTRY_SPEED = 0.2
  MOVING_SPEED = 0.05
  CREEP_SPEED = 0.15

  KP = 4.0
  APPROACH_ACCEL_MIN = -1.5
  CREEP_ACCEL_MAX = 0.12
  STOP_ACCEL = -0.35
  LAUNCH_ACCEL = 1.5
  HELD_POKE_ACCEL = 0.12

  CREEP_STOP_FRAMES = 10   # 200 ms HALTEN
  CREEP_GO_FRAMES = 7      # 120 ms RAMP + one 20 ms NONE frame
  CREEP_CHURN_FRAMES = 63  # 1.26 s, matching b6 phase A
  HELD_STOP_FRAMES = 5     # 100 ms HALTEN
  HELD_GO_FRAMES = 10      # 200 ms ANFAHREN
  HELD_CHURN_FRAMES = 108  # 2.16 s, matching b6 phase B
  SETTLE_FRAMES = 19       # TSK faulted 0.38 s after the b6 chatter ended

  def __init__(self):
    self.blocked_until_disengage = False
    self._reset_sequence()

  def _reset_sequence(self):
    self.phase = self.APPROACH
    self.phase_frames = 0

  @staticmethod
  def _replace_long_control_state(CC, long_control_state):
    return _ReproCarControl(CC, long_control_state)

  def _stopping(self, CC, accel):
    return accel, self._replace_long_control_state(CC, LongCtrlState.stopping)

  def _going(self, CC, accel):
    return accel, self._replace_long_control_state(CC, LongCtrlState.pid)

  def _creep_accel(self, speed):
    return min(max((self.CREEP_SPEED - speed) * self.KP, self.APPROACH_ACCEL_MIN), self.CREEP_ACCEL_MAX)

  def _advance_to_creep_churn(self):
    self.phase = self.CREEP_CHURN
    self.phase_frames = 0

  def update(self, CS, CC, accel):
    if not CC.enabled:
      self.blocked_until_disengage = False
      self._reset_sequence()
      return accel, CC

    driver_cancel = CS.out.brakePressed or CS.out.gasPressed
    unsafe_to_run = CS.out.accFaulted or CS.out.parkingBrake or CS.out.vEgo > self.ARM_SPEED
    if driver_cancel or unsafe_to_run:
      self.blocked_until_disengage = True
      self._reset_sequence()

    if self.blocked_until_disengage or not CC.longActive:
      return accel, CC

    if self.phase == self.APPROACH:
      if CS.out.vEgo < self.MOVING_SPEED:
        self.phase = self.LAUNCH
        self.phase_frames = 0
      elif CS.out.vEgo > self.CHURN_ENTRY_SPEED:
        approach_accel = max((self.CREEP_SPEED - CS.out.vEgo) * self.KP, self.APPROACH_ACCEL_MIN)
        return self._stopping(CC, approach_accel)
      else:
        self._advance_to_creep_churn()

    if self.phase == self.LAUNCH:
      if CS.esp_hold_confirmation or CS.out.vEgo < self.MOVING_SPEED:
        return self._going(CC, self.LAUNCH_ACCEL)
      self._advance_to_creep_churn()

    if self.phase == self.CREEP_CHURN:
      if CS.esp_hold_confirmation:
        self.phase = self.HELD_CHURN
        self.phase_frames = 0
      elif self.phase_frames >= self.CREEP_CHURN_FRAMES:
        return self._stopping(CC, self.STOP_ACCEL)
      else:
        period = self.CREEP_STOP_FRAMES + self.CREEP_GO_FRAMES
        cycle_frame = self.phase_frames % period
        self.phase_frames += 1
        if cycle_frame < self.CREEP_STOP_FRAMES:
          return self._stopping(CC, min(self._creep_accel(CS.out.vEgo), self.STOP_ACCEL))
        return self._going(CC, max(self._creep_accel(CS.out.vEgo), 0.0))

    if self.phase == self.HELD_CHURN:
      if not CS.esp_hold_confirmation:
        self._advance_to_creep_churn()
        return self._stopping(CC, self.STOP_ACCEL)
      if self.phase_frames >= self.HELD_CHURN_FRAMES:
        self.phase = self.SETTLE
        self.phase_frames = 0
      else:
        period = self.HELD_STOP_FRAMES + self.HELD_GO_FRAMES
        cycle_frame = self.phase_frames % period
        self.phase_frames += 1
        if cycle_frame < self.HELD_STOP_FRAMES:
          return self._stopping(CC, self.STOP_ACCEL)
        return self._going(CC, self.HELD_POKE_ACCEL)

    if self.phase == self.SETTLE:
      if not CS.esp_hold_confirmation:
        self._advance_to_creep_churn()
        return self._stopping(CC, self.STOP_ACCEL)
      if self.phase_frames < self.SETTLE_FRAMES:
        self.phase_frames += 1
        return self._stopping(CC, self.STOP_ACCEL)
      self.phase = self.LAUNCH
      self.phase_frames = 0
      return self._going(CC, self.LAUNCH_ACCEL)

    raise ValueError(f"unknown MEB repro phase: {self.phase}")
