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
  000000b6--48c9d2f02a/23 and 000000b8--ab902978ef/9. A closed-loop speed command first establishes
  forward creep around 0.1 m/s. While the car remains in the recorded 0.06-0.12 m/s band, a separate
  synthetic planner acceleration alternates across should_stop's 0.1 m/s^2 threshold and feeds the
  resulting pid/stopping state through the production MebLongStateMachine. The moving burst resets if
  the car stops, ESP hold grabs, or motion is not forwards. It is followed by the recorded held churn
  and quiet HALTEN delay.

  No ACC payload or fault status is replayed. Negative acceleration is requested only with
  longControlState=stopping, so the production state machine pairs it with HALTEN. Brake, gas,
  excessive speed, a parking brake, a TSK fault, or reverse motion blocks the experiment until the
  next engagement. Camera ACC faults are intentionally excluded from accFaulted on this branch.
  """

  APPROACH = "approach"
  ESTABLISH_CREEP = "establish_creep"
  CREEP_CHURN = "creep_churn"
  STOP_FOR_HOLD = "stop_for_hold"
  HELD_CHURN = "held_churn"
  SETTLE = "settle"

  ARM_SPEED = 10.0
  CREEP_SPEED_MIN = 0.06
  CREEP_SPEED_TARGET = 0.10
  CREEP_SPEED_MAX = 0.12
  STOPPING_CREEP_TARGET = 0.09
  GOING_CREEP_TARGET = 0.11

  MOTION_FORWARDS = 1
  MOTION_REVERSING = 2
  MOTION_STOPPED = 3

  SHOULD_STOP_ACCEL = 0.09
  SHOULD_GO_ACCEL = 0.11
  SHOULD_STOP_ACCEL_THRESHOLD = 0.10
  SHOULD_STOP_SPEED_THRESHOLD = 0.30

  KP = 4.0
  APPROACH_ACCEL_MIN = -1.5
  ESTABLISH_ACCEL_MAX = 0.6
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

  def _speed_control_accel(self, speed, target, accel_min, accel_max):
    return min(max((target - speed) * self.KP, accel_min), accel_max)

  def _approach(self, CC, speed):
    self.phase = self.APPROACH
    self.phase_frames = 0
    approach_accel = self._speed_control_accel(speed, self.CREEP_SPEED_TARGET, self.APPROACH_ACCEL_MIN, 0.0)
    return self._stopping(CC, approach_accel)

  def _establish_creep(self, CC, speed):
    self.phase = self.ESTABLISH_CREEP
    self.phase_frames = 0
    establish_accel = self._speed_control_accel(speed, self.CREEP_SPEED_TARGET, 0.0, self.ESTABLISH_ACCEL_MAX)
    return self._going(CC, establish_accel)

  def _enter_creep_churn(self):
    self.phase = self.CREEP_CHURN
    self.phase_frames = 0

  def _planner_wants_stop(self, speed, planner_accel):
    return speed < self.SHOULD_STOP_SPEED_THRESHOLD and planner_accel < self.SHOULD_STOP_ACCEL_THRESHOLD

  def _creep_churn_command(self, CS, CC):
    period = self.CREEP_STOP_FRAMES + self.CREEP_GO_FRAMES
    cycle_frame = self.phase_frames % period
    planner_accel = self.SHOULD_STOP_ACCEL if cycle_frame < self.CREEP_STOP_FRAMES else self.SHOULD_GO_ACCEL
    self.phase_frames += 1

    if self._planner_wants_stop(CS.out.vEgo, planner_accel):
      creep_accel = self._speed_control_accel(CS.out.vEgo, self.STOPPING_CREEP_TARGET,
                                              self.STOP_ACCEL, 0.0)
      return self._stopping(CC, creep_accel)

    creep_accel = self._speed_control_accel(CS.out.vEgo, self.GOING_CREEP_TARGET,
                                            0.0, self.CREEP_ACCEL_MAX)
    return self._going(CC, creep_accel)

  def update(self, CS, CC, accel):
    if not CC.enabled:
      self.blocked_until_disengage = False
      self._reset_sequence()
      return accel, CC

    driver_cancel = CS.out.brakePressed or CS.out.gasPressed
    reversing = CS.meb_motion_state == self.MOTION_REVERSING
    unsafe_to_run = CS.out.accFaulted or CS.out.parkingBrake or CS.out.vEgo > self.ARM_SPEED or reversing
    if driver_cancel or unsafe_to_run:
      self.blocked_until_disengage = True
      self._reset_sequence()

    if self.blocked_until_disengage or not CC.longActive:
      return accel, CC

    if self.phase == self.APPROACH:
      if CS.esp_hold_confirmation or CS.meb_motion_state == self.MOTION_STOPPED:
        self.phase = self.ESTABLISH_CREEP
        self.phase_frames = 0
      elif CS.out.vEgo > self.CREEP_SPEED_MAX:
        return self._approach(CC, CS.out.vEgo)
      elif CS.meb_motion_state == self.MOTION_FORWARDS and CS.out.vEgo >= self.CREEP_SPEED_MIN:
        self._enter_creep_churn()
      else:
        return self._establish_creep(CC, CS.out.vEgo)

    if self.phase == self.ESTABLISH_CREEP:
      if CS.esp_hold_confirmation or CS.meb_motion_state == self.MOTION_STOPPED:
        return self._going(CC, self.LAUNCH_ACCEL)
      if CS.out.vEgo > self.CREEP_SPEED_MAX:
        return self._approach(CC, CS.out.vEgo)
      if CS.meb_motion_state != self.MOTION_FORWARDS or CS.out.vEgo < self.CREEP_SPEED_MIN:
        return self._establish_creep(CC, CS.out.vEgo)
      self._enter_creep_churn()

    if self.phase == self.CREEP_CHURN:
      if CS.esp_hold_confirmation or CS.meb_motion_state == self.MOTION_STOPPED:
        self.phase = self.ESTABLISH_CREEP
        self.phase_frames = 0
        return self._going(CC, self.LAUNCH_ACCEL)
      if CS.meb_motion_state != self.MOTION_FORWARDS or CS.out.vEgo < self.CREEP_SPEED_MIN:
        return self._establish_creep(CC, CS.out.vEgo)
      if CS.out.vEgo > self.CREEP_SPEED_MAX:
        return self._approach(CC, CS.out.vEgo)
      if self.phase_frames >= self.CREEP_CHURN_FRAMES:
        self.phase = self.STOP_FOR_HOLD
        self.phase_frames = 0
        return self._stopping(CC, self.STOP_ACCEL)
      return self._creep_churn_command(CS, CC)

    if self.phase == self.STOP_FOR_HOLD:
      stopped_and_held = (CS.meb_motion_state == self.MOTION_STOPPED and
                          CS.esp_hold_confirmation and
                          CS.out.vEgo < self.CREEP_SPEED_MIN)
      if stopped_and_held:
        self.phase = self.HELD_CHURN
        self.phase_frames = 0
      else:
        return self._stopping(CC, self.STOP_ACCEL)

    if self.phase == self.HELD_CHURN:
      stopped_and_held = (CS.meb_motion_state == self.MOTION_STOPPED and
                          CS.esp_hold_confirmation and
                          CS.out.vEgo < self.CREEP_SPEED_MIN)
      if not stopped_and_held:
        self.phase = self.STOP_FOR_HOLD
        self.phase_frames = 0
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
      stopped_and_held = (CS.meb_motion_state == self.MOTION_STOPPED and
                          CS.esp_hold_confirmation and
                          CS.out.vEgo < self.CREEP_SPEED_MIN)
      if not stopped_and_held:
        self.phase = self.STOP_FOR_HOLD
        self.phase_frames = 0
        return self._stopping(CC, self.STOP_ACCEL)
      if self.phase_frames < self.SETTLE_FRAMES:
        self.phase_frames += 1
        return self._stopping(CC, self.STOP_ACCEL)
      self.phase = self.ESTABLISH_CREEP
      self.phase_frames = 0
      return self._going(CC, self.LAUNCH_ACCEL)

    raise ValueError(f"unknown MEB repro phase: {self.phase}")
