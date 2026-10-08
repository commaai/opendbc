# REPRO ONLY, do not merge
#
# On-car A/B for the MEB standstill TSK permanent faults (TSK_Status 7, 17/21, 18/6, 20/6).
#
# Engage with SET below 70 km/h. The harness sets its own speed and ignores the set speed, it slows the car down first.
# It ignores leads, brake for traffic. Every SET that engages starts a run, SET while engaged does not:
#   stop, go again at crawl (RAMP) and coast to a full stop, so the car stops without an ACC hold while the ESP
#   holds on its own, then the policy wants to drive off. Master requests ANFAHREN here and the hold manager refuses,
#   the fix doesn't. Stop again and hold, drive off, plain stop and hold. Expect no refusal, no fault and no rolling.
# Brake, gas, disengage or a TSK fault hand back to the driver. Speeding up past the SET speed, rolling back
# >0.5 m/s or not driving off stop and hold until disengage, so the car never goes back to the set speed.
#
# Only longControlState and accel are replaced, the production MebLongStateMachine still turns them into ACC_18.
# Status is shown on screen through alertDebug and logged, grep the rlog for "MEB repro".

from opendbc.car import DT_CTRL, structs
from opendbc.car.carlog import carlog

LongCtrlState = structs.CarControl.Actuators.LongControlState
ButtonType = structs.CarState.ButtonEvent.Type

MOTION_FORWARDS = 1
MOTION_REVERSING = 2
MOTION_STOPPED = 3

# (phase, hold fix, on screen)
PHASES = (
  ('ACCEL', True, 'rolling'),
  ('STOP', True, 'stopping'),
  ('CRAWL_GO', True, 'trigger'),
  ('RESTOP', True, 'hold 5 s'),
  ('DRIVEOFF', True, 'drive off'),
  ('STOP2', True, 'stop 2, hold 5 s'),
  ('FINAL', True, 'hold, disengage'),
)


class _ReproCarControl:
  # CarControl with longControlState replaced

  def __init__(self, CC, long_control_state):
    self.enabled = CC.enabled
    self.longActive = CC.longActive
    self.cruiseControl = CC.cruiseControl
    self.actuators = structs.CarControl.Actuators(longControlState=long_control_state)


class MebHoldRepro:
  ARM_SPEED = 70 / 3.6     # m/s, start below
  ARM_MIN_SPEED = 10 / 3.6  # m/s, start above, engaging from a hold that isn't ACC's never drives off
  ARM_WINDOW = 1.0         # s after SET to see the engagement
  ROLLBACK_ABORT = 0.5     # m/s while Motion_State is reversing
  CRUISE_SPEED = 1.5       # m/s, speed before each stop
  CRAWL_SPEED = 0.12       # m/s, go again while still rolling forward, like 18/6 at 0.06-0.16 m/s
  CRAWL_STOP_ACCEL = 0.0   # m/s^2, coast to a full stop during RAMP, like the natural stops in RAMP (p10 -0.02, p50 +0.12), braking here shut TSK down in 35
  CRAWL_ACCEL = 0.15       # m/s^2, the policy wants to go again once stopped, like 18/6
  ANFAHREN_TIME = 0.9      # s of weak ANFAHREN at standstill, 18/6 sent 0.93 s
  CRAWL_TIMEOUT = 3.0      # s to reach standstill during CRAWL_GO
  DWELL_TIME = 5.0         # s held at each stop
  DRIVEOFF_TIMEOUT = 6.0   # s to reach speed after a drive-off request
  HOLD_REQUEST_SPEED = 0.3  # m/s, plain braking above, a hold request at speed makes the hold manager refuse

  def __init__(self, step):
    self.dt = DT_CTRL * step
    self.set_time = None
    self.abort_speed = 0.
    self.hold_fix = True
    self.idx = None
    self.phase_time = 0.
    self.standstill_time = 0.
    self.fix_trigger = False  # fix run: stopped without an ACC hold while the policy wanted to drive off
    self.old_refused = False  # old run: the hold manager refused the ACC hold
    self.held_once = False
    self.crawl_stopped = False
    self.result = None
    self.result_detail = ''
    self.alert_text1 = None   # shown on screen through alertDebug once set
    self.alert_text2 = ''

  @property
  def phase(self):
    return PHASES[self.idx][0] if self.idx is not None else None

  def _state(self, CS):
    return (f'v={CS.out.vEgo:.2f} motion={CS.meb_motion_state} hms={CS.meb_hms_status} refused={CS.meb_hold_refused} ' +
            f'esp_hold={CS.meb_esp_hold} tsk={CS.meb_tsk_status}')

  def _log(self, CS, msg):
    carlog.warning(f'MEB repro [{"fix" if self.hold_fix else "old"}] {self.phase}: {msg} {self._state(CS)}')

  def _set_phase(self, CS, idx, msg=''):
    self.idx = idx if idx < len(PHASES) else None
    self.hold_fix = PHASES[self.idx][1] if self.idx is not None else True
    self.phase_time = 0.
    self.standstill_time = 0.
    self.crawl_stopped = False
    self._log(CS, f'enter {msg}')

  def _finish(self, CS, title, detail, hold=False):
    self._log(CS, f'RESULT: {title}, {detail}')
    self.result = title
    self.result_detail = f'{detail}, trigger fix {"yes" if self.fix_trigger else "no"} old {"yes" if self.old_refused else "no"}'
    self.alert_text1 = title
    self.alert_text2 = self.result_detail
    self.idx = len(PHASES) - 1 if hold else None  # hold: stop and hold in FINAL until disengage
    self.hold_fix = True

  def _abort(self, CS, reason, hold=False):
    if CS.out.accFaulted or CS.meb_tsk_status in (6, 7):
      if self.hold_fix:
        self._finish(CS, 'FIX FAILED', 'cruise fault with fix')
      elif self.fix_trigger:
        self._finish(CS, 'PASS', 'fix held, old faulted')
      else:
        self._finish(CS, 'REPEAT', 'old faulted, fix trigger not reached')
    elif not self.hold_fix and self.old_refused and self.fix_trigger:
      self._finish(CS, 'PASS', f'fix held, old refused ({reason})', hold)
    else:
      self._finish(CS, 'ABORTED', reason, hold)

  def observe(self, CS, CC):
    # every frame, button events only last one frame. only a SET that engages starts a run
    if self.idx is None and not CC.enabled and any(be.type == ButtonType.setCruise for be in CS.out.buttonEvents):
      self.set_time = 0.

  def update(self, CS, CC, accel):
    if self.set_time is not None:
      self.set_time += self.dt
      if self.set_time > self.ARM_WINDOW:
        self.set_time = None

    if self.idx is None:
      if self.set_time is None or not CC.enabled:
        return accel, CC
      self.set_time = None
      if CS.out.vEgo >= self.ARM_SPEED:
        self._finish(CS, 'NOT STARTED', f'SET at {CS.out.vEgo * 3.6:.0f} km/h, max 70')
        return accel, CC
      if CS.out.vEgo < self.ARM_MIN_SPEED:
        self._finish(CS, 'NOT STARTED', f'SET at {CS.out.vEgo * 3.6:.0f} km/h, min 10')
        return accel, CC
      if not CC.longActive:
        self._finish(CS, 'NOT STARTED', 'long not active')
        return accel, CC
      self.fix_trigger = self.old_refused = False
      self.abort_speed = max(CS.out.vEgo, self.CRUISE_SPEED) + 1.0
      self.result = None
      self._set_phase(CS, 0, 'armed')

    if self.hold_fix and CS.meb_hold_refused:
      self._finish(CS, 'FIX FAILED', 'hold refused with fix', hold=True)
      return self._stop_cmd(CS, CC)
    if not self.hold_fix and CS.meb_hold_refused and not self.old_refused:
      # master got refused: the car is no longer held for ACC and rolls on its own after ~1 s, end the run right away
      self.old_refused = True
      self._finish(CS, 'PASS' if self.fix_trigger else 'REPEAT', 'old refused, BRAKE NOW', hold=True)
      return self._stop_cmd(CS, CC)

    # driver and fault aborts hand back to the driver, the others stop and hold
    holding = self.phase == 'FINAL'
    if holding and self.result is not None and (CS.out.accFaulted or CS.meb_tsk_status in (6, 7) or not CC.enabled or
                                                not CC.longActive or CS.out.brakePressed or CS.out.gasPressed):
      self._log(CS, 'hold ended')
      self.idx = None
      return accel, CC
    if CS.out.accFaulted or CS.meb_tsk_status in (6, 7):
      self._abort(CS, 'cruise fault')
    elif not CC.enabled or not CC.longActive:
      self._abort(CS, 'disengaged')
    elif CS.out.brakePressed or CS.out.gasPressed:
      self._abort(CS, 'driver pedal')
    elif CS.out.parkingBrake:
      self._abort(CS, 'parking brake')
    elif not holding and CS.out.vEgo > self.abort_speed:
      self._abort(CS, 'too fast', hold=True)
    elif not holding and CS.meb_motion_state == MOTION_REVERSING and CS.out.vEgo > self.ROLLBACK_ABORT:
      self._abort(CS, 'rolling back, stopping', hold=True)
    if self.idx is None:
      return accel, CC

    self.phase_time += self.dt
    stopped = CS.meb_motion_state == MOTION_STOPPED
    self.standstill_time = self.standstill_time + self.dt if stopped else 0.

    accel, CC = self._step(CS, CC, accel, stopped)
    if self.phase == 'FINAL' and self.result is not None:
      self.alert_text1 = self.result
      self.alert_text2 = self.result_detail
    elif self.idx is not None:
      self.alert_text1 = f'{"FIX" if self.hold_fix else "OLD"} {self.idx + 1}/{len(PHASES)}: {PHASES[self.idx][2]}'
      self.alert_text2 = f'{CS.out.vEgo:.1f} m/s, hold {CS.meb_hms_status}, refused {CS.meb_hold_refused}'
    return accel, CC

  def _step(self, CS, CC, accel, stopped):
    phase = self.phase
    if phase in ('RESTOP', 'STOP2') and self.hold_fix:
      # once stopped and held, the car must not move again
      self.held_once |= self.standstill_time > 0.5
      if self.held_once and not stopped and CS.out.vEgo > 0.05:
        self._finish(CS, 'FIX FAILED', 'car rolled at hold', hold=True)
        return self._stop_cmd(CS, CC)
    else:
      self.held_once = False

    if phase in ('ACCEL', 'DRIVEOFF', 'DRIVEOFF2'):
      if CS.out.vEgo >= self.CRUISE_SPEED:
        self._set_phase(CS, self.idx + 1)
      elif self.phase_time > self.DRIVEOFF_TIMEOUT:
        self._finish(CS, 'FIX FAILED' if self.hold_fix else 'ABORTED', 'car did not drive off', hold=True)
        return -0.4, _ReproCarControl(CC, LongCtrlState.stopping)
      return 0.6, _ReproCarControl(CC, LongCtrlState.pid)

    if phase == 'STOP':
      if CS.meb_motion_state == MOTION_FORWARDS and CS.out.vEgo <= self.CRAWL_SPEED:
        self._set_phase(CS, self.idx + 1)
        return self._crawl(CS, CC)
      if stopped:
        self._set_phase(CS, self.idx + 1, 'stopped before crawl band')
        return self._crawl(CS, CC)
      # slow down, gently at the end so the crawl band is not skipped
      if self.abort_speed > self.CRUISE_SPEED + 1.0 and CS.out.vEgo < self.CRUISE_SPEED:
        self.abort_speed = self.CRUISE_SPEED + 1.0  # slowed down from the SET speed, abort if speeding up again
      return self._stop_cmd(CS, CC, accel)

    if phase == 'CRAWL_GO':
      return self._crawl(CS, CC)

    # RESTOP, STOP2, FINAL: stop and hold, FINAL holds until disengage
    if phase != 'FINAL' and self.standstill_time >= self.DWELL_TIME:
      self._set_phase(CS, self.idx + 1, 'dwell done')
      if self.phase == 'FINAL':
        self.result = 'PASS' if self.fix_trigger else 'REPEAT'
        self.result_detail = 'fix held, no refusal, disengage' if self.fix_trigger else 'trigger not reached, disengage'
        self._log(CS, f'RESULT: {self.result}, {self.result_detail}')
      return self._step(CS, CC, accel, stopped)
    return self._stop_cmd(CS, CC, accel)

  def _stop_cmd(self, CS, CC, accel=0.):
    # plain braking at speed like the planner, hold request only when nearly stopped
    if CS.out.vEgo > self.HOLD_REQUEST_SPEED:
      stop_accel = -1.0 if CS.out.vEgo > 3.0 else -0.5
      return min(accel, stop_accel), _ReproCarControl(CC, LongCtrlState.pid)
    return (-0.4 if CS.meb_motion_state == MOTION_STOPPED else -0.15), _ReproCarControl(CC, LongCtrlState.stopping)

  def _crawl(self, CS, CC):
    # policy aborts the stop at crawl (RAMP) and brakes gently to a full stop without a hold request, so the ESP holds
    # on its own. Then it wants to go: master requests ANFAHREN without an ACC hold, the fix gets the hold first
    if self.standstill_time > 0. and not self.crawl_stopped:
      self.crawl_stopped = True
      self.fix_trigger |= self.hold_fix and CS.meb_hms_status != 1  # stopped without an ACC hold, master requests ANFAHREN here
      self._log(CS, 'stopped after braking in the crawl')
    if self.standstill_time >= self.ANFAHREN_TIME:
      self._set_phase(CS, self.idx + 1, 'anfahren without movement done')
    elif not self.crawl_stopped and self.phase_time > self.CRAWL_TIMEOUT:
      self._set_phase(CS, self.idx + 1, 'no standstill during crawl')
    elif CS.meb_motion_state == MOTION_FORWARDS and CS.out.vEgo > 0.5:
      self._set_phase(CS, self.idx + 1, 'drove away during crawl')
    elif self.crawl_stopped and self.phase_time > self.CRAWL_TIMEOUT + self.ANFAHREN_TIME + 2.0:
      self._set_phase(CS, self.idx + 1, 'crawl timeout')
    else:
      if self.hold_fix and self.standstill_time > 0. and not CS.acc_hold_available:
        self.fix_trigger = True  # master would request a drive-off here and get the hold refused
      if not self.crawl_stopped:
        return self.CRAWL_STOP_ACCEL, _ReproCarControl(CC, LongCtrlState.pid)
      return self.CRAWL_ACCEL, _ReproCarControl(CC, LongCtrlState.pid)
    return self._stop_cmd(CS, CC)
