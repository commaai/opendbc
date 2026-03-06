"""
Simulation of MQB ESP + TSK behavior at low speed / standstill.

Used to validate ACC type 1 standstill logic without requiring a real vehicle.

The ESP and TSK are modeled as state machines that react to signals sent by
the TSK controller (ACC_06 / ACC_07) and produce the CarState fields consumed
by the standstill logic in CarController.
"""
import math
from dataclasses import dataclass

from opendbc.car import DT_CTRL
from opendbc.car.volkswagen.values import CarControllerParams

# Time step: ACC_06/07 run at 50Hz (ACC_CONTROL_STEP=2 at 100Hz DT_CTRL).
STEP_DT = DT_CTRL * CarControllerParams.ACC_CONTROL_STEP  # 0.02s

# ACC_Anhalteweg sentinel: value sent when stop is NOT requested.
# Anything strictly below this is treated by the ESP as an active stop distance.
ACC_ANHALTEWEG_NEUTRAL = 20.46

# Speed below which the ESP will acquire a brake hold.
ESP_HOLD_ACQUIRE_SPEED_MS = 0.33 / 3.6  # 0.33 km/h

# Maximum frames hold_confirmed may be continuously True before the ESP faults.
# 1.15s / 0.02s per step = 57.5 → 57 full frames.
ESP_HOLD_TIMER_LIMIT_FRAMES = math.floor(1.15 / STEP_DT)

# Maximum frames TSK may command radbremsmom while stopped on a hill before the ESP faults.
# Approximately 1 second; exact value is grade-dependent but we don't model that precision.
# 1.0s / 0.02s per step = 50 frames.
HILL_DECEL_TIMEOUT_FRAMES = math.floor(1.0 / STEP_DT)

# Ratio of actual wheel torque to ESP_Haltemoment required for a conditional
# hold release (starting=True) to succeed.
# Calibrated from one data point: 790 Nm hold torque, 453 Nm insufficient, 468.1 Nm sufficient.
# Midpoint ratio ≈ 0.583.  TODO: refine with more data.
TORQUE_RELEASE_RATIO = 0.583

# Rough engine torque model: first-order lag from commanded accel → wheel torque.
# Scale: F = m·a, torque = F·r_wheel ≈ 1500 kg × 0.32 m = 480 Nm/(m/s²).
ENGINE_ACCEL_TO_WHEEL_TORQUE_NM = 480.0  # Nm per m/s²
ENGINE_TORQUE_TAU_S = 0.3                # first-order lag time constant (seconds)


@dataclass
class ESPState:
  """Simulated ESP module state."""
  hold_confirmed: bool = False
  faulted: bool = False
  # Counts frames where hold_confirmed has been continuously True since the last
  # timer reset.  Resets on: (a) wheel impulse while not confirmed, or
  # (b) successful conditional release via starting=True.
  # Pauses (does NOT reset) on unconditional release (starting=False, stopping=False).
  hold_timer_frames: int = 0
  # Counts frames where TSK is commanding radbremsmom while the car is stopped on a
  # hill.  TSK commands radbremsmom whenever hold_confirmed is False.
  # Resets to zero as soon as hold_confirmed goes True (radbremsmom stops).
  hill_decel_frames: int = 0
  # Set when the ESP spontaneously reacquires a hold without us requesting stopping=True.
  # When True, the hill decel timeout applies even if esp_hold_torque_nm ≤ 600.
  # Cleared only when wheels move while hold is not confirmed (car drives away).
  spontaneous_uphill: bool = False


@dataclass
class SimInputs:
  """Signals sent to the ESP/TSK this frame (from ACC_06 / ACC_07)."""
  # ACC_07
  acc_anfahren: bool = False           # ACC_Anfahren  — start request
  acc_anhalten: bool = False           # ACC_Anhalten  — stop request
  acc_anhalteweg: float = ACC_ANHALTEWEG_NEUTRAL  # ACC_Anhalteweg stopping distance
  acc_sollbeschl_07: float = 0.0       # ACC_Sollbeschleunigung_02 from ACC_07
  # ACC_06
  acc_sollbeschl_06: float = 0.0       # ACC_Sollbeschleunigung_02 from ACC_06 (torque pre-request)


class ESPTSKSimulator:
  """
  Drives the ESP and TSK state machines forward one ACC_CONTROL_STEP frame at a time.

  Callers set `speed_ms`, `actual_torque_nm`, and `esp_hold_torque_nm` to reflect the
  physical scenario before each step().  `wheel_impulse_count` is updated internally
  when the simulator determines the car has moved.

  Usage::

    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=790.0)
    # --- approach and acquire hold ---
    sim.step(SimInputs(acc_anhalten=True, acc_anhalteweg=0.3))
    assert sim.car_state()["esp_hold_confirmation"]
  """

  def __init__(self, speed_ms: float = 0.0, esp_hold_torque_nm: float = 0.0,
               accel_to_torque_nm: float = ENGINE_ACCEL_TO_WHEEL_TORQUE_NM,
               torque_tau_s: float = ENGINE_TORQUE_TAU_S):
    self.esp = ESPState()
    self.speed_ms = speed_ms
    # Torque the ESP requires at the wheel to release a hold (from ESP_15).
    # Set by the test scenario to reflect hill grade.
    # Values ≤ 600 Nm indicate flat/downhill (ESP can hold indefinitely).
    self.esp_hold_torque_nm = esp_hold_torque_nm
    # Current actual wheel torque, driven by the engine model.
    self.actual_torque_nm: float = 0.0
    # Monotonically increasing impulse count; incremented whenever wheel movement is detected.
    self.wheel_impulse_count: int = 0
    self._prev_speed_ms = speed_ms
    self._accel_to_torque_nm = accel_to_torque_nm
    self._torque_alpha = STEP_DT / torque_tau_s  # first-order lag coefficient
    # Set to True to trigger a spontaneous hold reacquisition on the next eligible frame
    # (stopped, hold not currently confirmed).  Cleared automatically after firing.
    # Simulates the rare ESP behavior where it reacquires hold without a stopping=True request.
    self.trigger_spontaneous_reacquisition: bool = False

  def _check_faults(self, inp: SimInputs) -> bool:
    """Return True if this frame's inputs would cause an immediate ESP/TSK fault."""
    # Simultaneous start + stop is illegal.
    if inp.acc_anfahren and inp.acc_anhalten:
      return True
    # anhalten and anhalteweg must agree.
    anhalteweg_active = inp.acc_anhalteweg < ACC_ANHALTEWEG_NEUTRAL
    if inp.acc_anhalten != anhalteweg_active:
      return True
    return False

  def _update_wheel_impulses(self) -> bool:
    """Detect wheel movement and update impulse count.  Returns True if movement detected."""
    if self.speed_ms > 0 or self._prev_speed_ms > 0:
      self.wheel_impulse_count += 1
      return True
    return False

  def step(self, inp: SimInputs) -> None:
    """Advance the simulation by one ACC_CONTROL_STEP (0.02 s) frame."""
    if self.esp.faulted:
      return

    if self._check_faults(inp):
      self.esp.faulted = True
      return

    # Engine torque model: first-order lag toward target torque implied by ACC_06 accel command.
    # Only positive accel commands produce torque; idle/braking → target is 0.
    torque_target = max(0.0, inp.acc_sollbeschl_06) * self._accel_to_torque_nm
    self.actual_torque_nm += (torque_target - self.actual_torque_nm) * self._torque_alpha

    # Wheel impulse detection (before hold logic so timer-reset uses this frame's movement).
    moved = self._update_wheel_impulses()

    is_uphill = self.esp_hold_torque_nm > 600

    # --- Hill decel timeout ---
    # TSK commands radbremsmom whenever hold is not confirmed.  On a hill (or after a
    # spontaneous reacquisition), sustained radbremsmom while stopped faults the ESP.
    hill_active = is_uphill or self.esp.spontaneous_uphill
    if not self.esp.hold_confirmed and self.speed_ms == 0.0 and hill_active:
      self.esp.hill_decel_frames += 1
      if self.esp.hill_decel_frames > HILL_DECEL_TIMEOUT_FRAMES:
        self.esp.faulted = True
        return
    else:
      self.esp.hill_decel_frames = 0

    # --- Hold timer ---
    if self.esp.hold_confirmed:
      self.esp.hold_timer_frames += 1
      if self.esp.hold_timer_frames > ESP_HOLD_TIMER_LIMIT_FRAMES:
        self.esp.faulted = True
        return
    elif moved:
      # Wheels moved while hold not confirmed: reset hold timer and clear spontaneous_uphill
      # (car has driven away, ESP internal hill state is gone).
      self.esp.hold_timer_frames = 0
      self.esp.spontaneous_uphill = False

    # --- Hold release ---
    if not inp.acc_anfahren and not inp.acc_anhalten:
      # Unconditional release: always drops hold, timer is NOT reset.
      self.esp.hold_confirmed = False

    elif inp.acc_anfahren and not inp.acc_anhalten and self.esp.hold_confirmed:
      # Conditional release: only succeeds if actual torque is sufficient to prevent rollback.
      torque_sufficient = self.actual_torque_nm >= self.esp_hold_torque_nm * TORQUE_RELEASE_RATIO
      if torque_sufficient:
        self.esp.hold_confirmed = False
        self.esp.hold_timer_frames = 0  # successful release resets the timer

    # --- Hold acquisition ---
    elif inp.acc_anhalten and not inp.acc_anfahren and self.speed_ms < ESP_HOLD_ACQUIRE_SPEED_MS:
      self.esp.hold_confirmed = True

    # --- Spontaneous hold reacquisition ---
    # Rare ESP behavior: reacquires hold without a stopping=True request.
    # Only fires when stopped and hold is not currently confirmed.
    # Sets spontaneous_uphill so the hill decel timeout applies on subsequent radbremsmom phases.
    if self.trigger_spontaneous_reacquisition and not self.esp.hold_confirmed and self.speed_ms == 0.0:
      self.esp.hold_confirmed = True
      self.esp.spontaneous_uphill = True
      self.trigger_spontaneous_reacquisition = False

    self._prev_speed_ms = self.speed_ms


  def car_state(self) -> dict:
    """Return a dict of CS fields consumed by the CarController standstill logic."""
    is_uphill = self.esp_hold_torque_nm > 600
    return {
      "esp_hold_confirmation": self.esp.hold_confirmed,
      "esp_hold_torque_nm": self.esp_hold_torque_nm if is_uphill else 0.0,
      "road_grade": 5.0 if is_uphill else 0.0,
      "actual_torque_nm": self.actual_torque_nm,
      "wheel_impulse_count": self.wheel_impulse_count,
      "out.standstill": self.speed_ms == 0.0,
      "out.vEgo": self.speed_ms,
      "acc_type": 1,
      # Diagnostic fields for test assertions.
      "_faulted": self.esp.faulted,
      "_hold_timer_frames": self.esp.hold_timer_frames,
      "_hill_decel_frames": self.esp.hill_decel_frames,
      "_spontaneous_uphill": self.esp.spontaneous_uphill,
    }


# ──────────────────────────────────────────────────────────────────────────────
# ESPTSKSimulator test helpers
# ──────────────────────────────────────────────────────────────────────────────

def stopping(sim: ESPTSKSimulator, n: int = 1) -> None:
  """Send n frames of stopping (acc_anhalten=True, acc_anfahren=False)."""
  for _ in range(n):
    sim.step(SimInputs(acc_anhalten=True, acc_anhalteweg=0.3))


def starting(sim: ESPTSKSimulator, n: int = 1) -> None:
  """Send n frames of starting (acc_anfahren=True, acc_anhalten=False)."""
  for _ in range(n):
    sim.step(SimInputs(acc_anfahren=True))


def neutral(sim: ESPTSKSimulator, n: int = 1) -> None:
  """Send n frames of neither starting nor stopping."""
  for _ in range(n):
    sim.step(SimInputs())


def acquire_hold(sim: ESPTSKSimulator) -> None:
  """Send one stopping step and assert the hold is confirmed."""
  stopping(sim, 1)
  assert sim.car_state()["esp_hold_confirmation"], "hold should confirm after one stopping() step"


# ──────────────────────────────────────────────────────────────────────────────
# ESPTSKSimulator unit tests
# ──────────────────────────────────────────────────────────────────────────────

class TestImmediateFaults:
  def test_simultaneous_start_stop_faults(self):
    sim = ESPTSKSimulator()
    sim.step(SimInputs(acc_anfahren=True, acc_anhalten=True, acc_anhalteweg=0.3))
    assert sim.car_state()["_faulted"]

  def test_anhalten_without_anhalteweg_faults(self):
    sim = ESPTSKSimulator()
    sim.step(SimInputs(acc_anhalten=True, acc_anhalteweg=ACC_ANHALTEWEG_NEUTRAL))
    assert sim.car_state()["_faulted"]

  def test_anhalteweg_without_anhalten_faults(self):
    sim = ESPTSKSimulator()
    sim.step(SimInputs(acc_anhalten=False, acc_anhalteweg=0.3))
    assert sim.car_state()["_faulted"]

  def test_valid_stopping_no_fault(self):
    sim = ESPTSKSimulator()
    sim.step(SimInputs(acc_anhalten=True, acc_anhalteweg=0.3))
    assert not sim.car_state()["_faulted"]

  def test_valid_starting_no_fault(self):
    sim = ESPTSKSimulator()
    sim.step(SimInputs(acc_anfahren=True))
    assert not sim.car_state()["_faulted"]


class TestHoldMechanics:
  def test_hold_acquired_at_standstill(self):
    sim = ESPTSKSimulator(speed_ms=0.0)
    acquire_hold(sim)
    assert sim.car_state()["esp_hold_confirmation"]

  def test_hold_not_acquired_above_threshold_speed(self):
    sim = ESPTSKSimulator(speed_ms=ESP_HOLD_ACQUIRE_SPEED_MS + 0.01)
    stopping(sim, 1)
    assert not sim.car_state()["esp_hold_confirmation"]

  def test_hold_unconditional_release_by_neutral(self):
    sim = ESPTSKSimulator(speed_ms=0.0)
    acquire_hold(sim)
    neutral(sim, 1)
    assert not sim.car_state()["esp_hold_confirmation"]

  def test_conditional_release_succeeds_with_sufficient_torque(self):
    """starting=True releases hold when actual wheel torque meets threshold."""
    haltemoment = 790.0
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=haltemoment, torque_tau_s=0.1)
    acquire_hold(sim)
    # Build torque for 20 frames while holding (well within hold timer limit)
    for _ in range(20):
      sim.step(SimInputs(acc_anhalten=True, acc_anhalteweg=0.3, acc_sollbeschl_06=2.0))
    assert sim.actual_torque_nm >= haltemoment * TORQUE_RELEASE_RATIO
    # Conditional release while maintaining torque (torque must not drop to zero)
    sim.step(SimInputs(acc_anfahren=True, acc_sollbeschl_06=2.0))
    assert not sim.car_state()["esp_hold_confirmation"]

  def test_conditional_release_fails_with_insufficient_torque(self):
    """starting=True does NOT release hold when torque is insufficient."""
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=790.0)
    acquire_hold(sim)
    # No torque built — conditional release should fail
    starting(sim, 1)
    assert sim.car_state()["esp_hold_confirmation"]


class TestTimers:
  def test_hold_timer_increments_while_confirmed(self):
    sim = ESPTSKSimulator(speed_ms=0.0)
    acquire_hold(sim)
    stopping(sim, 5)
    assert sim.car_state()["_hold_timer_frames"] == 5

  def test_hold_timer_faults_at_limit(self):
    sim = ESPTSKSimulator(speed_ms=0.0)
    acquire_hold(sim)
    stopping(sim, ESP_HOLD_TIMER_LIMIT_FRAMES)
    assert not sim.car_state()["_faulted"]
    stopping(sim, 1)
    assert sim.car_state()["_faulted"]

  def test_hold_timer_pauses_on_unconditional_release(self):
    """Neutral (unconditional release) pauses the timer; it resumes on re-acquisition."""
    sim = ESPTSKSimulator(speed_ms=0.0)
    acquire_hold(sim)
    stopping(sim, 10)
    neutral(sim, 1)  # releases, timer was 10 → pauses
    # Timer increments once on the neutral step (hold was True at start of that step), so 11
    acquire_hold(sim)
    assert sim.car_state()["_hold_timer_frames"] == 11

  def test_hold_timer_resets_on_wheel_movement_without_hold(self):
    """Timer resets when wheels move while hold is not confirmed."""
    sim = ESPTSKSimulator(speed_ms=0.0)
    acquire_hold(sim)
    stopping(sim, 10)
    neutral(sim, 1)            # release hold
    sim.speed_ms = 0.5
    neutral(sim, 1)            # wheel movement detected, no hold
    sim.speed_ms = 0.0
    acquire_hold(sim)
    assert sim.car_state()["_hold_timer_frames"] == 0

  def test_hold_timer_resets_on_successful_conditional_release(self):
    """Successful conditional release resets the timer to 0."""
    haltemoment = 400.0
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=haltemoment, torque_tau_s=0.1)
    acquire_hold(sim)
    # Build torque for 20 frames (well within timer limit of 57)
    for _ in range(20):
      sim.step(SimInputs(acc_anhalten=True, acc_anhalteweg=0.3, acc_sollbeschl_06=2.0))
    assert sim.car_state()["_hold_timer_frames"] == 20
    assert sim.actual_torque_nm >= haltemoment * TORQUE_RELEASE_RATIO
    # Conditional release while maintaining torque
    sim.step(SimInputs(acc_anfahren=True, acc_sollbeschl_06=2.0))
    assert not sim.car_state()["esp_hold_confirmation"]
    assert sim.car_state()["_hold_timer_frames"] == 0

  def test_hill_decel_timer_counts_while_stopped_on_hill(self):
    """TSK radbremsmom (no hold, hill) increments the hill decel timer."""
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=790.0)
    for _ in range(10):
      sim.step(SimInputs())
    assert sim.car_state()["_hill_decel_frames"] == 10

  def test_hill_decel_timer_faults_at_limit(self):
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=790.0)
    for _ in range(HILL_DECEL_TIMEOUT_FRAMES):
      sim.step(SimInputs())
    assert not sim.car_state()["_faulted"]
    sim.step(SimInputs())
    assert sim.car_state()["_faulted"]

  def test_hill_decel_timer_resets_when_hold_confirms(self):
    """Hold confirmation stops radbremsmom; timer resets on second stopping step."""
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=790.0)
    for _ in range(10):
      sim.step(SimInputs())
    assert sim.car_state()["_hill_decel_frames"] == 10
    stopping(sim, 1)  # first stopping: hold acquires; hill_decel increments once more then...
    stopping(sim, 1)  # second stopping: hold was confirmed at start → hill_decel resets to 0
    assert sim.car_state()["_hill_decel_frames"] == 0

  def test_hill_decel_timer_flat_no_fault(self):
    """Flat ground (low haltemoment) does not accumulate hill decel timer."""
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=0.0)
    for _ in range(HILL_DECEL_TIMEOUT_FRAMES + 10):
      sim.step(SimInputs())
    assert not sim.car_state()["_faulted"]


class TestFlatStrategy:
  def test_neutral_releases_hold(self):
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=0.0)
    acquire_hold(sim)
    neutral(sim, 1)
    assert not sim.car_state()["esp_hold_confirmation"]

  def test_starting_prevents_reacquisition(self):
    """After release, starting=True prevents the ESP from re-acquiring hold."""
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=0.0)
    acquire_hold(sim)
    neutral(sim, 1)
    for _ in range(30):
      starting(sim, 1)
    assert not sim.car_state()["esp_hold_confirmation"]
    assert not sim.car_state()["_faulted"]

  def test_flat_hold_timer_does_not_expire(self):
    """On flat ground: acquire hold, immediately do neutral+starting; timer never expires."""
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=0.0)
    acquire_hold(sim)
    neutral(sim, 1)
    for _ in range(100):
      starting(sim, 1)
    assert not sim.car_state()["_faulted"]


class TestSpontaneousReacquisition:
  def test_spontaneous_reacquisition_sets_flag(self):
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=0.0)
    sim.trigger_spontaneous_reacquisition = True
    neutral(sim, 1)
    cs = sim.car_state()
    assert cs["esp_hold_confirmation"]
    assert cs["_spontaneous_uphill"]

  def test_spontaneous_reacquisition_triggers_hill_decel_timeout(self):
    """After spontaneous reacquisition, hill decel timeout applies even on flat grade."""
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=0.0)  # flat torque
    sim.trigger_spontaneous_reacquisition = True
    neutral(sim, 1)   # hold acquires with spontaneous_uphill=True
    neutral(sim, 1)   # unconditional release; spontaneous_uphill flag persists
    assert not sim.car_state()["esp_hold_confirmation"]
    # Hill decel timer now applies (spontaneous_uphill is set)
    faulted = False
    for _ in range(HILL_DECEL_TIMEOUT_FRAMES + 2):
      sim.step(SimInputs())
      if sim.car_state()["_faulted"]:
        faulted = True
        break
    assert faulted

  def test_spontaneous_uphill_clears_on_wheel_movement(self):
    """spontaneous_uphill flag clears once wheels move without a hold."""
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=0.0)
    sim.trigger_spontaneous_reacquisition = True
    neutral(sim, 1)   # acquires with spontaneous_uphill=True
    neutral(sim, 1)   # releases
    assert sim.car_state()["_spontaneous_uphill"]
    sim.speed_ms = 0.5
    neutral(sim, 1)   # wheel movement, no hold → clears flag
    sim.speed_ms = 0.0
    assert not sim.car_state()["_spontaneous_uphill"]
