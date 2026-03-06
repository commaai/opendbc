"""
Tests for:
  1. ESPTSKSimulator — unit tests for the ESP/TSK state machine
  2. MQBStandstillManager — closed-loop integration tests driven by the simulator
"""
import types

from opendbc.car import DT_CTRL
from opendbc.car.volkswagen.carcontroller import MQBStandstillManager
from opendbc.car.volkswagen.values import CarControllerParams as CCP, HOLD_MAX_FRAMES
from opendbc.car.volkswagen.tests.esp_tsk_sim import (
  ESPTSKSimulator, SimInputs,
  ACC_ANHALTEWEG_NEUTRAL, ESP_HOLD_TIMER_LIMIT_FRAMES, HILL_DECEL_TIMEOUT_FRAMES,
  TORQUE_RELEASE_RATIO,
)

STEP_DT = DT_CTRL * CCP.ACC_CONTROL_STEP  # 0.02 s per ACC_CONTROL_STEP frame


# ──────────────────────────────────────────────────────────────────────────────
# ESPTSKSimulator helpers
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
    from opendbc.car.volkswagen.tests.esp_tsk_sim import ESP_HOLD_ACQUIRE_SPEED_MS
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


# ──────────────────────────────────────────────────────────────────────────────
# MQBStandstillManager integration helpers
# ──────────────────────────────────────────────────────────────────────────────

def _make_cs(sim_state: dict, brake_pressed: bool = False):
  """Build a minimal CS-like SimpleNamespace from sim.car_state() output."""
  cs = types.SimpleNamespace()
  cs.esp_hold_confirmation = sim_state["esp_hold_confirmation"]
  cs.esp_hold_uphill = sim_state["esp_hold_uphill"]
  cs.esp_hold_torque_nm = sim_state["esp_hold_torque_nm"]
  cs.actual_torque_nm = sim_state["actual_torque_nm"]
  cs.wheel_impulse_count = sim_state["wheel_impulse_count"]
  cs.acc_type = sim_state["acc_type"]
  cs.out = types.SimpleNamespace()
  cs.out.standstill = sim_state["out.standstill"]
  cs.out.vEgo = sim_state["out.vEgo"]
  cs.out.brakePressed = brake_pressed
  return cs


def _to_sim_inputs(long_active: bool, accel: float, stopping_: bool, starting_: bool,
                   esp_starting_override, esp_stopping_override) -> SimInputs:
  """Mirror mqbcan.create_acc_accel_control signal mapping for the simulator."""
  acc07_stopping = esp_stopping_override if esp_stopping_override is not None else stopping_
  acc07_starting = esp_starting_override if esp_starting_override is not None else starting_
  return SimInputs(
    acc_anfahren=acc07_starting if long_active else False,
    acc_anhalten=acc07_stopping if long_active else False,
    acc_anhalteweg=0.3 if (acc07_stopping and long_active) else ACC_ANHALTEWEG_NEUTRAL,
    acc_sollbeschl_06=accel if long_active else 0.0,
  )


def _mgr_step(sim: ESPTSKSimulator, mgr: MQBStandstillManager,
              long_active: bool, accel: float, stopping_: bool, starting_: bool,
              brake_pressed: bool = False) -> tuple[dict, bool]:
  """One closed-loop update: manager outputs → simulator step. Returns (car_state, long_active_out)."""
  cs = _make_cs(sim.car_state(), brake_pressed)
  la, accel_out, stop_out, start_out, sstart, sstop = mgr.update(cs, long_active, accel, stopping_, starting_)
  sim.step(_to_sim_inputs(la, accel_out, stop_out, start_out, sstart, sstop))
  return sim.car_state(), la


# ──────────────────────────────────────────────────────────────────────────────
# MQBStandstillManager integration tests
# ──────────────────────────────────────────────────────────────────────────────

class TestMQBStandstillManagerIntegration:
  def test_constants_safe_relative_to_esp_timer(self):
    """HOLD_MAX_FRAMES must be < ESP_HOLD_TIMER_LIMIT_FRAMES so the safety cutoff fires first."""
    assert HOLD_MAX_FRAMES < ESP_HOLD_TIMER_LIMIT_FRAMES, (
      f"HOLD_MAX_FRAMES ({HOLD_MAX_FRAMES}) must be < ESP_HOLD_TIMER_LIMIT_FRAMES ({ESP_HOLD_TIMER_LIMIT_FRAMES})"
    )

  def test_flat_standstill_never_faults(self):
    """On flat ground manager overrides stopping to prevent hold acquisition; no fault for 300 frames."""
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=0.0)
    mgr = MQBStandstillManager(CCP)
    for frame in range(300):
      cs_after, _ = _mgr_step(sim, mgr, True, 0.0, True, False)
      assert not cs_after["_faulted"], f"flat ground fault at frame {frame}"
    assert not sim.car_state()["esp_hold_confirmation"], "no hold should persist on flat"

  def test_flat_briefly_acquired_hold_released_cleanly(self):
    """Hold briefly acquired during approach is released once at standstill; no fault over 200 frames."""
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=0.0)
    mgr = MQBStandstillManager(CCP)
    # Low speed — below ESP threshold — to exercise the brief-acquisition path
    sim.speed_ms = 0.05
    for _ in range(3):
      _mgr_step(sim, mgr, True, -1.0, True, False)
    sim.speed_ms = 0.0
    for frame in range(200):
      cs_after, _ = _mgr_step(sim, mgr, True, 0.0, True, False)
      assert not cs_after["_faulted"], f"fault after approach at frame {frame}"

  def test_brake_press_disables_long_active(self):
    """When driver brakes, manager returns long_active=False so no signals are sent to the ESP."""
    # Only test flat ground here: on a hill with no hold signals the hill decel timer
    # will fire after ~1 second, which is correct ESP behavior unrelated to the manager.
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=0.0)
    mgr = MQBStandstillManager(CCP)
    for frame in range(100):
      cs_after, la_out = _mgr_step(sim, mgr, True, 0.0, True, False, brake_pressed=True)
      assert not la_out, f"brake press should force long_active=False (frame {frame})"
      assert not cs_after["_faulted"], f"fault during brake press on flat (frame {frame})"

  def test_hill_safety_cutoff_fires_before_esp_fault(self):
    """On an extreme hill where conditional release can never succeed, manager disables long_active
    before the ESP hold timer would expire, preventing a cruise fault.

    After the manager disables we stop; the caller (openpilot) is responsible for not sending
    further signals that would trip the hill decel timer.
    """
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=9999.0)
    mgr = MQBStandstillManager(CCP)
    disabled_at = None
    for frame in range(200):
      cs_after, la_out = _mgr_step(sim, mgr, True, 0.0, True, False)
      assert not cs_after["_faulted"], (
        f"ESP faulted at frame {frame} before manager disabled (hold_timer={cs_after['_hold_timer_frames']})"
      )
      if not la_out:
        disabled_at = frame
        break  # stop here; further neutral signals would trip the hill decel timer
    assert disabled_at is not None, "manager should disable long_active on extreme hill"
    assert disabled_at <= ESP_HOLD_TIMER_LIMIT_FRAMES, (
      f"manager disabled too late at frame {disabled_at}; ESP limit {ESP_HOLD_TIMER_LIMIT_FRAMES}"
    )

  def test_hill_cycling_no_fault(self):
    """On a normal hill the I-controller builds enough torque for conditional release before the
    timer limit; the hold cycles and long_active stays True indefinitely."""
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=790.0)
    mgr = MQBStandstillManager(CCP)
    for frame in range(400):
      cs_after, la_out = _mgr_step(sim, mgr, True, 0.0, True, False)
      assert not cs_after["_faulted"], f"fault at frame {frame}"
      assert la_out, f"long_active dropped unexpectedly at frame {frame}"

  def test_spontaneous_reacquisition_detected_as_uphill(self):
    """When ESP spontaneously reacquires hold while manager is in flat starting mode,
    detected_uphill is set on the following frame so hill strategy takes over."""
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=0.0)
    mgr = MQBStandstillManager(CCP)
    # Enter flat starting mode: long_active, not uphill, starting=True, standstill, no hold
    for _ in range(5):
      _mgr_step(sim, mgr, True, 0.0, False, True)
    assert not mgr.detected_uphill
    # ESP spontaneously reacquires on the next sim.step()
    sim.trigger_spontaneous_reacquisition = True
    _mgr_step(sim, mgr, True, 0.0, False, True)  # hold acquires at end of this step
    assert not mgr.detected_uphill                 # not yet — detected one frame later
    _mgr_step(sim, mgr, True, 0.0, False, True)   # manager reads hold_confirmed=True → detects
    assert mgr.detected_uphill, "spontaneous reacquisition should trigger detected_uphill"
    # Continue without fault
    for frame in range(100):
      cs_after, la_out = _mgr_step(sim, mgr, True, 0.0, False, True)
      assert not cs_after["_faulted"], f"fault at frame {frame}"

  def test_disable_and_reenable_long_active_on_hill(self):
    """Briefly disabling long_active pauses the hold timer; re-enabling continues safely."""
    sim = ESPTSKSimulator(speed_ms=0.0, esp_hold_torque_nm=790.0)
    mgr = MQBStandstillManager(CCP)
    # Run for a few frames to acquire and hold
    for _ in range(5):
      _mgr_step(sim, mgr, True, 0.0, True, False)
    # Briefly disable (simulate driver take-over then re-engage)
    for _ in range(3):
      _mgr_step(sim, mgr, False, 0.0, False, False)
    # Re-enable and continue
    for frame in range(200):
      cs_after, _ = _mgr_step(sim, mgr, True, 0.0, True, False)
      assert not cs_after["_faulted"], f"fault after re-enable at frame {frame}"
