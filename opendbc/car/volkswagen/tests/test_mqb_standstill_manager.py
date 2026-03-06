"""
Integration tests for MQBStandstillManager, driven by ESPTSKSimulator.
"""
import types

from opendbc.car import DT_CTRL
from opendbc.car.volkswagen.carcontroller import MQBStandstillManager
from opendbc.car.volkswagen.values import CarControllerParams as CCP, HOLD_MAX_FRAMES
from opendbc.car.volkswagen.tests.esp_tsk_sim import (
  ESPTSKSimulator, SimInputs,
  ACC_ANHALTEWEG_NEUTRAL, ESP_HOLD_TIMER_LIMIT_FRAMES,
)

STEP_DT = DT_CTRL * CCP.ACC_CONTROL_STEP  # 0.02 s per ACC_CONTROL_STEP frame


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def _make_cs(sim_state: dict, brake_pressed: bool = False):
  """Build a minimal CS-like SimpleNamespace from sim.car_state() output."""
  cs = types.SimpleNamespace()
  cs.esp_hold_confirmation = sim_state["esp_hold_confirmation"]
  cs.road_grade = sim_state["road_grade"]
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
# Tests
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
