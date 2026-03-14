#!/usr/bin/env python3
import importlib
import unittest
from types import SimpleNamespace
from unittest.mock import patch


replay_drive_module = importlib.import_module("opendbc.safety.tests.safety_replay.replay_drive")


class FakeReplayMessage:
  def __init__(self, kind, log_mono_time=0, can=None, sendcan=None):
    self._kind = kind
    self.logMonoTime = log_mono_time
    self.can = can or []
    self.sendcan = sendcan or []

  def which(self):
    return self._kind


class FakeSafety:
  def __init__(self):
    self.alternative_experience = None
    self.autopark_active = False
    self.calls = []

  def set_safety_hooks(self, mode, param):
    self.calls.append(("set_safety_hooks", mode, param))
    self.autopark_active = True
    return 0

  def init_tests(self):
    self.calls.append(("init_tests",))
    self.alternative_experience = 0
    self.autopark_active = False

  def set_alternative_experience(self, alternative_experience):
    self.calls.append(("set_alternative_experience", alternative_experience))
    self.alternative_experience = alternative_experience

  def safety_tx_hook(self, _msg):
    return not self.autopark_active

  def set_timer(self, _timer):
    pass

  def safety_tick_current_safety_config(self):
    pass

  def safety_config_valid(self):
    return True

  def get_controls_allowed(self):
    return False

  def safety_fwd_hook(self, _bus_num, _addr):
    return 0

  def safety_rx_hook(self, _msg):
    return True


class TestSafetyReplay(unittest.TestCase):
  def test_replay_drive_resets_safety_before_initializing_segment(self):
    safety = FakeSafety()
    msgs = [FakeReplayMessage("can")]
    captured = {}

    def fake_init_segment(actual_safety, actual_msgs, _mode, _param):
      captured["calls"] = list(actual_safety.calls)
      captured["msgs"] = actual_msgs
      captured["tx_ready"] = actual_safety.safety_tx_hook(object())
      captured["alternative_experience"] = actual_safety.alternative_experience

    with patch.object(replay_drive_module, "libsafety_py", SimpleNamespace(libsafety=safety)), \
         patch.object(replay_drive_module, "init_segment", side_effect=fake_init_segment), \
         patch.object(replay_drive_module, "tqdm", side_effect=lambda it: it), \
         patch("builtins.print"):
      result = replay_drive_module.replay_drive(msgs, 10, 0, 7)

    self.assertTrue(result)
    self.assertIs(captured["msgs"], msgs)
    self.assertTrue(captured["tx_ready"])
    self.assertEqual(captured["alternative_experience"], 7)
    self.assertEqual(captured["calls"], [
      ("set_safety_hooks", 10, 0),
      ("init_tests",),
      ("set_alternative_experience", 7),
    ])


if __name__ == "__main__":
  unittest.main()
