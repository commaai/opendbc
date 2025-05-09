"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import unittest
import numpy as np
from unittest.mock import Mock

from opendbc.sunnypilot.car.hyundai.longitudinal.controller import LongitudinalController, LongitudinalState
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.car import DT_CTRL, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.hyundai.values import HyundaiFlags

LongCtrlState = structs.CarControl.Actuators.LongControlState


class TestLongitudinalTuningController(unittest.TestCase):
  def setUp(self):
    self.mock_CP = Mock(carFingerprint="KIA_NIRO_EV", flags=0)
    self.mock_CP.radarUnavailable = False            # ensure tuning branch
    self.mock_CP_SP = Mock(flags=0)
    self.controller = LongitudinalController(self.mock_CP, self.mock_CP_SP)

  def test_init(self):
    """Test controller initialization"""
    self.assertIsInstance(self.controller.tuning, LongitudinalState)
    self.assertEqual(self.controller.desired_accel, 0.0)
    self.assertEqual(self.controller.actual_accel, 0.0)
    self.assertEqual(self.controller.jerk_upper, 0.0)
    self.assertEqual(self.controller.jerk_lower, 0.0)
    self.assertEqual(self.controller.comfort_band_upper, 0.0)
    self.assertEqual(self.controller.comfort_band_lower, 0.0)

  def test_make_jerk_flag_off(self):
    """Test when LONG_TUNING_DYNAMIC flag is off"""
    mock_CC, mock_CS = Mock(spec=structs.CarControl), Mock(spec=CarStateBase)
    mock_CS.out = Mock()
    mock_CS.out.vEgo = 0.0
    mock_CS.out.aEgo = 0.0
    mock_CS.aBasis = 0.0

    # Test with PID state
    self.controller.calculate_jerk(mock_CC, mock_CS, LongCtrlState.pid)
    print(f"[PID state] jerk_upper={self.controller.jerk_upper:.2f}, jerk_lower={self.controller.jerk_lower:.2f}")
    self.assertEqual(self.controller.jerk_upper, 3.0)
    self.assertEqual(self.controller.jerk_lower, 5.0)

    # Test with non-PID state
    self.controller.calculate_jerk(mock_CC, mock_CS, LongCtrlState.stopping)
    print(f"[Non-PID state] jerk_upper={self.controller.jerk_upper:.2f}, jerk_lower={self.controller.jerk_lower:.2f}")
    self.assertEqual(self.controller.jerk_upper, 1.0)
    self.assertEqual(self.controller.jerk_lower, 5.0)

  def test_make_jerk_flag_on(self):
    """Only verify that limits update when flags are on."""
    self.controller.CP_SP.flags = HyundaiFlagsSP.LONG_TUNING_DYNAMIC
    self.controller.CP.flags = HyundaiFlags.CANFD
    mock_CC = Mock()
    mock_CC.actuators = Mock(accel=1.0)
    mock_CC.longActive = True
    self.controller.stopping = False
    mock_CS = Mock()
    mock_CS.out = Mock(aEgo=0.8, vEgo=3.0)
    mock_CS.aBasis = 0.8

    self.controller.calculate_jerk(mock_CC, mock_CS, LongCtrlState.pid)
    print(f"[FlagOn] jerk_upper={self.controller.jerk_upper:.3f}, jerk_lower={self.controller.jerk_lower:.3f}")
    self.assertGreater(self.controller.jerk_upper, 0.0)
    self.assertGreater(self.controller.jerk_lower, 0.0)

  def test_a_value_jerk_scaling(self):
    """Test a_value jerk scaling under tuning branch."""
    self.controller.CP_SP.flags = HyundaiFlagsSP.LONG_TUNING_DYNAMIC
    self.controller.CP.radarUnavailable = False
    mock_CC = Mock()
    mock_CC.actuators = Mock(accel=1.0)
    mock_CC.longActive = True
    print("[a_value] starting accel_last:", self.controller.tuning.accel_last)
    # first pass: limit to jerk_upper * DT_CTRL * 2 = 0.1
    self.controller.jerk_upper = 0.1 / (DT_CTRL * 2)
    self.controller.accel_cmd = 1.0  # ensure accel_cmd is set
    self.controller.calculate_accel(mock_CC)
    print(f"[a_value] pass1 actual_accel={self.controller.actual_accel:.5f}")
    self.assertAlmostEqual(self.controller.actual_accel, 0.1, places=5)

    # second pass: limit increment by new jerk_upper
    mock_CC.actuators.accel = 0.7
    self.controller.jerk_upper = 0.2 / (DT_CTRL * 2)
    self.controller.accel_cmd = 0.7  # update accel_cmd
    self.controller.calculate_accel(mock_CC)
    print(f"[a_value] pass2 actual_accel={self.controller.actual_accel:.5f}")
    self.assertAlmostEqual(self.controller.actual_accel, 0.3, places=5)

  def test_make_jerk_realistic_profile(self):
    """Test make_jerk with realistic velocity and acceleration profile"""
    np.random.seed(42)
    num_points = 30
    segments = [
      np.random.uniform(0.3, 0.8, num_points//4),
      np.random.uniform(0.8, 1.6, num_points//4),
      np.random.uniform(-0.2, 0.2, num_points//4),
      np.random.uniform(-1.2, -0.5, num_points//8),
      np.random.uniform(-2.2, -1.2, num_points//8)
    ]
    accels = np.concatenate(segments)[:num_points]
    vels = np.zeros_like(accels)
    vels[0] = 5.0
    for i in range(1, len(accels)):
      vels[i] = max(0.0, min(30.0, vels[i-1] + accels[i-1] * (DT_CTRL*2)))
    mock_CC, mock_CS = Mock(), Mock()
    mock_CC.actuators, mock_CS.out = Mock(), Mock()
    mock_CC.longActive = True
    self.controller.stopping = False

    # Test with LONG_TUNING_DYNAMIC only
    self.controller.CP_SP.flags = HyundaiFlagsSP.LONG_TUNING_DYNAMIC
    for v, a in zip(vels, accels, strict=True):
      mock_CS.out.vEgo = float(v)
      mock_CS.out.aEgo = float(a)
      mock_CS.aBasis = float(a)
      mock_CC.actuators.accel = float(a)
      self.controller.calculate_jerk(mock_CC, mock_CS, LongCtrlState.pid)
      print(f"[realistic][LONG_TUNING_DYNAMIC] v={v:.2f}, a={a:.2f}, jerk_upper={self.controller.jerk_upper:.2f}, jerk_lower={self.controller.jerk_lower:.2f}")
      self.assertGreater(self.controller.jerk_upper, 0.0)

    # Reset controller before next test
    self.controller.tuning = LongitudinalState()
    self.controller.jerk_upper = 0.5
    self.controller.jerk_lower = 0.5

    # Test with LONG_TUNING_DYNAMIC and LONG_TUNING_PREDICTIVE
    self.controller.CP_SP.flags = HyundaiFlagsSP.LONG_TUNING_DYNAMIC | HyundaiFlagsSP.LONG_TUNING_PREDICTIVE
    for v, a in zip(vels, accels, strict=True):
      mock_CS.out.vEgo = float(v)
      mock_CS.out.aEgo = float(a)
      mock_CS.aBasis = float(a)
      mock_CC.actuators.accel = float(a)
      self.controller.calculate_jerk(mock_CC, mock_CS, LongCtrlState.pid)
      print(f"[realistic][LONG_TUNING_PREDICTIVE] v={v:.2f}, a={a:.2f}, " +
            f"jerk_upper={self.controller.jerk_upper:.2f}, jerk_lower={self.controller.jerk_lower:.2f}")
      self.assertGreater(self.controller.jerk_upper, 0.0)


if __name__ == "__main__":
  unittest.main()
