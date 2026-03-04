"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
import pytest
from dataclasses import dataclass, field

from opendbc.car import structs
from opendbc.sunnypilot.car.hyundai.longitudinal.controller import LongitudinalController
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

LongCtrlState = structs.CarControl.Actuators.LongControlState


@dataclass
class CP:
  carFingerprint: str = "KIA_NIRO_EV"
  flags: int = 0
  radarUnavailable: bool = False


@dataclass
class Actuators:
  accel: float = 0.0
  longControlState = LongCtrlState


@dataclass
class CC:
  actuators: Actuators = field(default_factory=lambda: Actuators())
  longActive: bool = True


@dataclass
class Out:
  vEgo: float = 0.0
  aEgo: float = 0.0


@dataclass
class CS:
  out: Out = field(default_factory=lambda: Out())
  aBasis: float = 0.0


class TestLongitudinalTuningController:
  def setup_method(self):
    self.CP = CP(flags=0)
    self.CP_SP = CP(flags=0)
    self.CP_SP.flags = HyundaiFlagsSP.LONG_TUNING_DYNAMIC
    self.CS = CS()
    self.CC = CC()
    self.controller = LongitudinalController(self.CP, self.CP_SP)

  def test_enabled_and_disabled(self):
    assert self.controller.enabled
    self.CP_SP.flags = 0
    assert not self.controller.enabled

  def test_stopping_state(self):
    self.CC.actuators.longControlState = LongCtrlState.stopping
    self.controller.get_stopping_state(self.CC.actuators)
    assert self.controller.stopping
    assert self.controller.stopping_count == 0
    self.CC.actuators.longControlState = LongCtrlState.pid
    self.controller.get_stopping_state(self.CC.actuators)
    assert not self.controller.stopping

  def test_calc_speed_based_jerk(self):
    assert (0.5, 5.0) == self.controller._calculate_speed_based_jerk_limits(0.0, LongCtrlState.stopping)
    velocities: list = [0.0, 2.0, 5.0, 7.0, 10.0, 15.0, 20.0, 25.0, 30.0]

    for velocity in velocities:
      upper_limit = float(np.interp(velocity, [0.0, 5.0, 20.0], [2.0, 3.0, 2.0]))
      lower_limit = float(np.interp(velocity, [0.0, 5.0, 20.0], [5.0, 3.5, 3.0]))
      expected: tuple = (upper_limit, lower_limit)
      actual: tuple = self.controller._calculate_speed_based_jerk_limits(velocity, LongCtrlState.pid)
      assert expected == actual

  def test_calc_lookahead_jerk(self):
    assert pytest.approx((-1.1, -1.1), abs=0.1) == self.controller._calculate_lookahead_jerk(-0.5, 4.9)
    assert pytest.approx((1.1, 1.1), abs=0.1) == self.controller._calculate_lookahead_jerk(0.5, 5.0)

  def test_calc_dynamic_low_jerk(self):
    self.controller.car_config.jerk_limits = 3.3
    assert 0.5 == self.controller._calculate_dynamic_lower_jerk(0.0, 10.0)
    assert 3.3 == self.controller._calculate_dynamic_lower_jerk(-2.0, 10.0)

  def test_calc_jerk(self):
    self.CP_SP.flags = 0
    self.controller.calculate_jerk(self.CC, self.CS, LongCtrlState.pid)
    assert self.controller.jerk_upper == 3.0
    assert self.controller.jerk_lower == 5.0
    self.controller.calculate_jerk(self.CC, self.CS, LongCtrlState.off)
    assert self.controller.jerk_upper == 1.0

    self.CP_SP.flags = HyundaiFlagsSP.LONG_TUNING_PREDICTIVE
    self.controller.__init__(self.CP, self.CP_SP)
    self.CS.out.vEgo = 10.0
    self.controller.accel_cmd = -3.5
    self.controller.accel_last = -1.0
    self.controller.calculate_jerk(self.CC, self.CS, LongCtrlState.pid)
    assert self.controller.jerk_upper == 0.5
    assert self.controller.jerk_lower == pytest.approx(3.3, abs=0.01)

    self.CP_SP.flags = HyundaiFlagsSP.LONG_TUNING_DYNAMIC
    self.controller.__init__(self.CP, self.CP_SP)
    self.CS.out.vEgo = 10.0
    self.CS.aBasis = -3.3
    self.CS.out.aEgo = -3.5
    self.controller.accel_cmd = -3.5
    self.controller.accel_last = -1.0
    for _ in range(50):
      self.controller.calculate_jerk(self.CC, self.CS, LongCtrlState.pid)
    assert self.controller.jerk_upper == 0.5
    assert self.controller.jerk_lower == 3.3

  def test_calc_accel(self):
    self.CP_SP.flags = 0
    self.controller.accel_cmd = 1.5
    self.controller.calculate_accel(self.CC)
    assert self.controller.desired_accel == self.controller.accel_cmd

    self.CP_SP.flags = HyundaiFlagsSP.LONG_TUNING_DYNAMIC
    self.CC.longActive = False
    self.controller.calculate_accel(self.CC)
    assert self.controller.desired_accel == 0.0

    self.CC.longActive = True
    self.controller.stopping = True
    self.controller.calculate_accel(self.CC)
    assert self.controller.desired_accel == 0.0

  def test_calc_comfort_band(self):
    stock_decels_list: list = [-3.5, -2.5, -1.5, -1.0, -0.5, -0.05]
    stock_accels_list: list = [0.0, 0.3, 0.6, 0.9, 1.2, 2.0]
    stock_comfort_band_vals: list = [0.0, 0.02, 0.04, 0.06, 0.08, 0.10]

    decels_list: list = [-3.5, -3.1, -2.245, -1.853, -1.234, -0.64352, -0.06432, -0.00005]
    accels_list: list = [0.0, 0.23345, 0.456, 0.5677, 0.6788, 0.834, 1.0, 1.3456, 1.8]

    for decel in decels_list:
      self.CS.out.aEgo = decel
      self.controller.calculate_comfort_band(self.CC, self.CS)
      actual = self.controller.comfort_band_lower
      expected = float(np.interp(decel, stock_decels_list, [0.1, 0.08, 0.06, 0.04, 0.02, 0.0]))
      assert actual == expected
      assert self.controller.comfort_band_upper == 0.0

    for accel in accels_list:
      self.CS.out.aEgo = accel
      self.controller.calculate_comfort_band(self.CC, self.CS)
      actual = self.controller.comfort_band_upper
      expected = float(np.interp(accel, stock_accels_list, stock_comfort_band_vals))
      assert actual == expected
      assert self.controller.comfort_band_lower == 0.0

  def test_update(self):
    self.CC.actuators.accel = 2.0
    self.CC.actuators.longControlState = LongCtrlState.pid
    self.CS.aBasis = 1.75
    self.CS.out.aEgo = 2.0
    self.CS.out.vEgo = 5.0

    self.controller.update(self.CC, self.CS)
    assert self.controller.jerk_lower == 0.5
    assert self.controller.jerk_upper == 3.0
    assert self.controller.comfort_band_lower == 0.0
    assert self.controller.comfort_band_upper == 0.10
    assert self.controller.desired_accel == 2.0
