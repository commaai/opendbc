import math
import unittest

import numpy as np

from opendbc.car import structs
from opendbc.car.honda.carcontroller import compute_bosch_gas
from opendbc.car.honda.values import CAR, CarControllerParams, HondaFlags


A_CRUISE_MIN = -0.5  # openpilot longitudinal_planner.py (openpilot #39060); opendbc can't import openpilot


def _honda_params(car, flags):
  CP = structs.CarParams()
  CP.carFingerprint = car
  CP.flags = int(flags)
  return CarControllerParams(CP)


class TestHondaBoschCoast(unittest.TestCase):
  def setUp(self):
    self.hybrid = _honda_params(CAR.HONDA_ACCORD, HondaFlags.BOSCH | HondaFlags.HYBRID)
    self.default = _honda_params(CAR.HONDA_ACCORD, HondaFlags.BOSCH)

  def test_only_accord_hybrid(self):
    for car in CAR:
      for flags in (HondaFlags.BOSCH, HondaFlags.BOSCH | HondaFlags.HYBRID):
        params = _honda_params(car, flags)
        expected = car == CAR.HONDA_ACCORD and bool(flags & HondaFlags.HYBRID)
        assert bool(len(params.BOSCH_COAST_GAS_LOOKUP_BP)) == expected, car

  def test_default_lookup_unchanged(self):
    for v_ego in (0., 5., 30.):
      for accel in np.linspace(-3.5, 2.0, 56):
        for braking in (False, True):
          gas, min_gas_accel = compute_bosch_gas(self.default, accel, v_ego, braking)
          assert gas == np.interp(accel, CarControllerParams.BOSCH_GAS_LOOKUP_BP, CarControllerParams.BOSCH_GAS_LOOKUP_V)
          assert min_gas_accel == CarControllerParams.BOSCH_GAS_LOOKUP_BP[0]

  def test_coast_instead_of_brake_at_speed(self):
    # mild requests coast on regen instead of lighting the brake lights
    gas, min_gas_accel = compute_bosch_gas(self.hybrid, -0.3, 20., False)
    assert min_gas_accel == -0.45 and -0.3 > min_gas_accel
    assert 0 < gas < 40
    assert compute_bosch_gas(self.hybrid, -0.45, 20., False)[0] == 0

  def test_brake_point_above_cruise_min(self):
    # cruise never asks for less than A_CRUISE_MIN when over set speed; it must still be able to brake
    assert self.hybrid.BOSCH_COAST_GAS_LOOKUP_BP[0] > A_CRUISE_MIN

  def test_hysteresis(self):
    assert math.isclose(compute_bosch_gas(self.hybrid, -0.42, 20., True)[1], -0.40)
    assert math.isclose(compute_bosch_gas(self.hybrid, -0.42, 20., False)[1], -0.45)

  def test_default_lookup_at_low_speed(self):
    # near a stop, weak requests keep a brake request (and creep handling) instead of adding gas
    for v_ego in np.linspace(0., 4., 9):
      for accel in np.linspace(-3.5, 2.0, 56):
        for braking in (False, True):
          assert compute_bosch_gas(self.hybrid, accel, v_ego, braking) == compute_bosch_gas(self.default, accel, v_ego, braking)

  def test_blend(self):
    _, min_gas_accel = compute_bosch_gas(self.hybrid, 0., 5., False)
    assert math.isclose(min_gas_accel, (-0.2 - 0.45) / 2)
    _, min_gas_accel = compute_bosch_gas(self.hybrid, 0., 5., True)
    assert math.isclose(min_gas_accel, (-0.2 - 0.45) / 2 + 0.025)

  def test_accel_requests_unchanged(self):
    for v_ego in (0., 5., 30.):
      for accel in np.linspace(0., 2.0, 21):
        assert math.isclose(compute_bosch_gas(self.hybrid, accel, v_ego, False)[0], compute_bosch_gas(self.default, accel, v_ego, False)[0])

  def test_gas_monotonic(self):
    for v_ego in (0., 5., 30.):
      gas = [compute_bosch_gas(self.hybrid, a, v_ego, False)[0] for a in np.linspace(-3.5, 2.0, 111)]
      assert all(b >= a for a, b in zip(gas[:-1], gas[1:], strict=True))
