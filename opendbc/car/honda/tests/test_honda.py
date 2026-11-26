import math
import unittest

import numpy as np

from opendbc.car.honda.carcontroller import apply_bosch_creep_compensation
from opendbc.car.honda.values import CAR, HondaFlags


class TestHondaFingerprint(unittest.TestCase):
  def test_tja_bosch_only(self):
    for car_model in CAR:
      if car_model.config.flags & HondaFlags.BOSCH_TJA_CONTROL:
        assert car_model.config.flags & HondaFlags.BOSCH, "Nidec car found with TJA control"


class TestHondaBoschCreepCompensation(unittest.TestCase):
  def test_dead_band_requests_are_firmed_up(self):
    # below 1 m/s, any braking request ends up at -1.0 m/s^2 or firmer, clear of the car's ~-0.75 dead band
    for v_ego in np.linspace(0., 1., 11):
      for accel in np.linspace(-0.75, 0.1, 18):
        assert apply_bosch_creep_compensation(accel, v_ego, False) <= -1.0 + 1e-9

  def test_firm_requests_get_small_offset(self):
    assert math.isclose(apply_bosch_creep_compensation(-2.0, 0.5, False), -2.3)

  def test_never_less_braking(self):
    for v_ego in np.linspace(0., 5., 26):
      for accel in np.linspace(-3.5, 2.0, 56):
        for standstill in (False, True):
          assert apply_bosch_creep_compensation(accel, v_ego, standstill) <= accel + 1e-9

  def test_no_effect_above_3_ms(self):
    for v_ego in (3., 5., 20.):
      for accel in np.linspace(-3.5, 2.0, 56):
        assert apply_bosch_creep_compensation(accel, v_ego, False) == accel

  def test_no_effect_on_accel_requests(self):
    for v_ego in np.linspace(0., 3., 13):
      for accel in np.linspace(0.3, 2.0, 18):
        assert apply_bosch_creep_compensation(accel, v_ego, False) == accel

  def test_launch_from_standstill_untouched(self):
    for accel in np.linspace(0.01, 2.0, 20):
      assert apply_bosch_creep_compensation(accel, 0., True) == accel

  def test_holds_brake_at_standstill(self):
    assert apply_bosch_creep_compensation(-0.2, 0., True) <= -1.0 + 1e-9

  def test_blends_between_1_and_3_ms(self):
    # halfway through the speed fade, halfway between the raw and compensated request
    assert math.isclose(apply_bosch_creep_compensation(-0.5, 2.0, False), -0.75)
