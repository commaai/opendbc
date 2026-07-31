import unittest
from types import SimpleNamespace

from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.meb_repro import MebShouldStopChurnRepro
from opendbc.car.volkswagen.mebcan import MebLongStateMachine
from opendbc.car.volkswagen.values import CarControllerParams, VolkswagenFlags


class TestMebShouldStopChurnRepro(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.CP = CarParams(carFingerprint="VOLKSWAGEN_ID4_MK2",
                       flags=int(VolkswagenFlags.MEB | VolkswagenFlags.MEB_GEN2))
    cls.CCP = CarControllerParams(cls.CP)

  def setUp(self):
    self.repro = MebShouldStopChurnRepro()
    self.long_state = MebLongStateMachine(self.CP, self.CCP)

  @staticmethod
  def make_car_state(speed, held, brake=False, gas=False, acc_faulted=False, parking_brake=False):
    out = SimpleNamespace(vEgo=speed, brakePressed=brake, gasPressed=gas, accFaulted=acc_faulted,
                          parkingBrake=parking_brake, cruiseState=SimpleNamespace(available=True))
    return SimpleNamespace(out=out, esp_hold_confirmation=held)

  @staticmethod
  def make_car_control(enabled=True, long_active=True, accel=0.0):
    actuators = SimpleNamespace(longControlState=0, accel=accel)
    return SimpleNamespace(enabled=enabled, longActive=long_active,
                           cruiseControl=SimpleNamespace(override=not long_active), actuators=actuators)

  def step(self, speed, held, brake=False, gas=False, enabled=True, long_active=True, policy_accel=0.0):
    car_state = self.make_car_state(speed, held, brake, gas)
    car_control = self.make_car_control(enabled, long_active, policy_accel)
    accel, repro_control = self.repro.update(car_state, car_control, policy_accel)
    armed = repro_control is not car_control
    accel, _, hold_type, braking_to_stop = self.long_state.update(car_state, repro_control, accel)
    return accel, hold_type, braking_to_stop, armed

  def test_approach_brakes_through_halten(self):
    accel, hold_type, braking_to_stop, armed = self.step(5.0, False)

    self.assertTrue(armed)
    self.assertAlmostEqual(accel, self.repro.APPROACH_ACCEL_MIN)
    self.assertEqual(hold_type, self.long_state.acc_hold_type_vals["HALTEN"])
    self.assertTrue(braking_to_stop)

  def test_creep_churn_never_sends_negative_accel_without_halten(self):
    emitted = [self.step(0.11, False) for _ in range(self.repro.CREEP_CHURN_FRAMES)]
    halten = self.long_state.acc_hold_type_vals["HALTEN"]
    ramp = self.long_state.acc_hold_type_vals["LOESEN_UEBER_RAMPE"]
    none = self.long_state.acc_hold_type_vals["KEINE_ANFORDERUNG"]

    self.assertEqual({row[1] for row in emitted}, {halten, ramp, none})
    for accel, hold_type, _, armed in emitted:
      self.assertTrue(armed)
      if accel < 0:
        self.assertEqual(hold_type, halten)

  def test_held_churn_uses_only_halten_and_anfahren(self):
    for _ in range(self.repro.CREEP_CHURN_FRAMES):
      self.step(0.11, False)

    emitted = [self.step(0.0, True) for _ in range(self.repro.HELD_CHURN_FRAMES)]
    halten = self.long_state.acc_hold_type_vals["HALTEN"]
    anfahren = self.long_state.acc_hold_type_vals["ANFAHREN"]

    self.assertEqual({row[1] for row in emitted}, {halten, anfahren})
    for accel, hold_type, _, armed in emitted:
      self.assertTrue(armed)
      self.assertGreaterEqual(accel, 0)
      if hold_type == halten:
        self.assertEqual(accel, self.CCP.ACCEL_INACTIVE)
      else:
        self.assertAlmostEqual(accel, self.repro.HELD_POKE_ACCEL)

  def test_driver_cancel_blocks_until_disengagement(self):
    self.assertTrue(self.step(0.11, False)[3])
    self.assertFalse(self.step(0.11, False, gas=True)[3])
    self.assertFalse(self.step(0.11, False)[3])
    self.assertFalse(self.step(0.11, False, enabled=False, long_active=False)[3])
    self.assertTrue(self.step(0.11, False)[3])

  def test_gas_during_anfahren_disarms_through_ramp(self):
    _, hold_type, _, armed = self.step(0.0, True)
    self.assertTrue(armed)
    self.assertEqual(hold_type, self.long_state.acc_hold_type_vals["ANFAHREN"])

    _, hold_type, _, armed = self.step(0.0, True, gas=True, long_active=False)
    self.assertFalse(armed)
    self.assertEqual(hold_type, self.long_state.acc_hold_type_vals["LOESEN_UEBER_RAMPE"])
    self.assertNotEqual(hold_type, self.long_state.acc_hold_type_vals["KEINE_ANFORDERUNG"])

  def test_launch_from_hold_is_sustained_and_exits_through_halten(self):
    held_launch = [self.step(0.0, True) for _ in range(20)]
    anfahren = self.long_state.acc_hold_type_vals["ANFAHREN"]
    halten = self.long_state.acc_hold_type_vals["HALTEN"]

    self.assertTrue(all(row[0] == self.repro.LAUNCH_ACCEL and row[1] == anfahren for row in held_launch))
    accel, hold_type, _, armed = self.step(0.06, False)
    self.assertTrue(armed)
    self.assertEqual(hold_type, halten)
    self.assertLess(accel, 0)

  def test_closed_loop_reaches_every_phase_from_common_engage_speeds(self):
    for initial_speed in (0.0, 0.2, 2.0, 9.5):
      self.setUp()
      speed = initial_speed
      held = initial_speed == 0.0
      release_frames = 0
      phases = set()

      for _ in range(2500):
        accel, hold_type, braking_to_stop, armed = self.step(speed, held)
        phases.add(self.repro.phase)
        self.assertTrue(armed)

        halten = self.long_state.acc_hold_type_vals["HALTEN"]
        anfahren = self.long_state.acc_hold_type_vals["ANFAHREN"]
        if accel < 0:
          self.assertEqual(hold_type, halten)
        if held:
          self.assertIn(hold_type, (halten, anfahren))

        if held:
          speed = 0.0
          asking_to_launch = hold_type == anfahren and accel >= 1.0
          release_frames = release_frames + 1 if asking_to_launch else 0
          if release_frames >= 5:
            held = False
            release_frames = 0
        else:
          effective_accel = 0.0 if accel == self.CCP.ACCEL_INACTIVE else accel
          speed = max(0.0, speed + effective_accel * 0.02)
          held = speed < 0.03 and braking_to_stop

      self.assertTrue({self.repro.LAUNCH, self.repro.CREEP_CHURN,
                       self.repro.HELD_CHURN, self.repro.SETTLE}.issubset(phases))
