import unittest
from collections import deque
from types import SimpleNamespace

from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.meb_repro import MebCameraFaultPreconditioner, MebShouldStopChurnRepro
from opendbc.car.volkswagen.mebcan import MebLongStateMachine
from opendbc.car.volkswagen.values import CarControllerParams, VolkswagenFlags


class TestMebCameraFaultPreconditioner(unittest.TestCase):
  def test_two_fault_pulses_before_engagement(self):
    preconditioner = MebCameraFaultPreconditioner()
    wait = preconditioner.STARTUP_WAIT_FRAMES
    pulse = preconditioner.FAULT_PULSE_FRAMES
    gap = preconditioner.BETWEEN_PULSE_FRAMES
    second_start = wait + pulse + gap

    self.assertEqual(preconditioner.update(wait - 1, False, 2), 2)
    self.assertEqual(preconditioner.update(wait, False, 2), 6)
    self.assertEqual(preconditioner.update(wait + pulse - 1, False, 2), 6)
    self.assertEqual(preconditioner.update(wait + pulse, False, 2), 2)
    self.assertEqual(preconditioner.update(second_start - 1, False, 2), 2)
    self.assertEqual(preconditioner.update(second_start, False, 2), 6)
    self.assertEqual(preconditioner.update(second_start + pulse - 1, False, 2), 6)
    self.assertEqual(preconditioner.update(second_start + pulse, False, 2), 2)

  def test_never_overrides_while_engaged(self):
    preconditioner = MebCameraFaultPreconditioner()
    self.assertEqual(preconditioner.update(preconditioner.STARTUP_WAIT_FRAMES, True, 3), 3)


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
  def make_car_state(speed, held, motion_state, brake=False, gas=False, acc_faulted=False, parking_brake=False):
    out = SimpleNamespace(vEgo=speed, brakePressed=brake, gasPressed=gas, accFaulted=acc_faulted,
                          parkingBrake=parking_brake, cruiseState=SimpleNamespace(available=True))
    return SimpleNamespace(out=out, esp_hold_confirmation=held, meb_motion_state=motion_state)

  @staticmethod
  def make_car_control(enabled=True, long_active=True, accel=0.0):
    actuators = SimpleNamespace(longControlState=0, accel=accel)
    return SimpleNamespace(enabled=enabled, longActive=long_active,
                           cruiseControl=SimpleNamespace(override=not long_active), actuators=actuators)

  def step(self, speed, held, motion_state=None, brake=False, gas=False, enabled=True, long_active=True,
           policy_accel=0.0, acc_faulted=False, parking_brake=False):
    if motion_state is None:
      motion_state = self.repro.MOTION_STOPPED if held or speed == 0.0 else self.repro.MOTION_FORWARDS
    car_state = self.make_car_state(speed, held, motion_state, brake, gas, acc_faulted, parking_brake)
    car_control = self.make_car_control(enabled, long_active, policy_accel)
    accel, repro_control = self.repro.update(car_state, car_control, policy_accel)
    armed = repro_control is not car_control
    accel, _, hold_type, braking_to_stop = self.long_state.update(car_state, repro_control, accel)
    return accel, hold_type, braking_to_stop, armed

  def enter_creep_churn(self):
    self.step(self.repro.CREEP_SPEED_MAX + 0.01, False)
    for _ in range(self.repro.CREEP_STABLE_FRAMES):
      self.step(0.1, False)
    self.assertEqual(self.repro.phase, self.repro.CREEP_CHURN)
    self.assertEqual(self.repro.phase_frames, 0)

  def enter_stop_for_hold(self):
    self.enter_creep_churn()
    for _ in range(self.repro.CREEP_CHURN_FRAMES):
      self.step(0.1, False)
    self.step(0.1, False)
    self.assertEqual(self.repro.phase, self.repro.STOP_FOR_HOLD)

  def test_approach_brakes_through_halten(self):
    accel, hold_type, braking_to_stop, armed = self.step(5.0, False)

    self.assertTrue(armed)
    self.assertAlmostEqual(accel, self.repro.APPROACH_ACCEL_MIN)
    self.assertEqual(hold_type, self.long_state.acc_hold_type_vals["HALTEN"])
    self.assertTrue(braking_to_stop)

  def test_creep_must_be_stable_before_churn_counts(self):
    for _ in range(self.repro.CREEP_STABLE_FRAMES - 1):
      self.step(0.1, False)
    self.assertEqual(self.repro.phase, self.repro.STABILIZE_CREEP)

    self.step(0.13, False)
    self.assertEqual(self.repro.phase_frames, 0)

    self.enter_creep_churn()

  def test_should_stop_switches_on_recorded_planner_threshold(self):
    self.assertTrue(self.repro._planner_wants_stop(0.1, self.repro.SHOULD_STOP_ACCEL))
    self.assertFalse(self.repro._planner_wants_stop(0.1, self.repro.SHOULD_GO_ACCEL))
    self.assertFalse(self.repro._planner_wants_stop(0.3, self.repro.SHOULD_STOP_ACCEL))

  def test_creep_churn_never_sends_negative_accel_without_halten(self):
    self.enter_creep_churn()
    emitted = [self.step(0.1, False) for _ in range(self.repro.CREEP_CHURN_FRAMES)]
    halten = self.long_state.acc_hold_type_vals["HALTEN"]
    ramp = self.long_state.acc_hold_type_vals["LOESEN_UEBER_RAMPE"]
    none = self.long_state.acc_hold_type_vals["KEINE_ANFORDERUNG"]

    self.assertEqual({row[1] for row in emitted}, {halten, ramp, none})
    for accel, hold_type, _, armed in emitted:
      self.assertTrue(armed)
      if accel < 0:
        self.assertEqual(hold_type, halten)

  def test_moving_churn_restarts_stability_gate_outside_band(self):
    self.enter_creep_churn()
    for _ in range(20):
      self.step(0.1, False)
    self.assertEqual(self.repro.phase_frames, 20)

    accel, hold_type, _, armed = self.step(0.13, False)

    self.assertTrue(armed)
    self.assertEqual(self.repro.phase, self.repro.STABILIZE_CREEP)
    self.assertEqual(self.repro.phase_frames, 0)
    self.assertLess(accel, 0)
    self.assertEqual(hold_type, self.long_state.acc_hold_type_vals["HALTEN"])

  def test_held_churn_uses_only_halten_and_anfahren(self):
    self.enter_stop_for_hold()

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

  def test_held_churn_pauses_and_resumes_after_hold_release(self):
    self.enter_stop_for_hold()
    for _ in range(30):
      self.step(0.0, True)
    held_progress = self.repro.phase_frames

    accel, hold_type, _, armed = self.step(0.04, False)
    self.assertTrue(armed)
    self.assertEqual(self.repro.phase, self.repro.REACQUIRE_HOLD)
    self.assertEqual(self.repro.phase_frames, held_progress)
    self.assertEqual(accel, self.repro.STOP_ACCEL)
    self.assertEqual(hold_type, self.long_state.acc_hold_type_vals["HALTEN"])

    for _ in range(10):
      self.step(0.02, False)
    self.assertEqual(self.repro.phase_frames, held_progress)

    self.step(0.0, True)
    self.assertEqual(self.repro.phase, self.repro.HELD_CHURN)
    self.assertEqual(self.repro.phase_frames, held_progress + 1)

    for _ in range(self.repro.HELD_CHURN_FRAMES - self.repro.phase_frames):
      self.step(0.0, True)
    self.step(0.0, True)
    self.assertEqual(self.repro.phase, self.repro.SETTLE)

  def test_driver_cancel_blocks_until_disengagement(self):
    self.assertTrue(self.step(0.1, False)[3])
    self.assertFalse(self.step(0.1, False, gas=True)[3])
    self.assertFalse(self.step(0.1, False)[3])
    self.assertFalse(self.step(0.1, False, enabled=False, long_active=False)[3])
    self.assertTrue(self.step(0.1, False)[3])

  def test_reverse_blocks_until_disengagement(self):
    reverse = self.repro.MOTION_REVERSING

    self.assertFalse(self.step(0.1, False, motion_state=reverse)[3])
    self.assertFalse(self.step(0.1, False)[3])
    self.assertFalse(self.step(0.1, False, enabled=False, long_active=False)[3])
    self.assertTrue(self.step(0.1, False)[3])

  def test_tsk_fault_blocks_until_disengagement(self):
    self.assertFalse(self.step(0.1, False, acc_faulted=True)[3])
    self.assertFalse(self.step(0.1, False)[3])
    self.assertFalse(self.step(0.1, False, enabled=False, long_active=False)[3])
    self.assertTrue(self.step(0.1, False)[3])

  def test_gas_during_anfahren_disarms_through_ramp(self):
    _, hold_type, _, armed = self.step(0.0, True)
    self.assertTrue(armed)
    self.assertEqual(hold_type, self.long_state.acc_hold_type_vals["HALTEN"])

    _, hold_type, _, armed = self.step(0.0, True)
    self.assertTrue(armed)
    self.assertEqual(hold_type, self.long_state.acc_hold_type_vals["ANFAHREN"])

    _, hold_type, _, armed = self.step(0.0, True, gas=True, long_active=False)
    self.assertFalse(armed)
    self.assertEqual(hold_type, self.long_state.acc_hold_type_vals["LOESEN_UEBER_RAMPE"])
    self.assertNotEqual(hold_type, self.long_state.acc_hold_type_vals["KEINE_ANFORDERUNG"])

  def test_launch_settles_before_churn(self):
    held_launch = [self.step(0.0, True) for _ in range(20)]
    halten = self.long_state.acc_hold_type_vals["HALTEN"]
    anfahren = self.long_state.acc_hold_type_vals["ANFAHREN"]

    self.assertEqual(held_launch[0][0], self.CCP.ACCEL_INACTIVE)
    self.assertEqual(held_launch[0][1], halten)
    self.assertTrue(all(row[0] == self.repro.LAUNCH_ACCEL and row[1] == anfahren for row in held_launch[1:]))
    accel, _, _, armed = self.step(0.04, False)
    self.assertTrue(armed)
    self.assertEqual(self.repro.phase, self.repro.STABILIZE_CREEP)
    self.assertGreater(accel, 0)

    for speed in (0.07, 0.14, 0.20, 0.15, 0.11):
      self.step(speed, False)
      self.assertNotEqual(self.repro.phase, self.repro.CREEP_CHURN)

    self.enter_creep_churn()

  def test_delayed_plant_does_not_accelerate_to_one_mph(self):
    speed = 0.0
    held = True
    motion_state = self.repro.MOTION_STOPPED
    release_frames = 0
    delayed_accel = deque([0.0] * 20)  # adversarial 400 ms actuator delay
    maximum_speed_before_stop = 0.0

    for _ in range(1000):
      accel, hold_type, braking_to_stop, armed = self.step(speed, held, motion_state=motion_state)
      self.assertTrue(armed)
      if self.repro.phase in (self.repro.ESTABLISH_CREEP, self.repro.STABILIZE_CREEP, self.repro.CREEP_CHURN):
        maximum_speed_before_stop = max(maximum_speed_before_stop, speed)
      if self.repro.phase == self.repro.STOP_FOR_HOLD:
        break

      anfahren = self.long_state.acc_hold_type_vals["ANFAHREN"]
      command_accel = 0.0 if accel == self.CCP.ACCEL_INACTIVE else accel
      delayed_accel.append(command_accel)
      effective_accel = delayed_accel.popleft()
      if held:
        speed = 0.0
        asking_to_launch = hold_type == anfahren and accel >= 0.1
        release_frames = release_frames + 1 if asking_to_launch else 0
        if release_frames >= 5:
          held = False
          release_frames = 0
      else:
        speed = max(0.0, speed + effective_accel * 0.02)
        held = speed < 0.005 and braking_to_stop
      motion_state = self.repro.MOTION_STOPPED if held or speed == 0.0 else self.repro.MOTION_FORWARDS

    self.assertEqual(self.repro.phase, self.repro.STOP_FOR_HOLD)
    self.assertLess(maximum_speed_before_stop, 0.15)

  def test_closed_loop_sustains_forward_creep_before_held_churn(self):
    for initial_speed in (0.0, 0.2, 2.0, 9.5):
      self.setUp()
      speed = initial_speed
      held = initial_speed == 0.0
      motion_state = self.repro.MOTION_STOPPED if held else self.repro.MOTION_FORWARDS
      release_frames = 0
      consecutive_creep_frames = 0
      maximum_consecutive_creep_frames = 0
      phases = set()

      for _ in range(5000):
        accel, hold_type, braking_to_stop, armed = self.step(speed, held, motion_state=motion_state)
        phases.add(self.repro.phase)
        self.assertTrue(armed)

        halten = self.long_state.acc_hold_type_vals["HALTEN"]
        anfahren = self.long_state.acc_hold_type_vals["ANFAHREN"]
        ramp = self.long_state.acc_hold_type_vals["LOESEN_UEBER_RAMPE"]
        if accel < 0:
          self.assertEqual(hold_type, halten)
        if held:
          self.assertIn(hold_type, (halten, anfahren, ramp))

        if self.repro.phase == self.repro.CREEP_CHURN:
          self.assertEqual(motion_state, self.repro.MOTION_FORWARDS)
          self.assertFalse(held)
          self.assertGreaterEqual(speed, self.repro.CREEP_SPEED_MIN)
          self.assertLessEqual(speed, self.repro.CREEP_SPEED_MAX)
          consecutive_creep_frames += 1
          maximum_consecutive_creep_frames = max(maximum_consecutive_creep_frames, consecutive_creep_frames)
        else:
          consecutive_creep_frames = 0

        if held:
          speed = 0.0
          asking_to_launch = hold_type == anfahren and accel >= 0.1
          release_frames = release_frames + 1 if asking_to_launch else 0
          if release_frames >= 5:
            held = False
            release_frames = 0
        else:
          effective_accel = 0.0 if accel == self.CCP.ACCEL_INACTIVE else accel
          speed = max(0.0, speed + effective_accel * 0.02)
          held = speed < 0.005 and braking_to_stop

        motion_state = self.repro.MOTION_STOPPED if held or speed == 0.0 else self.repro.MOTION_FORWARDS

      self.assertGreaterEqual(maximum_consecutive_creep_frames, self.repro.CREEP_CHURN_FRAMES)
      self.assertTrue({self.repro.ESTABLISH_CREEP, self.repro.STABILIZE_CREEP, self.repro.CREEP_CHURN,
                       self.repro.STOP_FOR_HOLD, self.repro.HELD_CHURN, self.repro.SETTLE}.issubset(phases))
