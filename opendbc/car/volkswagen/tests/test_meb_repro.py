import hashlib
import struct
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

  @staticmethod
  def make_car_state(speed, held=False, motion_state=1, brake=False, gas=False,
                     acc_faulted=False, parking_brake=False):
    out = SimpleNamespace(vEgo=speed, brakePressed=brake, gasPressed=gas,
                          accFaulted=acc_faulted, parkingBrake=parking_brake,
                          cruiseState=SimpleNamespace(available=True))
    return SimpleNamespace(out=out, esp_hold_confirmation=held, meb_motion_state=motion_state)

  @staticmethod
  def make_car_control(enabled=True, long_active=True, accel=0.0):
    actuators = SimpleNamespace(longControlState=0, accel=accel)
    return SimpleNamespace(enabled=enabled, longActive=long_active,
                           cruiseControl=SimpleNamespace(override=not long_active),
                           actuators=actuators)

  @staticmethod
  def raw_long_control_state(car_control):
    state = car_control.actuators.longControlState
    if hasattr(state, "raw"):
      return state.raw
    return state

  def update(self, speed, held=False, motion_state=1, brake=False, gas=False,
             enabled=True, long_active=True, policy_accel=0.0,
             acc_faulted=False, parking_brake=False):
    car_state = self.make_car_state(speed, held, motion_state, brake, gas,
                                    acc_faulted, parking_brake)
    car_control = self.make_car_control(enabled, long_active, policy_accel)
    accel, repro_control = self.repro.update(car_state, car_control, policy_accel)
    return car_state, car_control, accel, repro_control

  def test_recorded_policy_fingerprint(self):
    inputs = self.repro.RECORDED_POLICY_INPUTS
    payload = b"".join(struct.pack("<Bf", state, accel) for state, accel in inputs)

    self.assertEqual(len(inputs), 225)
    self.assertEqual(inputs[0], (1, 0.054852768778800964))
    self.assertEqual(inputs[-1], (2, -0.4000000059604645))
    self.assertEqual(hashlib.sha256(payload).hexdigest(),
                     "1329d466e87768caa54a31533d1aca4a924d2edc2f004e01e720f2ca6b6786c8")

  def test_replays_every_recorded_policy_input_frame_for_frame(self):
    emitted = []
    for _ in self.repro.RECORDED_POLICY_INPUTS:
      _, original_control, accel, repro_control = self.update(self.repro.ENTRY_SPEED_TARGET)
      self.assertIsNot(repro_control, original_control)
      emitted.append((self.raw_long_control_state(repro_control), accel))

    self.assertEqual(tuple(emitted), self.repro.RECORDED_POLICY_INPUTS)
    self.assertEqual(self.repro.phase, self.repro.APPROACH)
    self.assertEqual(self.repro.replay_index, 0)

  def test_restarts_replay_at_first_frame(self):
    for _ in self.repro.RECORDED_POLICY_INPUTS:
      self.update(self.repro.ENTRY_SPEED_TARGET)

    _, _, accel, repro_control = self.update(self.repro.ENTRY_SPEED_TARGET)

    self.assertEqual((self.raw_long_control_state(repro_control), accel),
                     self.repro.RECORDED_POLICY_INPUTS[0])
    self.assertEqual(self.repro.phase, self.repro.REPLAY)
    self.assertEqual(self.repro.replay_index, 1)

  def test_approach_automatically_matches_entry_speed(self):
    _, _, accel, repro_control = self.update(5.0)
    self.assertEqual(accel, self.repro.APPROACH_ACCEL_MIN)
    self.assertEqual(self.raw_long_control_state(repro_control), 2)

    _, _, accel, repro_control = self.update(0.02)
    self.assertEqual(accel, self.repro.APPROACH_ACCEL_MAX)
    self.assertEqual(self.raw_long_control_state(repro_control), 1)

  def test_approach_releases_a_confirmed_hold_through_production_state_machine(self):
    car_state, _, accel, repro_control = self.update(0.0, held=True,
                                                     motion_state=self.repro.MOTION_STOPPED)
    long_state = MebLongStateMachine(self.CP, self.CCP)
    output_accel, _, hold_type, braking_to_stop = long_state.update(car_state, repro_control, accel)

    self.assertEqual(self.raw_long_control_state(repro_control), 1)
    self.assertEqual(accel, self.repro.LAUNCH_ACCEL)
    self.assertEqual(output_accel, self.CCP.ACCEL_INACTIVE)
    self.assertEqual(hold_type, long_state.acc_hold_type_vals["HALTEN"])
    self.assertFalse(braking_to_stop)

  def test_long_inactive_restarts_the_exact_sequence(self):
    for _ in range(20):
      self.update(self.repro.ENTRY_SPEED_TARGET)
    self.assertEqual(self.repro.replay_index, 20)

    _, original_control, accel, repro_control = self.update(
      self.repro.ENTRY_SPEED_TARGET, long_active=False, policy_accel=0.25)
    self.assertIs(repro_control, original_control)
    self.assertEqual(accel, 0.25)
    self.assertEqual(self.repro.phase, self.repro.APPROACH)

    _, _, accel, repro_control = self.update(self.repro.ENTRY_SPEED_TARGET)
    self.assertEqual((self.raw_long_control_state(repro_control), accel),
                     self.repro.RECORDED_POLICY_INPUTS[0])

  def test_driver_cancel_blocks_until_disengagement(self):
    _, original_control, _, repro_control = self.update(self.repro.ENTRY_SPEED_TARGET, gas=True)
    self.assertIs(repro_control, original_control)

    _, original_control, _, repro_control = self.update(self.repro.ENTRY_SPEED_TARGET)
    self.assertIs(repro_control, original_control)

    _, original_control, _, repro_control = self.update(
      self.repro.ENTRY_SPEED_TARGET, enabled=False, long_active=False)
    self.assertIs(repro_control, original_control)

    _, original_control, _, repro_control = self.update(self.repro.ENTRY_SPEED_TARGET)
    self.assertIsNot(repro_control, original_control)

  def test_unsafe_vehicle_states_block_until_disengagement(self):
    unsafe_states = (
      dict(acc_faulted=True),
      dict(parking_brake=True),
      dict(speed=self.repro.ARM_SPEED + 0.01),
      dict(motion_state=self.repro.MOTION_REVERSING),
    )
    for unsafe_state in unsafe_states:
      with self.subTest(unsafe_state=unsafe_state):
        self.repro = MebShouldStopChurnRepro()
        arguments = dict(speed=self.repro.ENTRY_SPEED_TARGET)
        arguments.update(unsafe_state)
        _, original_control, _, repro_control = self.update(**arguments)
        self.assertIs(repro_control, original_control)

        _, original_control, _, repro_control = self.update(self.repro.ENTRY_SPEED_TARGET)
        self.assertIs(repro_control, original_control)

        self.update(self.repro.ENTRY_SPEED_TARGET, enabled=False, long_active=False)
        _, original_control, _, repro_control = self.update(self.repro.ENTRY_SPEED_TARGET)
        self.assertIsNot(repro_control, original_control)


if __name__ == "__main__":
  unittest.main()
