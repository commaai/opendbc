import unittest
from types import SimpleNamespace

from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.mebcan import MebLongStateMachine
from opendbc.car.volkswagen.values import CarControllerParams, VolkswagenFlags

LongCtrlState = structs.CarControl.Actuators.LongControlState


class TestMebLongStateMachine(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.CP = CarParams(carFingerprint="VOLKSWAGEN_ID4_MK2",
                       flags=int(VolkswagenFlags.MEB | VolkswagenFlags.MEB_GEN2))
    cls.CCP = CarControllerParams(cls.CP)

  def setUp(self):
    self.state_machine = MebLongStateMachine(self.CP, self.CCP)
    self.none = self.state_machine.acc_hold_type_vals["KEINE_ANFORDERUNG"]
    self.halten = self.state_machine.acc_hold_type_vals["HALTEN"]
    self.anfahren = self.state_machine.acc_hold_type_vals["ANFAHREN"]
    self.ramp = self.state_machine.acc_hold_type_vals["LOESEN_UEBER_RAMPE"]

  @staticmethod
  def make_car_state(speed, held=False, acc_faulted=False):
    out = SimpleNamespace(vEgo=speed, accFaulted=acc_faulted,
                          cruiseState=SimpleNamespace(available=True))
    return SimpleNamespace(out=out, esp_hold_confirmation=held)

  @staticmethod
  def make_car_control(long_control_state, enabled=True, long_active=True):
    actuators = SimpleNamespace(longControlState=long_control_state)
    cruise_control = SimpleNamespace(override=enabled and not long_active)
    return SimpleNamespace(enabled=enabled, longActive=long_active,
                           cruiseControl=cruise_control, actuators=actuators)

  def update_hold_type(self, long_control_state, speed, held=False,
                       enabled=True, long_active=True, acc_faulted=False):
    car_state = self.make_car_state(speed, held, acc_faulted)
    car_control = self.make_car_control(long_control_state, enabled, long_active)
    _, _, hold_type, _ = self.state_machine.update(car_state, car_control, 0.0)
    return hold_type

  def start_hold_abort_ramp(self, speed=1 * CV.KPH_TO_MS):
    self.assertEqual(self.update_hold_type(LongCtrlState.stopping, speed), self.halten)
    self.assertEqual(self.update_hold_type(LongCtrlState.pid, speed), self.ramp)
    self.assertTrue(self.state_machine.hold_release_ramp_active)

  def test_hold_abort_ramp_has_no_timer(self):
    self.start_hold_abort_ramp()

    for _ in range(1000):
      hold_type = self.update_hold_type(LongCtrlState.pid, 4.99 * CV.KPH_TO_MS)
      self.assertEqual(hold_type, self.ramp)

    self.assertTrue(self.state_machine.hold_release_ramp_active)

  def test_new_halten_cancels_hold_abort_ramp(self):
    self.start_hold_abort_ramp()

    hold_type = self.update_hold_type(LongCtrlState.stopping, 2 * CV.KPH_TO_MS)

    self.assertEqual(hold_type, self.halten)
    self.assertFalse(self.state_machine.hold_release_ramp_active)

  def test_five_kph_cancels_hold_abort_ramp_while_engaged(self):
    self.start_hold_abort_ramp()
    self.assertEqual(self.update_hold_type(LongCtrlState.pid, 4.99 * CV.KPH_TO_MS), self.ramp)

    hold_type = self.update_hold_type(LongCtrlState.pid, 5 * CV.KPH_TO_MS)

    self.assertEqual(hold_type, self.none)
    self.assertFalse(self.state_machine.hold_release_ramp_active)

  def test_halten_abort_at_five_kph_needs_no_ramp(self):
    speed = 5 * CV.KPH_TO_MS
    self.assertEqual(self.update_hold_type(LongCtrlState.stopping, speed), self.halten)

    hold_type = self.update_hold_type(LongCtrlState.pid, speed)

    self.assertEqual(hold_type, self.none)
    self.assertFalse(self.state_machine.hold_release_ramp_active)
    self.assertEqual(self.state_machine.ramp_counter, 0)

  def assert_short_release_tail(self, enabled, long_active, acc_faulted=False):
    speed = 1 * CV.KPH_TO_MS
    self.assertEqual(self.update_hold_type(LongCtrlState.stopping, speed), self.halten)
    self.assertEqual(self.update_hold_type(LongCtrlState.pid, speed,
                                          enabled=enabled, long_active=long_active,
                                          acc_faulted=acc_faulted), self.ramp)

    for _ in range(self.state_machine.RAMP_FRAMES):
      self.assertEqual(self.update_hold_type(LongCtrlState.pid, speed,
                                            enabled=enabled, long_active=long_active,
                                            acc_faulted=acc_faulted), self.ramp)

    self.assertEqual(self.update_hold_type(LongCtrlState.pid, speed,
                                          enabled=enabled, long_active=long_active,
                                          acc_faulted=acc_faulted), self.none)

  def test_disengagement_from_halten_keeps_original_short_ramp(self):
    self.assert_short_release_tail(enabled=False, long_active=False)

  def test_gas_override_from_halten_keeps_original_short_ramp(self):
    self.assert_short_release_tail(enabled=True, long_active=False)

  def test_gas_override_converts_active_latch_to_short_ramp(self):
    speed = 1 * CV.KPH_TO_MS
    self.start_hold_abort_ramp(speed)

    hold_type = self.update_hold_type(LongCtrlState.pid, speed,
                                      enabled=True, long_active=False)

    self.assertEqual(hold_type, self.ramp)
    self.assertFalse(self.state_machine.hold_release_ramp_active)
    self.assertEqual(self.state_machine.ramp_counter, self.state_machine.RAMP_FRAMES)

  def test_acc_fault_converts_active_latch_to_short_ramp(self):
    speed = 1 * CV.KPH_TO_MS
    self.start_hold_abort_ramp(speed)

    hold_type = self.update_hold_type(LongCtrlState.pid, speed, acc_faulted=True)

    self.assertEqual(hold_type, self.ramp)
    self.assertFalse(self.state_machine.hold_release_ramp_active)
    self.assertEqual(self.state_machine.ramp_counter, self.state_machine.RAMP_FRAMES)

  def test_launch_release_keeps_existing_timed_ramp(self):
    speed = 0.0
    self.assertEqual(self.update_hold_type(LongCtrlState.pid, speed, held=True), self.halten)
    self.assertEqual(self.update_hold_type(LongCtrlState.pid, speed, held=True), self.anfahren)
    self.assertEqual(self.update_hold_type(LongCtrlState.pid, speed, held=False), self.ramp)

    for _ in range(self.state_machine.LAUNCH_RAMP_FRAMES):
      self.assertEqual(self.update_hold_type(LongCtrlState.pid, speed, held=False), self.ramp)

    self.assertEqual(self.update_hold_type(LongCtrlState.pid, speed, held=False), self.none)


if __name__ == "__main__":
  unittest.main()
