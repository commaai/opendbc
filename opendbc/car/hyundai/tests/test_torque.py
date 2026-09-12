import unittest

from opendbc.can import CANParser
from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarControl
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR


class TestHyundaiCanfdTorque(unittest.TestCase):
  def setUp(self):
    CP = CarInterface.get_params(CAR.KIA_EV6, gen_empty_fingerprint(), [], False, False, False)
    self.CI = CarInterface(CP)
    self.CC = CarControl(enabled=True, latActive=True)
    self.CC.actuators.torque = 1.
    self.parser = CANParser("hyundai_canfd_generated", [("LFA", 100)], 0)

  def update(self):
    timestamp = self.CI.CC.frame * 10_000_000
    actuators, msgs = self.CI.apply(self.CC.as_reader(), timestamp)
    self.parser.update([timestamp, msgs])
    assert self.parser.vl["LFA"]["StrTqReqVal"] == actuators.torqueOutputCan
    return actuators

  def test_curve_and_feedback(self):
    # vEgo deliberately differs: the curve must use unfiltered wheel speed.
    self.CI.CS.out.vEgo = 50.
    for speed, maximum in ((0., 310), (9., 310), (13., 310), (13.1, 309), (13.4, 306),
                           (14., 300), (15., 290), (16., 280), (16.9, 271), (17., 270), (30., 270)):
      for request in (-1., -0.5, 0.5, 1.):
        with self.subTest(speed=speed, request=request):
          self.CI.CS.out.vEgoRaw = speed
          self.CC.actuators.torque = request
          expected = round(request * maximum)
          self.CI.CC.apply_torque_last = expected
          actuators = self.update()
          assert actuators.torqueOutputCan == expected
          self.assertAlmostEqual(actuators.torque, expected / maximum, places=6)

  def test_driver_torque_limit_uses_dynamic_maximum(self):
    for speed, maximum in ((13., 310), (15., 290), (17., 270)):
      for sign in (-1, 1):
        self.CI.CS.out.vEgoRaw = speed
        self.CI.CS.out.steeringTorque = -sign * 260
        self.CC.actuators.torque = sign
        self.CI.CC.apply_torque_last = sign * maximum
        for _ in range(10):
          actuators = self.update()
        assert actuators.torqueOutputCan == sign * (maximum - 20)

  def test_rates_and_disengagement(self):
    for request in (1., -1.):
      self.CC.actuators.torque = request
      for _ in range(400):
        previous = self.CI.CC.apply_torque_last
        actuators = self.update()
        current = actuators.torqueOutputCan
        if current * previous >= 0 and abs(current) > abs(previous):
          assert abs(current - previous) <= 2
        else:
          assert abs(current - previous) <= 3
      assert current == request * 310

    self.CC.latActive = False
    assert self.update().torqueOutputCan == 0
    assert self.parser.vl["LFA"]["ActToiSta"] == 0

  def test_speed_transition(self):
    self.CI.CC.apply_torque_last = 310
    for step in range(401):
      self.CI.CS.out.vEgoRaw = 13. + step / 100
      previous = self.CI.CC.apply_torque_last
      actuators = self.update()
      assert 0 <= previous - actuators.torqueOutputCan <= 3
      if step % 10 == 0:
        assert actuators.torqueOutputCan == 310 - step // 10
    assert actuators.torqueOutputCan == 270

  def test_high_angle_fault_avoidance(self):
    self.CI.CS.out.steeringAngleDeg = 85.
    for frame in range(92):
      self.update()
      assert self.parser.vl["LFA"]["ActToiSta"] == (frame not in (89, 90))
