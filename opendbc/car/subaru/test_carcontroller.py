import unittest
from types import SimpleNamespace

from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.subaru.carcontroller import CarController
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR


class TestSubaruCarController(unittest.TestCase):
  def test_lkas_angle_rising_edge_uses_live_steering_angle(self):
    CP = CarInterface.get_non_essential_params(CAR.SUBARU_CROSSTREK_2025)
    controller = CarController({}, CP)

    controller.apply_angle_last = 2.46

    cs = SimpleNamespace(out=SimpleNamespace(
      vEgoRaw=8.791,
      steeringAngleDeg=2.61,
    ))
    cc = SimpleNamespace(
      latActive=True,
      actuators=SimpleNamespace(steeringAngleDeg=1.11),
    )

    controller.handle_angle_lateral(cc, cs)

    expected = apply_std_steer_angle_limits(
      cc.actuators.steeringAngleDeg,
      cs.out.steeringAngleDeg,
      cs.out.vEgoRaw,
      cs.out.steeringAngleDeg,
      cc.latActive,
      controller.p.ANGLE_LIMITS,
    )
    stale_reference = apply_std_steer_angle_limits(
      cc.actuators.steeringAngleDeg,
      2.46,
      cs.out.vEgoRaw,
      cs.out.steeringAngleDeg,
      cc.latActive,
      controller.p.ANGLE_LIMITS,
    )

    self.assertAlmostEqual(controller.apply_angle_last, expected)
    self.assertNotAlmostEqual(controller.apply_angle_last, stale_reference)


if __name__ == "__main__":
  unittest.main()
