import unittest
from types import SimpleNamespace

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car import Bus
from opendbc.car.byd import bydcan
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.values import DBC, CAR


class _Hud:
  leftLaneVisible = False
  rightLaneVisible = False
  leftLaneDepart = False
  rightLaneDepart = False


class _Actuators:
  steeringAngleDeg = 0.0

  def as_builder(self):
    return self


def _decode_hud(packer_msg):
  addr, dat, _bus = packer_msg
  cp = CANParser(DBC[CAR.BYD_ATTO_3][Bus.pt], [("LKAS_HUD_ADAS", 0)], 0)
  cp.update([(0, [(addr, dat, 0)])])
  return dict(cp.vl["LKAS_HUD_ADAS"])


class TestBydLkasHud(unittest.TestCase):
  def test_idle_clears_control_claim(self):
    packer = CANPacker(DBC[CAR.BYD_ATTO_3][Bus.pt])
    stock = {
      "LKAS_STATE": 0,
      "LKS_MODE": 2,
      "LKAS_ACTIVE": 1,
      "LKAS_REQ_PREPARE": 1,
      "TJA_ICA_STATE": 2,
      "LKAS_OUTPUT": 1,
      "LEFT_LANE_STATE": 1,
      "RIGHT_LANE_STATE": 1,
    }
    idle = _decode_hud(bydcan.create_lkas_hud(packer, False, 3, stock, _Hud()))
    active = _decode_hud(bydcan.create_lkas_hud(packer, True, 3, stock, _Hud()))

    self.assertEqual(idle["LKAS_ACTIVE"], 0)
    self.assertEqual(idle["LKAS_REQ_PREPARE"], 0)
    self.assertEqual(idle["TJA_ICA_STATE"], 0)
    self.assertEqual(idle["LKAS_OUTPUT"], 0)
    self.assertEqual(idle["LKAS_STATE"], stock["LKAS_STATE"])
    self.assertEqual(idle["LKS_MODE"], stock["LKS_MODE"])
    self.assertEqual(active["LKAS_STATE"], 2)
    self.assertEqual(active["LKS_MODE"], 2)


class TestBydSteerNotAccepted(unittest.TestCase):
  def test_debounce_fires_after_one_second(self):
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())

    def step(lat_active: bool, eps_engaged: bool) -> bool:
      CS = SimpleNamespace(
        eps_engaged=eps_engaged,
        steer_not_accepted=False,
        lkas_hud={},
        out=SimpleNamespace(vEgoRaw=20.0, steeringAngleDeg=0.0),
      )
      CC = SimpleNamespace(
        latActive=lat_active,
        actuators=_Actuators(),
        hudControl=_Hud(),
        cruiseControl=SimpleNamespace(cancel=False),
      )
      ctrl.update(CC, CS, 0)
      return CS.steer_not_accepted

    # Steer path runs on odd frames (frame % 2). Counter > 50 needs 51 odd ticks.
    flags = [step(True, False) for _ in range(110)]
    # After 50 odd updates (frames 1..99): counter == 50, still False
    self.assertFalse(flags[99])
    # After 51st odd update (frame 101): counter == 51, True
    self.assertTrue(flags[101])

    self.assertFalse(step(True, True))


class TestBydDbcObserve(unittest.TestCase):
  def test_drive_state_gear_nibble(self):
    packer = CANPacker(DBC[CAR.BYD_ATTO_3][Bus.pt])
    _, dat, _ = packer.make_can_msg("DRIVE_STATE", 0, {"GEAR": 4})
    self.assertEqual(dat[5] & 0x07, 4)

  def test_power_on_and_epb_bits(self):
    packer = CANPacker(DBC[CAR.BYD_ATTO_3][Bus.pt])
    _, power, _ = packer.make_can_msg("POWER_VCC", 0, {"POWER_ON": 1})
    self.assertEqual(power[4] & 0x02, 0x02)
    _, epb, _ = packer.make_can_msg("EPB_STATUS", 0, {"EPB_APPLIED": 1})
    self.assertEqual(epb[0] & 0x08, 0x08)


if __name__ == "__main__":
  unittest.main()
