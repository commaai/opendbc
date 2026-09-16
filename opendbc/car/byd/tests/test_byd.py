import unittest
from types import SimpleNamespace

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car import Bus
from opendbc.car.byd import bydcan
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import cruise_enabled
from opendbc.car import structs
from opendbc.car.byd.values import DBC, CAR, CarControllerParams as CCP

VisualAlert = structs.CarControl.HUDControl.VisualAlert


class _Hud:
  leftLaneVisible = False
  rightLaneVisible = False
  leftLaneDepart = False
  rightLaneDepart = False
  visualAlert = VisualAlert.none


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
  def test_active_sets_green_icons(self):
    packer = CANPacker(DBC[CAR.BYD_ATTO_3][Bus.pt])
    stock = {
      "LKAS_STATE": 0,
      "LKS_MODE": 0,
      "LKAS_ACTIVE": 1,
      "LEFT_LANE_STATE": 0,
      "RIGHT_LANE_STATE": 0,
    }
    active = _decode_hud(bydcan.create_lkas_hud(packer, 3, stock, _Hud(), True))
    self.assertEqual(active["LKAS_STATE"], 2)
    self.assertEqual(active["LKS_MODE"], 2)

  def test_active_preserves_unnamed_bits(self):
    # Bits 2,3,8,9,32,33,50,51 are not in the original DBC. Without SET_ME_*
    # the packer writes zeros and the EC sees a different 0x316 than the camera.
    raw = bytearray([0xFF] * 7 + [0])
    raw[7] = (~sum(raw[:7])) & 0xFF
    raw = bytes(raw)
    cp = CANParser(DBC[CAR.BYD_ATTO_3][Bus.pt], [("LKAS_HUD_ADAS", 0)], 0)
    cp.update([(0, [(0x316, raw, 0)])])
    stock = dict(cp.vl["LKAS_HUD_ADAS"])
    packer = CANPacker(DBC[CAR.BYD_ATTO_3][Bus.pt])
    _addr, dat, _bus = bydcan.create_lkas_hud(packer, 3, stock, _Hud(), True)
    for bit in (2, 3, 8, 9, 32, 33):
      self.assertEqual((dat[bit // 8] >> (bit % 8)) & 1, 1, f"bit {bit} zeroed")
    active = _decode_hud((_addr, dat, _bus))
    self.assertEqual(active["LKAS_STATE"], 2)
    self.assertEqual(active["HANDS_ON_WHEEL_REQ"], 0)
    self.assertEqual(active["SET_ME_50"], 0)

  def test_steer_required_sets_cluster_nag(self):
    packer = CANPacker(DBC[CAR.BYD_ATTO_3][Bus.pt])
    hud = _Hud()
    hud.visualAlert = VisualAlert.steerRequired
    out = _decode_hud(bydcan.create_lkas_hud(packer, 3, {"LKAS_STATE": 0, "LKS_MODE": 0}, hud, True))
    self.assertEqual(out["HANDS_ON_WHEEL_REQ"], 1)
    self.assertEqual(out["SET_ME_50"], 2)

  def test_heartbeat_hud_is_standby_wheel(self):
    packer = CANPacker(DBC[CAR.BYD_ATTO_3][Bus.pt])
    out = _decode_hud(bydcan.create_lkas_hud(packer, 3, {"LKAS_STATE": 0, "LKS_MODE": 0}, _Hud(), False))
    self.assertEqual(out["LKAS_STATE"], 1)
    self.assertEqual(out["LKS_MODE"], 2)

  def test_ldw_sets_req_without_escalate(self):
    packer = CANPacker(DBC[CAR.BYD_ATTO_3][Bus.pt])
    hud = _Hud()
    hud.visualAlert = VisualAlert.ldw
    out = _decode_hud(bydcan.create_lkas_hud(packer, 3, {"LKAS_STATE": 0, "LKS_MODE": 0}, hud, False))
    self.assertEqual(out["HANDS_ON_WHEEL_REQ"], 1)
    self.assertEqual(out["SET_ME_50"], 0)


def _cs(eps_engaged=True, lks_enabled=True, camera_lkas_state=0, angle=0.0):
  return SimpleNamespace(
    eps_engaged=eps_engaged,
    steer_not_accepted=False,
    lkas_hud={},
    lks_enabled=lks_enabled,
    camera_lkas_state=camera_lkas_state,
    out=SimpleNamespace(vEgoRaw=20.0, steeringAngleDeg=angle),
  )


def _cc(enabled=True, lat_active=True, cancel=False):
  return SimpleNamespace(
    enabled=enabled,
    latActive=lat_active,
    actuators=_Actuators(),
    hudControl=_Hud(),
    cruiseControl=SimpleNamespace(cancel=cancel),
  )


class TestBydSteerNotAccepted(unittest.TestCase):
  def test_debounce_fires_after_200ms(self):
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())

    def step(lat_active: bool, eps_engaged: bool) -> bool:
      CS = _cs(eps_engaged=eps_engaged)
      ctrl.update(_cc(enabled=lat_active, lat_active=lat_active), CS, 0)
      return CS.steer_not_accepted

    # Steer path runs on odd frames (frame % 2). 10 odd ticks is 200 ms.
    flags = [step(True, False) for _ in range(22)]
    self.assertFalse(flags[17])
    self.assertTrue(flags[19])

    self.assertFalse(step(True, True))

  def test_idle_does_not_tx_steer_or_hud(self):
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())

    def step(enabled: bool, lat_active: bool):
      CS = _cs(angle=12.0)
      _act, sends = ctrl.update(_cc(enabled=enabled, lat_active=lat_active), CS, 0)
      return {m[0]: m for m in sends}

    # disengaged: the camera owns 0x1E2 and 0x316
    step(False, False)
    idle = step(False, False)
    self.assertNotIn(0x1E2, idle)
    self.assertNotIn(0x316, idle)

    # engaged without lateral: idle heartbeat at the measured angle, we keep HUD
    step(True, False)
    heartbeat = step(True, False)
    self.assertIn(0x316, heartbeat)
    cp = CANParser(DBC[CAR.BYD_ATTO_3][Bus.pt], [("STEERING_MODULE_ADAS", 0)], 0)
    cp.update([(0, [(0x1E2, heartbeat[0x1E2][1], 0)])])
    self.assertEqual(cp.vl["STEERING_MODULE_ADAS"]["STEER_REQ"], 0)
    self.assertAlmostEqual(cp.vl["STEERING_MODULE_ADAS"]["STEER_ANGLE"], 12.0, places=1)

    step(True, True)
    active = step(True, True)
    self.assertIn(0x1E2, active)
    self.assertIn(0x316, active)
    cp.update([(0, [(0x1E2, active[0x1E2][1], 0)])])
    self.assertEqual(cp.vl["STEERING_MODULE_ADAS"]["STEER_REQ"], 1)


class TestBydCruiseGate(unittest.TestCase):
  def test_acc_without_lks_latch_does_not_enable(self):
    self.assertFalse(cruise_enabled(acc_state=3, lks_enabled=False))
    self.assertTrue(cruise_enabled(acc_state=3, lks_enabled=True))
    self.assertTrue(cruise_enabled(acc_state=5, lks_enabled=True))
    self.assertFalse(cruise_enabled(acc_state=2, lks_enabled=True))


class TestBydLksNeutralize(unittest.TestCase):
  def test_pulses_lks_button_on_bus2_when_camera_on(self):
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
    CS = _cs(lks_enabled=True, camera_lkas_state=2)
    _act, sends = ctrl.update(_cc(enabled=False, lat_active=False), CS, 0)
    bus2 = [m for m in sends if m[0] == 0x3B0]
    self.assertEqual(len(bus2), 1)
    self.assertEqual(bus2[0][2], 2)
    cp = CANParser(DBC[CAR.BYD_ATTO_3][Bus.pt], [("PCM_BUTTONS", 0)], 0)
    cp.update([(0, [(0x3B0, bus2[0][1], 0)])])
    self.assertEqual(cp.vl["PCM_BUTTONS"]["LKAS_ON_BTN"], 1)
    self.assertEqual(cp.vl["PCM_BUTTONS"]["ACC_ON_BTN"], 0)

  def test_no_pulse_when_latch_off_or_camera_off(self):
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
    for lks, cam in ((False, 2), (True, 0)):
      CS = _cs(lks_enabled=lks, camera_lkas_state=cam)
      _act, sends = ctrl.update(_cc(enabled=False, lat_active=False), CS, 0)
      self.assertFalse(any(m[0] == 0x3B0 for m in sends), (lks, cam))

  def test_cancel_does_not_set_lks_button(self):
    packer = CANPacker(DBC[CAR.BYD_ATTO_3][Bus.pt])
    _addr, dat, bus = bydcan.create_buttons(packer, cancel=True)
    self.assertEqual(bus, 0)
    cp = CANParser(DBC[CAR.BYD_ATTO_3][Bus.pt], [("PCM_BUTTONS", 0)], 0)
    cp.update([(0, [(0x3B0, dat, 0)])])
    self.assertEqual(cp.vl["PCM_BUTTONS"]["ACC_ON_BTN"], 1)
    self.assertEqual(cp.vl["PCM_BUTTONS"]["LKAS_ON_BTN"], 0)


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

  def test_radar_dbc_is_mapped_but_unavailable(self):
    self.assertEqual(DBC[CAR.BYD_ATTO_3][Bus.radar], "byd_radar_fd")
    self.assertEqual(CCP.EPB_DEBOUNCE_FRAMES, 8)


if __name__ == "__main__":
  unittest.main()
