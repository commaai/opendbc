import unittest
from types import SimpleNamespace

import numpy as np

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car import Bus
from opendbc.car.byd import bydcan
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import cruise_enabled
from opendbc.car.byd.interface import CarInterface
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car import structs
from opendbc.car.byd.values import DBC, CAR, BydSafetyFlags, CarControllerParams as CCP

VisualAlert = structs.CarControl.HUDControl.VisualAlert


class _Hud:
  leftLaneVisible = False
  rightLaneVisible = False
  leftLaneDepart = False
  rightLaneDepart = False
  visualAlert = VisualAlert.none
  setSpeed = 0.0
  leadDistanceBars = 0


class _Actuators:
  steeringAngleDeg = 0.0
  accel = 0.0

  def as_builder(self):
    return self


def _ctrl(long_control=False):
  CP = SimpleNamespace(openpilotLongitudinalControl=long_control)
  return CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, CP)


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
    for bit in (0, 1, 2, 3, 8, 9, 15, 32, 33):
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

  def test_lks_off_paints_icon_off(self):
    packer = CANPacker(DBC[CAR.BYD_ATTO_3][Bus.pt])
    hud = _Hud()
    hud.visualAlert = VisualAlert.steerRequired
    stock = {"LKAS_STATE": 2, "LKS_MODE": 2, "LEFT_LANE_STATE": 1, "RIGHT_LANE_STATE": 1}
    out = _decode_hud(bydcan.create_lkas_hud(packer, 3, stock, hud, True, lks_on=False))
    self.assertEqual(out["LKAS_STATE"], 0)
    self.assertEqual(out["LKS_MODE"], 2)
    self.assertEqual(out["LEFT_LANE_STATE"], 1)
    self.assertEqual(out["RIGHT_LANE_STATE"], 1)
    self.assertEqual(out["HANDS_ON_WHEEL_REQ"], 0)
    self.assertEqual(out["SET_ME_50"], 0)


def _decode_bsd(dat: bytes) -> dict:
  cp = CANParser(DBC[CAR.BYD_ATTO_3][Bus.pt], [("BSD_RADAR", 0)], 0)
  cp.update([(0, [(0x418, dat, 0)])])
  return dict(cp.vl["BSD_RADAR"])


def _decode_pcw(dat: bytes) -> dict:
  cp = CANParser(DBC[CAR.BYD_ATTO_3][Bus.pt], [("PCW_ADAS", 0)], 0)
  cp.update([(0, [(0x32F, dat, 0)])])
  return dict(cp.vl["PCW_ADAS"])


class TestBydRcw(unittest.TestCase):
  def test_rcw_side_agnostic_not_bsm(self):
    idle = _decode_bsd(bytes.fromhex("fd8156c0f5c81f8c"))
    self.assertEqual(idle["RCW"], 0)
    self.assertEqual(idle["RCW_2"], 0)
    self.assertEqual(idle["RCTA_RIGHT"], 0)
    self.assertEqual(idle["RCTA_RIGHT_2"], 0)
    # left HUD+chime
    left = _decode_bsd(bytes.fromhex("fd8166d0f5c81f8c"))
    self.assertEqual(left["RCW"], 1)
    self.assertEqual(left["RCW_2"], 1)
    self.assertEqual(left["RCTA_RIGHT"], 0)
    self.assertNotEqual(left["LEFT_APPROACH"], 0)
    self.assertEqual(left["RIGHT_APPROACH"], 0)
    # right HUD+chime (XOR vs left is only byte 1)
    right_rcw = _decode_bsd(bytes.fromhex("fd8466d0f5c81f8c"))
    self.assertEqual(right_rcw["RCW"], 1)
    self.assertEqual(right_rcw["RCW_2"], 1)
    self.assertEqual(right_rcw["RCTA_RIGHT"], 0)
    self.assertEqual(right_rcw["LEFT_APPROACH"], 0)
    self.assertNotEqual(right_rcw["RIGHT_APPROACH"], 0)
    bsm = _decode_bsd(bytes.fromhex("fd8456c0f5c81f8c"))
    self.assertEqual(bsm["RCW"], 0)
    self.assertEqual(bsm["RCW_2"], 0)
    self.assertNotEqual(bsm["RIGHT_APPROACH"], 0)
    # reverse-only RCTA_RIGHT
    right = _decode_bsd(bytes.fromhex("fd9059c0f5c81f8c"))
    self.assertEqual(right["RCTA_RIGHT"], 1)
    self.assertEqual(right["RCTA_RIGHT_2"], 0)
    self.assertEqual(right["RCW"], 0)
    sib = _decode_bsd(bytes.fromhex("fda059c0f5c81f8c"))
    self.assertEqual(sib["RCTA_RIGHT"], 1)
    park = _decode_bsd(bytes.fromhex("fd8095c4f5c81f8c"))
    self.assertEqual(park["RCTA_RIGHT"], 0)
    self.assertEqual(park["RCTA_RIGHT_2"], 1)


class TestBydPcw(unittest.TestCase):
  def test_pcw_prewarn_byte2(self):
    idle = _decode_pcw(bytes.fromhex("0580020ce57f0000"))
    self.assertEqual(idle["PCW_STATE"], 0x02)
    warn = _decode_pcw(bytes.fromhex("0580360ce57f0000"))
    self.assertEqual(warn["PCW_STATE"], 0x36)


def _cs(eps_engaged=True, lks_enabled=True, camera_lkas_state=0, angle=0.0, lks_btn_rising=False, eps_standby=False,
        buttons=None, buttons_ts=0, steering_pressed=False, stock_aeb=False, acc_cmd=None, standstill=False, v_ego=20.0):
  return SimpleNamespace(
    eps_engaged=eps_engaged,
    eps_standby=eps_standby,
    eps_idle=(not eps_engaged) and (not eps_standby),
    steer_not_accepted=False,
    lkas_hud={},
    lks_enabled=lks_enabled,
    lks_btn_rising=lks_btn_rising,
    camera_lkas_state=camera_lkas_state,
    pcm_buttons_stock=buttons if buttons is not None else {},
    pcm_buttons_ts=buttons_ts,
    pcm_buttons_new=[],
    camera_acc_state=0,
    acc_hud_stock={},
    acc_cmd_stock=acc_cmd if acc_cmd is not None else {},
    out=SimpleNamespace(vEgoRaw=v_ego, steeringAngleDeg=angle, steeringPressed=steering_pressed, stockAeb=stock_aeb,
                        standstill=standstill),
  )


def _decode(name, packer_msg):
  addr, dat, _bus = packer_msg
  cp = CANParser(DBC[CAR.BYD_ATTO_3][Bus.pt], [(name, 0)], 0)
  cp.update([(0, [(addr, dat, 0)])])
  return dict(cp.vl[name])


def _cc(enabled=True, lat_active=True, cancel=False, long_active=False):
  return SimpleNamespace(
    enabled=enabled,
    latActive=lat_active,
    longActive=long_active,
    actuators=_Actuators(),
    hudControl=_Hud(),
    cruiseControl=SimpleNamespace(cancel=cancel),
  )


def _steer_frames(ctrl, n, enabled=True, lat=True, eps_engaged=True, eps_standby=False, angle=0.0, desired=0.0,
                  pressed=False, v_ego=20.0):
  # Returns the decoded 0x1E2 sent on each 50 Hz slot (None if none) and the last CS.
  frames = []
  CS = None
  for _ in range(n):
    CS = _cs(eps_engaged=eps_engaged, eps_standby=eps_standby, angle=angle, steering_pressed=pressed, v_ego=v_ego)
    CC = _cc(enabled=enabled, lat_active=lat)
    CC.actuators.steeringAngleDeg = desired(ctrl.frame) if callable(desired) else desired
    _act, sends = ctrl.update(CC, CS, 0)
    if ctrl.frame % 2 == 0:  # frame already advanced: odd frames carry the steer slot
      steer = [m for m in sends if m[0] == 0x1E2]
      frames.append(_decode("STEERING_MODULE_ADAS", steer[0]) if steer else None)
  return frames, CS


class TestBydSteerNotAccepted(unittest.TestCase):
  def test_debounce_fires_after_200ms(self):
    ctrl = _ctrl()

    def step(lat_active: bool, eps_engaged: bool) -> bool:
      CS = _cs(eps_engaged=eps_engaged)
      ctrl.update(_cc(enabled=lat_active, lat_active=lat_active), CS, 0)
      return CS.steer_not_accepted

    # Steer path runs on odd frames (frame % 2). Two warm-up slots first, then 10 REQ=1 slots is 200 ms.
    warmup = CCP.STEER_WARMUP_FRAMES * 2
    flags = [step(True, False) for _ in range(22 + warmup)]
    self.assertFalse(flags[17 + warmup])
    self.assertTrue(flags[19 + warmup])

    self.assertFalse(step(True, True))

  def test_idle_does_not_tx_steer_or_hud(self):
    ctrl = _ctrl()

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
    steer = _decode("STEERING_MODULE_ADAS", heartbeat[0x1E2])
    self.assertEqual(steer["STEER_REQ"], 0)
    self.assertEqual(steer["STEER_REQ_ACTIVE_LOW"], 1)
    self.assertAlmostEqual(steer["STEER_ANGLE"], 12.0, places=1)

    # lateral comes on: the heartbeat above was warm-up slot one, the rest follow, then REQ=1
    reqs = []
    for _ in range(2 * (CCP.STEER_WARMUP_FRAMES + 1)):
      active = step(True, True)
      if 0x1E2 in active:
        self.assertIn(0x316, active)
        reqs.append(_decode("STEERING_MODULE_ADAS", active[0x1E2])["STEER_REQ"])
    self.assertEqual(reqs, [0] * (CCP.STEER_WARMUP_FRAMES - 1) + [1, 1])

  def test_lks_off_hold_sends_icon_off(self):
    ctrl = _ctrl()
    cam = {"LKAS_STATE": 2, "LKS_MODE": 2, "LEFT_LANE_STATE": 1, "RIGHT_LANE_STATE": 1}
    CS = _cs(lks_enabled=False, lks_btn_rising=True, camera_lkas_state=2)
    CS.lkas_hud = cam
    ctrl.update(_cc(enabled=False, lat_active=False), CS, 0)
    hud = None
    for _ in range(4):
      hold = _cs(lks_enabled=False, camera_lkas_state=2)
      hold.lkas_hud = cam
      _act, sends = ctrl.update(_cc(enabled=False, lat_active=False), hold, 0)
      for m in sends:
        if m[0] == 0x316:
          hud = _decode("LKAS_HUD_ADAS", m)
    self.assertIsNotNone(hud)
    self.assertEqual(hud["LKAS_STATE"], 0)
    self.assertEqual(hud["LKS_MODE"], 2)
    self.assertEqual(hud["LEFT_LANE_STATE"], 1)

  def test_warmup_then_first_req_repeats_angle(self):
    # After a TX gap: REQ=0 at the measured angle, then the first REQ=1 at that same angle,
    # then the ramp toward the desired angle.
    ctrl = _ctrl()
    frames, _ = _steer_frames(ctrl, 2 * (CCP.STEER_WARMUP_FRAMES + 3), angle=5.0, desired=20.0)
    self.assertEqual([f["STEER_REQ"] for f in frames], [0] * CCP.STEER_WARMUP_FRAMES + [1, 1, 1])
    for f in frames[:CCP.STEER_WARMUP_FRAMES + 1]:
      self.assertAlmostEqual(f["STEER_ANGLE"], 5.0, places=1)
    self.assertGreater(frames[CCP.STEER_WARMUP_FRAMES + 1]["STEER_ANGLE"], 5.0)
    self.assertGreater(frames[CCP.STEER_WARMUP_FRAMES + 2]["STEER_ANGLE"], frames[CCP.STEER_WARMUP_FRAMES + 1]["STEER_ANGLE"])

    # A lateral pause resyncs the same way: REQ=0 at measured, then REQ=1 without a step
    frames, _ = _steer_frames(ctrl, 2, lat=False, angle=7.0, desired=20.0)
    self.assertEqual(frames[0]["STEER_REQ"], 0)
    self.assertAlmostEqual(frames[0]["STEER_ANGLE"], 7.0, places=1)
    frames, _ = _steer_frames(ctrl, 4, angle=7.0, desired=20.0)
    self.assertEqual(frames[0]["STEER_REQ"], 1)
    self.assertAlmostEqual(frames[0]["STEER_ANGLE"], 7.0, places=1)
    self.assertGreater(frames[1]["STEER_ANGLE"], 7.0)

  def test_standby_ack_pair_then_resume(self):
    ctrl = _ctrl()
    _steer_frames(ctrl, 2 * (CCP.STEER_WARMUP_FRAMES + 2), angle=3.0, desired=3.0)

    # EPS drops to standby while we steer: seen on two slots, then idle frame + ack, then REQ=1 again
    frames, CS = _steer_frames(ctrl, 2 * 4, eps_engaged=False, eps_standby=True, angle=3.0, desired=3.0)
    reqs = [(f["STEER_REQ"], f["STEER_REQ_ACTIVE_LOW"]) for f in frames]
    self.assertEqual(reqs, [(1, 0), (0, 1), (0, 0), (1, 0)])
    for f in frames[1:3]:
      self.assertAlmostEqual(f["STEER_ANGLE"], 3.0, places=1)
      self.assertEqual(f["ANGLE_RATE_LIMIT_UPPER"], 0)
      self.assertEqual(f["ANGLE_RATE_LIMIT_LOWER"], 0)
    self.assertFalse(CS.steer_not_accepted)

    # EPS acks: REQ=1 resumes at the same angle, no second pair inside the period
    frames, CS = _steer_frames(ctrl, 2 * CCP.STEER_ACK_PERIOD, angle=3.0, desired=3.0)
    self.assertEqual({f["STEER_REQ"] for f in frames}, {1})
    self.assertAlmostEqual(frames[0]["STEER_ANGLE"], 3.0, places=1)
    self.assertFalse(CS.steer_not_accepted)

  def test_standby_acks_are_rate_limited_then_fault_latches(self):
    ctrl = _ctrl()
    _steer_frames(ctrl, 2 * (CCP.STEER_WARMUP_FRAMES + 2), angle=0.0)

    acks = 0
    latched_at = None
    for i in range(2 * CCP.STEER_ACK_PERIOD * (CCP.STEER_ACK_ATTEMPTS + 2)):
      frames, CS = _steer_frames(ctrl, 1, eps_engaged=False, eps_standby=True)
      if frames and frames[0] is not None and frames[0]["STEER_REQ"] == 0 and frames[0]["STEER_REQ_ACTIVE_LOW"] == 0:
        acks += 1
      if CS.steer_not_accepted and ctrl.steer_fault_latched and latched_at is None:
        latched_at = i
    self.assertEqual(acks, CCP.STEER_ACK_ATTEMPTS)
    self.assertIsNotNone(latched_at)

    # Latched: no more acks, fault stays while enabled, clears on disengage
    frames, CS = _steer_frames(ctrl, 2 * CCP.STEER_ACK_PERIOD, eps_engaged=False, eps_standby=True)
    self.assertFalse(any(f["STEER_REQ"] == 0 and f["STEER_REQ_ACTIVE_LOW"] == 0 for f in frames))
    self.assertTrue(CS.steer_not_accepted)
    _frames, CS = _steer_frames(ctrl, 2, enabled=False, lat=False, eps_engaged=False, eps_standby=True)
    self.assertFalse(CS.steer_not_accepted)
    self.assertEqual(ctrl.ack_attempts, 0)

  def test_not_accepted_counts_only_sent_req_frames(self):
    ctrl = _ctrl()
    # warm-up slots with the EPS idle do not count
    _steer_frames(ctrl, 2 * CCP.STEER_WARMUP_FRAMES, eps_engaged=False)
    self.assertEqual(ctrl.not_accepted_frames, 0)
    # standby ack slots do not count either
    _steer_frames(ctrl, 2 * 4, eps_engaged=False, eps_standby=True)
    self.assertEqual(ctrl.not_accepted_frames, 2)

  def test_pressed_yields_req_and_skips_standby_fault(self):
    ctrl = _ctrl()
    _steer_frames(ctrl, 2 * (CCP.STEER_WARMUP_FRAMES + 2), angle=10.0, desired=10.0)

    frames, CS = _steer_frames(ctrl, 2 * 6, angle=40.0, desired=10.0, pressed=True)
    self.assertTrue(all(f["STEER_REQ"] == 0 for f in frames))
    self.assertTrue(all(f["STEER_REQ_ACTIVE_LOW"] == 1 for f in frames))
    self.assertFalse(any(f["STEER_REQ"] == 0 and f["STEER_REQ_ACTIVE_LOW"] == 0 for f in frames))
    for f in frames:
      self.assertAlmostEqual(f["STEER_ANGLE"], 40.0, places=1)
    self.assertEqual(ctrl.not_accepted_frames, 0)
    self.assertFalse(CS.steer_not_accepted)

    frames, CS = _steer_frames(ctrl, 2 * CCP.STEER_ACK_PERIOD * (CCP.STEER_ACK_ATTEMPTS + 2),
                               eps_engaged=False, eps_standby=True, angle=40.0, desired=10.0, pressed=True)
    self.assertTrue(all(f["STEER_REQ"] == 0 for f in frames))
    self.assertFalse(any(f["STEER_REQ"] == 0 and f["STEER_REQ_ACTIVE_LOW"] == 0 for f in frames))
    self.assertEqual(ctrl.ack_attempts, 0)
    self.assertFalse(ctrl.steer_fault_latched)
    self.assertFalse(CS.steer_not_accepted)

    # hands off: REQ stays 0 through the yield hold, then comes back at the held angle
    frames, CS = _steer_frames(ctrl, 2 * (CCP.STEER_YIELD_HOLD_SLOTS - 1), angle=40.0, desired=40.0)
    self.assertTrue(all(f["STEER_REQ"] == 0 for f in frames))
    frames, CS = _steer_frames(ctrl, 2 * (CCP.STEER_WARMUP_FRAMES + 2), angle=40.0, desired=40.0)
    self.assertEqual(frames[0]["STEER_REQ"], 1)
    self.assertAlmostEqual(frames[0]["STEER_ANGLE"], 40.0, places=1)
    self.assertFalse(CS.steer_not_accepted)

  def test_yield_hold_after_release(self):
    ctrl = _ctrl()
    warm, _ = _steer_frames(ctrl, 2 * (CCP.STEER_WARMUP_FRAMES + 2))
    self.assertEqual(warm[-1]["STEER_REQ"], 1)
    pressed, _ = _steer_frames(ctrl, 4, pressed=True)
    self.assertEqual([f["STEER_REQ"] for f in pressed], [0, 0])
    self.assertEqual([f["STEER_REQ_ACTIVE_LOW"] for f in pressed], [1, 1])
    held, _ = _steer_frames(ctrl, 2 * (CCP.STEER_YIELD_HOLD_SLOTS - 1))
    self.assertEqual(len(held), CCP.STEER_YIELD_HOLD_SLOTS - 1)
    self.assertEqual([f["STEER_REQ"] for f in held], [0] * len(held))
    back, _ = _steer_frames(ctrl, 4)
    self.assertEqual([f["STEER_REQ"] for f in back], [1, 1])
    # chatter inside the hold does not re-engage
    _steer_frames(ctrl, 2, pressed=True)
    for _ in range(3):
      a, _ = _steer_frames(ctrl, 10)
      b, _ = _steer_frames(ctrl, 2, pressed=True)
      self.assertEqual([f["STEER_REQ"] for f in a + b], [0] * 6)


class TestBydCruiseGate(unittest.TestCase):
  def test_acc_without_lks_latch_does_not_enable(self):
    self.assertFalse(cruise_enabled(acc_state=3, lks_enabled=False))
    self.assertTrue(cruise_enabled(acc_state=3, lks_enabled=True))
    self.assertTrue(cruise_enabled(acc_state=5, lks_enabled=True))
    self.assertFalse(cruise_enabled(acc_state=2, lks_enabled=True))


def _bus2_lks(sends):
  return [m for m in sends if m[0] == 0x3B0 and m[2] == 2]


def _step(ctrl, n, enabled=True, lat=False, cam=0, lks=True, rising=False,
          eps_engaged=True, eps_standby=False):
  pulses = 0
  last = []
  for i in range(n):
    CS = _cs(lks_enabled=lks, camera_lkas_state=cam, lks_btn_rising=rising and i == 0,
             eps_engaged=eps_engaged, eps_standby=eps_standby)
    _act, last = ctrl.update(_cc(enabled=enabled, lat_active=lat), CS, 0)
    pulses += len(_bus2_lks(last))
  return pulses, last


class TestBydLksCamera(unittest.TestCase):
  def test_neutralize_on_enabled_when_camera_on(self):
    ctrl = _ctrl()
    seen = None
    for _ in range(CCP.LKS_CAM_DEBOUNCE_FRAMES + CCP.LKS_PULSE_PERIOD + 2):
      CS = _cs(lks_enabled=True, camera_lkas_state=2)
      _act, sends = ctrl.update(_cc(enabled=True, lat_active=False), CS, 0)
      bus2 = _bus2_lks(sends)
      if bus2:
        seen = bus2[0]
    self.assertIsNotNone(seen)
    cp = CANParser(DBC[CAR.BYD_ATTO_3][Bus.pt], [("PCM_BUTTONS", 0)], 0)
    cp.update([(0, [(0x3B0, seen[1], 0)])])
    self.assertEqual(cp.vl["PCM_BUTTONS"]["LKAS_ON_BTN"], 1)
    self.assertEqual(cp.vl["PCM_BUTTONS"]["ACC_ON_BTN"], 0)

  def test_no_pulse_when_camera_already_off(self):
    ctrl = _ctrl()
    pulses, _ = _step(ctrl, CCP.LKS_CAM_DEBOUNCE_FRAMES + 10, enabled=True, lat=True, cam=0)
    self.assertEqual(pulses, 0)

  def test_no_immediate_restore_or_idle_glitch(self):
    ctrl = _ctrl()
    pulses, _ = _step(ctrl, CCP.LKS_HUD_QUIET_FRAMES - 1, enabled=False, cam=2, lks=True)
    self.assertEqual(pulses, 0)

  def test_restore_after_hud_quiet_when_latch_on(self):
    ctrl = _ctrl()
    _step(ctrl, CCP.LKS_CAM_DEBOUNCE_FRAMES + 2, enabled=False, cam=0, lks=True, eps_engaged=False)
    pulses, _ = _step(ctrl, CCP.LKS_HUD_QUIET_FRAMES + CCP.LKS_PULSE_PERIOD + 2,
                      enabled=False, cam=0, lks=True, eps_engaged=False)
    self.assertGreater(pulses, 0)

  def test_no_restore_while_eps_engaged(self):
    ctrl = _ctrl()
    pulses, _ = _step(ctrl, CCP.LKS_HUD_QUIET_FRAMES + CCP.LKS_PULSE_PERIOD + 2,
                      enabled=False, cam=0, lks=True, eps_engaged=True)
    self.assertEqual(pulses, 0)

  def test_restore_after_eps_idle_stable(self):
    ctrl = _ctrl()
    _step(ctrl, CCP.LKS_CAM_DEBOUNCE_FRAMES + CCP.LKS_HUD_QUIET_FRAMES,
          enabled=False, cam=0, lks=True, eps_engaged=True)
    short, _ = _step(ctrl, CCP.LKS_EPS_IDLE_FRAMES - 1, enabled=False, cam=0, lks=True, eps_engaged=False)
    self.assertEqual(short, 0)
    more, _ = _step(ctrl, CCP.LKS_PULSE_PERIOD + 2, enabled=False, cam=0, lks=True, eps_engaged=False)
    self.assertGreater(more, 0)

  def test_no_tick_while_lkas_state_4(self):
    ctrl = _ctrl()
    pulses, _ = _step(ctrl, CCP.LKS_CAM_DEBOUNCE_FRAMES + CCP.LKS_PULSE_PERIOD * 4, enabled=True, cam=4)
    self.assertEqual(pulses, 0)
    more, _ = _step(ctrl, CCP.LKS_PULSE_PERIOD + 2, enabled=True, cam=2)
    self.assertEqual(more, 1)

  def test_no_restore_when_latch_off(self):
    ctrl = _ctrl()
    pulses, _ = _step(ctrl, CCP.LKS_HUD_QUIET_FRAMES + 20, enabled=False, cam=0, lks=False)
    self.assertEqual(pulses, 0)

  def test_latch_off_turns_camera_off_at_idle(self):
    # Persist off + stock camera still on: snap camera off without waiting for engage.
    ctrl = _ctrl()
    pulses, _ = _step(ctrl, CCP.LKS_CAM_DEBOUNCE_FRAMES + CCP.LKS_PULSE_PERIOD + 2, enabled=False, cam=2, lks=False)
    self.assertGreater(pulses, 0)

  def test_failed_camera_off_retries_after_recover(self):
    ctrl = _ctrl()
    pulses = 0
    quiet = 0
    # First snap+retry, then a full quiet recover gap, then another snap.
    for _ in range(800):
      n, _ = _step(ctrl, 1, enabled=False, cam=2, lks=False)
      if n:
        pulses += n
        quiet = 0
      else:
        quiet += 1
      if pulses >= CCP.LKS_PULSE_TICKS * 2 and quiet == CCP.LKS_CONFIRM_FRAMES + CCP.LKS_RECOVER_FRAMES - 1:
        break
    else:
      self.fail("did not reach recover quiet")
    self.assertLessEqual(pulses, CCP.LKS_PULSE_TICKS * 2)
    more, _ = _step(ctrl, CCP.LKS_PULSE_PERIOD + 2, enabled=False, cam=2, lks=False)
    self.assertGreater(more, 0)

  def test_rapid_lks_presses_snap_once_after_lockout(self):
    ctrl = _ctrl()
    lks = True
    pulses = 0
    for i in range(30):
      rising = i % 3 == 0
      if rising:
        lks = not lks
      CS = _cs(lks_enabled=lks, camera_lkas_state=2 if lks else 0, lks_btn_rising=rising)
      _act, sends = ctrl.update(_cc(enabled=True, lat_active=True), CS, 0)
      pulses += len(_bus2_lks(sends))
    self.assertEqual(pulses, 0)
    # Last press left latch off, camera on. After lockout, one snap off.
    settle = CCP.LKS_LOCKOUT_FRAMES + CCP.LKS_CAM_DEBOUNCE_FRAMES + CCP.LKS_PULSE_TICKS * CCP.LKS_PULSE_PERIOD + 5
    more, _ = _step(ctrl, settle, enabled=True, cam=2, lks=False)
    self.assertGreater(more, 0)
    self.assertLessEqual(more, CCP.LKS_PULSE_TICKS * 2)

  def test_rapid_acc_does_not_restore_or_storm(self):
    ctrl = _ctrl()
    pulses = 0
    for i in range(80):
      enabled = (i % 10) < 5
      CS = _cs(lks_enabled=True, camera_lkas_state=2)
      _act, sends = ctrl.update(_cc(enabled=enabled, lat_active=enabled), CS, 0)
      pulses += len(_bus2_lks(sends))
    self.assertLessEqual(pulses, CCP.LKS_PULSE_TICKS * 2)

  def test_acc_back_on_aborts_restore(self):
    ctrl = _ctrl()
    _step(ctrl, CCP.LKS_HUD_QUIET_FRAMES - 1, enabled=False, cam=0, lks=True)
    _step(ctrl, CCP.LKS_PULSE_PERIOD, enabled=False, cam=0, lks=True)
    pulses_on, _ = _step(ctrl, CCP.LKS_PULSE_TICKS * CCP.LKS_PULSE_PERIOD + 10, enabled=True, cam=0, lks=True)
    self.assertEqual(pulses_on, 0)

  def test_button_lockout_then_snap_camera_off(self):
    ctrl = _ctrl()
    _step(ctrl, CCP.LKS_CAM_DEBOUNCE_FRAMES + 2, enabled=True, cam=0, lks=True)
    pulses_lock, last_lock = _step(ctrl, CCP.LKS_LOCKOUT_FRAMES, enabled=False, cam=2, lks=False, rising=True)
    self.assertEqual(pulses_lock, 0)
    self.assertIn(0x1E2, {m[0] for m in last_lock})
    pulses_snap, _ = _step(ctrl, CCP.LKS_CAM_DEBOUNCE_FRAMES + CCP.LKS_PULSE_PERIOD + 2, enabled=False, cam=2, lks=False)
    self.assertGreater(pulses_snap, 0)

  def test_one_retry_then_stop(self):
    ctrl = _ctrl()
    pulses, _ = _step(ctrl, CCP.LKS_CAM_DEBOUNCE_FRAMES + (CCP.LKS_PULSE_PERIOD + 1) * 2 +
                      CCP.LKS_CONFIRM_FRAMES * 2 + 20, enabled=True, cam=2)
    # one tick per pulse, two pulses (first + one retry), then give up
    self.assertEqual(pulses, CCP.LKS_PULSE_TICKS * 2)

  def test_tick_mirrors_stock_frame_right_after_it(self):
    # Our bus-2 frame is the car's latest 0x3B0 (same COUNTER) with only LKAS_ON_BTN added,
    # sent on the frame the car's 0x3B0 arrived.
    ctrl = _ctrl()
    stock = {"SET_ME_1_1": 1, "SET_ME_1_2": 1, "SET_BTN": 1, "RES_BTN": 0, "LKAS_ON_BTN": 0,
             "DEC_DISTANCE_BTN": 0, "INC_DISTANCE_BTN": 0, "ACC_ON_BTN": 1, "COUNTER": 11, "CHECKSUM": 0}
    sent = []
    for i in range(CCP.LKS_CAM_DEBOUNCE_FRAMES + CCP.LKS_PULSE_PERIOD * 3):
      fresh = i % CCP.LKS_PULSE_PERIOD == 3
      if fresh:
        stock = {**stock, "COUNTER": (stock["COUNTER"] + 1) % 16}
      # pcm_buttons_ts only moves on the frames a stock 0x3B0 arrived
      CS = _cs(camera_lkas_state=2, buttons=stock, buttons_ts=(i - 3) // CCP.LKS_PULSE_PERIOD)
      _act, sends = ctrl.update(_cc(enabled=True, lat_active=False), CS, 0)
      for m in _bus2_lks(sends):
        sent.append((i, fresh, stock["COUNTER"], _decode("PCM_BUTTONS", m), m[1]))
    self.assertEqual(len(sent), 1)
    i, fresh, counter, values, dat = sent[0]
    self.assertTrue(fresh)
    self.assertEqual(values["COUNTER"], counter)
    self.assertEqual(values["LKAS_ON_BTN"], 1)
    self.assertEqual(values["ACC_ON_BTN"], 0)
    self.assertEqual(values["SET_BTN"], 0)
    self.assertEqual(values["SET_ME_1_1"], 1)
    self.assertEqual(values["SET_ME_1_2"], 1)
    self.assertEqual(dat[7], (~sum(dat[:7])) & 0xFF)

  def test_tick_falls_back_without_fresh_stock_frame(self):
    ctrl = _ctrl()
    pulses, _ = _step(ctrl, CCP.LKS_CAM_DEBOUNCE_FRAMES + CCP.LKS_PULSE_PERIOD + 2, enabled=True, cam=2)
    self.assertEqual(pulses, 1)

  def test_no_tick_while_eps_standby(self):
    ctrl = _ctrl()
    pulses = 0
    for _ in range(CCP.LKS_CAM_DEBOUNCE_FRAMES + CCP.LKS_PULSE_PERIOD * 4):
      CS = _cs(camera_lkas_state=2, eps_standby=True)
      _act, sends = ctrl.update(_cc(enabled=True, lat_active=False), CS, 0)
      pulses += len(_bus2_lks(sends))
    self.assertEqual(pulses, 0)
    more, _ = _step(ctrl, CCP.LKS_PULSE_PERIOD + 2, enabled=True, cam=2)
    self.assertEqual(more, 1)

  def test_cancel_does_not_set_lks_button(self):
    packer = CANPacker(DBC[CAR.BYD_ATTO_3][Bus.pt])
    _addr, dat, bus = bydcan.create_buttons(packer, {"COUNTER": 7, "LKAS_ON_BTN": 1}, cancel=True)
    self.assertEqual(bus, 0)
    values = _decode("PCM_BUTTONS", (_addr, dat, bus))
    self.assertEqual(values["ACC_ON_BTN"], 1)
    self.assertEqual(values["LKAS_ON_BTN"], 0)
    self.assertEqual(values["COUNTER"], 7)


class TestBydDbcObserve(unittest.TestCase):
  def test_drive_state_gear_nibble(self):
    packer = CANPacker(DBC[CAR.BYD_ATTO_3][Bus.pt])
    _, dat, _ = packer.make_can_msg("DRIVE_STATE", 0, {"GEAR": 4})
    self.assertEqual(dat[5] & 0x07, 4)
    self.assertEqual(dat[7], (~sum(dat[:7])) & 0xFF)

  def test_power_on_and_epb_bits(self):
    packer = CANPacker(DBC[CAR.BYD_ATTO_3][Bus.pt])
    _, power, _ = packer.make_can_msg("POWER_VCC", 0, {"POWER_ON": 1})
    self.assertEqual(power[4] & 0x02, 0x02)
    _, epb, _ = packer.make_can_msg("EPB_STATUS", 0, {"EPB_APPLIED": 1})
    self.assertEqual(epb[0] & 0x08, 0x08)

  def test_charge_status_layout(self):
    # 03 32 09 93 .. = charging, 50 %, 12-bit 777, flags 9
    packer = CANPacker(DBC[CAR.BYD_ATTO_3][Bus.pt])
    values = {"CHARGE_STATE": 3, "CHARGE_SOC": 50, "CHARGE_UNKNOWN_12BIT": 777, "CHARGE_SESSION_FLAGS": 9, "COUNTER": 0}
    _, dat, _ = packer.make_can_msg("CHARGE_STATUS", 0, values)
    self.assertEqual(bytes(dat[:4]), bytes.fromhex("03320993"))
    self.assertEqual(dat[7], (~sum(dat[:7])) & 0xFF)
    _, power, _ = packer.make_can_msg("POWER_VCC", 0, {"CHARGE_PLUGGED": 1})
    self.assertEqual(power[0] & 0x20, 0x20)
    _, sess, _ = packer.make_can_msg("CHARGE_SESSION", 0, {"CHARGE_SESSION_ACTIVE": 3})
    self.assertEqual(sess[6] & 0x03, 0x03)

  def test_radar_dbc_is_mapped_and_available(self):
    self.assertEqual(DBC[CAR.BYD_ATTO_3][Bus.radar], "byd_radar_fd")
    CP = CarInterface.get_params(CAR.BYD_ATTO_3, {0: {}, 1: {}, 2: {}}, [], False, False, False)
    self.assertFalse(CP.radarUnavailable)
    self.assertEqual(CCP.EPB_DEBOUNCE_FRAMES, 8)

  def test_get_params_safety_param_alpha_long(self):
    fp = {0: {}, 1: {}, 2: {}}
    CP = CarInterface.get_params(CAR.BYD_ATTO_3, fp, [], False, False, False)
    self.assertFalse(CP.openpilotLongitudinalControl)
    self.assertTrue(CP.pcmCruise)
    self.assertEqual(CP.safetyConfigs[0].safetyParam, int(BydSafetyFlags.LKS_ON))

    CP_long = CarInterface.get_params(CAR.BYD_ATTO_3, fp, [], True, False, False)
    self.assertTrue(CP_long.openpilotLongitudinalControl)
    self.assertFalse(CP_long.pcmCruise)
    self.assertEqual(
      CP_long.safetyConfigs[0].safetyParam,
      int(BydSafetyFlags.LKS_ON | BydSafetyFlags.LONG_CONTROL),
    )


class TestBydLowSpeedSmoothing(unittest.TestCase):
  LEAD = CCP.STEER_WARMUP_FRAMES + 1  # warmup REQ=0 plus the first REQ=1

  def _angles(self, frames):
    return [f["STEER_ANGLE"] for f in frames if f is not None]

  def test_crawl_step_is_rate_capped_and_settles(self):
    ctrl = _ctrl()
    frames, _ = _steer_frames(ctrl, 2 * (self.LEAD + 150), angle=0.0, desired=30.0, v_ego=1.5)
    angles = self._angles(frames)[self.LEAD:]
    steps = [b - a for a, b in zip(angles, angles[1:], strict=False)]
    self.assertAlmostEqual(max(steps), CCP.LOW_SPEED_RATE_V[0], delta=0.01)
    self.assertLess(steps[10], CCP.LOW_SPEED_RATE_V[0])
    self.assertGreater(angles[-1], 29.5)

  def test_crawl_chatter_is_filtered(self):
    ctrl = _ctrl()
    frames, _ = _steer_frames(ctrl, 2 * (self.LEAD + 100), angle=0.0, desired=lambda f: 5.0 if (f // 2) % 2 else -5.0, v_ego=1.5)
    angles = self._angles(frames)[self.LEAD + 25:]
    steps = [abs(b - a) for a, b in zip(angles, angles[1:], strict=False)]
    self.assertLess(max(steps), 0.5)
    self.assertLess(max(abs(a) for a in angles), 1.0)

  def test_identity_from_30_kph(self):
    v = 9.0
    self.assertEqual(float(np.interp(v, CCP.LOW_SPEED_TAU_BP, CCP.LOW_SPEED_TAU_V)), 0.0)
    self.assertEqual(float(np.interp(v, CCP.LOW_SPEED_RATE_BP, CCP.LOW_SPEED_RATE_V)), CCP.ANGLE_LIMITS.MAX_ANGLE_RATE)
    ctrl = _ctrl()
    req = lambda f: 2.0 if (f // 2) % 2 else -2.0  # noqa: E731
    frames, _ = _steer_frames(ctrl, 2 * (self.LEAD + 40), angle=0.0, desired=req, v_ego=v)
    angles = self._angles(frames)
    last = angles[self.LEAD - 1]
    for i, a in enumerate(angles[self.LEAD:], start=self.LEAD):
      last = apply_steer_angle_limits_vm(req(2 * i + 1), last, v, 0.0, True, CCP, ctrl.VM)
      self.assertAlmostEqual(a, last, delta=0.11)

  def test_filter_resets_to_measured_when_not_steering(self):
    ctrl = _ctrl()
    _steer_frames(ctrl, 2 * (self.LEAD + 50), angle=0.0, desired=30.0, v_ego=1.5)
    _steer_frames(ctrl, 4, lat=False, angle=12.0, desired=30.0, v_ego=1.5)
    self.assertAlmostEqual(ctrl.angle_filt, 12.0)
    frames, _ = _steer_frames(ctrl, 6, angle=12.0, desired=30.0, v_ego=1.5)
    angles = self._angles(frames)
    self.assertAlmostEqual(angles[0], 12.0, places=1)
    for a, b in zip(angles, angles[1:], strict=False):
      self.assertLessEqual(b - a, CCP.LOW_SPEED_RATE_V[0] + 0.01)


def _acc_frame(hex7):
  dat = bytes.fromhex(hex7)
  return dat + bytes([(~sum(dat)) & 0xFF])


class TestBydAccCmd(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker(DBC[CAR.BYD_ATTO_3][Bus.pt])

  def test_idle_matches_stock_template(self):
    addr, dat, bus = bydcan.create_acc_cmd(self.packer, 0.0, False, False, 5)
    self.assertEqual((addr, bus), (0x32E, 0))
    self.assertEqual(bytes(dat[:6]), bytes.fromhex("646464805000"))
    self.assertEqual(dat[6], 0xF5)
    self.assertEqual(dat[7], (~sum(dat[:7])) & 0xFF)

  def test_active_layout_and_scale(self):
    _, dat, _ = bydcan.create_acc_cmd(self.packer, -0.5, True, False, 0)
    self.assertEqual(bytes(dat[:6]), bytes.fromhex("5a6666834c18"))
    self.assertEqual(dat[7], (~sum(dat[:7])) & 0xFF)
    _, dat, _ = bydcan.create_acc_cmd(self.packer, 1.0, True, False, 0)
    self.assertEqual(dat[0], 120)
    self.assertAlmostEqual(_decode("ACC_CMD", (0x32E, dat, 0))["ACCEL_CMD"], 1.0, places=6)

  def test_standstill_hold_bits(self):
    _, dat, _ = bydcan.create_acc_cmd(self.packer, -0.5, True, True, 0)
    self.assertEqual(dat[5], 0x31)
    values = _decode("ACC_CMD", (0x32E, dat, 0))
    self.assertEqual(values["STANDSTILL_STATE"], 1)
    self.assertEqual(values["ACC_REQ_NOT_STANDSTILL"], 0)
    self.assertEqual(values["ACC_OVERRIDE_OR_STANDSTILL"], 1)

  def test_stock_aeb_threshold(self):
    idle = _decode("ACC_CMD", (0x32E, _acc_frame("646464805000f0"), 0))
    self.assertGreaterEqual(idle["ACCEL_CMD"], CCP.STOCK_AEB_ACCEL)
    hard = _decode("ACC_CMD", (0x32E, _acc_frame("286666834c18f0"), 0))
    self.assertAlmostEqual(hard["ACCEL_CMD"], -3.0, places=6)
    self.assertLess(hard["ACCEL_CMD"], CCP.STOCK_AEB_ACCEL)

  def test_passthrough_keeps_stock_payload_with_our_counter(self):
    stock = _decode("ACC_CMD", (0x32E, _acc_frame("286666834c18f3"), 0))
    _, dat, _ = bydcan.create_acc_cmd_passthrough(self.packer, stock, 9)
    self.assertEqual(bytes(dat[:6]), bytes.fromhex("286666834c18"))
    self.assertEqual(dat[6], 0xF9)
    self.assertEqual(dat[7], (~sum(dat[:7])) & 0xFF)

  def test_passthrough_roundtrip_covers_spare_bits(self):
    for raw6 in (bytes.fromhex("286666834c18"), bytes.fromhex("ffffffffffff"), bytes.fromhex("010204081020")):
      frame = raw6 + bytes([0xF3])
      frame += bytes([(~sum(frame)) & 0xFF])
      stock = _decode("ACC_CMD", (0x32E, frame, 0))
      _, dat, _ = bydcan.create_acc_cmd_passthrough(self.packer, stock, 9)
      self.assertEqual(bytes(dat[:6]), raw6)


class TestBydLongControl(unittest.TestCase):
  def _slots(self, ctrl, n, **kw):
    out = []
    for _ in range(n):
      cs_kw = {k: kw[k] for k in ("stock_aeb", "acc_cmd", "standstill") if k in kw}
      CC = _cc(enabled=kw.get("enabled", True), lat_active=False, long_active=kw.get("long_active", False))
      CC.actuators.accel = kw.get("accel", 0.0)
      act, sends = ctrl.update(CC, _cs(**cs_kw), 0)
      if ctrl.frame % 2 == 0:
        acc = [m for m in sends if m[0] == 0x32E]
        out.append((act, _decode("ACC_CMD", acc[0]) if acc else None))
    return out

  def test_no_acc_cmd_without_long_control(self):
    ctrl = _ctrl(long_control=False)
    for _act, msg in self._slots(ctrl, 4, long_active=True, accel=-1.0):
      self.assertIsNone(msg)

  def test_idle_frame_every_slot_when_not_long_active(self):
    ctrl = _ctrl(long_control=True)
    slots = self._slots(ctrl, 6, enabled=False, long_active=False, accel=-1.0)
    self.assertEqual(len(slots), 3)
    for act, msg in slots:
      self.assertEqual(msg["ACC_ON_1"], 0)
      self.assertEqual(msg["ACCEL_CMD"], 0.0)
      self.assertEqual(act.accel, 0.0)
    self.assertEqual([m["COUNTER"] for _a, m in slots], [0, 1, 2])

  def test_active_clips_accel(self):
    ctrl = _ctrl(long_control=True)
    act, msg = self._slots(ctrl, 2, long_active=True, accel=-9.0)[0]
    self.assertEqual(msg["ACC_ON_1"], 1)
    self.assertEqual(msg["ACC_CONTROLLABLE_AND_ON"], 1)
    self.assertAlmostEqual(msg["ACCEL_CMD"], CCP.ACCEL_MIN, places=6)
    self.assertEqual(act.accel, CCP.ACCEL_MIN)
    act, msg = self._slots(ctrl, 2, long_active=True, accel=9.0)[0]
    self.assertAlmostEqual(msg["ACCEL_CMD"], CCP.ACCEL_MAX, places=6)

  def test_standstill_sets_hold(self):
    ctrl = _ctrl(long_control=True)
    _act, msg = self._slots(ctrl, 2, long_active=True, accel=-0.5, standstill=True)[0]
    self.assertEqual(msg["STANDSTILL_STATE"], 1)
    self.assertEqual(msg["ACC_REQ_NOT_STANDSTILL"], 0)

  def test_stock_aeb_is_passed_through(self):
    ctrl = _ctrl(long_control=True)
    stock = _decode("ACC_CMD", (0x32E, _acc_frame("286666834c18f3"), 0))
    _act, msg = self._slots(ctrl, 2, long_active=True, accel=1.0, stock_aeb=True, acc_cmd=stock)[0]
    self.assertAlmostEqual(msg["ACCEL_CMD"], -3.0, places=6)
    self.assertEqual(msg["ACC_ON_1"], 1)
    self.assertEqual(msg["COUNTER"], 0)

  def test_buttons_relay_strips_acc_and_keeps_counter(self):
    ctrl = _ctrl(long_control=True)
    stock = {
      "SET_BTN": 1, "RES_BTN": 1, "DEC_DISTANCE_BTN": 1, "INC_DISTANCE_BTN": 1,
      "ACC_ON_BTN": 1, "LKAS_ON_BTN": 1, "COUNTER": 7, "SET_ME_1_1": 1, "SET_ME_1_2": 1,
    }
    CS = _cs()
    CS.pcm_buttons_new = [stock]
    _act, sends = ctrl.update(_cc(lat_active=False), CS, 0)
    relay = [m for m in sends if m[0] == 0x3B0 and m[2] == 2]
    self.assertEqual(len(relay), 1)
    dec = _decode("PCM_BUTTONS", relay[0])
    self.assertEqual(dec["SET_BTN"], 0)
    self.assertEqual(dec["RES_BTN"], 0)
    self.assertEqual(dec["DEC_DISTANCE_BTN"], 0)
    self.assertEqual(dec["INC_DISTANCE_BTN"], 0)
    self.assertEqual(dec["ACC_ON_BTN"], 1)
    self.assertEqual(dec["LKAS_ON_BTN"], 0)
    self.assertEqual(dec["COUNTER"], 7)
    self.assertEqual(dec["SET_ME_1_1"], 1)

  def test_lks_pulse_rides_on_relay_frame(self):
    # Long mode: the camera sync press goes out on the relayed stock 0x3B0 (same
    # counter, one frame), never as a second frame with a duplicate counter.
    ctrl = _ctrl(long_control=True)
    stock = {"SET_ME_1_1": 1, "SET_ME_1_2": 1, "SET_BTN": 0, "RES_BTN": 0, "LKAS_ON_BTN": 0,
             "DEC_DISTANCE_BTN": 0, "INC_DISTANCE_BTN": 0, "ACC_ON_BTN": 0, "COUNTER": 3}
    sent = []
    ts = 0
    for i in range(CCP.LKS_CAM_DEBOUNCE_FRAMES + CCP.LKS_PULSE_PERIOD * 3):
      fresh = i % CCP.LKS_PULSE_PERIOD == 2
      if fresh:
        stock = {**stock, "COUNTER": (stock["COUNTER"] + 1) % 16}
        ts += 1
      CS = _cs(camera_lkas_state=2, buttons=stock, buttons_ts=ts)
      CS.pcm_buttons_new = [stock] if fresh else []
      _act, sends = ctrl.update(_cc(enabled=True, lat_active=False), CS, 0)
      bus2 = _bus2_lks(sends)
      self.assertLessEqual(len(bus2), 1)
      self.assertEqual(len(bus2), 1 if fresh else 0)
      for m in bus2:
        sent.append((stock["COUNTER"], _decode("PCM_BUTTONS", m)))
    pulses = [(c, v) for c, v in sent if v["LKAS_ON_BTN"] == 1]
    self.assertEqual(len(pulses), CCP.LKS_PULSE_TICKS)
    counter, values = pulses[0]
    self.assertEqual(values["COUNTER"], counter)
    self.assertEqual(values["ACC_ON_BTN"], 0)
    self.assertEqual(values["SET_BTN"], 0)

  def test_relay_strips_driver_lks_press(self):
    ctrl = _ctrl(long_control=True)
    stock = {"SET_ME_1_1": 1, "SET_ME_1_2": 1, "LKAS_ON_BTN": 1, "ACC_ON_BTN": 0, "COUNTER": 9}
    CS = _cs(lks_enabled=False, lks_btn_rising=True, camera_lkas_state=2, buttons=stock, buttons_ts=1)
    CS.pcm_buttons_new = [stock]
    _act, sends = ctrl.update(_cc(enabled=False, lat_active=False), CS, 0)
    relay = _bus2_lks(sends)
    self.assertEqual(len(relay), 1)
    self.assertEqual(_decode("PCM_BUTTONS", relay[0])["LKAS_ON_BTN"], 0)

  def test_no_button_relay_without_long_control(self):
    ctrl = _ctrl(long_control=False)
    CS = _cs()
    CS.pcm_buttons_new = [{"SET_BTN": 1, "COUNTER": 7, "SET_ME_1_1": 1, "SET_ME_1_2": 1, "ACC_ON_BTN": 1}]
    _act, sends = ctrl.update(_cc(enabled=False, lat_active=False), CS, 0)
    self.assertFalse(any(m[0] == 0x3B0 for m in sends))

  def test_acc_hud_state_speed_and_stock_bytes(self):
    ctrl = _ctrl(long_control=True)
    raw = bytes([0x00, 108, 4, 1, 244, 255, 0xF0])
    raw += bytes([(~sum(raw)) & 0xFF])
    stock = _decode("ACC_HUD_ADAS", (0x32D, raw, 2))
    CS = _cs()
    CS.acc_hud_stock = stock
    CS.camera_acc_state = 2
    CC = _cc(lat_active=False, long_active=True)
    CC.hudControl.setSpeed = 80 / 3.6
    CC.hudControl.leadDistanceBars = 2
    ctrl.update(CC, CS, 0)
    _act, sends = ctrl.update(CC, CS, 0)
    hud = [m for m in sends if m[0] == 0x32D]
    self.assertEqual(len(hud), 1)
    dat = bytes(hud[0][1])
    self.assertEqual(dat[0], 160)
    self.assertEqual(dat[2], 0x5C)
    self.assertEqual(dat[3:6], bytes([1, 244, 255]))
    self.assertEqual((dat[1] ^ 108) & ~0x1C, 0)
    dec = _decode("ACC_HUD_ADAS", hud[0])
    self.assertEqual(dec["ACC_STATE"], 3)
    self.assertEqual(dec["SET_DISTANCE"], 2)
    self.assertEqual(dec["COUNTER"], 0)
    self.assertEqual(dat[7], (~sum(dat[:7])) & 0xFF)

    CS.camera_acc_state = 0
    CC.hudControl.leadDistanceBars = 0
    ctrl.update(CC, CS, 0)
    _act, sends = ctrl.update(CC, CS, 0)
    dat = bytes([m for m in sends if m[0] == 0x32D][0][1])
    self.assertEqual(dat[2], 0x04)
    self.assertEqual(_decode("ACC_HUD_ADAS", (0x32D, dat, 0))["SET_DISTANCE"], 3)

  def test_unset_cruise_keeps_stock_set_speed(self):
    ctrl = _ctrl(long_control=True)
    raw = bytes([72, 108, 4, 1, 244, 255, 0xF0])
    raw += bytes([(~sum(raw)) & 0xFF])
    stock = _decode("ACC_HUD_ADAS", (0x32D, raw, 2))
    CS = _cs()
    CS.acc_hud_stock = stock
    CS.camera_acc_state = 2
    CC = _cc(lat_active=False, long_active=False)
    CC.hudControl.setSpeed = 255 / 3.6
    ctrl.update(CC, CS, 0)
    _act, sends = ctrl.update(CC, CS, 0)
    dat = bytes([m for m in sends if m[0] == 0x32D][0][1])
    self.assertEqual(dat[0], 72)
    self.assertEqual(dat[2], 0x54)

  def test_no_cancel_spoof_in_long_control(self):
    ctrl = _ctrl(long_control=True)
    CS = _cs(buttons={"SET_ME_1_1": 1, "SET_ME_1_2": 1, "COUNTER": 3})
    bus0 = []
    for _ in range(10):
      _act, sends = ctrl.update(_cc(cancel=True, lat_active=False), CS, 0)
      bus0.extend(m for m in sends if m[0] == 0x3B0 and m[2] == 0)
    self.assertEqual(bus0, [])


class TestBydLongCarState(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_non_essential_params("BYD_ATTO_3")
    self.packer = CANPacker(DBC[CAR.BYD_ATTO_3][Bus.pt])

  def _cs(self, long_control):
    from opendbc.car.byd.carstate import CarState
    self.CP.openpilotLongitudinalControl = long_control
    self.CP.pcmCruise = not long_control
    cs = CarState(self.CP)
    parsers = cs.get_can_parsers(self.CP)
    for bus, name in (
      (Bus.pt, "WHEELSPEED_CLEAN"), (Bus.pt, "STEER_MODULE_2"), (Bus.pt, "STEERING_TORQUE"),
      (Bus.pt, "PEDAL"), (Bus.pt, "DRIVE_STATE"), (Bus.pt, "STALKS"), (Bus.pt, "BSD_RADAR"),
      (Bus.pt, "METER_CLUSTER"), (Bus.cam, "PCW_ADAS"), (Bus.cam, "ACC_CMD"),
      (Bus.cam, "ACC_HUD_ADAS"), (Bus.cam, "LKAS_HUD_ADAS"),
    ):
      parsers[bus].vl[name]
    return cs, parsers

  def _feed(self, parsers, bus, name, values):
    addr, dat, _bus = self.packer.make_can_msg(name, 0 if bus == Bus.pt else 2, values)
    parsers[bus].update([(0, [(addr, dat, 0 if bus == Bus.pt else 2)])])

  def test_button_events_and_enable_edge(self):
    from opendbc.car import structs
    cs, parsers = self._cs(True)
    self._feed(parsers, Bus.cam, "ACC_HUD_ADAS", {"ACC_STATE": 2, "ACC_ON2": 1})
    self._feed(parsers, Bus.pt, "PCM_BUTTONS", {"SET_BTN": 1, "SET_ME_1_1": 1, "SET_ME_1_2": 1, "COUNTER": 1})
    ret = cs.update(parsers)
    self.assertEqual([(e.type, e.pressed) for e in ret.buttonEvents],
                     [(structs.CarState.ButtonEvent.Type.decelCruise, True)])
    self.assertFalse(cs.update_button_enable(ret.buttonEvents))

    self._feed(parsers, Bus.pt, "PCM_BUTTONS", {"SET_ME_1_1": 1, "SET_ME_1_2": 1, "COUNTER": 2})
    ret = cs.update(parsers)
    self.assertEqual([(e.type, e.pressed) for e in ret.buttonEvents],
                     [(structs.CarState.ButtonEvent.Type.decelCruise, False)])
    self.assertTrue(cs.update_button_enable(ret.buttonEvents))
    self.assertFalse(ret.cruiseState.enabled)
    self.assertTrue(ret.cruiseState.available)

  def test_lks_off_cancels_and_blocks_enable(self):
    from opendbc.car import structs
    cs, parsers = self._cs(True)
    self._feed(parsers, Bus.cam, "ACC_HUD_ADAS", {"ACC_STATE": 2, "ACC_ON2": 1, "COUNTER": 1})
    self._feed(parsers, Bus.pt, "PCM_BUTTONS", {"LKAS_ON_BTN": 1, "SET_ME_1_1": 1, "SET_ME_1_2": 1, "COUNTER": 1})
    ret = cs.update(parsers)
    self.assertFalse(cs.lks_enabled)
    self.assertIn((structs.CarState.ButtonEvent.Type.cancel, False),
                  [(e.type, e.pressed) for e in ret.buttonEvents])
    self.assertFalse(cs.update_button_enable(ret.buttonEvents))

    self._feed(parsers, Bus.pt, "PCM_BUTTONS", {"SET_BTN": 1, "LKAS_ON_BTN": 1, "SET_ME_1_1": 1, "SET_ME_1_2": 1, "COUNTER": 2})
    ret = cs.update(parsers)
    self._feed(parsers, Bus.pt, "PCM_BUTTONS", {"LKAS_ON_BTN": 1, "SET_ME_1_1": 1, "SET_ME_1_2": 1, "COUNTER": 3})
    ret = cs.update(parsers)
    self.assertFalse(cs.update_button_enable(ret.buttonEvents))

    self._feed(parsers, Bus.pt, "PCM_BUTTONS", {"SET_ME_1_1": 1, "SET_ME_1_2": 1, "COUNTER": 4})
    cs.update(parsers)
    self._feed(parsers, Bus.pt, "PCM_BUTTONS", {"LKAS_ON_BTN": 1, "SET_ME_1_1": 1, "SET_ME_1_2": 1, "COUNTER": 5})
    ret = cs.update(parsers)
    self.assertTrue(cs.lks_enabled)
    self.assertNotIn(structs.CarState.ButtonEvent.Type.cancel, [e.type for e in ret.buttonEvents])
    self._feed(parsers, Bus.pt, "PCM_BUTTONS", {"SET_BTN": 1, "LKAS_ON_BTN": 1, "SET_ME_1_1": 1, "SET_ME_1_2": 1, "COUNTER": 6})
    cs.update(parsers)
    self._feed(parsers, Bus.pt, "PCM_BUTTONS", {"LKAS_ON_BTN": 1, "SET_ME_1_1": 1, "SET_ME_1_2": 1, "COUNTER": 7})
    ret = cs.update(parsers)
    self.assertTrue(cs.update_button_enable(ret.buttonEvents))

  def test_cruise_follows_acc_main_and_not_active_state(self):
    cs, parsers = self._cs(True)
    self._feed(parsers, Bus.cam, "ACC_HUD_ADAS", {"ACC_STATE": 3, "ACC_ON2": 1, "SET_SPEED": 80})
    ret = cs.update(parsers)
    self.assertFalse(ret.cruiseState.enabled)
    self.assertTrue(ret.cruiseState.available)
    self.assertEqual(ret.cruiseState.speed, 0)

    self._feed(parsers, Bus.cam, "ACC_HUD_ADAS", {"ACC_STATE": 0, "SET_SPEED": 80})
    ret = cs.update(parsers)
    self.assertFalse(ret.cruiseState.available)

  def test_res_with_set_bit_is_accel_cruise(self):
    from opendbc.car import structs
    T = structs.CarState.ButtonEvent.Type
    cs, parsers = self._cs(True)
    self._feed(parsers, Bus.cam, "ACC_HUD_ADAS", {"ACC_STATE": 2, "ACC_ON2": 1})
    self._feed(parsers, Bus.pt, "PCM_BUTTONS", {"SET_BTN": 1, "RES_BTN": 1, "SET_ME_1_1": 1, "SET_ME_1_2": 1, "COUNTER": 1})
    ret = cs.update(parsers)
    self.assertEqual([(e.type, e.pressed) for e in ret.buttonEvents], [(T.accelCruise, True)])
    self._feed(parsers, Bus.pt, "PCM_BUTTONS", {"SET_ME_1_1": 1, "SET_ME_1_2": 1, "COUNTER": 2})
    ret = cs.update(parsers)
    self.assertEqual([(e.type, e.pressed) for e in ret.buttonEvents], [(T.accelCruise, False)])
    self.assertTrue(cs.update_button_enable(ret.buttonEvents))
    self._feed(parsers, Bus.pt, "PCM_BUTTONS", {"SET_BTN": 1, "SET_ME_1_1": 1, "SET_ME_1_2": 1, "COUNTER": 3})
    ret = cs.update(parsers)
    self.assertEqual([(e.type, e.pressed) for e in ret.buttonEvents], [(T.decelCruise, True)])


if __name__ == "__main__":
  unittest.main()
