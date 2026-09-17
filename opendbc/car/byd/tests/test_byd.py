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


def _cs(eps_engaged=True, lks_enabled=True, camera_lkas_state=0, angle=0.0, lks_btn_rising=False, eps_standby=False,
        buttons=None, buttons_ts=0):
  return SimpleNamespace(
    eps_engaged=eps_engaged,
    eps_standby=eps_standby,
    steer_not_accepted=False,
    lkas_hud={},
    lks_enabled=lks_enabled,
    lks_btn_rising=lks_btn_rising,
    camera_lkas_state=camera_lkas_state,
    pcm_buttons_stock=buttons if buttons is not None else {},
    pcm_buttons_ts=buttons_ts,
    out=SimpleNamespace(vEgoRaw=20.0, steeringAngleDeg=angle),
  )


def _decode(name, packer_msg):
  addr, dat, _bus = packer_msg
  cp = CANParser(DBC[CAR.BYD_ATTO_3][Bus.pt], [(name, 0)], 0)
  cp.update([(0, [(addr, dat, 0)])])
  return dict(cp.vl[name])


def _cc(enabled=True, lat_active=True, cancel=False):
  return SimpleNamespace(
    enabled=enabled,
    latActive=lat_active,
    actuators=_Actuators(),
    hudControl=_Hud(),
    cruiseControl=SimpleNamespace(cancel=cancel),
  )


def _steer_frames(ctrl, n, enabled=True, lat=True, eps_engaged=True, eps_standby=False, angle=0.0, desired=0.0):
  # Returns the decoded 0x1E2 sent on each 50 Hz slot (None if none) and the last CS.
  frames = []
  CS = None
  for _ in range(n):
    CS = _cs(eps_engaged=eps_engaged, eps_standby=eps_standby, angle=angle)
    CC = _cc(enabled=enabled, lat_active=lat)
    CC.actuators.steeringAngleDeg = desired
    _act, sends = ctrl.update(CC, CS, 0)
    if ctrl.frame % 2 == 0:  # frame already advanced: odd frames carry the steer slot
      steer = [m for m in sends if m[0] == 0x1E2]
      frames.append(_decode("STEERING_MODULE_ADAS", steer[0]) if steer else None)
  return frames, CS


class TestBydSteerNotAccepted(unittest.TestCase):
  def test_debounce_fires_after_200ms(self):
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())

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

  def test_warmup_then_first_req_repeats_angle(self):
    # After a TX gap: REQ=0 at the measured angle, then the first REQ=1 at that same angle,
    # then the ramp toward the desired angle.
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
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
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
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
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
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
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
    # warm-up slots with the EPS idle do not count
    _steer_frames(ctrl, 2 * CCP.STEER_WARMUP_FRAMES, eps_engaged=False)
    self.assertEqual(ctrl.not_accepted_frames, 0)
    # standby ack slots do not count either
    _steer_frames(ctrl, 2 * 4, eps_engaged=False, eps_standby=True)
    self.assertEqual(ctrl.not_accepted_frames, 2)


class TestBydCruiseGate(unittest.TestCase):
  def test_acc_without_lks_latch_does_not_enable(self):
    self.assertFalse(cruise_enabled(acc_state=3, lks_enabled=False))
    self.assertTrue(cruise_enabled(acc_state=3, lks_enabled=True))
    self.assertTrue(cruise_enabled(acc_state=5, lks_enabled=True))
    self.assertFalse(cruise_enabled(acc_state=2, lks_enabled=True))


def _bus2_lks(sends):
  return [m for m in sends if m[0] == 0x3B0 and m[2] == 2]


def _step(ctrl, n, enabled=True, lat=False, cam=0, lks=True, rising=False):
  pulses = 0
  last = []
  for i in range(n):
    CS = _cs(lks_enabled=lks, camera_lkas_state=cam, lks_btn_rising=rising and i == 0)
    _act, last = ctrl.update(_cc(enabled=enabled, lat_active=lat), CS, 0)
    pulses += len(_bus2_lks(last))
  return pulses, last


class TestBydLksCamera(unittest.TestCase):
  def test_neutralize_on_enabled_when_camera_on(self):
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
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
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
    pulses, _ = _step(ctrl, CCP.LKS_CAM_DEBOUNCE_FRAMES + 10, enabled=True, lat=True, cam=0)
    self.assertEqual(pulses, 0)

  def test_no_immediate_restore_or_idle_glitch(self):
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
    pulses, _ = _step(ctrl, CCP.LKS_HUD_QUIET_FRAMES - 1, enabled=False, cam=2, lks=True)
    self.assertEqual(pulses, 0)

  def test_restore_after_hud_quiet_when_latch_on(self):
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
    _step(ctrl, CCP.LKS_CAM_DEBOUNCE_FRAMES + 2, enabled=False, cam=0, lks=True)
    pulses, _ = _step(ctrl, CCP.LKS_HUD_QUIET_FRAMES + CCP.LKS_PULSE_PERIOD + 2, enabled=False, cam=0, lks=True)
    self.assertGreater(pulses, 0)

  def test_no_restore_when_latch_off(self):
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
    pulses, _ = _step(ctrl, CCP.LKS_HUD_QUIET_FRAMES + 20, enabled=False, cam=0, lks=False)
    self.assertEqual(pulses, 0)

  def test_latch_off_turns_camera_off_at_idle(self):
    # Persist off + stock camera still on: snap camera off without waiting for engage.
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
    pulses, _ = _step(ctrl, CCP.LKS_CAM_DEBOUNCE_FRAMES + CCP.LKS_PULSE_PERIOD + 2, enabled=False, cam=2, lks=False)
    self.assertGreater(pulses, 0)

  def test_failed_camera_off_retries_after_recover(self):
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
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
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
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
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
    pulses = 0
    for i in range(80):
      enabled = (i % 10) < 5
      CS = _cs(lks_enabled=True, camera_lkas_state=2)
      _act, sends = ctrl.update(_cc(enabled=enabled, lat_active=enabled), CS, 0)
      pulses += len(_bus2_lks(sends))
    self.assertLessEqual(pulses, CCP.LKS_PULSE_TICKS * 2)

  def test_acc_back_on_aborts_restore(self):
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
    _step(ctrl, CCP.LKS_HUD_QUIET_FRAMES - 1, enabled=False, cam=0, lks=True)
    _step(ctrl, CCP.LKS_PULSE_PERIOD, enabled=False, cam=0, lks=True)
    pulses_on, _ = _step(ctrl, CCP.LKS_PULSE_TICKS * CCP.LKS_PULSE_PERIOD + 10, enabled=True, cam=0, lks=True)
    self.assertEqual(pulses_on, 0)

  def test_button_lockout_then_snap_camera_off(self):
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
    _step(ctrl, CCP.LKS_CAM_DEBOUNCE_FRAMES + 2, enabled=True, cam=0, lks=True)
    pulses_lock, last_lock = _step(ctrl, CCP.LKS_LOCKOUT_FRAMES, enabled=False, cam=2, lks=False, rising=True)
    self.assertEqual(pulses_lock, 0)
    self.assertIn(0x1E2, {m[0] for m in last_lock})
    pulses_snap, _ = _step(ctrl, CCP.LKS_CAM_DEBOUNCE_FRAMES + CCP.LKS_PULSE_PERIOD + 2, enabled=False, cam=2, lks=False)
    self.assertGreater(pulses_snap, 0)

  def test_one_retry_then_stop(self):
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
    pulses, _ = _step(ctrl, CCP.LKS_CAM_DEBOUNCE_FRAMES + (CCP.LKS_PULSE_PERIOD + 1) * 2 +
                      CCP.LKS_CONFIRM_FRAMES * 2 + 20, enabled=True, cam=2)
    # one tick per pulse, two pulses (first + one retry), then give up
    self.assertEqual(pulses, CCP.LKS_PULSE_TICKS * 2)

  def test_tick_mirrors_stock_frame_right_after_it(self):
    # Our bus-2 frame is the car's latest 0x3B0 (same COUNTER) with only LKAS_ON_BTN added,
    # sent on the frame the car's 0x3B0 arrived.
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
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
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
    pulses, _ = _step(ctrl, CCP.LKS_CAM_DEBOUNCE_FRAMES + CCP.LKS_PULSE_PERIOD + 2, enabled=True, cam=2)
    self.assertEqual(pulses, 1)

  def test_no_tick_while_eps_standby(self):
    ctrl = CarController({Bus.pt: DBC[CAR.BYD_ATTO_3][Bus.pt]}, SimpleNamespace())
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

  def test_radar_dbc_is_mapped_but_unavailable(self):
    self.assertEqual(DBC[CAR.BYD_ATTO_3][Bus.radar], "byd_radar_fd")
    self.assertEqual(CCP.EPB_DEBOUNCE_FRAMES, 8)


if __name__ == "__main__":
  unittest.main()
