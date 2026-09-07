#!/usr/bin/env python3

import hashlib
import json
import math
import os
import sys
import time
import unittest
from collections import Counter, defaultdict
from pathlib import Path
from urllib.parse import quote, urlparse
from urllib.request import urlopen

from opendbc.can.dbc import DBC, SignalType
from opendbc.can.packer import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car import DT_CTRL, gen_empty_fingerprint, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import FRAME_FINGERPRINT, interfaces
from opendbc.car.fingerprints import MIGRATION
from opendbc.car.honda.values import HondaFlags
from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
from opendbc.car.logreader import LogReader
from opendbc.car.structs import car
from opendbc.car.tests.routes import CarTestRoute, non_tested_cars, routes
from opendbc.car.toyota.values import ToyotaFlags
from opendbc.car.values import PLATFORMS, Platform
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.volkswagen.values import VolkswagenFlags
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.testing import fuzzy_test


SafetyModel = car.CarParams.SafetyModel
SteerControlType = structs.CarParams.SteerControlType
LongControlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert

# panda safety stores angle_meas in brand-specific CAN units (angle_deg_to_can in opendbc/safety/modes/*.h).
ANGLE_DEG_TO_CAN = {
  "tesla": -10,
  "toyota": 17.452007,
  "nissan": 100,
  "psa": 10,
}

# TX fuzzing: route CAN replayed so openpilot and panda derive the same vehicle state, then 100Hz control frames
TX_FUZZ_WARMUP = 300
TX_FUZZ_FRAMES = 200
# MAX_CURVATURE in openpilot's clip_curvature: the bound controlsd keeps its desired curvature within.
# The angle command is derived from it through the vehicle model, so it also bounds the angle controllers.
MAX_ACTUATOR_CURVATURE = 0.2  # 1/m

# The steering command message and the signal panda checks, per brand, for the TX edge probe. The probe re-sends an
# accepted frame with that signal at the DBC's own extreme and asserts panda rejects it, so a panda that stopped
# enforcing the absolute cap is caught even though openpilot never emits such a command itself. Each entry lists
# candidates; the first whose message exists in one of the platform's DBCs is used, so a brand with several CAN
# generations needs one line per generation. Platforms with no matching candidate skip the probe.
STEER_CMD_SIGNALS = {
  "toyota": {"angle": [("STEERING_LTA", "STEER_ANGLE_CMD")], "torque": [("STEERING_LKA", "STEER_TORQUE_CMD")]},
  "nissan": {"angle": [("LKAS", "DESIRED_ANGLE")]},
  "psa": {"angle": [("LANE_KEEP_ASSIST", "SET_ANGLE")]},
  "hyundai": {"torque": [("LKAS11", "CR_Lkas_StrToqReq"),  # CAN
                         ("LKAS", "StrTqReqVal"), ("LKAS_ALT", "StrTqReqVal"), ("LFA", "StrTqReqVal")]},  # CAN FD
  "subaru": {"torque": [("ES_LKAS", "LKAS_Output")]},
  "tesla": {"angle": [("DAS_steeringControl", "DAS_steeringAngleRequest")]},
  "volkswagen": {"curvature": [("HCA_03", "Curvature")]},
}
# Absolute angle caps enforced by panda (max_angle / angle_deg_to_can in opendbc/safety/modes/*.h). The edge probe
# needs the DBC extreme to lie strictly beyond the cap for the rejection to be meaningful.
STEER_ANGLE_CAP_DEG = {"toyota": 94.9461, "nissan": 600., "psa": 390., "tesla": 360.}

NUM_JOBS = int(os.environ.get("NUM_JOBS", "1"))
JOB_ID = int(os.environ.get("JOB_ID", "0"))
RELAY_TRANSITION_TIMEOUT_US = 10_000_000
DOWNLOAD_CACHE_ROOT = Path(os.environ.get("COMMA_CACHE", "/tmp/comma_download_cache"))
OPENPILOT_CI_URL = "https://commadataci.blob.core.windows.net/openpilotci"
COMMA_API_URL = "https://api.commadotai.com"


def get_test_cases() -> list[tuple[str, CarTestRoute | None]]:
  routes_by_car = defaultdict(set)
  for route in routes:
    routes_by_car[str(route.car_model)].add(route)

  test_cases = []
  for i, platform in enumerate(sorted(PLATFORMS)):
    if i % NUM_JOBS == JOB_ID:
      test_cases.extend(sorted((platform, route) for route in routes_by_car.get(platform, (None,))))
  return test_cases


def get_cached_segment(route: str, segment: int) -> Path:
  for filename in ("rlog.zst", "rlog.bz2"):
    url = f"{OPENPILOT_CI_URL}/{route.replace('|', '/')}/{segment}/{filename}"
    try:
      return get_cached_url(url)
    except OSError:
      pass

  canonical_route = route.replace("/", "|")
  with urlopen(f"{COMMA_API_URL}/v1/route/{quote(canonical_route, safe='')}/files") as response:
    route_files = json.load(response)
  for url in route_files["logs"]:
    path = Path(urlparse(url).path)
    if path.parent.name == str(segment) and path.name.startswith("rlog."):
      return get_cached_url(url)

  raise FileNotFoundError(f"no log found for {route}/{segment}")


def get_cached_url(url: str) -> Path:
  cache_path = DOWNLOAD_CACHE_ROOT / f"{hashlib.sha256(url.encode()).hexdigest()}{Path(url).suffix}"
  if not cache_path.exists():
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = cache_path.with_suffix(f".{os.getpid()}.tmp")
    with urlopen(url) as response, open(tmp_path, "wb") as output:
      output.write(response.read())
    tmp_path.replace(cache_path)
  return cache_path


def normalize_can_buses(can: tuple[int, list[CanData]], raw_can_keys: set[tuple[int, int]]) -> tuple[int, list[CanData]]:
  timestamp, messages = can
  return timestamp, [
    CanData(msg.address, msg.dat, msg.src % 128) for msg in messages
    if msg.src < 128 or (msg.address, msg.src % 128) not in raw_can_keys
  ]


def fuzzy_controls(fuzzy) -> dict:
  """Everything controlsd is free to pick when it builds a CarControl. Add new state here to fuzz it."""
  return {
    "lat_active": fuzzy.boolean(),
    "long_active": fuzzy.boolean(),
    "torque": fuzzy.real(-1., 1.),
    "curvature": fuzzy.real(-MAX_ACTUATOR_CURVATURE, MAX_ACTUATOR_CURVATURE),
    "accel": fuzzy.real(ACCEL_MIN, ACCEL_MAX),
    "long_control_state": fuzzy.choice(tuple(LongControlState.schema.enumerants)),
    "resume": fuzzy.boolean(),
    "left_blinker": fuzzy.boolean(),
    "right_blinker": fuzzy.boolean(),
    "visual_alert": fuzzy.choice(tuple(VisualAlert.schema.enumerants)),
    "lead_visible": fuzzy.boolean(),
    "lanes_visible": fuzzy.boolean(),
    "lane_depart": fuzzy.boolean(),
    "lead_distance_bars": fuzzy.integer(1, 3),
    "set_speed": fuzzy.real(0., 40.),
  }


def build_car_control(controls: dict, enabled: bool, CP, VM: VehicleModel, CS) -> structs.CarControl:
  """The CarControl controlsd would send for these controls, mirroring selfdrive/controls/controlsd.py.

  enabled is passed in rather than drawn: the caller derives it from panda's own engagement state so that
  a rejected frame is a limits mismatch and never an artifact of the test forcing the two sides to agree.
  """
  standstill = abs(CS.vEgo) <= max(CP.minSteerSpeed, 0.3) or CS.standstill
  lat_active = enabled and controls["lat_active"] and not CS.steerFaultTemporary and \
               not CS.steerFaultPermanent and (not standstill or CP.steerAtStandstill)
  # a pressed gas pedal is an OVERRIDE_LONGITUDINAL event, which matches panda's longitudinal_allowed gate
  long_active = enabled and controls["long_active"] and CP.openpilotLongitudinalControl and not CS.gasPressed

  # while inactive the controllers command the car's current state, not the planner's, see
  # selfdrive/controls/lib/latcontrol_angle.py and longcontrol.py
  curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - CS.steeringAngleOffsetDeg), CS.vEgo, 0.)
  angle = math.degrees(VM.get_steer_from_curvature(-controls["curvature"], CS.vEgo, 0.))

  return structs.CarControl(
    enabled=enabled,
    latActive=lat_active,
    longActive=long_active,
    leftBlinker=controls["left_blinker"],
    rightBlinker=controls["right_blinker"],
    currentCurvature=curvature,
    actuators=structs.CarControl.Actuators(
      torque=controls["torque"] if lat_active else 0.,
      steeringAngleDeg=angle if lat_active else CS.steeringAngleDeg,
      curvature=controls["curvature"] if lat_active else curvature,
      accel=controls["accel"] if long_active else 0.,
      longControlState=controls["long_control_state"] if long_active else "off",
    ),
    cruiseControl=structs.CarControl.CruiseControl(
      cancel=CS.cruiseState.enabled and (not enabled or not CP.pcmCruise),
      resume=enabled and CS.cruiseState.standstill and controls["resume"],
      override=enabled and not long_active and CP.openpilotLongitudinalControl,
    ),
    hudControl=structs.CarControl.HUDControl(
      speedVisible=enabled,
      setSpeed=controls["set_speed"],
      lanesVisible=controls["lanes_visible"],
      leadVisible=controls["lead_visible"],
      visualAlert=controls["visual_alert"],
      leftLaneVisible=controls["lanes_visible"],
      rightLaneVisible=controls["lanes_visible"],
      leftLaneDepart=controls["lane_depart"],
      rightLaneDepart=controls["lane_depart"],
      leadDistanceBars=controls["lead_distance_bars"],
    ),
  )


def decode_frame(dbc, address: int, dat: bytes) -> dict[str, float]:
  """Physical values of every plain signal in a frame. Checksums and counters are left out so CANPacker regenerates them."""
  values = {}
  for sig in dbc.addr_to_msg[address].sigs.values():
    if sig.type != SignalType.DEFAULT:
      continue
    raw = get_raw_value(dat, sig)
    if sig.is_signed and raw >= 1 << (sig.size - 1):
      raw -= 1 << sig.size
    values[sig.name] = raw * sig.factor + sig.offset
  return values


def signal_extreme(dbc, address: int, name: str, positive: bool) -> float:
  """The largest (or smallest) physical value the signal's bit field can carry."""
  sig = dbc.addr_to_msg[address].sigs[name]
  if sig.is_signed:
    raw = (1 << (sig.size - 1)) - 1 if positive else -(1 << (sig.size - 1))
  else:
    raw = (1 << sig.size) - 1 if positive else 0
  return raw * sig.factor + sig.offset


class TestCarModelBase(unittest.TestCase):
  platform: Platform | None = None
  test_route: CarTestRoute | None = None

  can_msgs: list[tuple[int, list[CanData]]]
  fingerprint: dict[int, dict[int, int]]
  elm_frame: int | None
  car_safety_mode_frame: int | None

  @classmethod
  def get_testing_data_from_logreader(cls, lr):
    car_fw = []
    can_msgs = []
    cls.elm_frame = None
    cls.car_safety_mode_frame = None
    cls.fingerprint = gen_empty_fingerprint()
    alpha_long = False

    for msg in lr:
      if msg.which() == "can":
        can_msgs.append((msg.logMonoTime, [CanData(can.address, can.dat, can.src) for can in msg.can]))
        if len(can_msgs) <= FRAME_FINGERPRINT:
          for can in msg.can:
            if can.src < 64:
              cls.fingerprint[can.src][can.address] = len(can.dat)

      elif msg.which() == "carParams":
        car_fw = msg.carParams.carFw
        alpha_long |= msg.carParams.openpilotLongitudinalControl
        if cls.platform is None:
          live_fingerprint = msg.carParams.carFingerprint
          cls.platform = MIGRATION.get(live_fingerprint, live_fingerprint)

      # Log which CAN frame panda safety left ELM327, for CAN validity checks.
      elif msg.which() == "pandaStates":
        for panda_state in msg.pandaStates:
          if cls.elm_frame is None and panda_state.safetyModel != SafetyModel.elm327:
            cls.elm_frame = len(can_msgs)
          if cls.car_safety_mode_frame is None and panda_state.safetyModel not in (SafetyModel.elm327, SafetyModel.noOutput):
            cls.car_safety_mode_frame = len(can_msgs)

      elif msg.which() == "pandaStateDEPRECATED":
        panda_state = msg.pandaStateDEPRECATED
        if cls.elm_frame is None and panda_state.safetyModel != SafetyModel.elm327:
          cls.elm_frame = len(can_msgs)
        if cls.car_safety_mode_frame is None and panda_state.safetyModel not in (SafetyModel.elm327, SafetyModel.noOutput):
          cls.car_safety_mode_frame = len(can_msgs)

    assert len(can_msgs) > int(50 / DT_CTRL), "no CAN data found"
    return car_fw, can_msgs, alpha_long

  @classmethod
  def get_testing_data(cls):
    test_segments = (2, 1, 0) if cls.test_route.segment is None else (cls.test_route.segment,)
    for segment in test_segments:
      try:
        log_path = get_cached_segment(cls.test_route.route, segment)
        return cls.get_testing_data_from_logreader(LogReader(str(log_path), only_union_types=True, sort_by_time=True))
      except (OSError, AssertionError):
        pass

    raise Exception(f"Route: {cls.test_route.route!r} with segments: {test_segments} not found or no CAN messages found")

  @classmethod
  def setUpClass(cls):
    if cls.__name__.endswith("Base"):
      raise unittest.SkipTest

    if cls.test_route is None:
      if cls.platform in non_tested_cars:
        raise unittest.SkipTest(f"missing route for {cls.platform}")
      raise Exception(f"missing test route for {cls.platform}")

    car_fw, cls.can_msgs, alpha_long = cls.get_testing_data()
    cls.raw_can_keys = {(msg.address, msg.src) for _, messages in cls.can_msgs for msg in messages if msg.src < 128}
    cls.CarInterface = interfaces[cls.platform]
    cls.CP = cls.CarInterface.get_params(cls.platform, cls.fingerprint, car_fw, alpha_long, False, docs=False)
    assert cls.CP
    assert cls.CP.carFingerprint == cls.platform

  @classmethod
  def tearDownClass(cls):
    del cls.can_msgs

  def setUp(self):
    self.CI = self.CarInterface(self.CP.copy())
    assert self.CI

    self.safety = libsafety_py.libsafety
    cfg = self.CP.safetyConfigs[-1]
    set_status = self.safety.set_safety_hooks(cfg.safetyModel.raw, cfg.safetyParam)
    self.assertEqual(0, set_status, f"failed to set safetyModel {cfg}")
    self.safety.init_tests()

  def test_car_params(self):
    if self.CP.dashcamOnly:
      self.skipTest("no need to check carParams for dashcamOnly")

    self.assertGreater(self.CP.mass, 1)
    if self.CP.steerControlType not in (SteerControlType.angle, SteerControlType.curvature):
      tuning = self.CP.lateralTuning.which()
      if tuning == "pid":
        self.assertTrue(len(self.CP.lateralTuning.pid.kpV))
      elif tuning == "torque":
        self.assertGreater(self.CP.lateralTuning.torque.latAccelFactor, 0)
      else:
        raise Exception("unknown tuning")

  def test_car_interface(self):
    can_invalid_cnt = 0
    CC = structs.CarControl().as_reader()
    for i, msg in enumerate(self.can_msgs):
      CS = self.CI.update(normalize_can_buses(msg, self.raw_can_keys))
      self.CI.apply(CC, msg[0])
      if i > 250:
        can_invalid_cnt += not CS.canValid
    self.assertEqual(can_invalid_cnt, 0)

  def test_radar_interface(self):
    RI = self.CarInterface.RadarInterface(self.CP)
    assert RI

    error_cnt = 0
    radar_initialized = False
    for msg in self.can_msgs[self.elm_frame:]:
      rr: structs.RadarData | None = RI.update(msg)
      if rr is not None and not rr.errors.canError:
        radar_initialized = True
      if rr is not None and radar_initialized:
        error_cnt += rr.errors.canError
    self.assertTrue(radar_initialized or self.CP.radarUnavailable)
    self.assertEqual(error_cnt, 0)

  def test_panda_safety_rx_checks(self):
    if self.CP.dashcamOnly:
      self.skipTest("no need to check panda safety for dashcamOnly")

    start_ts = self.can_msgs[0][0]
    failed_addrs = Counter()
    last_relay_malfunction_us = 0.
    relay_open_inferred = False
    for can_idx, can in enumerate(self.can_msgs):
      t = (can[0] - start_ts) / 1e3
      self.safety.set_timer(int(t))

      for msg in can[1]:
        if msg.src >= 64:
          continue
        packet = libsafety_py.make_CANPacket(msg.address, msg.src % 4, msg.dat)
        if self.safety.safety_rx_hook(packet) != 1:
          failed_addrs[hex(msg.address)] += 1

      # Some logs contain a bus only as panda's returned bus (bus + 128).
      # Replay returned-only messages, but ignore rejected TX echoes.
      relay_malfunction = self.safety.get_relay_malfunction()
      for msg in can[1]:
        if msg.src >= 128 and (msg.address, msg.src % 128) not in self.raw_can_keys:
          packet = libsafety_py.make_CANPacket(msg.address, msg.src % 4, msg.dat)
          self.safety.safety_rx_hook(packet)
      self.safety.set_relay_malfunction(relay_malfunction)

      self.safety.safety_tick_current_safety_config()
      if t > 1e6:
        self.assertTrue(self.safety.safety_config_valid())

      if self.car_safety_mode_frame is not None:
        if can_idx >= self.car_safety_mode_frame:
          self.assertFalse(self.safety.get_relay_malfunction())
        else:
          self.safety.set_relay_malfunction(False)
      elif relay_open_inferred:
        self.assertFalse(self.safety.get_relay_malfunction())
      elif self.safety.get_relay_malfunction():
        # Without recorded panda state, infer the transition once relay-check traffic disappears.
        last_relay_malfunction_us = t
        self.safety.set_relay_malfunction(False)
      elif t - last_relay_malfunction_us > RELAY_TRANSITION_TIMEOUT_US:
        relay_open_inferred = True
      else:
        self.safety.set_relay_malfunction(False)

    self.assertFalse(failed_addrs, f"panda safety RX check failed: {failed_addrs}")
    self.safety.set_timer(int(t + 2e6))
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.safety_config_valid())

  def test_panda_safety_tx_cases(self):
    """Asserts we can transmit common messages."""
    if self.CP.dashcamOnly:
      self.skipTest("no need to check panda safety for dashcamOnly")
    if self.CP.notCar:
      self.skipTest("skipping test for notCar")
    if self.CP.flags & ToyotaFlags.SECOC:
      self.skipTest("SecOC transmit tests require the vehicle key")

    controller_params = self.CP
    if self.CP.brand == "volkswagen" and self.CP.flags & VolkswagenFlags.MLB and self.CP.openpilotLongitudinalControl:
      # Some archived MLB routes record alpha longitudinal, which current MLB safety does not support.
      controller_params = self.CarInterface.get_params(self.platform, self.fingerprint, self.CP.carFw, False, False, docs=False)

    def test_car_controller(car_control):
      now_nanos = 0
      msgs_sent = 0
      CI = self.CarInterface(controller_params)
      for _ in range(round(10.0 / DT_CTRL)):
        CI.update([])
        _, sendcan = CI.apply(car_control, now_nanos)
        now_nanos += DT_CTRL * 1e9
        msgs_sent += len(sendcan)
        for addr, dat, bus in sendcan:
          packet = libsafety_py.make_CANPacket(addr, bus % 4, dat)
          self.assertTrue(self.safety.safety_tx_hook(packet), (addr, dat, bus))
      self.assertGreater(msgs_sent, 50)

    test_car_controller(structs.CarControl().as_reader())

    self.safety.set_cruise_engaged_prev(True)
    CC = structs.CarControl(cruiseControl=structs.CarControl.CruiseControl(cancel=True))
    test_car_controller(CC.as_reader())

    self.safety.set_controls_allowed(True)
    CC = structs.CarControl(cruiseControl=structs.CarControl.CruiseControl(resume=True))
    test_car_controller(CC.as_reader())

  def _tx_test_params(self):
    """Skips cars we can't transmit for and returns the CarParams to build the car controller with."""
    if self.CP.dashcamOnly:
      self.skipTest("no need to check panda safety for dashcamOnly")
    if self.CP.notCar:
      self.skipTest("skipping test for notCar")
    if self.CP.flags & ToyotaFlags.SECOC:
      self.skipTest("SecOC transmit tests require the vehicle key")
    if self.CP.brand == "volkswagen" and self.CP.flags & VolkswagenFlags.MLB and self.CP.openpilotLongitudinalControl:
      # Some archived MLB routes record alpha longitudinal, which current MLB safety does not support.
      return self.CarInterface.get_params(self.platform, self.fingerprint, self.CP.carFw, False, False, docs=False)
    return self.CP

  def _tx_replay(self, CI, can):
    """Feeds one CAN frame to both sides so they derive the same vehicle state, and returns openpilot's CarState."""
    self.safety.set_timer(int((can[0] - self.can_msgs[0][0]) / 1e3))
    for msg in (msg for msg in can[1] if msg.src < 64):
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(msg.address, msg.src % 4, msg.dat))
    return CI.update(normalize_can_buses(can, self.raw_can_keys))

  @fuzzy_test(max_examples=25)
  def test_panda_safety_tx_fuzzy(self, fuzzy):
    """Fuzzes what openpilot sends and asserts panda safety accepts all of it.

    Both sides read the car's state off the same route CAN, and openpilot's engagement is taken from panda's own
    controls_allowed rather than forced, so every message the car controller emits for a CarControl controlsd can
    reach has to pass panda's TX checks. A rejection is a mismatch between the limits openpilot applies to its
    commands and the ones panda enforces on them.

    Scenarios push the two sides to the places they are most likely to disagree: winding the rate limiters up
    to the actuator bound and stepping back, toggling engagement while a command is live, and skipping control
    frames so the real time rate checks see an irregular cadence.
    """
    controller_params = self._tx_test_params()
    CI = self.CarInterface(controller_params)
    VM = VehicleModel(controller_params)
    cfg = self.CP.safetyConfigs[-1]
    self.safety.set_safety_hooks(cfg.safetyModel.raw, cfg.safetyParam)
    self.safety.init_tests()

    scenario = fuzzy.choice(("random", "hold_windup", "churn", "cadence_jitter"))
    # fuzz a random part of the route so examples cover different driving conditions
    start = fuzzy.integer(0, max(len(self.can_msgs) - TX_FUZZ_WARMUP - 2 * TX_FUZZ_FRAMES, 0))
    controls = fuzzy.list(lambda: fuzzy_controls(fuzzy), min_size=1, max_size=8)
    # holding a control lets openpilot's rate limiters wind up; the steps between holds are what panda's rate limits check
    hold_frames = fuzzy.integer(1, TX_FUZZ_FRAMES)
    skip_every = fuzzy.integer(2, 7)

    for can in self.can_msgs[start:start + TX_FUZZ_WARMUP]:
      self._tx_replay(CI, can)

    frame, sent, rejected = 0, Counter(), []
    next_apply_ts = self.can_msgs[start + TX_FUZZ_WARMUP][0]
    for can in self.can_msgs[start + TX_FUZZ_WARMUP:]:
      CS = self._tx_replay(CI, can)

      # openpilot controls at 100Hz however fast the route logged CAN, and panda checks its real time
      # rate limits against the log's timestamps, so both have to run on the same clock
      if can[0] < next_apply_ts:
        continue
      next_apply_ts += DT_CTRL * 1e9

      frame += 1
      if frame > TX_FUZZ_FRAMES:
        break
      if scenario == "cadence_jitter" and frame % skip_every == 0:
        continue

      control = controls[frame // hold_frames % len(controls)]
      if scenario == "hold_windup":
        # first half: hold the actuator bound so the rate limiters wind all the way up. second half: step to zero.
        windup = frame <= TX_FUZZ_FRAMES // 2
        control = {**control, "lat_active": True,
                   "curvature": math.copysign(MAX_ACTUATOR_CURVATURE, control["curvature"] or 1.) if windup else 0.,
                   "torque": math.copysign(1., control["torque"] or 1.) if windup else 0.}

      # panda's engagement comes from the replayed CAN, exactly as it would on the car. never forced.
      enabled = bool(self.safety.get_controls_allowed())
      if scenario == "churn":
        enabled = enabled and (frame // max(hold_frames // 4, 1)) % 2 == 0
      CC = build_car_control(control, enabled, controller_params, VM, CS)

      # relay malfunction is an RX side check covered by test_panda_safety_rx_checks; clear it so it can't mask a TX check
      self.safety.set_relay_malfunction(False)
      _, sendcan = CI.apply(CC.as_reader(), can[0])
      for addr, dat, bus in sendcan:
        sent[(hex(addr), bus)] += 1
        if not self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus % 4, dat)):
          actuation = {k: control[k] for k in ("lat_active", "long_active", "torque", "curvature", "accel")}
          rejected.append((frame, hex(addr), bus, enabled, actuation))

    self.assertGreater(sum(sent.values()), 0, f"no TX messages sent ({scenario=}, {frame=})")
    self.assertFalse(rejected, f"panda safety rejected {len(rejected)} TX frames ({scenario=}), first: {rejected[:3]}, sent: {dict(sent)}")

  def test_panda_safety_tx_edge(self):
    """Re-sends accepted steering commands pushed past the absolute limit and asserts panda rejects them.

    The fuzz test can only show that panda accepts what openpilot sends. This shows the other direction: a frame
    openpilot would never emit, the steering command at the DBC's own extreme, is refused. If panda stopped
    enforcing a cap, the fuzz test would stay green and this would go red. The unmodified frame is re-packed and
    sent first, so a rejection can't be an artifact of the decode and re-pack round trip.
    """
    signals = STEER_CMD_SIGNALS.get(self.CP.brand)
    # capnp enum values compare equal to the schema enumerants but do not hash equal, so no dict lookup here
    if self.CP.steerControlType == SteerControlType.angle:
      kind = "angle"
    elif self.CP.steerControlType == SteerControlType.curvature:
      kind = "curvature"
    else:
      kind = "torque"
    if not signals or kind not in signals:
      self.skipTest(f"no steering command signal registered for {self.CP.brand} {kind} control")

    controller_params = self._tx_test_params()
    CI = self.CarInterface(controller_params)
    VM = VehicleModel(controller_params)
    cfg = self.CP.safetyConfigs[-1]
    self.safety.set_safety_hooks(cfg.safetyModel.raw, cfg.safetyParam)
    self.safety.init_tests()

    # every candidate present in one of the platform's DBCs, keyed by address: which one the car actually sends is
    # decided by flags inside the car controller, so the first accepted frame that matches picks the candidate
    candidates = {}
    for dbc in (DBC(cp.dbc_name) for cp in CI.can_parsers.values()):
      for msg, sig in signals[kind]:
        if msg in dbc.name_to_msg:
          candidates.setdefault(dbc.name_to_msg[msg].address, (msg, sig, dbc))
    if not candidates:
      self.skipTest(f"none of {[msg for msg, _ in signals[kind]]} in the DBCs used by {self.platform}")

    control = {"lat_active": True, "long_active": False, "torque": 0.5, "curvature": 0.05, "accel": 0.,
               "long_control_state": "off", "resume": False, "left_blinker": False, "right_blinker": False,
               "visual_alert": "none", "lead_visible": False, "lanes_visible": True, "lane_depart": False,
               "lead_distance_bars": 2, "set_speed": 30.}

    for can in self.can_msgs[:TX_FUZZ_WARMUP]:
      self._tx_replay(CI, can)

    probes = 0
    next_apply_ts = self.can_msgs[TX_FUZZ_WARMUP][0]
    for can in self.can_msgs[TX_FUZZ_WARMUP:]:
      CS = self._tx_replay(CI, can)
      if can[0] < next_apply_ts:
        continue
      next_apply_ts += DT_CTRL * 1e9

      # engagement is needed for the command to be live; take it from panda and, if the route never engages, from the
      # test override that test_panda_safety_tx_cases also relies on
      if not self.safety.get_controls_allowed():
        self.safety.set_controls_allowed(True)
      CC = build_car_control(control, True, controller_params, VM, CS)
      self.safety.set_relay_malfunction(False)
      _, sendcan = CI.apply(CC.as_reader(), can[0])

      for addr, dat, bus in sendcan:
        if addr not in candidates or not self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus % 4, dat)):
          continue
        msg_name, sig_name, dbc = candidates[addr]
        address = addr
        packer = CANPacker(dbc.name)

        values = decode_frame(dbc, address, dat)
        # round trip: the same values re-packed must still be accepted, or the harness is what's broken
        _, dat_rt, _ = packer.make_can_msg(msg_name, bus, values)
        self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus % 4, dat_rt)),
                        f"re-packed {msg_name} was rejected: decode/pack round trip is not faithful ({values})")

        for positive in (True, False):
          extreme = signal_extreme(dbc, address, sig_name, positive)
          if extreme == 0:
            continue  # an unsigned magnitude (sign carried in another signal) bottoms out at zero, never past a cap
          if kind == "angle" and abs(extreme) <= STEER_ANGLE_CAP_DEG[self.CP.brand]:
            continue  # the field can't represent anything past the cap in this direction
          _, dat_x, _ = packer.make_can_msg(msg_name, bus, {**values, sig_name: extreme})
          self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus % 4, dat_x)),
                           f"panda accepted {msg_name}.{sig_name}={extreme} (was {values[sig_name]})")
          probes += 1
        break

      if probes:
        break

    self.assertGreater(probes, 0, f"never got an accepted {[msg for msg, _, _ in candidates.values()]} frame to probe")

  @fuzzy_test(max_examples=300)
  def test_panda_safety_carstate_fuzzy(self, fuzzy):
    if self.CP.dashcamOnly:
      self.skipTest("no need to check panda safety for dashcamOnly")

    valid_addrs = [(addr, bus, size) for bus, addrs in self.fingerprint.items() for addr, size in addrs.items()]
    address, bus, size = fuzzy.choice(valid_addrs)
    msgs = fuzzy.list(lambda: fuzzy.binary(min_size=size, max_size=size), min_size=20)
    vehicle_speed_seen = self.CP.steerControlType == SteerControlType.angle and not self.CP.notCar

    for n, dat in enumerate(msgs):
      prev_panda_gas = self.safety.get_gas_pressed_prev()
      prev_panda_brake = self.safety.get_brake_pressed_prev()
      prev_panda_regen_braking = self.safety.get_regen_braking_prev()
      prev_panda_steering_disengage = self.safety.get_steering_disengage_prev()
      prev_panda_vehicle_moving = self.safety.get_vehicle_moving()
      prev_panda_vehicle_speed_min = self.safety.get_vehicle_speed_min()
      prev_panda_vehicle_speed_max = self.safety.get_vehicle_speed_max()
      prev_panda_cruise_engaged = self.safety.get_cruise_engaged_prev()
      prev_panda_acc_main_on = self.safety.get_acc_main_on()

      packet = libsafety_py.make_CANPacket(address, bus, dat)
      self.safety.safety_rx_hook(packet)

      can = [(time.monotonic_ns(), [CanData(address=address, dat=dat, src=bus)])]
      CS = self.CI.update(can)
      if n < 5:
        continue

      if self.safety.get_gas_pressed_prev() != prev_panda_gas:
        self.assertEqual(CS.gasPressed, self.safety.get_gas_pressed_prev())
      if self.safety.get_brake_pressed_prev() != prev_panda_brake:
        self.assertEqual(CS.brakePressed, self.safety.get_brake_pressed_prev())
      if self.safety.get_regen_braking_prev() != prev_panda_regen_braking:
        self.assertEqual(CS.regenBraking, self.safety.get_regen_braking_prev())
      if self.safety.get_steering_disengage_prev() != prev_panda_steering_disengage:
        self.assertEqual(CS.steeringDisengage, self.safety.get_steering_disengage_prev())
      if self.safety.get_vehicle_moving() != prev_panda_vehicle_moving and not self.CP.notCar:
        self.assertEqual(not CS.standstill, self.safety.get_vehicle_moving())

      if self.safety.get_vehicle_speed_min() > 0 or self.safety.get_vehicle_speed_max() > 0:
        vehicle_speed_seen = True
      if vehicle_speed_seen and (self.safety.get_vehicle_speed_min() != prev_panda_vehicle_speed_min or
                                 self.safety.get_vehicle_speed_max() != prev_panda_vehicle_speed_max):
        v_ego_raw = CS.vEgoRaw / self.CP.wheelSpeedFactor
        self.assertFalse(v_ego_raw > self.safety.get_vehicle_speed_max() + 1e-3 or
                         v_ego_raw < self.safety.get_vehicle_speed_min() - 1e-3)

      if not (self.CP.brand == "honda" and not (self.CP.flags & HondaFlags.BOSCH)):
        if self.safety.get_cruise_engaged_prev() != prev_panda_cruise_engaged:
          self.assertEqual(CS.cruiseState.enabled, self.safety.get_cruise_engaged_prev())
      if self.CP.brand == "honda" and self.safety.get_acc_main_on() != prev_panda_acc_main_on:
        self.assertEqual(CS.cruiseState.available, self.safety.get_acc_main_on())

  def test_panda_safety_carstate(self):
    if self.CP.dashcamOnly:
      self.skipTest("no need to check panda safety for dashcamOnly")

    for can in self.can_msgs[:300]:
      self.CI.update(can)
      for msg in (msg for msg in can[1] if msg.src < 64):
        self.safety.safety_rx_hook(libsafety_py.make_CANPacket(msg.address, msg.src % 4, msg.dat))

    controls_allowed_prev = False
    CS_prev = car.CarState.new_message()
    checks = defaultdict(int)
    vehicle_speed_seen = self.CP.steerControlType == SteerControlType.angle and not self.CP.notCar

    for idx, can in enumerate(self.can_msgs):
      CS = self.CI.update(can).as_reader()
      for msg in (msg for msg in can[1] if msg.src < 64):
        packet = libsafety_py.make_CANPacket(msg.address, msg.src % 4, msg.dat)
        ret = self.safety.safety_rx_hook(packet)
        self.assertEqual(1, ret, f"safety RX failed ({ret=}): {(msg.address, msg.src % 4)}")

      if idx == 0:
        CS_prev = CS
        if not self.CP.pcmCruise:
          self.safety.set_controls_allowed(0)
        continue

      checks["gasPressed"] += CS.gasPressed != self.safety.get_gas_pressed_prev()
      checks["standstill"] += (CS.standstill == self.safety.get_vehicle_moving()) and not self.CP.notCar

      if self.safety.get_vehicle_speed_min() > 0 or self.safety.get_vehicle_speed_max() > 0:
        vehicle_speed_seen = True
      if vehicle_speed_seen:
        v_ego_raw = CS.vEgoRaw / self.CP.wheelSpeedFactor
        checks["vEgoRaw"] += (v_ego_raw > self.safety.get_vehicle_speed_max() + 1e-3 or
                              v_ego_raw < self.safety.get_vehicle_speed_min() - 1e-3)

      if self.CP.steerControlType == SteerControlType.angle and not self.CP.notCar and self.CP.brand not in ("ford", "volkswagen"):
        angle_can = (CS.steeringAngleDeg + CS.steeringAngleOffsetDeg) * ANGLE_DEG_TO_CAN[self.CP.brand]
        checks["steeringAngleDeg"] += (angle_can > self.safety.get_angle_meas_max() + 1 or
                                       angle_can < self.safety.get_angle_meas_min() - 1)

      checks["brakePressed"] += CS.brakePressed != self.safety.get_brake_pressed_prev()
      checks["regenBraking"] += CS.regenBraking != self.safety.get_regen_braking_prev()
      checks["steeringDisengage"] += CS.steeringDisengage != self.safety.get_steering_disengage_prev()

      if self.CP.pcmCruise:
        if self.CP.brand == "honda" and not (self.CP.flags & HondaFlags.BOSCH):
          if CS.cruiseState.enabled and not CS_prev.cruiseState.enabled:
            checks["controlsAllowed"] += not self.safety.get_controls_allowed()
        else:
          checks["controlsAllowed"] += not CS.cruiseState.enabled and self.safety.get_controls_allowed()
        if not self.CP.notCar:
          checks["cruiseState"] += CS.cruiseState.enabled != self.safety.get_cruise_engaged_prev()
      else:
        button_enable = CS.buttonEnable and (not CS.brakePressed or CS.standstill)
        mismatch = button_enable != (self.safety.get_controls_allowed() and not controls_allowed_prev)
        checks["controlsAllowed"] += mismatch
        controls_allowed_prev = self.safety.get_controls_allowed()
        if button_enable and not mismatch:
          self.safety.set_controls_allowed(False)

      if self.CP.brand == "honda":
        checks["mainOn"] += CS.cruiseState.available != self.safety.get_acc_main_on()
      CS_prev = CS

    failed_checks = {key: value for key, value in checks.items() if value > 0}
    self.assertFalse(failed_checks, f"panda safety doesn't agree with CarState: {failed_checks}")


DIRECTLY_CALLED = Path(sys.argv[0]).resolve() == Path(__file__).resolve()

if DIRECTLY_CALLED:
  for index, (platform, test_route) in enumerate(get_test_cases()):
    name = f"TestCarModel_{index}_{platform}"
    globals()[name] = type(name, (TestCarModelBase,), {"platform": platform, "test_route": test_route})


if __name__ == "__main__":
  from unittest_parallel.main import main

  main(["-j0", "--level", "class", "-s", str(Path(__file__).parent), "-p", Path(__file__).name])
