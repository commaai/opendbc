import os
os.environ['LOGPRINT'] = 'CRITICAL'

import pickle
import zstandard as zstd
from pathlib import Path

TOLERANCE = 1e-2

CARSTATE_FIELDS = [
  "vEgo", "aEgo", "vEgoRaw", "yawRate", "standstill",
  "gasPressed", "brake", "brakePressed", "regenBraking", "parkingBrake", "brakeHoldActive",
  "steeringAngleDeg", "steeringAngleOffsetDeg", "steeringRateDeg", "steeringTorque", "steeringTorqueEps",
  "steeringPressed", "steerFaultTemporary", "steerFaultPermanent",
  "stockAeb", "stockFcw", "stockLkas", "espDisabled", "espActive", "accFaulted",
  "cruiseState.enabled", "cruiseState.available", "cruiseState.speed", "cruiseState.standstill",
  "cruiseState.nonAdaptive", "cruiseState.speedCluster",
  "gearShifter", "leftBlinker", "rightBlinker", "genericToggle",
  "doorOpen", "seatbeltUnlatched", "leftBlindspot", "rightBlindspot",
  "canValid", "canTimeout",
]


def get_value(obj, field):
  for p in field.split("."):
    obj = getattr(obj, p, None)
  return obj.raw if hasattr(obj, "raw") else obj


def differs(v1, v2):
  if isinstance(v1, float) and isinstance(v2, float):
    return abs(v1 - v2) > TOLERANCE
  return v1 != v2


def save_ref(path, states, timestamps):
  data = list(zip(timestamps, states, strict=True))
  Path(path).write_bytes(zstd.compress(pickle.dumps(data)))


def load_ref(path):
  return pickle.loads(zstd.decompress(Path(path).read_bytes()))


def load_can_messages(seg):
  from opendbc.car.can_definitions import CanData
  from openpilot.selfdrive.pandad import can_capnp_to_list
  from openpilot.tools.lib.logreader import LogReader
  from openpilot.tools.lib.comma_car_segments import get_url

  parts = seg.split("/")
  url = get_url(f"{parts[0]}/{parts[1]}", parts[2])

  can_msgs = []
  for msg in LogReader(url):
    if msg.which() == "can":
      can = can_capnp_to_list((msg.as_builder().to_bytes(),))[0]
      can_msgs.append((can[0], [CanData(*c) for c in can[1]]))
  return can_msgs


def replay_segment(platform, can_msgs):
  from opendbc.car import gen_empty_fingerprint, structs
  from opendbc.car.car_helpers import FRAME_FINGERPRINT, interfaces

  fingerprint = gen_empty_fingerprint()
  for _, frames in can_msgs[:FRAME_FINGERPRINT]:
    for msg in frames:
      if msg.src < 64:
        fingerprint[msg.src][msg.address] = len(msg.dat)

  CarInterface = interfaces[platform]
  car_interface = CarInterface(CarInterface.get_params(platform, fingerprint, [], False, False, False))
  car_control = structs.CarControl().as_reader()

  states = []
  timestamps = []
  for ts, frames in can_msgs:
    states.append(car_interface.update([(ts, frames)]))
    car_interface.apply(car_control, ts)
    timestamps.append(ts)
  return states, timestamps


def process_segment(args):
  platform, seg, ref_path, update = args
  try:
    can_msgs = load_can_messages(seg)
    states, timestamps = replay_segment(platform, can_msgs)
    ref_file = Path(ref_path) / f"{platform}_{seg.replace('/', '_')}.zst"

    if update:
      save_ref(ref_file, states, timestamps)
      return (platform, seg, [], None)

    if not ref_file.exists():
      return (platform, seg, [], "no ref")

    ref = load_ref(ref_file)
    diffs = [(field, i, get_value(ref_state, field), get_value(state, field), ts)
             for i, ((ts, ref_state), state) in enumerate(zip(ref, states, strict=True))
             for field in CARSTATE_FIELDS
             if differs(get_value(state, field), get_value(ref_state, field))]
    return (platform, seg, diffs, None)
  except Exception as e:
    return (platform, seg, [], str(e))
