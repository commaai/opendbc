import os
os.environ['LOGPRINT'] = 'ERROR'

from pathlib import Path

from opendbc.car.tests.diff.compare import CARSTATE_FIELDS, get_value, differs
from opendbc.car.tests.diff.loader import load_can_messages, load_ref, save_ref


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
