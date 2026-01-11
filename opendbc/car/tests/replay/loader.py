import pickle
import zstandard as zstd
from pathlib import Path


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
