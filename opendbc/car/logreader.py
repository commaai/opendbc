#!/usr/bin/env python3
import subprocess
from pathlib import Path
from urllib.request import urlopen
import capnp


def _zstd_decompress(dat):
  proc = subprocess.run(['zstd', '-d'], input=dat, capture_output=True, check=True)
  return proc.stdout


class LogReader:
  def __init__(self, fn, sort_by_time=False):
    if fn.startswith("http"):
      with urlopen(fn) as f:
        dat = f.read()
    else:
      dat = Path(fn).read_bytes()

    if dat.startswith(b'\x28\xB5\x2F\xFD'):
      dat = _zstd_decompress(dat)

    rlog = capnp.load(str(Path(__file__).parent / "rlog.capnp"))
    self._ents = list(rlog.Event.read_multiple_bytes(dat))

    if sort_by_time:
      self._ents.sort(key=lambda x: x.logMonoTime)

  def __iter__(self):
    yield from self._ents

  def filter(self, msg_type: str):
    return (getattr(m, m.which()) for m in filter(lambda m: m.which() == msg_type, self))

  def first(self, msg_type: str):
    return next(self.filter(msg_type), None)
