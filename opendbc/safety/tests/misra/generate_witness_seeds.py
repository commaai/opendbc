#!/usr/bin/env python3
"""Generates witness_seeds.txt, the call sequences misrable's check of MISRA
C:2012 Rules 2.1 and 14.3 starts from (test_misra.sh passes --witness-seeds).

misrable executes the safety code from its entry points to show that code is
reachable and that conditions are variant. Seeds only guide that search:
misrable executes every seed call again on its model of the target, so a seed
cannot hide a finding. This script

1. runs the safety tests and records every call of an entry point
   (external_api.txt) with its arguments and the timer value it reads; test
   helpers that set state directly are not entry points and are left out, so
   a recorded sequence is what the entry points alone do (before a test
   allows controls or sets the last steering torque, the car's cruise
   status engages them and the torque ramps up to it);
2. has misrable replay the recording and search on from it, for the release
   and ALLOW_DEBUG configurations, writing back (--write-witness-seeds) the
   shortest sequences that cover what the search covered; rounds that start
   from the previous round's sequences search further.

Run it after changing the safety code or its tests:

  opendbc/safety/tests/misra/generate_witness_seeds.py

Seed format: one call per line, the entry point's name and the call's input
in hex: the argument bytes in parameter order (a pointer argument: the bytes
of the object it points to), then the bytes of each volatile read, in order.
A line '---' starts a sequence from the initial state.
"""
import argparse
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
TESTS = HERE.parent
ROOT = TESTS.parents[2]
OUT = HERE / "witness_seeds.txt"


def record(path: Path) -> None:
  """Runs the safety tests and writes their entry point calls to path."""
  os.environ["SAFETY_COVERAGE"] = "1"  # no UBSan runtime needed
  from opendbc.safety.tests.libsafety import libsafety_py
  libsafety_py.ffi.cdef("extern uint32_t timer_cnt;")
  libsafety_py.load(libsafety_py._build_libsafety())
  lib = libsafety_py.libsafety
  ffi = libsafety_py.ffi
  # CANPacket_t is packed and aligned(4); cffi's declaration leaves out the
  # alignment and the padding it adds at the end
  packet_size = ffi.sizeof("CANPacket_t")
  padding = bytes(-packet_size % 4)

  seqs: list[list[str]] = []

  def emit(name: str, data: bytes) -> None:
    seqs[-1].append(f"{name} {data.hex()}")

  def timer() -> bytes:
    return struct.pack("<I", lib.timer_cnt)

  def packet(msg) -> bytes:
    return bytes(ffi.buffer(msg, packet_size)) + padding

  # the input of each entry point call, after the call
  inputs = {
    "safety_rx_hook": lambda msg: packet(msg) + timer(),
    "safety_tx_hook": lambda msg: packet(msg) + timer(),
    "ignition_can_hook": packet,
    "safety_tick": lambda: timer(),
    "safety_fwd_hook": lambda bus, addr: struct.pack("<ii", bus, addr),
    "set_safety_hooks": lambda mode, param: struct.pack("<HH", mode, param),
    "safety_heartbeat": lambda engaged: struct.pack("<B", bool(engaged)),
    "safety_alternative_experience": lambda mode: struct.pack("<i", mode),
    "safety_heartbeat_check": lambda: b"",
    "ignition_can_reset": lambda: b"",
    "get_longitudinal_allowed": lambda: b"",
  }
  # test helpers that call entry points
  helpers = {
    "set_alternative_experience": lambda mode: [("safety_alternative_experience", struct.pack("<i", mode))],
    "init_tests": lambda: [("safety_alternative_experience", struct.pack("<i", 0)), ("ignition_can_reset", b"")],
  }

  test = None  # the running test
  ramps = 0  # torque ramps recorded in it

  def engage() -> None:
    """Records the car engaging before a test allows controls directly, so
    the entry points alone reach the state the test builds on. The messages
    are only recorded: the test runs on the state it expects."""
    if lib.get_controls_allowed():
      return
    try:
      msgs = [test._pcm_status_msg(False), test._pcm_status_msg(True)]
    except Exception:  # no PCM cruise status message in this test
      return
    try:  # Hyundai engages only after a recent button press
      msgs.insert(0, test._button_msg(0, main_button=1))
    except Exception:
      pass
    for msg in msgs:
      if msg is not None:
        emit("safety_rx_hook", inputs["safety_rx_hook"](msg))

  def ramp(torque: int) -> None:
    """Records the steering torque ramping up within the rate limits before
    a test sets the last torque directly, at 50 Hz from the current time
    (for the first few times in a test, or since it called init_tests)."""
    nonlocal ramps
    step = getattr(test, "MAX_RATE_UP", 0)
    if torque == 0 or step <= 0 or not lib.get_controls_allowed() or ramps >= 3:
      return
    ramps += 1
    t = lib.timer_cnt
    for i in range(1, abs(torque) // step + 1):
      value = i * step if torque > 0 else -i * step
      t = (t + 20000) & 0xFFFFFFFF
      try:
        if hasattr(test, "_torque_meas_msg"):
          emit("safety_rx_hook", packet(test._torque_meas_msg(value)) + struct.pack("<I", t))
        emit("safety_tx_hook", packet(test._torque_cmd_msg(value, steer_req=1)) + struct.pack("<I", t))
      except Exception:  # no torque messages in this test
        return

  class Recorder:
    def __getattr__(self, name):
      attr = getattr(lib, name)
      if not isinstance(attr, ffi.CData) or ffi.typeof(attr).kind != "function":
        return attr

      def call(*args):
        nonlocal ramps
        if name == "set_controls_allowed" and args[0]:
          engage()
        elif name == "set_desired_torque_last":
          ramp(args[0])
        elif name == "init_tests":
          ramps = 0
        ret = attr(*args)
        if name in inputs:
          emit(name, inputs[name](*args))
        elif name in helpers:
          for entry, data in helpers[name](*args):
            emit(entry, data)
        return ret
      return call

    def __setattr__(self, name, value):
      setattr(lib, name, value)

  libsafety_py.libsafety = Recorder()
  from opendbc.safety.tests import common
  common.state_buffer = lambda copy: ffi.buffer(ffi.addressof(lib, copy))

  # every test starts a sequence from the initial state
  run = unittest.TestCase.run

  def run_recorded(self, result=None):
    nonlocal test, ramps
    test, ramps = self, 0
    seqs.append([f"# {self.id()}"])
    return run(self, result)
  unittest.TestCase.run = run_recorded

  suite = unittest.defaultTestLoader.discover(str(TESTS), top_level_dir=str(ROOT))
  # the messages recorded for test helpers can make a test's later counter
  # or checksum checks fail; the recording is still a guide
  result = unittest.TextTestRunner(verbosity=0, stream=open(os.devnull, "w")).run(suite)
  print(f"recorded {len(seqs)} tests, {len(result.failures) + len(result.errors)} failed")
  with open(path, "w") as f:
    for s in seqs:
      if len(s) > 1:
        f.write("---\n" + "\n".join(s) + "\n")


def search(misrable: str, seeds: list[Path], out: Path, flags: list[str]) -> None:
  """Has misrable replay seeds, search on, and write the corpus to out."""
  cmd = [misrable, "-I", str(ROOT), "--rules", "2.1,14.3",
         "--external-api", str(HERE / "external_api.txt"),
         *[a for s in seeds for a in ("--witness-seeds", str(s))],
         "--write-witness-seeds", str(out), *flags, str(HERE / "main.c")]
  # exits 1 when there are findings, which the seeds are not about
  if subprocess.run(cmd, stdout=subprocess.DEVNULL).returncode not in (0, 1):
    sys.exit(f"misrable failed: {' '.join(cmd)}")


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--rounds", type=int, default=2, help="search rounds per configuration")
  parser.add_argument("--misrable", default=os.environ.get("MISRABLE", "misrable"))
  args = parser.parse_args()
  misrable = shutil.which(args.misrable) or sys.exit(f"{args.misrable} not found")

  sys.path.insert(0, str(ROOT))
  with tempfile.TemporaryDirectory() as tmp:
    recorded = Path(tmp) / "recorded.txt"
    record(recorded)
    parts = []
    for name, flags in (("release", []), ("debug", ["-DALLOW_DEBUG"])):
      seeds = [recorded]
      for i in range(args.rounds):
        out = Path(tmp) / f"{name}{i}.txt"
        search(misrable, seeds, out, flags)
        seeds = [out]
        print(f"{name} round {i + 1}: {out.read_text().count('---')} sequences")
      parts.append(seeds[0].read_text())
    OUT.write_text("".join(parts))


if __name__ == "__main__":
  main()
