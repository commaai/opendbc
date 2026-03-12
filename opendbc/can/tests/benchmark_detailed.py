#!/usr/bin/env python3
"""Detailed benchmark breaking down mypyc vs Python paths."""
import time
import sys

from opendbc.can import CANPacker, CANParser
from opendbc.can.packer import set_value
from opendbc.can.parser import get_raw_value, MessageState
from opendbc.can._types import Signal, SignalType


def check_compiled():
  """Check which modules are mypyc-compiled vs interpreted."""
  from opendbc.can import _types, packer, parser, _vldict
  modules = {
    '_types': _types,
    'packer': packer,
    'parser': parser,
    '_vldict': _vldict,
  }
  for name, mod in modules.items():
    so = hasattr(mod, '__loader__') and 'extension' in type(mod.__loader__).__name__.lower()
    # mypyc-compiled modules have a __mypyc_attrs__ or similar marker
    # simplest check: see if the .so file is loaded
    origin = getattr(mod, '__file__', '') or ''
    compiled = origin.endswith('.so')
    print(f"  {name:12s}: {'COMPILED (C)' if compiled else 'INTERPRETED (Python)'}  ({origin.split('/')[-1]})")


def bench_get_raw_value(n=500_000):
  sig = Signal("TEST", 7, 7, 0, 8, False, 1.0, 0.0, False)
  dat = b'\xAB\xCD\xEF\x01\x23\x45\x67\x89'
  t1 = time.perf_counter_ns()
  for _ in range(n):
    get_raw_value(dat, sig)
  t2 = time.perf_counter_ns()
  return (t2 - t1) / n


def bench_set_value(n=500_000):
  sig = Signal("TEST", 7, 7, 0, 8, False, 1.0, 0.0, False)
  msg = bytearray(8)
  t1 = time.perf_counter_ns()
  for _ in range(n):
    set_value(msg, sig, 0xAB)
  t2 = time.perf_counter_ns()
  return (t2 - t1) / n


def bench_message_state_parse(n=200_000):
  sigs = [
    Signal("SIG1", 7, 7, 0, 8, False, 1.0, 0.0, False),
    Signal("SIG2", 15, 15, 8, 8, False, 1.0, 0.0, False),
    Signal("SIG3", 23, 23, 16, 8, True, 0.1, -10.0, False),
  ]
  state = MessageState(address=0x100, name="TEST", size=8, signals=sigs)
  dat = b'\xAB\xCD\xEF\x01\x23\x45\x67\x89'
  t1 = time.perf_counter_ns()
  for _ in range(n):
    state.parse(0, dat)
  t2 = time.perf_counter_ns()
  return (t2 - t1) / n


def bench_pack(n=100_000):
  packer = CANPacker('toyota_new_mc_pt_generated')
  t1 = time.perf_counter_ns()
  for _ in range(n):
    packer.make_can_msg("ACC_CONTROL", 0, {"ACC_TYPE": 1, "ALLOW_LONG_PRESS": 3})
  t2 = time.perf_counter_ns()
  return (t2 - t1) / n


def bench_parse(n=100_000):
  packer = CANPacker('toyota_new_mc_pt_generated')
  parser = CANParser('toyota_new_mc_pt_generated', [('ACC_CONTROL', 10)], 0)
  msgs = []
  for i in range(n):
    msg = packer.make_can_msg("ACC_CONTROL", 0, {"ACC_TYPE": 1, "ALLOW_LONG_PRESS": 3})
    msgs.append([int(0.01 * i * 1e9), [msg]])

  t1 = time.perf_counter_ns()
  for m in msgs:
    parser.update([m])
  t2 = time.perf_counter_ns()
  return (t2 - t1) / n


if __name__ == "__main__":
  print("Module compilation status:")
  check_compiled()
  print()

  # Warm up
  bench_get_raw_value(1000)
  bench_set_value(1000)
  bench_message_state_parse(1000)

  print("Per-call timings (ns):")
  print(f"  get_raw_value:       {bench_get_raw_value():8.0f} ns")
  print(f"  set_value:           {bench_set_value():8.0f} ns")
  print(f"  MessageState.parse:  {bench_message_state_parse():8.0f} ns  (3 signals)")
  print(f"  CANPacker.pack:      {bench_pack():8.0f} ns  (full msg w/ checksum)")
  print(f"  CANParser.update:    {bench_parse():8.0f} ns  (full msg w/ update)")
