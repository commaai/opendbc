from opendbc.car import *
from opendbc.car.structs import car
import pytest
# pytest attempts to execute shell scripts while collecting

collect_ignore_glob = [
  "opendbc/safety/tests/misra/*.sh",
  "opendbc/safety/tests/misra/cppcheck/",
]

from concurrent.futures import ThreadPoolExecutor
import xdist.workermanage as wm

# def setup_nodes_concurrent(self, putevent):
#     self.config.hook.pytest_xdist_setupnodes(config=self.config, specs=self.specs)
#     self.trace("setting up nodes (concurrent)")
#     t = time.monotonic()
#     with ThreadPoolExecutor(max_workers=len(self.specs)) as pool:
#         futs = [pool.submit(self.setup_node, spec, putevent) for spec in self.specs]
#         ret = [f.result() for f in futs]
#         print(f"setup_nodes_concurrent took {time.monotonic() - t:.3f} seconds")
#         return ret

# wm.NodeManager.setup_nodes = setup_nodes_concurrent

# conftest.py  (fixed)
import time, pytest

_T0 = None        # will hold the perf‑counter at startup
_PHASES = {}      # elapsed durations we actually care about


# ---------- bootstrap ----------
def pytest_cmdline_main(config):
    global _T0
    _T0 = time.perf_counter()          # mark very first moment


# ---------- collection ----------
def pytest_sessionstart(session):
    now = time.perf_counter()
    _PHASES['collection'] = now - _T0
    _PHASES['_run_start'] = now        # internal: start of test run


# ---------- tests done ----------
def pytest_sessionfinish(session, exitstatus):
    end = time.perf_counter()
    _PHASES['tests'] = end - _PHASES['_run_start']
    _PHASES['total'] = end - _T0
    _PHASES.pop('_run_start', None)    # remove internal key



@pytest.hookimpl(trylast=True)
def pytest_terminal_summary(terminalreporter, exitstatus, config):
    tr = terminalreporter
    tr.section("pytest overhead")
    for k, v in _PHASES.items():
        tr.write_line(f"{k:>10}: {v:8.3f} s")
