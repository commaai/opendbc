"""Run the shared observable safety scenarios on stock Panda and a CAN station."""
import argparse
import datetime
import json
import platform
import subprocess
import sys
import unittest

from opendbc.safety.tests import test_observable
from opendbc.safety.tests.hardware import HardwareBackend


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--dut-command', required=True, help='JSON-line hardware_service dut command (SSH permitted)')
  parser.add_argument('--station-command', required=True, help='JSON-line hardware_service station command')
  parser.add_argument('--profile', default='all', choices=['all', 'toyota', 'hyundai'])
  parser.add_argument('--failfast', action='store_true', help='Stop on the first test or transport failure')
  parser.add_argument('--json-output', help='Write results and run provenance to this file')
  args = parser.parse_args()
  runs = []

  def make_backend():
    backend = HardwareBackend(args.dut_command, args.station_command)
    try:
      provenance = backend.provenance()
    except Exception:
      backend.close()
      raise
    runs.append({'provenance': provenance, 'evidence': backend.evidence})
    return backend

  test_observable.backend_factory = make_backend
  suite = unittest.defaultTestLoader.loadTestsFromModule(test_observable)
  if args.profile != 'all':
    case = test_observable.TestToyotaObservable if args.profile == 'toyota' else test_observable.TestHyundaiObservable
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(case)
  started = datetime.datetime.now(datetime.UTC).isoformat()
  result = unittest.TextTestRunner(verbosity=2, failfast=args.failfast).run(suite)
  successful = result.wasSuccessful() and not result.skipped
  if args.json_output:
    revision = subprocess.run(['git', 'rev-parse', 'HEAD'], check=True, capture_output=True, text=True).stdout.strip()
    report = {'profile': args.profile, 'backend': 'stock-panda', 'started': started, 'revision': revision,
              'python': platform.python_version(),
              'hardware_runs': runs,
              'tests_run': result.testsRun, 'failures': [(str(t), err) for t, err in result.failures],
              'errors': [(str(t), err) for t, err in result.errors], 'skipped': [(str(t), why) for t, why in result.skipped],
              'successful': successful}
    with open(args.json_output, 'w') as output:
      json.dump(report, output, indent=2)
      output.write('\n')
  return 0 if successful else 1


if __name__ == '__main__':
  sys.exit(main())
