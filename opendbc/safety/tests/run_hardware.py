"""Run the original safety tests on stock Panda and a CAN station."""
import argparse
from collections import Counter, defaultdict, deque
import copy
import datetime
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import unittest
from unittest.mock import patch

from opendbc.safety.tests import common
from opendbc.safety.tests.hardware import HardwareBackend
from opendbc.safety.tests.coverage import method_coverage


def _cases(suite):
  for test in suite:
    if isinstance(test, unittest.TestSuite):
      yield from _cases(test)
    else:
      yield test


def _definition_reason(test):
  """Recognize declared fixture definitions without executing setUpClass.

  Never interpret a caught SkipTest (or an empty skip reason) as abstractness:
  concrete fixtures can skip for real semantic or hardware reasons.
  """
  cls = type(test)
  if not isinstance(test, common.SafetyTestBase) or getattr(cls, '__unittest_skip__', False):
    return None
  if cls.__dict__.get('__test__') is False:
    return 'Parameterized fixture template declares __test__=False; generated subclasses declare True'
  # This is the documented SafetyTestBase.setUpClass contract, not a generic
  # rule that every unittest class named Base is abstract.
  if cls.__module__ == common.__name__ or cls.__name__.endswith('Base'):
    return 'Shared fixture definition excluded by SafetyTestBase.setUpClass contract'
  return None


def _partition_cases(cases):
  concrete, definitions = [], []
  for test in cases:
    reason = _definition_reason(test)
    if reason is None:
      concrete.append(test)
    else:
      definitions.append({'id': test.id(), 'kind': 'definition-only', 'reason': reason})
  return concrete, definitions


class _HostOnlyTest(unittest.TestCase):
  def __init__(self, test_id, reason):
    super().__init__('runTest')
    self.test_id, self.reason = test_id, reason

  def id(self):
    return self.test_id

  def __str__(self):
    return self.test_id

  def runTest(self):
    raise RuntimeError(self.reason)


class _RecordedResult(unittest.TextTestResult):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.executed_ids = []
    self.executions = []
    self.current_execution = None

  def startTest(self, test):
    self.executed_ids.append(test.id())
    self.current_execution = {'id': test.id(), 'status': 'passed', 'execution_index': len(self.executions)}
    self.executions.append(self.current_execution)
    super().startTest(test)

  def stopTest(self, test):
    # Preserve partial counts on failures too. Only copy the small coverage
    # manifest; the backend's full scan/sequence audit stays in hardware_runs.
    self.current_execution['scan_coverage'] = copy.deepcopy(getattr(test, '_scan_coverage', None))
    super().stopTest(test)
    self.current_execution = None

  def addError(self, test, err):
    if self.current_execution is not None:
      self.current_execution['status'] = 'error'
    super().addError(test, err)

  def addFailure(self, test, err):
    if self.current_execution is not None:
      self.current_execution['status'] = 'failure'
    super().addFailure(test, err)

  def addSkip(self, test, reason):
    if self.current_execution is not None:
      self.current_execution['status'] = 'skipped'
    super().addSkip(test, reason)

  def addExpectedFailure(self, test, err):
    self.current_execution['status'] = 'expected-failure'
    super().addExpectedFailure(test, err)

  def addUnexpectedSuccess(self, test):
    self.current_execution['status'] = 'unexpected-success'
    super().addUnexpectedSuccess(test)

  def addSubTest(self, test, subtest, err):
    if err is not None:
      self.current_execution['status'] = 'failure' if issubclass(err[0], test.failureException) else 'error'
    super().addSubTest(test, subtest, err)


def main(argv=None):
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--suite', default='legacy', choices=['legacy'], help='Compatibility option; the original suite is always selected')
  parser.add_argument('--test', action='append', default=[], help='Full legacy unittest test ID (repeatable)')
  parser.add_argument('--list', action='store_true', help='List selected test IDs without opening hardware')
  parser.add_argument('--dut-command', help='JSON-line hardware_service dut command (SSH permitted)')
  parser.add_argument('--station-command', help='JSON-line hardware_service station command')
  parser.add_argument('--failfast', action='store_true', help='Stop on the first test or transport failure')
  parser.add_argument('--json-output', help='Write results and run provenance to this file')
  args = parser.parse_args(argv)
  if not args.list and (not args.dut_command or not args.station_command):
    parser.error('--dut-command and --station-command are required unless --list is used')

  loader = unittest.TestLoader()
  directory = Path(__file__).resolve().parent
  discovered = list(_cases(loader.discover(str(directory), pattern='test_*.py',
                                         top_level_dir=str(directory.parents[2]))))
  discovered_ids = [test.id() for test in discovered]
  if args.test:
    unknown = set(args.test) - set(discovered_ids)
    if unknown:
      parser.error('Unknown discovered test IDs: ' + ', '.join(sorted(unknown)))
    requested = [test for test in discovered if test.id() in set(args.test)]
  else:
    requested = discovered
  selected, excluded_definitions = _partition_cases(requested)
  _, definition_inventory = _partition_cases(discovered)
  requested_ids = [test.id() for test in requested]
  selected_ids = [test.id() for test in selected]
  if args.list:
    for test_id in selected_ids:
      print(test_id)
    if excluded_definitions:
      print(f'{len(excluded_definitions)} definition-only entries excluded; concrete IDs listed above', file=sys.stderr)
    return 0 if selected_ids else 1

  # Non-fixture tests can compile/load host hooks or test the runner itself.
  # Keep their original IDs and make unsupported physical execution an error.
  classifications = {}
  runnable = []
  for test in selected:
    if not isinstance(test, common.SafetyTestBase):
      module = type(test).__module__
      kind = ('host-only-build' if module.endswith('.test_release_build') else
              'unmigrated-direct-hooks' if module.endswith('.test_can_scenarios') else
              'host-only-internal-tooling')
      reason = f'{kind}: cannot execute physically through the legacy SafetyTestBase backend'
      classifications[test.id()] = {'kind': kind, 'reason': reason}
      # Preserve unittest's original import/collection error rather than hiding it.
      runnable.append(test if isinstance(test, unittest.loader._FailedTest) else _HostOnlyTest(test.id(), reason))
    else:
      classifications[test.id()] = {'kind': 'physical-backend-required'}
      runnable.append(test)
  runs = []
  active_test = [None, None]

  def make_backend():
    backend = HardwareBackend(args.dut_command, args.station_command)
    try:
      provenance = backend.provenance()
    except Exception:
      backend.close()
      raise
    runs.append({'test_id': active_test[0], 'execution_index': active_test[1], 'provenance': provenance,
                 'evidence': backend.evidence,
                 # Backend retains these append-only lists across reset/grace
                 # transitions, including partial evidence on failed scans.
                 'scan_observations': getattr(backend, 'scan_observations', []),
                 'sequence_observations': getattr(backend, 'sequence_observations', []),
                 'rx_group_observations': getattr(backend, 'rx_group_observations', []),
                 'transport_observations': getattr(backend, 'transport_observations', [])})
    return backend

  def legacy_backend(case, selection):
    if selection != 'hardware':
      raise RuntimeError('Legacy hardware runner requires SAFETY_BACKEND=hardware')
    return make_backend()

  class Result(_RecordedResult):
    def startTest(self, test):
      active_test[0] = test.id()
      super().startTest(test)
      active_test[1] = self.current_execution['execution_index']

  started = datetime.datetime.now(datetime.UTC).isoformat()
  # Scoped overrides avoid credentials in environment variables and restore all
  # hooks even if discovery/execution fails in a caller using main repeatedly.
  with patch.dict(os.environ, {'SAFETY_BACKEND': 'hardware'}), \
       patch.object(common.SafetyTestBase, 'backend_factory', legacy_backend):
    result = unittest.TextTestRunner(verbosity=2, failfast=args.failfast, resultclass=Result).run(unittest.TestSuite(runnable))
  executed_ids = result.executed_ids
  successful = (result.wasSuccessful() and not result.skipped and not result.expectedFailures and not loader.errors
                and bool(selected_ids) and len(executed_ids) == len(selected_ids))
  if args.json_output:
    revision = subprocess.run(['git', 'rev-parse', 'HEAD'], check=True, capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(['git', 'status', '--porcelain'], check=True, capture_output=True, text=True).stdout)

    def events(items):
      return [(test.id(), detail) for test, detail in items]

    errors, failures, skipped = events(result.errors), events(result.failures), events(result.skipped)
    records = []
    executions = defaultdict(deque)
    for execution in result.executions:
      executions[execution['id']].append(execution)
    occurrences = Counter()
    for test in requested:
      test_id = test.id()
      definition_reason = _definition_reason(test)
      classification = ({'kind': 'definition-only', 'reason': definition_reason} if definition_reason
                        else classifications[test_id])
      occurrences[test_id] += 1
      execution = executions[test_id].popleft() if executions[test_id] else None
      class_id = test_id.rsplit('.', 1)[0]
      module_id = class_id.rsplit('.', 1)[0]
      fixture_events = [(name, detail) for name, detail in errors + skipped
                        if name.endswith(('(' + class_id + ')', '(' + module_id + ')'))]
      status = 'definition-only' if definition_reason else execution['status'] if execution else 'not-executed'
      evidence = [run['evidence'] for run in runs
                  if execution is not None and run['execution_index'] == execution['execution_index']]
      can_observed = any(any(value for value in counters.values()) for counters in evidence)
      # Traffic may come entirely from setup. It is never an assertion-coverage verdict.
      scope = 'backend-can-evidence-present' if can_observed else 'backend-without-can-evidence' if evidence else 'no-hardware-execution'
      assertion_coverage = method_coverage(test)
      relation = assertion_coverage['relation']
      assertion_coverage['shared_execution'] = ('unclassified' if relation == 'unclassified' else
                                                'not-implemented' if relation == 'host-only' else
                                                'passed' if status == 'passed' else 'not-completed')
      equivalent = status == 'passed' and relation == 'equivalent' and not assertion_coverage['host_supplements']
      records.append({'id': test_id, 'occurrence': occurrences[test_id], **classification,
                      'selected': definition_reason is None,
                      'execution_index': execution['execution_index'] if execution else None,
                      'assertion_coverage': assertion_coverage, 'original_assertion_equivalence': equivalent,
                      'host_only_supplements': assertion_coverage['host_supplements'],
                      'scan_coverage': execution.get('scan_coverage') if execution else None,
                      'executed': execution is not None, 'status': status, 'fixture_events': fixture_events,
                      'not_executed_reason': (None if execution else definition_reason or
                                              ('class/module fixture event' if fixture_events else 'failfast or fixture failure')),
                      'hardware_backend_created': bool(evidence), 'execution_scope': scope,
                      'can_evidence_observed': can_observed})
    selected_records = [record for record in records if record['selected']]
    selected_equivalence = successful and all(record['original_assertion_equivalence'] for record in selected_records)
    report = {'suite': args.suite, 'profile': None,
              'backend': 'stock-panda', 'started': started, 'revision': revision, 'dirty_tree': dirty,
              'python': platform.python_version(), 'discovered_ids': discovered_ids, 'requested_ids': requested_ids,
              'definition_inventory': definition_inventory, 'excluded_definitions': excluded_definitions,
              'selected_ids': selected_ids,
              'accounting': {'discovered': len(discovered_ids), 'requested': len(requested_ids),
                             'definition_only': len(excluded_definitions), 'concrete_selected': len(selected_ids),
                             'executed': len(executed_ids), 'semantic_skip_events': len(skipped),
                             'unclassified_original_assertion_tests': sum(r['assertion_coverage']['relation'] == 'unclassified'
                                                                        for r in selected_records),
                             'tests_with_host_supplements': sum(bool(r['host_only_supplements']) for r in selected_records)},
              'executed_ids': executed_ids, 'not_executed_ids': [record['id'] for record in records if record['selected'] and not record['executed']],
              'discovery_errors': loader.errors, 'test_records': records, 'hardware_runs': runs,
              'tests_run': result.testsRun, 'failures': failures, 'errors': errors, 'skipped': skipped,
              'expected_failures': events(result.expectedFailures),
              'unexpected_successes': [test.id() for test in result.unexpectedSuccesses],
              'execution_successful': successful, 'successful': successful,
              'selected_original_assertion_equivalence': selected_equivalence,
              'original_suite_equivalence': selected_equivalence and args.suite == 'legacy' and not args.test}
    # Tracebacks from transports may contain the supplied command; never persist
    # command strings (which may embed credentials) in the run artifact.
    redactions = [json.dumps(command)[1:-1] for command in (args.dut_command, args.station_command) if command]
    with open(args.json_output, 'w') as output:
      # iterencode emits each JSON string as a complete chunk, allowing command
      # redaction without materializing a second full copy of a large raw audit.
      for chunk in json.JSONEncoder(indent=2).iterencode(report):
        for command in redactions:
          chunk = chunk.replace(command, '[REDACTED HARDWARE COMMAND]')
        output.write(chunk)
      output.write('\n')

  return 0 if successful else 1


if __name__ == '__main__':
  sys.exit(main())
