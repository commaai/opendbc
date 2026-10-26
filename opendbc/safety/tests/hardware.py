"""Physical CAN backend. Every verdict comes from stock firmware and wire evidence."""
import collections
import json
import math
import queue
import shlex
import subprocess
import threading
import time

from opendbc.safety.tests.hardware_sequence import _frame as _sequence_frame


def _steering_plan(events, max_lateness_us):
  if type(max_lateness_us) is not int or not 0 <= max_lateness_us <= 10000:
    raise ValueError('max_lateness_us must be between 0 and 10000')
  plan, identities = [], set()
  for event in events:
    if set(event) != {'offset_us', 'frame'}:
      raise ValueError('Only scheduled TX frame events are supported')
    offset = event['offset_us']
    if type(offset) is not int or not 0 <= offset <= 5000000 or (plan and offset <= plan[-1]['offset_us']):
      raise ValueError('Offsets must strictly increase within five seconds')
    frame = _sequence_frame(event['frame'])
    if frame in identities:
      raise ValueError('Duplicate sequence command identity')
    identities.add(frame)
    plan.append({'offset_us': offset, 'frame': list(frame)})
    if len(plan) > 1000:
      raise ValueError('Sequence exceeds 1000 commands')
  if not plan:
    raise ValueError('Empty sequence')
  return plan


def _scan_chunks(frames):
  """Normalize the whole domain before I/O; repeated identities start a new batch."""
  chunks, chunk, identities = [], [], set()
  for raw in frames:
    frame = list(_sequence_frame(raw))
    identity = _identity(frame)
    if len(chunk) == 256 or identity in identities:
      chunks.append(chunk)
      chunk, identities = [], set()
    chunk.append(frame)
    identities.add(identity)
  if chunk:
    chunks.append(chunk)
  return chunks


def _accepted_ramp_batches(frames, spacing_us):
  # paced_scan currently admits gaps >=10ms and nominal offsets <=5 seconds.
  if type(spacing_us) is not int or not 10000 <= spacing_us <= 5000000:
    raise ValueError('Ramp spacing_us must be between 10000 and 5000000')
  chunks = _scan_chunks(frames)
  if not chunks:
    raise ValueError('Empty accepted ramp')
  limit = min(256, 5000000 // spacing_us + 1)
  return [[{'offset_us': i * spacing_us, 'frame': frame} for i, frame in enumerate(chunk[start:start + limit])]
          for chunk in chunks for start in range(0, len(chunk), limit)]


def _rx_group_plan(events, sources):
  events = list(events)
  if not 1 <= len(events) <= 32:
    raise ValueError('RX group requires 1..32 events')
  plan, cycles = [], dict(sources)
  for event in events:
    if set(event) != {'frame', 'updates', 'stream_name'}:
      raise ValueError('RX group event requires frame, updates, stream_name')
    frame = None if event['frame'] is None and event['stream_name'] is not None else \
      list(_sequence_frame(event['frame']))
    updates = []
    for spec in event['updates']:
      if set(spec) != {'name', 'period', 'frames'} or spec['name'] not in cycles:
        raise ValueError('RX group updates require existing streams')
      if type(spec['period']) not in (int, float) or not math.isfinite(spec['period']) or spec['period'] <= 0:
        raise ValueError('RX group stream period must be positive and finite')
      frames = [list(_sequence_frame(raw)) for raw in spec['frames']]
      if not frames:
        raise ValueError('RX group stream requires frames')
      updates.append(dict(spec, frames=frames))
      cycles[spec['name']] = frames
    name = event['stream_name']
    if name is not None and name not in cycles:
      raise ValueError('RX group injection requires existing stream')
    plan.append({'frame': frame, 'updates': updates, 'stream_name': name})
  return plan


def _forward_scan_plan(cases):
  frames, targets = [], []
  for frame, target in cases:
    normalized = list(_sequence_frame(frame))
    if target is not None and (type(target) is not int or target not in (0, 1, 2) or target == normalized[2]):
      raise ValueError('Forwarding target must be a different physical bus')
    frames.append(normalized)
    targets.append(target)
  return frames, targets


def _identity(frame):
  addr, data, bus = frame
  return addr, bytes.fromhex(data).hex(), bus


class RPC:
  def __init__(self, command, timeout=10):
    self.timeout = timeout
    self.lock = threading.Lock()
    self.responses = queue.Queue(maxsize=2)
    self.process = subprocess.Popen(shlex.split(command) if isinstance(command, str) else command,
                                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
    self.reader = threading.Thread(target=self._read, daemon=True)
    self.reader.start()

  def _read(self):
    try:
      for line in self.process.stdout:
        self.responses.put(json.loads(line), timeout=self.timeout)
      self.responses.put({'ok': False, 'error': 'Service closed'}, timeout=self.timeout)
    except Exception as exc:
      try:
        self.responses.put({'ok': False, 'error': str(exc)}, timeout=self.timeout)
      except queue.Full:
        pass

  def call(self, op, *, rpc_timeout=None, **kwargs):
    with self.lock:
      self.process.stdin.write(json.dumps(dict(op=op, **kwargs)) + '\n')
      self.process.stdin.flush()
      try:
        response = self.responses.get(timeout=self.timeout if rpc_timeout is None else rpc_timeout)
      except queue.Empty:
        self.process.kill()
        raise TimeoutError('Hardware RPC timed out: ' + op) from None
      if not response['ok']:
        raise RuntimeError(response['error'])
      return response.get('result')

  def close(self):
    if self.process.poll() is None:
      self.process.stdin.close()
      try:
        self.process.wait(timeout=3)
      except subprocess.TimeoutExpired:
        self.process.kill()
        self.process.wait(timeout=3)
    self.process.stdout.close()


class HardwareBackend:
  def __init__(self, dut_command, station_command, timeout=2):
    self.dut = RPC(dut_command)
    try:
      self.station = RPC(station_command)
    except Exception:
      self.dut.close()
      raise
    self.timeout = timeout
    self._command_lock = threading.Lock()
    self._station_streams = {}
    self.sequence_observations = []
    self.scan_observations = []
    self.rx_group_observations = []
    self.transport_observations = []
    self.mode = None
    self.param = 0
    self.evidence = {'rx_physical': 0, 'tx_physical': 0, 'tx_rejected': 0,
                     'forward_physical': 0, 'forward_blocked': 0}

  def health(self):
    return self._transport_call('dut', 'health', validate_health=True)

  def _validate_health(self, health):
    if hasattr(self, '_overflow_baseline'):
      for key, baseline in self._overflow_baseline.items():
        assert health[key] == baseline, f'DUT {key} changed: {baseline} -> {health[key]}'
    if hasattr(self, '_expected_rx_invalid'):
      assert health['safety_rx_invalid'] == self._expected_rx_invalid, health

  def provenance(self):
    return self.dut.call('provenance')

  def set_alternative_experience(self, value):
    self.dut.call('configure', setting='alternative_experience', value=int(value))
    health = self.health()
    assert health['alternative_experience'] == value, health

  def set_safety_mode(self, mode, param=0):
    """Request a mode and retain the firmware's reported fallback, if any."""
    self.dut.call('mode', mode=int(mode), param=int(param))
    health = self.health()
    self.mode, self.param = health['safety_mode'], health['safety_param']

  def restart(self):
    """Normal stock MCU reset; callers select the next safety mode explicitly."""
    with self._command_lock:
      self.station.call('clear')
      self._station_streams.clear()
      result = self.dut.call('restart', rpc_timeout=30)
      for name in ('_overflow_baseline', '_expected_rx_invalid'):
        if hasattr(self, name):
          delattr(self, name)
      self.mode, self.param = None, 0
      self.station.call('clear')
      return result

  def reset(self, mode, param=0):
    # Configuration may drain stale traffic from the preceding mode. Establish
    # evidence baselines only after that lifecycle transition has settled.
    self.station.call('clear', reset_stats=True)
    self._station_streams = {}
    self.dut.call('mode', mode=0)
    self.dut.call('clear')
    self.dut.call('mode', mode=int(mode), param=int(param))
    time.sleep(.1)
    self.dut.call('clear')
    self.station.call('clear', reset_stats=True)
    health = self._transport_call('dut', 'health')
    self._overflow_baseline = {key: health[key] for key in ('rx_buffer_overflow', 'tx_buffer_overflow')}
    self._expected_rx_invalid = health['safety_rx_invalid']
    self.mode, self.param = int(mode), int(param)
    self._check(None, False)

  def _check(self, controls, healthy):
    health = self.health()
    self._check_snapshot(health, controls, healthy)
    return health

  def _check_snapshot(self, health, controls, healthy):
    assert (health['safety_mode'], health['safety_param']) == (self.mode, self.param), health
    assert not health.get('heartbeat_disabled', False), health
    if controls is not None:
      assert bool(health['controls_allowed']) == controls, health
    if healthy:
      assert not health['safety_rx_checks_invalid'], health
      assert not health.get('faults', 0), health
    return health

  @staticmethod
  def _frame(frame):
    addr, data, bus = frame
    if isinstance(data, bytes):
      data = data.hex()
    if bus not in (0, 1, 2):
      raise ValueError('No physical CAN bus ' + str(bus))
    return [int(addr), data, int(bus)]

  def _recv(self, endpoint):
    """Drain without losing raw traffic unrelated to the current verdict."""
    return self._transport_call(endpoint, 'recv')

  def _transport_call(self, endpoint, operation, validate_health=False):
    # Host RPC bounds only: neither wire arrival nor MCU execution timestamps.
    if not hasattr(self, 'transport_observations'):
      self.transport_observations = []
    record = {'endpoint': endpoint, 'operation': operation, 'clock_scope': 'coordinator_monotonic',
              'start_monotonic_s': time.monotonic(), 'frames': []}
    try:
      result = (self.dut if endpoint == 'dut' else self.station).call(operation)
      record['frames' if operation == 'recv' else 'health'] = result
      if validate_health:
        self._validate_health(result)
      return result
    except Exception as exc:
      record['error'] = f'{type(exc).__name__}: {exc}'
      raise
    finally:
      record['end_monotonic_s'] = time.monotonic()
      self.transport_observations.append(record)

  def _collect(self, record, stage):
    record[stage] = {}
    for endpoint in ('dut', 'station'):
      try:
        record[stage][endpoint] = self._recv(endpoint)
      except Exception as exc:
        record.setdefault('collection_errors', []).append({'stage': stage, 'endpoint': endpoint, 'error': f'{type(exc).__name__}: {exc}'})
        raise
    return record[stage]

  def _receive(self, expected):
    pending = list(expected)
    deadline = time.monotonic() + self.timeout
    while pending and time.monotonic() < deadline:
      for frame in self._recv('dut'):
        if frame in pending:
          pending.remove(frame)
          self.evidence['rx_physical'] += 1
      if pending:
        time.sleep(.001)
    if pending:
      raise TimeoutError('DUT did not physically receive frames: ' + repr(pending))
    return list(expected)

  def streams(self, streams):
    self._recv('dut')
    self._recv('station')
    before = self.health()['safety_rx_invalid']
    emitted = self.station.call('streams', streams=streams)
    for spec in streams:
      self._station_streams[spec['name']] = [self._frame(frame) for frame in spec['frames']]
    observed = self._receive(emitted)
    after = self.health()['safety_rx_invalid']
    assert after == before, f'Stream update rejected by safety RX: {before} -> {after}'
    self._recv('station')
    return observed

  def pause_stream(self, name):
    self.station.call('pause_stream', name=name)

  def resume_stream(self, name):
    self.station.call('resume_stream', name=name)

  def drop_stream(self, name):
    self.station.call('drop', name=name)
    self._station_streams.pop(name, None)

  def rx(self, frame):
    return self._rx(frame=self._frame(frame))

  def rx_group(self, events):
    """All-valid RX only: exact emitted multiplicity and per-bus ordering."""
    plan = _rx_group_plan(events, self._station_streams)
    with self._command_lock:
      self.station.call('pause')
      record = {'events': plan, 'collection_errors': []}
      self.rx_group_observations.append(record)

      try:
        time.sleep(.020)
        self._collect(record, 'baseline')
        before = self._check(None, True)
        record['health_before'] = before
        # A partial failure can leave any submitted update active. Retain the
        # union conservatively for collision guards until reset or replacement.
        cycles = dict(self._station_streams)
        allowed = []
        for event in plan:
          for spec in event['updates']:
            cycles[spec['name']] = spec['frames']
            self._station_streams[spec['name']] = self._station_streams[spec['name']] + spec['frames']
          allowed.append(cycles[event['stream_name']] if event['stream_name'] is not None else [event['frame']])
        report = self.station.call('rx_group', events=plan)
        record['timing'] = report
        self._collect(record, 'received')
        record['health_after'] = self._transport_call('dut', 'health')
        assert report['complete'] is True, f'Incomplete RX group: {report}'
        self._station_streams = cycles
        emitted = [_identity(frame) for frame in report['frames']]
        assert len(emitted) == len(plan), 'RX group emitted unexpected frame count'
        for actual, candidates in zip(emitted, allowed, strict=True):
          assert actual in map(_identity, candidates), f'Unrequested RX group frame: {actual}'
        physical = [_identity(frame) for frame in record['received']['dut'] if frame[2] in (0, 1, 2)]
        assert collections.Counter(physical) == collections.Counter(emitted), 'RX group physical multiplicity mismatch'
        for bus in range(3):
          assert [frame for frame in physical if frame[2] == bus] == [frame for frame in emitted if frame[2] == bus], 'RX group bus order mismatch'
        after = record['health_after']
        self._validate_health(after)
        self._check_snapshot(after, None, True)
        assert after['safety_rx_invalid'] == before['safety_rx_invalid'], 'RX group contains invalid input'
        self._check(None, True)
        self.evidence['rx_physical'] += len(emitted)
        return True
      finally:
        self.station.call('resume')

  def rx_stream(self, name):
    return self._rx(stream=name)

  def _rx(self, frame=None, stream=None):
    self.station.call('pause')
    try:
      # Let already transmitted periodic inputs finish crossing the physical
      # link before attributing an invalid-counter change to this injection.
      time.sleep(.020)
      self._recv('dut')
      self._recv('station')
      before = self.health()['safety_rx_invalid']
      if stream is None:
        self.station.call('send', frame=frame)
      else:
        frame = self.station.call('rx_stream', name=stream)
      self._receive([frame])
      self._recv('station')
      health = self._transport_call('dut', 'health')
      delta = health['safety_rx_invalid'] - before
      assert delta in (0, 1), f'Ambiguous safety RX invalid delta: {delta}'
      self._expected_rx_invalid = before + delta
      self._validate_health(health)
      return delta == 0
    finally:
      self.station.call('resume')

  def tx(self, frame, controls=True, healthy=True):
    with self._command_lock:
      return self._tx(frame, controls, healthy)

  def _tx(self, frame, controls, healthy):
    frame = self._frame(frame)
    before = self._check(controls, healthy)
    self._recv('dut')
    self._recv('station')
    receive_start = len(self.transport_observations)
    self.dut.call('send', frame=frame)
    addr, data, bus = frame
    accepted = observed = rejected = False
    quiet_deadline = None
    deadline = time.monotonic() + self.timeout
    while time.monotonic() < deadline:
      received = self._recv('dut')
      accepted |= [addr, data, 128 + bus] in received
      rejected |= [addr, data, 192 + bus] in received
      observed |= frame in self._recv('station')
      assert not (rejected and (accepted or observed)), 'Rejected TX also had physical or accepted evidence'
      if rejected and quiet_deadline is None:
        quiet_deadline = time.monotonic() + .020
      if (accepted and observed) or (quiet_deadline is not None and time.monotonic() >= quiet_deadline):
        after = self._check(controls, healthy)
        if healthy:
          assert after['safety_rx_invalid'] == before['safety_rx_invalid'], (before, after)
        self.evidence['tx_rejected' if rejected else 'tx_physical'] += 1
        return not rejected
      time.sleep(.001)
    record = {'endpoint': 'dut', 'operation': 'tx_timeout', 'frames': [], 'frame': frame, 'receipt_bus': 128 + bus,
              'rejection_bus': 192 + bus, 'wire_bus': bus,
              'accepted': accepted, 'rejected': rejected, 'observed': observed,
              'receive_span': [receive_start, len(self.transport_observations)]}
    self.transport_observations.append(record)
    try:
      self.health()
    except Exception as exc:
      record['health_error'] = f'{type(exc).__name__}: {exc}'
    raise TimeoutError('TX lacks rejection receipt or accepted receipt plus physical observation')

  def steering_sequence(self, events, max_lateness_us=2000):
    """Run exclusive DUT commands with unchanged valid input streams and controls on.

    Queue snapshots retain unrelated evidence. Service timing is host dispatch
    timing, not wire or MCU timing; only stock receipts and wire counts decide TX.
    """
    plan = _steering_plan(events, max_lateness_us)
    return self._steering_sequence(plan, max_lateness_us)

  def _steering_sequence(self, plan, max_lateness_us, *, scheduling='strict'):
    identities = {_identity(event['frame']) for event in plan}
    sources = {_identity(frame) for frames in self._station_streams.values() for frame in frames}
    assert not identities & sources, 'Station stream emits sequence command identity'
    with self._command_lock:
      before = self._check(True, True)
      time.sleep(.020)
      baseline = {'dut': self._recv('dut'), 'station': self._recv('station')}
      record = {'baseline': baseline}
      self.sequence_observations.append(record)
      receipt_ids = {(a, d, bus + flag) for a, d, bus in identities for flag in (128, 192)}
      assert not receipt_ids & {_identity(frame) for frame in baseline['dut']}, 'Outstanding matching DUT receipt'
      assert not identities & {_identity(frame) for frame in baseline['dut']}, 'Outstanding matching DUT RX'
      assert not identities & {_identity(frame) for frame in baseline['station']}, 'Outstanding matching wire evidence'
      self._check(True, True)
      options = {'scheduling': scheduling} if scheduling != 'strict' else {}
      report = self.dut.call('sequence', events=plan, exclusive_commands=True, max_lateness_us=max_lateness_us, **options)
      record['timing'] = report
      # Always drain and retain post-run evidence, including failed/partial runs.
      received = self._collect(record, 'received')
      assert report['complete'] is True, f'Incomplete steering sequence: {report}'
      for health in (report['health_before'], report['health_after']):
        self._validate_health(health)
        self._check_snapshot(health, True, True)
        assert health['safety_rx_invalid'] == before['safety_rx_invalid'], (before, health)
      after = self._check(True, True)
      assert after['safety_rx_invalid'] == before['safety_rx_invalid'], (before, after)
      receipts = collections.Counter(_identity(frame) for frame in received['dut'])
      wire = collections.Counter(_identity(frame) for frame in received['station'])
      verdicts = []
      for event in plan:
        addr, data, bus = _identity(event['frame'])
        accepted, rejected = receipts[(addr, data, 128 + bus)], receipts[(addr, data, 192 + bus)]
        assert accepted + rejected == 1, f'Expected exactly one receipt: {event}, A={accepted}, R={rejected}'
        assert wire[(addr, data, bus)] == accepted, f'Wire/receipt count mismatch: {event}, A={accepted}, wire={wire[(addr, data, bus)]}'
        verdicts.append(bool(accepted))
      self.evidence['tx_physical'] += sum(verdicts)
      self.evidence['tx_rejected'] += len(verdicts) - sum(verdicts)
      return {'verdicts': verdicts, 'timing': report}

  def accepted_ramp(self, frames, spacing_us):
    """Accepted-only setup commands, paced from send completion with live inputs.

    This is not a real-time deadline test. Every exact input frame is retained
    in order, and every command must have accepted receipt plus wire evidence.
    """
    batches = _accepted_ramp_batches(frames, spacing_us)
    sources = {_identity(frame) for frames in self._station_streams.values() for frame in frames}
    identities = {_identity(event['frame']) for batch in batches for event in batch}
    assert not identities & sources, 'Station stream emits ramp command identity'
    verdicts, reports = [], []
    for index, batch in enumerate(batches):
      if index:
        # The preceding RPC completed after its last send and quiet interval;
        # this extra gap guarantees cadence across independently timed batches.
        time.sleep(spacing_us / 1_000_000)
      result = self._steering_sequence(batch, 10000, scheduling='paced_scan')
      reports.append(result['timing'])
      assert all(result['verdicts']), f'Setup ramp command rejected: {result}'
      verdicts.extend(result['verdicts'])
    return {'verdicts': verdicts, 'timing': reports}

  def rx_scan(self, frames):
    """Inject every input, returning actual final health and aggregate evidence.

    Deliberately invalid inputs are counted collectively, never assigned an
    invented per-frame RX verdict. No safety mode is changed by a scan.
    """
    return self._scan(frames, 'rx')[0]

  def forward_scan(self, cases):
    frames, targets = _forward_scan_plan(cases)
    self._scan(frames, 'forward', targets)
    return True

  def tx_scan(self, frames):
    return self._scan(frames, 'tx')[1]

  def _scan(self, frames, kind, targets=None):
    chunks = _scan_chunks(frames)
    verdicts, records, cursor = [], [], 0
    with self._command_lock:
      self.station.call('pause')
      try:
        time.sleep(.020)
        final = self._check(None, False)
        for chunk in chunks:
          baseline = {'dut': self._recv('dut'), 'station': self._recv('station')}
          before = self._check(None, False)
          record = {'kind': kind, 'frames': chunk, 'baseline': baseline, 'health_before': before}
          self.scan_observations.append(record)
          records.append(record)
          events = [{'offset_us': i * 10000, 'frame': frame} for i, frame in enumerate(chunk)]
          sender = self.dut if kind == 'tx' else self.station
          report = sender.call('sequence', events=events, exclusive_commands=True,
                               scheduling='paced_scan', max_duration_us=8000000)
          record['timing'] = report
          received = self._collect(record, 'received')
          health = self._transport_call('dut', 'health')
          record['health_after'] = health
          assert report['complete'] is True, f'Incomplete {kind} scan: {report}'
          self._check_snapshot(health, None, False)
          delta = health['safety_rx_invalid'] - before['safety_rx_invalid']
          assert 0 <= delta <= (0 if kind == 'tx' else len(chunk)), f'Ambiguous scan RX invalid delta: {delta}'
          # This adjustment is valid only while all background input is paused.
          self._expected_rx_invalid = before['safety_rx_invalid'] + delta
          self._validate_health(health)
          if kind == 'tx':
            for snapshot in (report['health_before'], report['health_after']):
              self._validate_health(snapshot)
              self._check_snapshot(snapshot, None, False)
          receipts = collections.Counter(_identity(frame) for frame in received['dut'])
          physical = collections.Counter(_identity(frame) for frame in received['dut'] if frame[2] in (0, 1, 2))
          wire = collections.Counter(_identity(frame) for frame in received['station'])
          if kind == 'tx':
            batch_verdicts, expected_wire, expected_receipts = [], collections.Counter(), collections.Counter()
            for addr, data, bus in map(_identity, chunk):
              accepted, rejected = receipts[(addr, data, 128 + bus)], receipts[(addr, data, 192 + bus)]
              assert accepted + rejected == 1, f'Expected one scan receipt: {(addr, data, bus)}, A={accepted}, R={rejected}'
              expected_receipts[(addr, data, (128 if accepted else 192) + bus)] += 1
              if accepted:
                expected_wire[(addr, data, bus)] += 1
              batch_verdicts.append(bool(accepted))
            assert receipts == expected_receipts, f'Unexpected/missing scan DUT receipts: {receipts - expected_receipts}'
            assert wire == expected_wire, f'Scan TX wire mismatch: expected={expected_wire}, received={wire}'
            verdicts.extend(batch_verdicts)
            self.evidence['tx_physical'] += sum(batch_verdicts)
            self.evidence['tx_rejected'] += len(chunk) - sum(batch_verdicts)
          else:
            expected_rx = collections.Counter(map(_identity, chunk))
            assert physical == expected_rx, f'Scan physical RX mismatch: expected={expected_rx}, received={physical}'
            if kind == 'forward':
              expected_wire = collections.Counter((a, d, target) for (a, d, _), target in
                                                  zip(map(_identity, chunk), targets[cursor:cursor + len(chunk)], strict=True)
                                                  if target is not None)
              # Local station TX echoes are already excluded by the service.
              # Do not drop a bus globally: another input may forward onto it.
              assert wire == expected_wire, f'Scan forwarding mismatch: expected={expected_wire}, received={wire}'
              count = sum(expected_wire.values())
              self.evidence['forward_physical'] += count
              self.evidence['forward_blocked'] += len(chunk) - count
            self.evidence['rx_physical'] += len(chunk)
          cursor += len(chunk)
          final = self._check(None, False)
        # Keep all raw baseline, partial and complete evidence for audit.
        return dict(final, scan_evidence=records), verdicts
      finally:
        self.station.call('resume')

  def forward(self, frame, expected_bus, controls=None, healthy=True, rx_valid=True):
    """Check routing independently of RX validity; None leaves RX validity unspecified."""
    frame = self._frame(frame)
    if expected_bus is not None and (expected_bus not in (0, 1, 2) or expected_bus == frame[2]):
      raise ValueError('Forwarding target must be a different physical bus')
    self.station.call('pause')
    try:
      time.sleep(.020)
      before = self._check(controls, healthy)
      self._recv('dut')
      self._recv('station')
      self.station.call('send', frame=frame)
      self._receive([frame])
      observed = set()
      deadline = time.monotonic() + (.020 if expected_bus is None else self.timeout)
      quiet_deadline = None
      while True:
        for addr, data, bus in self._recv('station'):
          if [addr, data] == frame[:2] and bus != frame[2]:
            observed.add(bus)
        expected = set() if expected_bus is None else {expected_bus}
        assert not observed - expected, f'Unexpected physical forwarding buses: {observed}'
        now = time.monotonic()
        if expected_bus is not None and observed == expected and quiet_deadline is None:
          quiet_deadline = now + .020
        if (expected_bus is None and now >= deadline) or (quiet_deadline is not None and now >= quiet_deadline):
          health = self._transport_call('dut', 'health')
          delta = health['safety_rx_invalid'] - before['safety_rx_invalid']
          assert delta in (0, 1), f'Ambiguous safety RX invalid delta: {delta}'
          self._expected_rx_invalid = before['safety_rx_invalid'] + delta
          self._validate_health(health)
          self._check(controls, healthy)
          if rx_valid is not None:
            assert (delta == 0) == rx_valid, f'Forwarded input RX validity mismatch: delta={delta}, expected={rx_valid}'
          self.evidence['forward_blocked' if expected_bus is None else 'forward_physical'] += 1
          return True
        if now >= deadline and quiet_deadline is None:
          raise TimeoutError('Expected forwarding frame was not physically observed')
        time.sleep(.001)
    finally:
      self.station.call('resume')

  def wait(self, seconds):
    if not math.isfinite(seconds) or seconds < 0:
      raise ValueError('Wait duration must be finite and nonnegative')
    deadline = time.monotonic() + seconds
    while True:
      # Both service workers continue receiving while the caller waits. Keep
      # their bounded queues drained, with every batch retained for audit.
      with self._command_lock:
        self._recv('dut')
        self._recv('station')
      remaining = deadline - time.monotonic()
      if remaining <= 0:
        break
      time.sleep(min(remaining, .1))
    self.health()

  def close(self):
    errors = []
    for action in (lambda: self.station.call('clear'), lambda: self.dut.call('mode', mode=0), self.dut.close, self.station.close):
      try:
        action()
      except Exception as exc:
        errors.append(str(exc))
    if errors:
      raise RuntimeError('Hardware cleanup failed: ' + '; '.join(errors))
