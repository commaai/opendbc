"""Physical CAN backend. Every verdict comes from stock firmware and wire evidence."""
import json
import queue
import shlex
import subprocess
import threading
import time


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

  def call(self, op, **kwargs):
    with self.lock:
      self.process.stdin.write(json.dumps(dict(op=op, **kwargs)) + '\n')
      self.process.stdin.flush()
      try:
        response = self.responses.get(timeout=self.timeout)
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
    self.mode = None
    self.param = 0
    self.evidence = {'rx_physical': 0, 'tx_physical': 0, 'tx_rejected': 0,
                     'forward_physical': 0, 'forward_blocked': 0}

  def health(self):
    health = self.dut.call('health')
    self._validate_health(health)
    return health

  def _validate_health(self, health):
    if hasattr(self, '_overflow_baseline'):
      for key, baseline in self._overflow_baseline.items():
        assert health[key] == baseline, f'DUT {key} changed: {baseline} -> {health[key]}'
    if hasattr(self, '_expected_rx_invalid'):
      assert health['safety_rx_invalid'] == self._expected_rx_invalid, health

  def provenance(self):
    return self.dut.call('provenance')

  def reset(self, mode, param=0):
    # Configuration may drain stale traffic from the preceding mode. Establish
    # evidence baselines only after that lifecycle transition has settled.
    self.station.call('clear', reset_stats=True)
    self.dut.call('mode', mode=0)
    self.dut.call('clear')
    self.dut.call('mode', mode=int(mode), param=int(param))
    time.sleep(.1)
    self.dut.call('clear')
    self.station.call('clear', reset_stats=True)
    health = self.dut.call('health')
    self._overflow_baseline = {key: health[key] for key in ('rx_buffer_overflow', 'tx_buffer_overflow')}
    self._expected_rx_invalid = health['safety_rx_invalid']
    self.mode, self.param = int(mode), int(param)
    self._check(None, False)

  def _check(self, controls, healthy):
    health = self.health()
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

  def _receive(self, expected):
    pending = list(expected)
    deadline = time.monotonic() + self.timeout
    while pending and time.monotonic() < deadline:
      for frame in self.dut.call('recv'):
        if frame in pending:
          pending.remove(frame)
          self.evidence['rx_physical'] += 1
      if pending:
        time.sleep(.001)
    if pending:
      raise TimeoutError('DUT did not physically receive frames: ' + repr(pending))
    return list(expected)

  def streams(self, streams):
    self.dut.call('recv')
    before = self.health()['safety_rx_invalid']
    emitted = self.station.call('streams', streams=streams)
    observed = self._receive(emitted)
    after = self.health()['safety_rx_invalid']
    assert after == before, f'Stream update rejected by safety RX: {before} -> {after}'
    return observed

  def drop_stream(self, name):
    self.station.call('drop', name=name)

  def rx(self, frame):
    frame = self._frame(frame)
    self.station.call('pause')
    try:
      # Let already transmitted periodic inputs finish crossing the physical
      # link before attributing an invalid-counter change to this injection.
      time.sleep(.020)
      self.dut.call('recv')
      before = self.health()['safety_rx_invalid']
      self.station.call('send', frame=frame)
      self._receive([frame])
      health = self.dut.call('health')
      delta = health['safety_rx_invalid'] - before
      assert delta in (0, 1), f'Ambiguous safety RX invalid delta: {delta}'
      self._expected_rx_invalid = before + delta
      self._validate_health(health)
      return delta == 0
    finally:
      self.station.call('resume')

  def tx(self, frame, controls=True, healthy=True):
    frame = self._frame(frame)
    before = self._check(controls, healthy)
    self.dut.call('recv')
    self.station.call('recv')
    self.dut.call('send', frame=frame)
    addr, data, bus = frame
    accepted = observed = rejected = False
    quiet_deadline = None
    deadline = time.monotonic() + self.timeout
    while time.monotonic() < deadline:
      received = self.dut.call('recv')
      accepted |= [addr, data, 128 + bus] in received
      rejected |= [addr, data, 192 + bus] in received
      observed |= frame in self.station.call('recv')
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
    raise TimeoutError('TX lacks rejection receipt or accepted receipt plus physical observation')

  def forward(self, frame, expected_bus, controls=None, healthy=True):
    frame = self._frame(frame)
    if expected_bus is not None and (expected_bus not in (0, 1, 2) or expected_bus == frame[2]):
      raise ValueError('Forwarding target must be a different physical bus')
    before = self._check(controls, healthy)
    self.dut.call('recv')
    self.station.call('recv')
    self.station.call('send', frame=frame)
    self._receive([frame])
    observed = set()
    deadline = time.monotonic() + (.020 if expected_bus is None else self.timeout)
    quiet_deadline = None
    while True:
      for addr, data, bus in self.station.call('recv'):
        if [addr, data] == frame[:2] and bus != frame[2]:
          observed.add(bus)
      expected = set() if expected_bus is None else {expected_bus}
      assert not observed - expected, f'Unexpected physical forwarding buses: {observed}'
      now = time.monotonic()
      if expected_bus is not None and observed == expected and quiet_deadline is None:
        quiet_deadline = now + .020
      if (expected_bus is None and now >= deadline) or (quiet_deadline is not None and now >= quiet_deadline):
        after = self._check(controls, healthy)
        if healthy:
          assert after['safety_rx_invalid'] == before['safety_rx_invalid'], (before, after)
        self.evidence['forward_blocked' if expected_bus is None else 'forward_physical'] += 1
        return True
      if now >= deadline and quiet_deadline is None:
        raise TimeoutError('Expected forwarding frame was not physically observed')
      time.sleep(.001)

  def wait(self, seconds):
    time.sleep(seconds)
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
