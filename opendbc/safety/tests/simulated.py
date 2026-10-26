"""Virtual-time transport exercising the same C hooks used by Panda firmware.

Time and health adaptation live here; observable scenarios never inspect private
safety state or manipulate the firmware clock.
"""
from opendbc.safety.tests.hardware import _accepted_ramp_batches, _forward_scan_plan, _identity, _rx_group_plan, _scan_chunks, _steering_plan
from opendbc.safety.tests.libsafety import libsafety_py


# init_tests resets these firmware globals; changing safety mode must preserve them.
libsafety_py.ffi.cdef('extern bool ignition_can; extern uint32_t ignition_can_cnt;')


class SimulatedBackend:
  def __init__(self):
    self.safety = libsafety_py.libsafety
    self.scan_observations = []
    self.rx_group_observations = []
    self.reset(0)

  def restart(self):
    """Restart lifecycle, including ignition globals, rather than select a mode."""
    # A new image also resets function-local static ignition counters. The
    # ordinary test initializer cannot model a complete MCU restart.
    self.safety = libsafety_py.ffi.dlopen(libsafety_py._build_libsafety())
    self.reset(0)

  def reset(self, mode, param=0):
    assert self.safety.set_safety_hooks(mode, param) == 0
    ignition = (self.safety.ignition_can, self.safety.ignition_can_cnt) if hasattr(self, 'now') else None
    self.safety.init_tests()
    if ignition is not None:
      self.safety.ignition_can, self.safety.ignition_can_cnt = ignition
    self.now = 0
    self.next_tick = 1_000_000
    self.active_streams = {}
    self.invalid = 0
    self.streams_paused = False

  def _clock(self, now):
    self.now = now
    self.safety.set_timer(now % (2 ** 32))

  @staticmethod
  def _packet(frame):
    addr, data, bus = frame
    return libsafety_py.make_CANPacket(addr, bus, bytes.fromhex(data))

  def streams(self, streams):
    for spec in streams:
      assert spec['frames'] and spec['period'] > 0
      previous = self.active_streams.get(spec['name'])
      index = previous['index'] if previous else 0
      if previous is None:
        assert self.rx(spec['frames'][index % len(spec['frames'])]), spec['name']
        index += 1
      self.active_streams[spec['name']] = dict(spec, index=index,
                                             next=previous['next'] if previous else self.now + round(spec['period'] * 1_000_000),
                                             paused=previous.get('paused', False) if previous else False)

  def rx_group(self, events):
    plan = _rx_group_plan(events, {name: stream['frames'] for name, stream in self.active_streams.items()})
    # The station worker runs independently between RPCs. Give already-due
    # background inputs that same opportunity before the next global pause.
    # Do not advance time or dispatch held/future inputs.
    if not self.streams_paused:
      self.wait(0)
    paused = self.streams_paused
    self.streams_paused = True
    record = {'events': plan, 'frames': [], 'complete': False, 'source': 'simulated_virtual_time', 'physical': False}
    self.rx_group_observations.append(record)
    try:
      self.wait(.020)
      before = self._guard(None, True)
      record['health_before'] = before
      for index, event in enumerate(plan):
        if index:
          self.wait(.001)
        self.streams(event['updates'])
        name = event['stream_name']
        if name is None:
          frame = event['frame']
        else:
          stream = self.active_streams[name]
          frame = stream['frames'][stream['index'] % len(stream['frames'])]
        self.safety.safety_fwd_hook(frame[2], frame[0])
        valid = self.rx(frame) if name is None else self.rx_stream(name)
        record['frames'].append(list(frame))
        assert valid, f'RX group contains invalid input: {frame}'
      self.wait(.020)
      after = self._guard(None, True)
      record['health_after'] = after
      assert after['safety_rx_invalid'] == before['safety_rx_invalid'], 'RX group invalid counter changed'
      record['complete'] = True
      return True
    except Exception as exc:
      record['error'] = f'{type(exc).__name__}: {exc}'
      raise
    finally:
      self.streams_paused = paused
      if not paused:
        for stream in self.active_streams.values():
          if not stream.get('paused', False):
            stream['next'] = max(self.now, stream['next'])

  def rx_stream(self, name):
    stream = self.active_streams[name]
    frame = stream['frames'][stream['index'] % len(stream['frames'])]
    stream['index'] += 1
    stream['next'] = self.now + round(stream['period'] * 1_000_000)
    return self.rx(frame)

  def pause_stream(self, name):
    self.active_streams[name]['paused'] = True

  def resume_stream(self, name):
    stream = self.active_streams[name]
    stream['paused'] = False
    stream['next'] = max(self.now, stream['next'])

  def drop_stream(self, name):
    del self.active_streams[name]

  def rx(self, frame):
    packet = self._packet(frame)
    accepted = bool(self.safety.safety_rx_hook(packet))
    self.safety.ignition_can_hook(packet)
    self.invalid += not accepted
    return accepted

  def health(self):
    return {'safety_mode': int(self.safety.get_current_safety_mode()),
            'safety_param': int(self.safety.get_current_safety_param()),
            'ignition_can': bool(self.safety.get_ignition_can()),
            'controls_allowed': bool(self.safety.get_controls_allowed()),
            'safety_rx_checks_invalid': bool(self.safety.safety_rx_checks_invalid),
            'safety_rx_invalid': self.invalid, 'faults': int(self.safety.get_relay_malfunction())}

  def set_alternative_experience(self, value):
    self.safety.set_alternative_experience(value)

  def set_safety_mode(self, mode, param=0):
    self.safety.set_safety_hooks(mode, param)

  def _guard(self, controls, healthy):
    health = self.health()
    if controls is not None:
      assert health['controls_allowed'] == controls, health
    if healthy:
      assert not health['safety_rx_checks_invalid'], health
      assert not health['faults'], health
    return health

  def tx(self, frame, controls=True, healthy=True):
    before = self._guard(controls, healthy)
    accepted = bool(self.safety.safety_tx_hook(self._packet(frame)))
    after = self._guard(controls, healthy)
    if healthy:
      assert before['safety_rx_invalid'] == after['safety_rx_invalid']
    return accepted

  def steering_sequence(self, events, max_lateness_us=2000):
    """Virtual-time C verdicts; this report makes no physical timing claims."""
    plan = _steering_plan(events, max_lateness_us)
    return self._steering_sequence(plan)

  def _steering_sequence(self, plan, *, scheduling='strict'):
    identities = {_identity(event['frame']) for event in plan}
    sources = {_identity(frame) for stream in self.active_streams.values() for frame in stream['frames']}
    assert not identities & sources, 'Input stream emits sequence command identity'
    before = self._guard(True, True)
    start = self.now
    verdicts = []
    for event in plan:
      self.wait((start + event['offset_us'] - self.now) / 1_000_000)
      verdicts.append(self.tx(event['frame']))
    self.wait(.020)
    after = self._guard(True, True)
    assert after['safety_rx_invalid'] == before['safety_rx_invalid'], (before, after)
    return {'verdicts': verdicts,
            'timing': {'source': 'simulated_virtual_time', 'physical': False, 'complete': True,
                       'events': plan, 'start_us': start, 'quiet_end_us': self.now, 'scheduling': scheduling}}

  def accepted_ramp(self, frames, spacing_us):
    """Real C verdicts at virtual cadence; never physical timing evidence."""
    batches = _accepted_ramp_batches(frames, spacing_us)
    sources = {_identity(frame) for stream in self.active_streams.values() for frame in stream['frames']}
    identities = {_identity(event['frame']) for batch in batches for event in batch}
    assert not identities & sources, 'Input stream emits ramp command identity'
    verdicts, reports = [], []
    for index, batch in enumerate(batches):
      if index:
        self.wait(spacing_us / 1_000_000)
      result = self._steering_sequence(batch, scheduling='paced_scan')
      reports.append(result['timing'])
      assert all(result['verdicts']), f'Setup ramp command rejected: {result}'
      verdicts.extend(result['verdicts'])
    return {'verdicts': verdicts, 'timing': reports}

  def rx_scan(self, frames):
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
    paused = self.streams_paused
    self.streams_paused = True
    try:
      self._guard(None, False)
      for chunk in chunks:
        start = self.now
        record = {'kind': kind, 'frames': chunk, 'source': 'simulated_virtual_time', 'physical': False,
                  'health_before': self.health(), 'events': [], 'complete': False}
        self.scan_observations.append(record)
        records.append(record)
        for i, frame in enumerate(chunk):
          self.wait((start + i * 10000 - self.now) / 1_000_000)
          if kind == 'tx':
            verdict = self.tx(frame, controls=None, healthy=False)
            verdicts.append(verdict)
            record['events'].append({'offset_us': i * 10000, 'frame': frame, 'accepted': verdict})
          else:
            # The stock driver routes first even when RX subsequently rejects
            # the frame or latches a relay fault. Ignition always sees the RX.
            actual = self.safety.safety_fwd_hook(frame[2], frame[0])
            self.rx(frame)
            record['events'].append({'offset_us': i * 10000, 'frame': frame, 'forward_bus': actual})
            if kind == 'forward':
              target = targets[cursor + i]
              assert actual == (-1 if target is None else target), f'Scan forwarding mismatch: {frame}, actual={actual}, expected={target}'
        self.wait(.020)
        cursor += len(chunk)
        record['health_after'] = self._guard(None, False)
        record['complete'] = True
      return dict(self._guard(None, False), scan_evidence=records), verdicts
    finally:
      self.streams_paused = paused
      if not paused:
        for stream in self.active_streams.values():
          if not stream.get('paused', False):
            stream['next'] = max(self.now, stream['next'])

  def forward(self, frame, expected_bus, controls=None, healthy=True, rx_valid=True):
    """Check routing independently of RX validity; None leaves RX validity unspecified."""
    self._guard(controls, healthy)
    actual = self.safety.safety_fwd_hook(frame[2], frame[0])
    valid = self.rx(frame)
    self._guard(controls, healthy)
    if rx_valid is not None:
      assert valid == rx_valid, f'Forwarded input RX validity mismatch: actual={valid}, expected={rx_valid}'
    return actual == (-1 if expected_bus is None else expected_bus)

  def wait(self, seconds):
    end = self.now + round(seconds * 1_000_000)
    while True:
      event = min([self.next_tick] + [s['next'] for s in self.active_streams.values() if not self.streams_paused and not s.get('paused', False)])
      if event > end:
        break
      self._clock(event)
      for stream in self.active_streams.values():
        if not self.streams_paused and not stream.get('paused', False) and stream['next'] <= event:
          assert self.rx(stream['frames'][stream['index'] % len(stream['frames'])]), stream['name']
          stream['index'] += 1
          stream['next'] += round(stream['period'] * 1_000_000)
      if self.next_tick <= event:
        self.safety.safety_tick()
        self.next_tick += 1_000_000
    self._clock(end)

  def close(self):
    self.active_streams.clear()
