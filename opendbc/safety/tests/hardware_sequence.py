"""Coarse host-monotonic Panda / station TX scheduling; no MCU timing claims."""
import time


def _frame(raw):
  addr, data, bus = raw
  if type(addr) is not int or not 0 <= addr <= 0x1FFFFFFF or type(bus) is not int or bus not in range(3):
    raise ValueError('Invalid sequence CAN address/bus')
  if not isinstance(data, str):
    raise ValueError('Sequence payload must be hex text')
  payload = bytes.fromhex(data)
  if len(payload) not in (*range(9), 12, 16, 20, 24, 32, 48, 64):
    raise ValueError('Invalid CAN payload length')
  return addr, payload.hex(), bus


def execute_sequence(service, events, *, exclusive_commands=False, max_lateness_us=2000,
                     clock=time.monotonic_ns, sleep=time.sleep, scheduling='strict', max_duration_us=8000000, receipt_timeout_us=1000000):
  """Run on Service's dispatch thread, acquiring its lock only for I/O.

  Caller must establish a clean evidence baseline and guarantee no other sender
  (including background streams) emits these identities for the whole batch.
  Unique identities permit pipelining. Existing worker queues retain ALL receipts
  and background RX. Backend must collect both DUT and station queues after this
  returns and require one Panda receipt per event and one wire frame iff accepted.
  Missing evidence never implies rejection. This function supplies timing only.
  Explicit scheduling='paced_scan' uses completion-relative minimum gaps, never
  catches up, and audits nominal jitter without a per-event lateness failure.
  Its overall duration is bounded to <=8s (blocking I/O is ultimately bounded by
  coordinator RPC timeout). Strict steering semantics remain the default.
  Station scans require globally paused streams and >=10ms event spacing. The
  coordinator separately checks DUT health / physical RX evidence; station health
  fields stay None. Station transport checks run before/after and at each send.

  Returns complete=False plus error/partial timing on execution failure; callers
  MUST reject that result. All timestamps are service host monotonic nanoseconds.
  DUT completion requires all unique accepted/rejected receipts observed in the
  retained service queue within receipt_timeout_us after the last send. The local
  receipts_observed_ns cutoff upper-bounds MCU processing; it is conservative,
  not an MCU timestamp. quiet_end_ns follows this cutoff plus >=20ms and health.
  Final quiet period bounds wire absence evidence, not physical delivery.
  """
  if scheduling not in ('strict', 'paced_scan'):
    raise ValueError('Unsupported scheduling policy')
  paced = scheduling == 'paced_scan'
  if type(max_duration_us) is not int or not 20000 <= max_duration_us <= 8000000:
    raise ValueError('max_duration_us must be between 20000 and 8000000')
  if type(receipt_timeout_us) is not int or not 1000 <= receipt_timeout_us <= 1000000:
    raise ValueError('receipt_timeout_us must be between 1000 and 1000000')
  if exclusive_commands is not True:
    raise ValueError('Explicit exclusive_commands guarantee required')
  if service.role not in ('dut', 'station'):
    raise ValueError('Unsupported sequence service role')
  station = service.role == 'station'
  if station:
    with service.lock:
      if not service.streams_paused:
        raise ValueError('Station sequence requires globally paused streams')
  if type(max_lateness_us) is not int or not 0 <= max_lateness_us <= 10000:
    raise ValueError('max_lateness_us must be between 0 and 10000')
  plan = []
  identities = set()
  for event in events:
    if set(event) != {'offset_us', 'frame'}:
      raise ValueError('Only scheduled TX frame events are supported')
    offset = event['offset_us']
    if type(offset) is not int or not 0 <= offset <= 5000000 or (plan and offset <= plan[-1][0]):
      raise ValueError('Offsets must strictly increase within five seconds')
    if (station or paced) and plan and offset - plan[-1][0] < 10000:
      raise ValueError('Scan events require at least 10000us spacing')
    frame = _frame(event['frame'])
    if frame in identities:
      raise ValueError('Duplicate sequence command identity')
    identities.add(frame)
    plan.append((offset, frame))
    if len(plan) > 1000:
      raise ValueError('Sequence exceeds 1000 commands')
  if not plan:
    raise ValueError('Empty sequence')
  report = {'events': [], 'health_before': None, 'health_after': None,
            'start_ns': None, 'max_jitter_ns': 0, 'complete': False, 'scheduling': scheduling}
  budget_start = clock()

  def check_worker():
    if paced and clock() - budget_start >= max_duration_us * 1000:
      raise RuntimeError('Paced scan overall deadline exceeded')
    if station and not service.streams_paused:
      raise RuntimeError('Station streams resumed during sequence')
    if service.error or service.stopped.is_set():
      raise RuntimeError('CAN worker failed/stopped: ' + str(service.error))

  def wait_until(deadline):
    while clock() < deadline:
      with service.lock:
        check_worker()
      remaining = deadline - clock()
      if remaining > 0:
        sleep(min(remaining / 1e9, .001))

  try:
    with service.lock:
      check_worker()
      if station:
        service.check_transport()
      else:
        report['health_before'] = service.panda.health()
        if any((a, bytes.fromhex(d).hex(), b - base) in identities
               for a, d, b in service.queue for base in (128, 192)):
          raise RuntimeError('Stale sequence receipt in service queue')
    report['start_ns'] = clock()
    for offset, frame in plan:
      deadline = report['start_ns'] + offset * 1000
      target = deadline
      if paced and report['events']:
        previous = report['events'][-1]
        gap_ns = (offset - previous['offset_us']) * 1000
        target = max(deadline, previous['send_end_ns'] + gap_ns)
      wait_until(target)
      with service.lock:
        check_worker()
        if station:
          service.check_transport()
        started = clock()  # Include lock acquisition delay in dispatch jitter.
        jitter = started - deadline
        report['max_jitter_ns'] = max(report['max_jitter_ns'], jitter)
        if not paced and jitter > max_lateness_us * 1000:
          raise RuntimeError('Late sequence dispatch')
        event = {'offset_us': offset, 'frame': frame, 'send_start_ns': started, 'jitter_ns': jitter, 'target_ns': target}
        report['events'].append(event)
        service.send(frame)
        event['send_end_ns'] = clock()
        check_worker()
        if not paced and event['send_end_ns'] - deadline > max_lateness_us * 1000:
          raise RuntimeError('Late sequence send completion')
    quiet_start = report['events'][-1]['send_end_ns']
    if not station:
      receipt_deadline = quiet_start + receipt_timeout_us * 1000
      while True:
        with service.lock:
          check_worker()
          counts = {frame: 0 for frame in identities}
          for addr, data, bus in service.queue:
            for base in (128, 192):
              identity = (addr, bytes.fromhex(data).hex(), bus - base)
              if identity in counts:
                counts[identity] += 1
          if any(count > 1 for count in counts.values()):
            raise RuntimeError('Duplicate/conflicting local sequence receipts')
          if all(count == 1 for count in counts.values()):
            # Reading the retained queue proves the MCU produced every receipt
            # by this service-local time. No remote collection clock is involved.
            report['receipts_observed_ns'] = clock()
            if report['receipts_observed_ns'] > receipt_deadline:
              raise RuntimeError('Sequence receipt observation timed out')
            quiet_start = max(quiet_start, report['receipts_observed_ns'])
            break
        if clock() >= receipt_deadline:
          raise RuntimeError('Sequence receipt observation timed out')
        wait_until(min(receipt_deadline, clock() + 1000000))
    wait_until(quiet_start + 20000000)
    with service.lock:
      check_worker()
      if station:
        service.check_transport()
      else:
        report['health_after'] = service.panda.health()
      check_worker()
    report['quiet_end_ns'] = clock()
    report['complete'] = True
  except Exception as exc:
    report['error'] = f'{type(exc).__name__}: {exc}'
  report['elapsed_ns'] = clock() - budget_start
  return report


def execute_rx_group(service, events, *, clock=time.monotonic_ns, sleep=time.sleep):
  """Emit <=32 all-valid RX inputs; coordinator owns physical evidence/verdicts.

  Existing stream dispatch preserves counter phase and replacement semantics.
  Queues are retained. Runtime errors return partial ordered frames and timing;
  the caller must reject incomplete groups and resume streams in its finally.
  """
  if service.role != 'station':
    raise ValueError('rx_group is station-only')
  if not isinstance(events, list) or not 1 <= len(events) <= 32:
    raise ValueError('rx_group requires 1..32 events')
  with service.lock:
    if not service.streams_paused:
      raise ValueError('rx_group requires globally paused streams')
    for event in events:
      if set(event) != {'frame', 'updates', 'stream_name'}:
        raise ValueError('Invalid rx_group event schema')
      name = event['stream_name']
      if name is not None and (not isinstance(name, str) or name not in service.streams):
        raise ValueError('rx_group stream must already exist')
      if event['frame'] is not None or name is None:
        _frame(event['frame'])
      if not isinstance(event['updates'], list):
        raise ValueError('rx_group updates must be stream specs')
      for spec in event['updates']:
        if spec['name'] not in service.streams:
          raise ValueError('rx_group cannot create streams')
        period = spec['period']
        if type(period) not in (int, float) or not 0 < period < float('inf') or not spec['frames']:
          raise ValueError('Invalid rx_group stream spec')
        for frame in spec['frames']:
          _frame(frame)
  report = {'complete': False, 'frames': [], 'events': [], 'start_ns': clock()}
  deadline = report['start_ns'] + 1000000000

  def check():
    if clock() >= deadline:
      raise RuntimeError('rx_group overall deadline exceeded')
    if not service.streams_paused:
      raise RuntimeError('rx_group requires globally paused streams')
    if service.error or service.stopped.is_set():
      raise RuntimeError('CAN worker failed/stopped: ' + str(service.error))

  def wait_until(target):
    while clock() < target:
      with service.lock:
        check()
      remaining = target - clock()
      if remaining > 0:
        sleep(min(remaining / 1e9, .001))

  try:
    for event in events:
      if report['events']:
        wait_until(report['events'][-1]['send_end_ns'] + 1000000)
      with service.lock:
        check()
        service.check_transport()
        # Recheck admission under lock before replacement: never initialize a
        # stream if another producer removed it while the scheduler waited.
        if any(spec['name'] not in service.streams for spec in event['updates']):
          raise RuntimeError('rx_group stream removed during group')
        emitted = service.dispatch({'op': 'streams', 'streams': event['updates']})
        if emitted:
          raise RuntimeError('rx_group unexpectedly initialized streams')
        check()
        timing = {'send_start_ns': clock()}
        report['events'].append(timing)
        if event['stream_name'] is None:
          frame = list(_frame(event['frame']))
          service.dispatch({'op': 'send', 'frame': frame})
        else:
          frame = service.dispatch({'op': 'rx_stream', 'name': event['stream_name']})
        report['frames'].append(list(frame))
        timing['send_end_ns'] = clock()
        check()
    wait_until(report['events'][-1]['send_end_ns'] + 20000000)
    with service.lock:
      check()
      service.check_transport()
      check()
    report['quiet_end_ns'] = clock()
    report['complete'] = True
  except Exception as exc:
    report['error'] = f'{type(exc).__name__}: {exc}'
  report['elapsed_ns'] = clock() - report['start_ns']
  return report
