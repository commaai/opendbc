"""Scheduler tests, including the real Service worker and stock receipt routing."""
import collections
import socket
import struct
import threading
import time
import unittest
from unittest.mock import patch

from opendbc.safety.tests.hardware_sequence import execute_rx_group, execute_sequence
from opendbc.safety.tests.hardware_service import Service


class PandaTransport:
  """Public-method transport harness, not an implementation of safety hooks."""
  def __init__(self):
    self.receipts = collections.deque()
    self.wire = []
    self.heartbeats = []

  def health(self):
    return {'controls_allowed': True, 'heartbeat_disabled': False}

  def send_heartbeat(self, controls):
    self.heartbeats.append(time.monotonic_ns())

  def can_send(self, addr, data, bus):
    accepted = data != b'\xff'
    self.receipts.append((addr, data, bus + (128 if accepted else 192)))
    if accepted:
      self.wire.append([addr, data.hex(), bus])

  def can_recv(self):
    result = list(self.receipts)
    self.receipts.clear()
    return result


def service(role='dut', paused=False):
  obj = Service.__new__(Service)
  obj.role = role
  obj.lock = threading.RLock()
  obj.stopped = threading.Event()
  obj.error = None
  if role == 'dut':
    obj.panda = PandaTransport()
  obj.fd_buses = set()
  obj.queue = collections.deque()
  obj.streams = {}
  obj.streams_paused = paused
  obj.net_stats = {}
  return obj


class Clock:
  def __init__(self, obj):
    self.ns = 0
    self.obj = obj

  def __call__(self):
    return self.ns

  def sleep(self, seconds):
    # Every deterministic scheduler wait must release the service lock.
    assert not self.obj.lock._is_owned()
    self.ns += round(seconds * 1e9)


def events(count=38):
  return [{'offset_us': i * 6000, 'frame': [0x123, bytes([i]).hex(), 0]} for i in range(count)]


class TestSequence(unittest.TestCase):
  def test_service_pipeline_keeps_worker_heartbeat_and_receipts(self):
    obj = service()
    worker = threading.Thread(target=obj.run)
    worker.start()
    try:
      batch = events()
      batch[-1]['frame'][1] = 'ff'
      result = obj.dispatch({'op': 'sequence', 'events': batch, 'exclusive_commands': True, 'max_lateness_us': 10000})
      self.assertTrue(result['complete'], result)
      self.assertEqual(len(result['events']), 38)
      self.assertGreaterEqual(len(obj.panda.heartbeats), 2)
      with obj.lock:
        receipts = list(obj.queue)
      self.assertEqual(len(receipts), 38)
      for event, receipt in zip(batch, receipts, strict=True):
        addr, data, bus = event['frame']
        self.assertEqual(receipt, [addr, data, bus + (192 if data == 'ff' else 128)])
      self.assertEqual(obj.panda.wire, [event['frame'] for event in batch[:-1]])
      self.assertGreaterEqual(result['quiet_end_ns'] - result['events'][-1]['send_end_ns'], 20000000)
    finally:
      obj.stopped.set()
      worker.join(timeout=2)

  def test_deterministic_six_ms_schedule_and_timestamps(self):
    obj = service()
    now = Clock(obj)

    def sleep(seconds):
      now.sleep(seconds)
      obj.queue.extend(obj.receive())
    result = execute_sequence(obj, events(), exclusive_commands=True, clock=now, sleep=sleep)
    self.assertTrue(result['complete'])
    self.assertEqual([event['send_start_ns'] for event in result['events']], [i * 6000000 for i in range(38)])
    self.assertEqual(result['max_jitter_ns'], 0)
    self.assertEqual(now.ns, 243000000)

  def test_late_schedule_aborts_without_retiming(self):
    obj = service()
    now = Clock(obj)

    def sleep(seconds):
      now.sleep(seconds)
      now.ns += 10000000
    result = execute_sequence(obj, events(3), exclusive_commands=True, clock=now, sleep=sleep)
    self.assertFalse(result['complete'])
    self.assertIn('Late sequence', result['error'])
    self.assertEqual(len(result['events']), 1)
    self.assertEqual(len(obj.panda.receipts), 1)

  def test_send_delay_and_worker_failure_abort(self):
    for failure in ('send', 'worker'):
      obj = service()
      now = Clock(obj)
      original = obj.send

      def send(frame, original=original, failure=failure, now=now, obj=obj):
        original(frame)
        if failure == 'send':
          now.ns += 3000000
        else:
          obj.error = 'receive queue overflow'
      obj.send = send
      result = execute_sequence(obj, events(3), exclusive_commands=True, clock=now)
      self.assertFalse(result['complete'])
      self.assertEqual(len(result['events']), 1)

  def test_station_socket_batch_keeps_worker_polling_and_streams_paused(self):
    obj = service('station', paused=True)
    local, peer = socket.socketpair(type=socket.SOCK_DGRAM)
    local.setblocking(False)
    peer.settimeout(.5)
    obj.sockets = [local]
    obj.streams = {'background': {'next': 0, 'period': .01, 'index': 0, 'frames': [[99, 'ff', 0]]}}
    worker = threading.Thread(target=obj.run)
    worker.start()
    try:
      # Preexisting evidence must survive scheduling; receive worker also drains
      # actual SocketCAN-layout datagrams while the dispatch thread waits.
      obj.queue.append([77, 'aa', 0])
      peer.send(struct.pack('=IB3x8s', 88, 1, b'b'))
      batch = [{'offset_us': i * 10000, 'frame': [0x100 + i, '00', 0]} for i in range(4)]
      result = obj.dispatch({'op': 'sequence', 'events': batch, 'exclusive_commands': True, 'max_lateness_us': 10000})
      self.assertTrue(result['complete'], result)
      self.assertIsNone(result['health_before'])
      self.assertIsNone(result['health_after'])
      packets = [struct.unpack('=IB3x8s', peer.recv(16)) for _ in batch]
      self.assertEqual([packet[0] for packet in packets], [0x100 + i for i in range(4)])
      self.assertEqual(obj.streams['background']['index'], 0)
      with obj.lock:
        self.assertEqual(list(obj.queue), [[77, 'aa', 0], [88, '62', 0]])
    finally:
      obj.stopped.set()
      worker.join(timeout=2)
      local.close()
      peer.close()

  def test_station_scan_preserves_500_addresses_at_100hz(self):
    obj = service('station', paused=True)
    now = Clock(obj)
    sent = []
    checks = []
    obj.check_transport = lambda: checks.append(now.ns)
    obj.send = lambda frame: sent.append(frame)
    batch = [{'offset_us': i * 10000, 'frame': [0x1000 + i, '00', i % 3]} for i in range(500)]

    result = execute_sequence(obj, batch, exclusive_commands=True, clock=now, sleep=now.sleep)
    self.assertTrue(result['complete'], result)
    self.assertEqual(sent, [tuple(event['frame']) for event in batch])
    self.assertEqual(len(checks), 502)
    self.assertEqual(now.ns, 5010000000)

  def test_station_pause_and_transport_guards(self):
    obj = service('station')
    with self.assertRaisesRegex(ValueError, 'paused'):
      execute_sequence(obj, events(1), exclusive_commands=True)
    obj.streams_paused = True
    with self.assertRaisesRegex(ValueError, 'spacing'):
      execute_sequence(obj, events(2), exclusive_commands=True)
    now = Clock(obj)
    obj.check_transport = lambda: None
    obj.send = lambda frame: None

    def sleep(seconds):
      now.sleep(seconds)
      obj.streams_paused = False
    result = execute_sequence(obj, events(1), exclusive_commands=True, clock=now, sleep=sleep)
    self.assertFalse(result['complete'])
    self.assertIn('resumed', result['error'])
    obj.streams_paused = True

    def loss():
      raise RuntimeError('Station transport loss')
    obj.check_transport = loss
    result = execute_sequence(obj, events(1), exclusive_commands=True)
    self.assertFalse(result['complete'])
    self.assertIn('transport loss', result['error'])
    self.assertEqual(result['events'], [])

  def test_paced_scan_accepts_slow_sends_and_jitter_without_catchup(self):
    for role in ('dut', 'station'):
      obj = service(role, paused=True)
      now = Clock(obj)
      sent = []

      def send(frame, now=now, sent=sent, obj=obj):
        sent.append((now.ns, frame))
        if obj.role == 'dut':
          obj.queue.append([frame[0], frame[1], frame[2] + 128])
        now.ns += 15000000  # Normal USB stalls exceed steering's 2ms budget.

      def sleep(seconds, now=now, obj=obj):
        now.sleep(seconds)
        now.ns += 4000000
      obj.send = send
      batch = [{'offset_us': i * 10000, 'frame': [i, '00', 0]} for i in range(12)]
      result = execute_sequence(obj, batch, exclusive_commands=True, scheduling='paced_scan',
                                clock=now, sleep=sleep)
      self.assertTrue(result['complete'], result)
      self.assertEqual([frame for _, frame in sent], [tuple(event['frame']) for event in batch])
      self.assertGreater(result['max_jitter_ns'], 10000000)
      for previous, event in zip(result['events'][:-1], result['events'][1:], strict=True):
        self.assertGreaterEqual(event['send_start_ns'] - previous['send_end_ns'], 10000000)
        self.assertGreaterEqual(event['target_ns'] - previous['send_end_ns'], 10000000)
      self.assertGreaterEqual(result['quiet_end_ns'] - result['events'][-1]['send_end_ns'], 20000000)

  def test_paced_scan_overall_deadline_and_transport_loss_still_fail(self):
    for failure in ('deadline', 'transport'):
      obj = service('station', paused=True)
      now = Clock(obj)
      calls = []

      def send(frame, now=now, calls=calls):
        calls.append(frame)
        now.ns += 25000000

      def check_transport(failure=failure, calls=calls):
        if failure == 'transport' and calls:
          raise RuntimeError('Station transport loss')

      obj.send = send
      obj.check_transport = check_transport
      result = execute_sequence(obj, [{'offset_us': i * 10000, 'frame': [i, '00', 0]} for i in range(5)],
                                exclusive_commands=True, scheduling='paced_scan', max_duration_us=60000,
                                clock=now, sleep=now.sleep)
      self.assertFalse(result['complete'])
      self.assertIn('deadline' if failure == 'deadline' else 'transport loss', result['error'])
      self.assertLess(len(calls), 5)

  def test_dut_cutoff_waits_for_delayed_local_receipts_and_retains_queue(self):
    obj = service()
    now = Clock(obj)
    obj.queue.append([99, 'aa', 0])
    batch = [{'offset_us': 0, 'frame': [1, '00', 0]}, {'offset_us': 6000, 'frame': [2, 'ff', 0]}]

    def sleep(seconds):
      now.sleep(seconds)
      if now.ns >= 80000000:
        obj.queue.extend(obj.receive())
    result = execute_sequence(obj, batch, exclusive_commands=True, clock=now, sleep=sleep)
    self.assertTrue(result['complete'], result)
    self.assertEqual(result['receipts_observed_ns'], 80000000)
    self.assertGreaterEqual(result['quiet_end_ns'], 100000000)
    self.assertEqual(list(obj.queue), [[99, 'aa', 0], [1, '00', 128], [2, 'ff', 192]])
    self.assertGreater(result['receipts_observed_ns'], result['events'][-1]['send_end_ns'] + 20000000)

  def test_missing_duplicate_and_stale_local_receipts_fail_without_rejection(self):
    for failure in ('missing', 'duplicate', 'stale'):
      obj = service()
      now = Clock(obj)
      if failure == 'stale':
        obj.queue.append([0x123, '00', 128])

      def sleep(seconds, obj=obj, now=now, failure=failure):
        now.sleep(seconds)
        if failure == 'duplicate' and not obj.queue:
          receipts = obj.receive()
          obj.queue.extend(receipts + receipts)
      result = execute_sequence(obj, events(1), exclusive_commands=True, receipt_timeout_us=30000,
                                clock=now, sleep=sleep)
      self.assertFalse(result['complete'])
      self.assertNotIn('quiet_end_ns', result)
      self.assertIn({'missing': 'timed out', 'duplicate': 'Duplicate', 'stale': 'Stale'}[failure], result['error'])
      if failure == 'missing':
        self.assertEqual(now.ns, 30000000)
        self.assertEqual(list(obj.panda.receipts), [(0x123, b'\x00', 128)])
      if failure == 'stale':
        self.assertEqual(result['events'], [])

  def test_rx_group_phase_replacements_duplicates_and_no_extra_emission(self):
    obj = service('station', paused=True)
    local, peer = socket.socketpair(type=socket.SOCK_DGRAM)
    peer.setblocking(False)
    obj.sockets = [local]
    cycle = [[1, bytes([i]).hex(), 0] for i in range(4)]
    replacement = [[1, bytes([i + 10]).hex(), 0] for i in range(4)]
    obj.streams = {'measurement': {'frames': cycle, 'period': .1, 'index': 5, 'next': 123, 'paused': True},
                   'other': {'frames': cycle, 'period': .1, 'index': 7, 'next': 456, 'paused': False}}
    now = Clock(obj)
    sent_at = []
    original_send = obj.send

    def send(frame):
      sent_at.append(now.ns)
      original_send(frame)

    obj.send = send
    spec = {'name': 'measurement', 'frames': replacement, 'period': .2}
    other = {'name': 'other', 'frames': replacement, 'period': .3}
    batch = [{'frame': cycle[0], 'updates': [spec, other], 'stream_name': 'measurement'},
             {'frame': cycle[0], 'updates': [], 'stream_name': 'measurement'},
             {'frame': replacement[2], 'updates': [], 'stream_name': None}]
    try:
      result = execute_rx_group(obj, batch, clock=now, sleep=now.sleep)
      self.assertTrue(result['complete'], result)
      self.assertEqual(result['frames'], [replacement[1], replacement[2], replacement[2]])
      self.assertEqual(sent_at, [0, 1000000, 2000000])
      self.assertEqual(obj.streams['measurement']['index'], 7)
      self.assertTrue(obj.streams['measurement']['paused'])
      self.assertEqual(obj.streams['other']['index'], 7)
      self.assertEqual(obj.streams['other']['next'], 456)
      self.assertFalse(obj.streams['other']['paused'])
      self.assertTrue(obj.streams_paused)
      packets = [struct.unpack('=IB3x8s', peer.recv(16)) for _ in batch]
      self.assertEqual([packet[2][0] for packet in packets], [11, 12, 12])
      with self.assertRaises(BlockingIOError):
        peer.recv(16)
    finally:
      local.close()
      peer.close()

  def test_rx_group_rpc_allows_worker_to_receive(self):
    obj = service('station', paused=True)
    local, peer = socket.socketpair(type=socket.SOCK_DGRAM)
    local.setblocking(False)
    obj.sockets = [local]
    worker = threading.Thread(target=obj.run)
    worker.start()
    try:
      peer.send(struct.pack('=IB3x8s', 55, 1, b'a'))
      result = obj.dispatch({'op': 'rx_group', 'events': [
        {'frame': [1, '00', 0], 'updates': [], 'stream_name': None} for _ in range(4)]})
      self.assertTrue(result['complete'], result)
      self.assertEqual(result['frames'], [[1, '00', 0]] * 4)
      self.assertEqual(list(obj.queue), [[55, '61', 0]])
      for previous, current in zip(result['events'][:-1], result['events'][1:], strict=True):
        self.assertGreaterEqual(current['send_start_ns'] - previous['send_end_ns'], 1000000)
    finally:
      obj.stopped.set()
      worker.join(timeout=2)
      local.close()
      peer.close()

  def test_rx_group_partial_failure_timeout_and_pause_guard(self):
    for failure in ('send', 'loss', 'resume', 'timeout'):
      obj = service('station', paused=True)
      now = Clock(obj)
      sent = []

      def send(frame, sent=sent, now=now, failure=failure):
        if sent and failure == 'send':
          raise RuntimeError('send failed')
        sent.append(frame)
        if failure == 'timeout':
          now.ns += 1000000001

      def check_transport(sent=sent, failure=failure):
        if sent and failure == 'loss':
          raise RuntimeError('Station transport loss')

      def sleep(seconds, obj=obj, now=now, failure=failure):
        now.sleep(seconds)
        if failure == 'resume':
          obj.streams_paused = False
      obj.send = send
      obj.check_transport = check_transport
      result = execute_rx_group(obj, [{'frame': [1, '00', 0], 'updates': [], 'stream_name': None}] * 3,
                                clock=now, sleep=sleep)
      self.assertFalse(result['complete'])
      self.assertIn('error', result)
      self.assertEqual(result['frames'], [[1, '00', 0]])
      self.assertEqual(len(sent), 1)

  def test_rx_group_admission_rejects_new_streams_and_unpaused_or_large_groups(self):
    event = {'frame': [1, '00', 0], 'updates': [], 'stream_name': None}
    obj = service()
    with self.assertRaisesRegex(ValueError, 'station-only'):
      execute_rx_group(obj, [event])
    obj.role = 'station'
    with self.assertRaisesRegex(ValueError, 'paused'):
      execute_rx_group(obj, [event])
    obj.streams_paused = True
    for batch in ([], [event] * 33, [dict(event, extra=True)],
                  [dict(event, stream_name='missing')],
                  [dict(event, updates=[{'name': 'new', 'frames': [[1, '00', 0]], 'period': .1}])]):
      with patch.object(obj, 'send') as send:
        with self.assertRaises(ValueError):
          execute_rx_group(obj, batch)
        send.assert_not_called()
    self.assertEqual(obj.streams, {})

  def test_validation_before_any_send(self):
    invalid = [events(1) * 2, [], [{'offset_us': 0, 'frame': [1, '00', 0], 'kind': 'rx'}],
               [{'offset_us': -1, 'frame': [1, '00', 0]}],
               [{'offset_us': 0, 'frame': [1, 'aa', 0]}, {'offset_us': 6000, 'frame': [1, 'AA', 0]}]]
    for batch in invalid:
      obj = service()
      with self.assertRaises(ValueError):
        execute_sequence(obj, batch, exclusive_commands=True)
      self.assertFalse(obj.panda.receipts)
    with self.assertRaisesRegex(ValueError, 'exclusive_commands'):
      execute_sequence(service(), events())
    with self.assertRaises(ValueError):
      execute_sequence(service(), events(), exclusive_commands=True, max_lateness_us=10001)


if __name__ == '__main__':
  unittest.main()
