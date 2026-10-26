"""Scheduler tests, including the real Service worker and stock receipt routing."""
import collections
import socket
import struct
import threading
import time
import unittest

from opendbc.safety.tests.hardware_sequence import execute_sequence
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


def service():
  obj = Service.__new__(Service)
  obj.role = 'dut'
  obj.lock = threading.RLock()
  obj.stopped = threading.Event()
  obj.error = None
  obj.panda = PandaTransport()
  obj.fd_buses = set()
  obj.queue = collections.deque()
  obj.streams = {}
  obj.streams_paused = False
  return obj


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
    now = [0]

    def sleep(seconds):
      # Verify scheduler never sleeps with Service.lock held.
      self.assertFalse(obj.lock._is_owned())
      now[0] += round(seconds * 1e9)
      obj.queue.extend(obj.receive())
    result = execute_sequence(obj, events(), exclusive_commands=True, clock=lambda: now[0], sleep=sleep)
    self.assertTrue(result['complete'])
    self.assertEqual([event['send_start_ns'] for event in result['events']], [i * 6000000 for i in range(38)])
    self.assertEqual(result['max_jitter_ns'], 0)
    self.assertEqual(now[0], 243000000)

  def test_late_schedule_aborts_without_retiming(self):
    obj = service()
    now = [0]

    def sleep(seconds):
      now[0] += round(seconds * 1e9) + 10000000
    result = execute_sequence(obj, events(3), exclusive_commands=True, clock=lambda: now[0], sleep=sleep)
    self.assertFalse(result['complete'])
    self.assertIn('Late sequence', result['error'])
    self.assertEqual(len(result['events']), 1)
    self.assertEqual(len(obj.panda.receipts), 1)

  def test_send_delay_and_worker_failure_abort(self):
    for failure in ('send', 'worker'):
      obj = service()
      now = [0]
      original = obj.send

      def send(frame, original=original, failure=failure, now=now, obj=obj):
        original(frame)
        if failure == 'send':
          now[0] += 3000000
        else:
          obj.error = 'receive queue overflow'
      obj.send = send
      result = execute_sequence(obj, events(3), exclusive_commands=True, clock=lambda now=now: now[0])
      self.assertFalse(result['complete'])
      self.assertEqual(len(result['events']), 1)

  def test_station_socket_batch_keeps_worker_polling_and_streams_paused(self):
    obj = service()
    obj.role = 'station'
    del obj.panda  # Station execution must never access Panda health.
    obj.streams_paused = True
    obj.net_stats = {}
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
    obj = service()
    obj.role = 'station'
    obj.streams_paused = True
    del obj.panda
    now = [0]
    sent = []
    checks = []
    obj.check_transport = lambda: checks.append(now[0])
    obj.send = lambda frame: sent.append(frame)
    batch = [{'offset_us': i * 10000, 'frame': [0x1000 + i, '00', i % 3]} for i in range(500)]

    def sleep(seconds):
      self.assertFalse(obj.lock._is_owned())
      now[0] += round(seconds * 1e9)
    result = execute_sequence(obj, batch, exclusive_commands=True, clock=lambda: now[0], sleep=sleep)
    self.assertTrue(result['complete'], result)
    self.assertEqual(sent, [tuple(event['frame']) for event in batch])
    self.assertEqual(len(checks), 502)
    self.assertEqual(now[0], 5010000000)

  def test_station_pause_and_transport_guards(self):
    obj = service()
    obj.role = 'station'
    with self.assertRaisesRegex(ValueError, 'paused'):
      execute_sequence(obj, events(1), exclusive_commands=True)
    obj.streams_paused = True
    with self.assertRaisesRegex(ValueError, 'spacing'):
      execute_sequence(obj, events(2), exclusive_commands=True)
    now = [0]
    obj.check_transport = lambda: None
    obj.send = lambda frame: None

    def sleep(seconds):
      now[0] += round(seconds * 1e9)
      obj.streams_paused = False
    result = execute_sequence(obj, events(1), exclusive_commands=True, clock=lambda: now[0], sleep=sleep)
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
      obj = service()
      obj.role = role
      obj.streams_paused = True
      obj.net_stats = {}
      now = [0]
      sent = []

      def send(frame, now=now, sent=sent, obj=obj):
        sent.append((now[0], frame))
        if obj.role == 'dut':
          obj.queue.append([frame[0], frame[1], frame[2] + 128])
        now[0] += 15000000  # Normal USB stalls exceed steering's 2ms budget.

      def sleep(seconds, now=now, obj=obj):
        self.assertFalse(obj.lock._is_owned())
        now[0] += round(seconds * 1e9) + 4000000
      obj.send = send
      batch = [{'offset_us': i * 10000, 'frame': [i, '00', 0]} for i in range(12)]
      result = execute_sequence(obj, batch, exclusive_commands=True, scheduling='paced_scan',
                                clock=lambda now=now: now[0], sleep=sleep)
      self.assertTrue(result['complete'], result)
      self.assertEqual([frame for _, frame in sent], [tuple(event['frame']) for event in batch])
      self.assertGreater(result['max_jitter_ns'], 10000000)
      for previous, event in zip(result['events'][:-1], result['events'][1:], strict=True):
        self.assertGreaterEqual(event['send_start_ns'] - previous['send_end_ns'], 10000000)
        self.assertGreaterEqual(event['target_ns'] - previous['send_end_ns'], 10000000)
      self.assertGreaterEqual(result['quiet_end_ns'] - result['events'][-1]['send_end_ns'], 20000000)

  def test_paced_scan_overall_deadline_and_transport_loss_still_fail(self):
    for failure in ('deadline', 'transport'):
      obj = service()
      obj.role = 'station'
      obj.streams_paused = True
      now = [0]
      calls = []

      def send(frame, now=now, calls=calls):
        calls.append(frame)
        now[0] += 25000000

      def check_transport(failure=failure, calls=calls):
        if failure == 'transport' and calls:
          raise RuntimeError('Station transport loss')

      def sleep(seconds, now=now):
        now[0] += round(seconds * 1e9)
      obj.send = send
      obj.check_transport = check_transport
      result = execute_sequence(obj, [{'offset_us': i * 10000, 'frame': [i, '00', 0]} for i in range(5)],
                                exclusive_commands=True, scheduling='paced_scan', max_duration_us=60000,
                                clock=lambda now=now: now[0], sleep=sleep)
      self.assertFalse(result['complete'])
      self.assertIn('deadline' if failure == 'deadline' else 'transport loss', result['error'])
      self.assertLess(len(calls), 5)

  def test_dut_cutoff_waits_for_delayed_local_receipts_and_retains_queue(self):
    obj = service()
    now = [0]
    obj.queue.append([99, 'aa', 0])
    batch = [{'offset_us': 0, 'frame': [1, '00', 0]}, {'offset_us': 6000, 'frame': [2, 'ff', 0]}]

    def sleep(seconds):
      self.assertFalse(obj.lock._is_owned())
      now[0] += round(seconds * 1e9)
      if now[0] >= 80000000:
        obj.queue.extend(obj.receive())
    result = execute_sequence(obj, batch, exclusive_commands=True, clock=lambda: now[0], sleep=sleep)
    self.assertTrue(result['complete'], result)
    self.assertEqual(result['receipts_observed_ns'], 80000000)
    self.assertGreaterEqual(result['quiet_end_ns'], 100000000)
    self.assertEqual(list(obj.queue), [[99, 'aa', 0], [1, '00', 128], [2, 'ff', 192]])
    self.assertGreater(result['receipts_observed_ns'], result['events'][-1]['send_end_ns'] + 20000000)

  def test_missing_duplicate_and_stale_local_receipts_fail_without_rejection(self):
    for failure in ('missing', 'duplicate', 'stale'):
      obj = service()
      now = [0]
      if failure == 'stale':
        obj.queue.append([0x123, '00', 128])

      def sleep(seconds, obj=obj, now=now, failure=failure):
        now[0] += round(seconds * 1e9)
        if failure == 'duplicate' and not obj.queue:
          receipts = obj.receive()
          obj.queue.extend(receipts + receipts)
      result = execute_sequence(obj, events(1), exclusive_commands=True, receipt_timeout_us=30000,
                                clock=lambda now=now: now[0], sleep=sleep)
      self.assertFalse(result['complete'])
      self.assertNotIn('quiet_end_ns', result)
      self.assertIn({'missing': 'timed out', 'duplicate': 'Duplicate', 'stale': 'Stale'}[failure], result['error'])
      if failure == 'missing':
        self.assertEqual(now[0], 30000000)
        self.assertEqual(list(obj.panda.receipts), [(0x123, b'\x00', 128)])
      if failure == 'stale':
        self.assertEqual(result['events'], [])

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
