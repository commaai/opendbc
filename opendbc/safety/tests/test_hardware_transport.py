"""Wire-level tests independent of physical hardware and Panda installation."""
import collections
import socket
import struct
import threading
import unittest
from unittest.mock import Mock, patch

from opendbc.safety.tests.hardware_service import CANFD_LENGTHS, CAN_RAW_FD_FRAMES, SO_RXQ_OVFL, Service


def station(fd=True):
  service = Service.__new__(Service)
  service.role = 'station'
  service.fd_buses = {0} if fd else set()
  service.sockets = [Mock()]
  service.net_stats = {}
  service.sockets[0].send.side_effect = len
  service.sockets[0].recvmsg.side_effect = BlockingIOError
  return service


class TestHardwareTransport(unittest.TestCase):
  def test_fd_wire_layout(self):
    for length in CANFD_LENGTHS:
      with self.subTest(length=length):
        service = station()
        data = bytes(range(length))
        service.send([0x18DAF110, data.hex(), 0])
        packet = service.sockets[0].send.call_args.args[0]
        self.assertEqual(len(packet), 72)
        self.assertEqual(packet[:8], struct.pack('=IBB2x', 0x98DAF110, length, 0))
        self.assertEqual(packet[8:], data.ljust(64, b'\0'))

  def test_classic_wire_layout(self):
    service = station(False)
    service.send([0x123, '010203', 0])
    self.assertEqual(service.sockets[0].send.call_args.args[0], struct.pack('=IB3x8s', 0x123, 3, b'\1\2\3'))
    with self.assertRaises(ValueError):
      service.send([0x123, bytes(12).hex(), 0])

  def test_invalid_send(self):
    service = station()
    for length in (9, 10, 11, 13, 63, 65):
      with self.assertRaises(ValueError):
        service.send([0x123, bytes(length).hex(), 0])
    for addr, bus in ((-1, 0), (0x20000000, 0), (1, -1), (1, 1)):
      with self.assertRaises(ValueError):
        service.send([addr, '', bus])
    service.sockets[0].send.side_effect = None
    service.sockets[0].send.return_value = 1
    with self.assertRaisesRegex(RuntimeError, 'Short'):
      service.send([1, '', 0])

  def test_mixed_receive(self):
    service = station()
    packets = [(struct.pack('=IB3x8s', 0x123, 3, b'abc'), [], 0, None),
               (struct.pack('=IBB2x64s', 0x98DAF110, 12, 3, b'abcdefghijkl'), [], 0, None)]
    service.sockets[0].recvmsg.side_effect = [*packets, BlockingIOError()]
    self.assertEqual(service.receive(), [[0x123, b'abc'.hex(), 0], [0x18DAF110, b'abcdefghijkl'.hex(), 0]])
    service.sockets[0].recvmsg.assert_called_with(72, socket.CMSG_SPACE(4))

  def test_echo_is_not_physical_evidence(self):
    service = station()
    packet = struct.pack('=IB3x8s', 1, 1, b'a')
    service.sockets[0].recvmsg.side_effect = [(packet, [], flag, None) for flag in (socket.MSG_DONTROUTE, socket.MSG_CONFIRM)] + [BlockingIOError()]
    self.assertEqual(service.receive(), [])

  def test_receive_rejects_bad_evidence(self):
    valid = struct.pack('=IBB2x64s', 1, 12, 0, b'a')
    cases = [(valid, [], socket.MSG_TRUNC), (valid, [], socket.MSG_CTRUNC),
             (valid, [(socket.SOL_SOCKET, SO_RXQ_OVFL, struct.pack('=I', 1))], socket.MSG_CONFIRM),
             (b'bad', [], 0), (struct.pack('=IB3x8s', 1, 9, b'a'), [], 0),
             (struct.pack('=IBB2x64s', 1, 9, 0, b'a'), [], 0),
             (struct.pack('=IBB2x64s', 1, 12, 8, b'a'), [], 0),
             (struct.pack('=IB3x8s', 0x40000001, 1, b'a'), [], 0)]
    for packet, ancillary, flags in cases:
      with self.subTest(packet=packet, flags=flags):
        service = station()
        service.sockets[0].recvmsg.side_effect = [(packet, ancillary, flags, None), BlockingIOError()]
        with self.assertRaises(RuntimeError):
          service.receive()
    service = station(False)
    service.sockets[0].recvmsg.side_effect = [(valid, [], 0, None)]
    with self.assertRaises(RuntimeError):
      service.receive()

  def test_explicit_socket_opt_in(self):
    with patch('opendbc.safety.tests.hardware_service.socket.socket') as factory, \
         patch('opendbc.safety.tests.hardware_service.Path.read_text', return_value='0'), \
         patch('opendbc.safety.tests.hardware_service.threading.Thread'):
      service = Service('station', interfaces='can0', fd_buses=(0,))
      factory.return_value.setsockopt.assert_any_call(socket.SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
      factory.return_value.setsockopt.assert_any_call(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
      service.close()

  def test_stock_configuration_and_capabilities(self):
    service = Service.__new__(Service)
    service.role = 'dut'
    service.lock = threading.RLock()
    service.error = None
    service.configuration = {}
    service.fd_buses = set()
    service.panda = Mock(spec=['set_alternative_experience', 'set_can_speed_kbps', 'set_can_data_speed_kbps',
                               'set_canfd_auto', 'set_canfd_non_iso', 'set_obd', 'can_send'])
    for setting, args, method, values in (
      ('alternative_experience', {'value': 4}, 'set_alternative_experience', (4,)),
      ('can_speed_kbps', {'bus': 0, 'speed': 500}, 'set_can_speed_kbps', (0, 500)),
      ('can_data_speed_kbps', {'bus': 1, 'speed': 2000}, 'set_can_data_speed_kbps', (1, 2000)),
      ('canfd_auto', {'bus': 1, 'enabled': False}, 'set_canfd_auto', (1, False)),
      ('canfd_non_iso', {'bus': 1, 'enabled': True}, 'set_canfd_non_iso', (1, True)),
      ('obd', {'enabled': True}, 'set_obd', (True,)),
    ):
      service.dispatch({'op': 'configure', 'setting': setting, **args})
      getattr(service.panda, method).assert_called_once_with(*values)
    caps = service.dispatch({'op': 'capabilities'})
    self.assertEqual(len(caps['configuration_applied']), 6)
    del service.panda.set_obd
    self.assertNotIn('obd', service.dispatch({'op': 'capabilities'})['configuration'])
    with self.assertRaisesRegex(ValueError, 'lacks'):
      service.dispatch({'op': 'configure', 'setting': 'obd', 'enabled': True})
    with self.assertRaises(ValueError):
      service.dispatch({'op': 'configure', 'setting': 'canfd_auto', 'bus': 0, 'enabled': 1})

  def test_panda_long_frames_request_fd(self):
    service = station()
    service.role = 'dut'
    service.fd_buses = set()
    service.panda = Mock()
    service.panda.can_send = Mock(spec=lambda addr, data, bus, *, fd=False: None)
    # A real method signature is essential for detecting installed API support.
    calls = []
    service.panda.can_send = lambda addr, data, bus, *, fd=False: calls.append((addr, data, bus, fd))
    service.send([1, bytes(12).hex(), 0])
    service.send([1, '00', 0])
    self.assertEqual([call[3] for call in calls], [True, False])
    service.panda.can_send = lambda addr, data, bus: None
    with self.assertRaisesRegex(ValueError, 'explicit CAN FD'):
      service.send([1, bytes(12).hex(), 0])

  @staticmethod
  def stream_station():
    service = station()
    service.lock = threading.RLock()
    service.error = None
    service.streams = {}
    service.streams_paused = False
    service.queue = collections.deque()
    return service

  @staticmethod
  def run_one_iteration(service):
    service.stopped = Mock()
    service.stopped.is_set.side_effect = [False, True]
    service.run()
    if service.error:
      raise AssertionError(service.error)

  def test_stream_replacement_preserves_counter_and_deadline(self):
    service = self.stream_station()
    frames = [[0x123, bytes([counter]).hex(), 0] for counter in range(4)]
    spec = {'name': 'counter', 'frames': frames, 'period': .01}
    with patch('opendbc.safety.tests.hardware_service.time.monotonic', return_value=10):
      self.assertEqual(service.dispatch({'op': 'streams', 'streams': [spec]}), [frames[0]])
      # Initial installation reports the first frame, but only the worker sends it.
      service.sockets[0].send.assert_not_called()
      self.run_one_iteration(service)
    self.assertEqual(service.streams['counter']['index'], 1)
    deadline = service.streams['counter']['next']
    replacement = dict(spec, frames=[[0x123, bytes([counter + 16]).hex(), 0] for counter in range(4)], period=.02)
    service.sockets[0].send.reset_mock()
    with patch('opendbc.safety.tests.hardware_service.time.monotonic', return_value=10.005):
      self.assertEqual(service.dispatch({'op': 'streams', 'streams': [replacement]}), [])
      self.assertEqual(service.streams['counter']['index'], 1)
      self.assertEqual(service.streams['counter']['next'], deadline)
      self.run_one_iteration(service)
      service.sockets[0].send.assert_not_called()
    with patch('opendbc.safety.tests.hardware_service.time.monotonic', return_value=deadline):
      self.run_one_iteration(service)
    service.sockets[0].send.assert_called_once()
    packet = service.sockets[0].send.call_args.args[0]
    self.assertEqual(packet[8], 17)
    self.assertEqual(service.streams['counter']['index'], 2)

  def test_paused_injection_emits_exactly_one_without_incidental_streams(self):
    service = self.stream_station()
    frames = [[0x123, bytes([counter]).hex(), 0] for counter in range(4)]
    spec = {'name': 'counter', 'frames': frames, 'period': .01}
    other = {'name': 'other', 'frames': [[0x124, 'ff', 0]], 'period': .02}
    with patch('opendbc.safety.tests.hardware_service.time.monotonic', return_value=10):
      service.dispatch({'op': 'streams', 'streams': [spec, other]})
      with self.assertRaisesRegex(ValueError, 'paused'):
        service.dispatch({'op': 'rx_stream', 'name': 'counter'})
      service.sockets[0].send.assert_not_called()
      service.dispatch({'op': 'pause'})
    service.streams['counter']['index'] = 7
    other_before = dict(service.streams['other'])
    with patch('opendbc.safety.tests.hardware_service.time.monotonic', return_value=20):
      new = {'name': 'new', 'frames': [[0x125, 'ee', 0]], 'period': .03}
      self.assertEqual(service.dispatch({'op': 'streams', 'streams': [spec, new]}), new['frames'])
      service.sockets[0].send.assert_not_called()
      self.assertEqual(service.streams['counter']['index'], 7)
      self.assertEqual(service.streams['counter']['next'], 10)
      self.assertEqual(service.dispatch({'op': 'rx_stream', 'name': 'counter'}), frames[3])
      self.run_one_iteration(service)
    service.sockets[0].send.assert_called_once()
    self.assertEqual(service.sockets[0].send.call_args.args[0][8], 3)
    self.assertEqual(service.streams['counter']['index'], 8)
    self.assertEqual(service.streams['counter']['next'], 20.01)
    self.assertEqual(service.streams['other'], other_before)
    self.assertEqual(service.streams['new']['index'], 0)

  def test_simulated_replacement_preserves_counter_and_deadline(self):
    from opendbc.safety.tests.simulated import SimulatedBackend

    backend = SimulatedBackend()
    backend.rx = Mock(return_value=True)
    frames = [[0x123, bytes([counter]).hex(), 0] for counter in range(4)]
    spec = {'name': 'counter', 'frames': frames, 'period': .01}
    backend.streams([spec])
    backend.wait(.02)
    self.assertEqual(backend.rx.call_args_list, [unittest.mock.call(frame) for frame in frames[:3]])
    previous = dict(backend.active_streams['counter'])
    backend.rx.reset_mock()
    replacement = dict(spec, frames=list(reversed(frames)), period=.02)
    new = {'name': 'new', 'frames': [[0x124, 'ff', 0]], 'period': .1}
    backend.streams([replacement, new])
    backend.rx.assert_called_once_with(new['frames'][0])
    self.assertEqual(backend.active_streams['counter']['index'], previous['index'])
    self.assertEqual(backend.active_streams['counter']['next'], previous['next'])
    backend.rx.reset_mock()
    backend.wait(.009)
    backend.rx.assert_not_called()
    backend.wait(.001)
    backend.rx.assert_called_once_with(replacement['frames'][3])
    self.assertEqual(backend.active_streams['counter']['index'], 4)
    self.assertEqual(backend.active_streams['counter']['next'], 50000)

  def test_simulated_replacement_does_not_add_c_measurement_sample(self):
    from opendbc.car.structs import CarParams
    from opendbc.safety.tests.simulated import SimulatedBackend

    backend = SimulatedBackend()
    backend.reset(CarParams.SafetyModel.toyota, 100)

    def measurement(torque):
      data = bytearray(8)
      data[5:7] = torque.to_bytes(2, 'big', signed=True)
      data[7] = (0x02 + 0x60 + len(data) + sum(data[:7])) & 0xff
      return [0x260, data.hex(), 0]

    # The C measurement window has six slots. The high sample must survive
    # replacement, then expire on exactly the next real receive hook call.
    self.assertTrue(backend.rx(measurement(600)))
    zero = measurement(0)
    spec = {'name': 'measurement', 'frames': [zero], 'period': .01}
    backend.streams([spec])
    backend.wait(.04)
    self.assertEqual(backend.safety.get_torque_meas_max(), 601)
    previous = dict(backend.active_streams['measurement'])
    backend.streams([dict(spec, period=.02)])
    self.assertEqual(backend.safety.get_torque_meas_max(), 601)
    self.assertEqual(backend.active_streams['measurement']['index'], previous['index'])
    self.assertEqual(backend.active_streams['measurement']['next'], previous['next'])
    self.assertTrue(backend.rx_stream('measurement'))
    self.assertEqual(backend.safety.get_torque_meas_max(), 1)

  def test_simulated_forward_orders_hooks_and_allows_invalid_rx(self):
    from opendbc.safety.tests.simulated import SimulatedBackend

    backend = SimulatedBackend()
    events = []
    backend.safety = Mock()
    backend.safety.get_current_safety_mode.return_value = 0
    backend.safety.get_current_safety_param.return_value = 0
    backend.safety.get_controls_allowed.return_value = False
    backend.safety.safety_rx_checks_invalid = False
    backend.safety.get_relay_malfunction.return_value = False
    backend.safety.safety_fwd_hook.side_effect = lambda bus, addr: events.append('forward') or 2
    backend.safety.safety_rx_hook.side_effect = lambda packet: events.append('rx') or False
    backend.safety.ignition_can_hook.side_effect = lambda packet: events.append('ignition')
    self.assertTrue(backend.forward([0x123, '00', 0], 2, controls=False, rx_valid=False))
    self.assertEqual(events, ['forward', 'rx', 'ignition'])
    self.assertEqual(backend.health()['safety_rx_invalid'], 1)
    with self.assertRaisesRegex(AssertionError, 'validity mismatch'):
      backend.forward([0x123, '00', 0], 2)
    self.assertTrue(backend.forward([0x123, '00', 0], 2, rx_valid=None))
    backend.safety.get_relay_malfunction.return_value = True
    with self.assertRaises(AssertionError):
      backend.forward([0x123, '00', 0], 2, rx_valid=False)
    self.assertTrue(backend.forward([0x123, '00', 0], 2, healthy=False, rx_valid=False))

  def test_simulated_c_forward_precedes_relay_fault_and_invalid_rx(self):
    from opendbc.car.structs import CarParams
    from opendbc.safety.tests.simulated import SimulatedBackend

    backend = SimulatedBackend()
    backend.reset(CarParams.SafetyModel.toyota, 100)
    # An invalid checksum does not prevent the driver forwarding this frame.
    self.assertTrue(backend.forward([0x260, '0000000000000000', 0], 2, rx_valid=False))
    self.assertEqual(backend.health()['safety_rx_invalid'], 1)
    backend.reset(CarParams.SafetyModel.toyota, 100)
    # This relay-triggering frame is forwarded before its RX hook latches the
    # fault; the subsequent frame is blocked by that latched fault.
    self.assertTrue(backend.forward([0x2e4, '0000000000000000', 0], 2, healthy=False))
    self.assertTrue(backend.health()['faults'])
    self.assertTrue(backend.forward([0x123, '00', 0], None, healthy=False))

  def test_simulated_ignition_hook_and_mode_reset_preserves_state(self):
    from opendbc.safety.tests.simulated import SimulatedBackend

    backend = SimulatedBackend()
    try:
      backend.rx([0x1f1, '0200000000000000', 0])
      self.assertTrue(backend.health()['ignition_can'])
      backend.safety.ignition_can_cnt = 2
      backend.reset(0)
      self.assertTrue(backend.health()['ignition_can'])
      self.assertEqual(backend.safety.ignition_can_cnt, 2)
      backend.rx([0x1f1, '0000000000000000', 0])
      self.assertFalse(backend.health()['ignition_can'])
      self.assertEqual(backend.safety.ignition_can_cnt, 0)
    finally:
      backend.rx([0x1f1, '0000000000000000', 0])

  @staticmethod
  def forwarding_hardware(delta=1, overflow=0, faults=0):
    from opendbc.safety.tests.hardware import HardwareBackend

    backend = HardwareBackend.__new__(HardwareBackend)
    backend.dut, backend.station = Mock(), Mock()
    backend.mode, backend.param, backend.timeout = 1, 0, .1
    backend._overflow_baseline = {'rx_buffer_overflow': 0, 'tx_buffer_overflow': 0}
    backend._expected_rx_invalid = 0
    backend.evidence = {'rx_physical': 0, 'forward_physical': 0, 'forward_blocked': 0}
    base = {'safety_mode': 1, 'safety_param': 0, 'controls_allowed': False, 'safety_rx_checks_invalid': False,
            'safety_rx_invalid': 0, 'rx_buffer_overflow': 0, 'tx_buffer_overflow': 0, 'faults': 0}
    after = dict(base, safety_rx_invalid=delta, rx_buffer_overflow=overflow, faults=faults)
    frame = [0x123, '00', 0]
    reads = iter([[], [frame]])
    healths = iter([base, after, after])
    events = []

    def dut_call(op):
      events.append(('dut', op))
      return next(healths) if op == 'health' else next(reads, [])

    received = iter([[], [[0x123, '00', 2]]])

    def station_call(op, **kwargs):
      events.append(('station', op))
      return next(received, []) if op == 'recv' else None

    backend.dut.call.side_effect = dut_call
    backend.station.call.side_effect = station_call
    return backend, frame, events

  def test_hardware_forward_attributes_invalid_rx_while_paused(self):
    backend, frame, events = self.forwarding_hardware()
    clock = iter(i * .005 for i in range(100))
    with patch('opendbc.safety.tests.hardware.time.sleep'), \
         patch('opendbc.safety.tests.hardware.time.monotonic', side_effect=lambda clock=clock: next(clock)):
      self.assertTrue(backend.forward(frame, 2, controls=False, rx_valid=False))
    self.assertEqual(backend._expected_rx_invalid, 1)
    self.assertEqual(backend.evidence['forward_physical'], 1)
    self.assertEqual(backend.evidence['rx_physical'], 1)
    self.assertEqual(events[0], ('station', 'pause'))
    self.assertEqual(events[-1], ('station', 'resume'))
    self.assertEqual(events.count(('station', 'send')), 1)
    self.assertGreater(events.index(('station', 'send')), events.index(('dut', 'health')))

  def test_hardware_forward_keeps_validity_health_and_loss_guards(self):
    for delta, overflow, faults, healthy, rx_valid, succeeds in (
      (0, 0, 0, True, True, True),
      (1, 0, 0, True, True, False),
      (2, 0, 0, False, None, False),
      (1, 1, 0, False, None, False),
      (1, 0, 1, True, False, False),
      (1, 0, 1, False, False, True),
    ):
      with self.subTest(delta=delta, overflow=overflow, faults=faults, healthy=healthy, rx_valid=rx_valid):
        backend, frame, events = self.forwarding_hardware(delta, overflow, faults)
        clock = iter(i * .005 for i in range(100))
        with patch('opendbc.safety.tests.hardware.time.sleep'), \
             patch('opendbc.safety.tests.hardware.time.monotonic', side_effect=lambda clock=clock: next(clock)):
          if succeeds:
            self.assertTrue(backend.forward(frame, 2, controls=False, healthy=healthy, rx_valid=rx_valid))
          else:
            with self.assertRaises(AssertionError):
              backend.forward(frame, 2, controls=False, healthy=healthy, rx_valid=rx_valid)
        self.assertEqual(events[-1], ('station', 'resume'))

  def test_hardware_forward_blocked_evidence_and_failure_cleanup(self):
    for scenario in ('blocked', 'wrong_bus', 'transport_loss', 'controls'):
      with self.subTest(scenario=scenario):
        backend, frame, events = self.forwarding_hardware()
        station_call = backend.station.call.side_effect

        def call(op, scenario=scenario, station_call=station_call, **kwargs):
          result = station_call(op, **kwargs)
          if op == 'recv':
            if scenario == 'transport_loss':
              raise RuntimeError('Station transport loss')
            if scenario == 'blocked':
              return []
            if scenario == 'wrong_bus' and result:
              return [[0x123, '00', 1]]
          return result

        backend.station.call.side_effect = call
        clock = iter(i * .005 for i in range(100))
        with patch('opendbc.safety.tests.hardware.time.sleep'), \
             patch('opendbc.safety.tests.hardware.time.monotonic', side_effect=lambda clock=clock: next(clock)):
          if scenario == 'blocked':
            self.assertTrue(backend.forward(frame, None, rx_valid=False))
            self.assertEqual(backend.evidence['forward_blocked'], 1)
          elif scenario == 'transport_loss':
            with self.assertRaisesRegex(RuntimeError, 'transport loss'):
              backend.forward(frame, 2, rx_valid=False)
          else:
            with self.assertRaises(AssertionError):
              backend.forward(frame, 2, controls=True if scenario == 'controls' else False, healthy=False, rx_valid=False)
        self.assertEqual(events[-1], ('station', 'resume'))
        if scenario == 'controls':
          self.assertNotIn(('station', 'send'), events)

  @staticmethod
  def sequence_hardware(receipts=None, wire=None, baseline=None, complete=True):
    from opendbc.safety.tests.hardware import HardwareBackend

    backend = HardwareBackend.__new__(HardwareBackend)
    backend.dut, backend.station = Mock(), Mock()
    backend.mode, backend.param = 1, 0
    backend._command_lock = threading.Lock()
    backend._station_streams = {}
    backend.sequence_observations = []
    backend._overflow_baseline = {'rx_buffer_overflow': 0, 'tx_buffer_overflow': 0}
    backend._expected_rx_invalid = 0
    backend.evidence = {'tx_physical': 0, 'tx_rejected': 0}
    health = {'safety_mode': 1, 'safety_param': 0, 'controls_allowed': True, 'safety_rx_checks_invalid': False,
              'safety_rx_invalid': 0, 'rx_buffer_overflow': 0, 'tx_buffer_overflow': 0, 'faults': 0}
    report = {'complete': complete, 'health_before': dict(health), 'health_after': dict(health),
              'start_ns': 123, 'events': [], 'quiet_end_ns': 20000123}
    events = [{'offset_us': 0, 'frame': [0x123, 'AA', 0]}, {'offset_us': 6000, 'frame': [0x123, 'bb', 0]}]
    dut_reads = iter([baseline or [], receipts if receipts is not None else [[0x123, 'aa', 128], [0x123, 'bb', 192]]])
    station_reads = iter([[], wire if wire is not None else [[0x123, 'aa', 0]]])

    def dut_call(op, **kwargs):
      if op == 'health':
        return dict(health)
      if op == 'sequence':
        return report
      return next(dut_reads)

    backend.dut.call.side_effect = dut_call
    backend.station.call.side_effect = lambda op, **kwargs: next(station_reads)
    return backend, events, report

  def test_sequence_exact_counts_and_retained_timing(self):
    backend, events, report = self.sequence_hardware()
    with patch('opendbc.safety.tests.hardware.time.sleep'):
      result = backend.steering_sequence(events)
    self.assertEqual(result['verdicts'], [True, False])
    self.assertIs(result['timing'], report)
    self.assertEqual(backend.evidence, {'tx_physical': 1, 'tx_rejected': 1})
    backend.dut.call.assert_any_call('sequence', events=[dict(events[0], frame=[0x123, 'aa', 0]), events[1]],
                                     exclusive_commands=True, max_lateness_us=2000)
    self.assertEqual(backend.sequence_observations[0]['received']['station'], [[0x123, 'aa', 0]])
    self.assertNotIn(unittest.mock.call('clear'), backend.station.call.call_args_list)

  def test_sequence_rejects_missing_duplicate_conflicting_and_false_rejection_evidence(self):
    accepted, rejected = [0x123, 'aa', 128], [0x123, 'bb', 192]
    for receipts, wire in (
      ([accepted], [[0x123, 'aa', 0]]),
      ([accepted, accepted, rejected], [[0x123, 'aa', 0]]),
      ([accepted, rejected, [0x123, 'aa', 192]], [[0x123, 'aa', 0]]),
      ([accepted, rejected], []),
      ([accepted, rejected], [[0x123, 'aa', 0], [0x123, 'aa', 0]]),
      ([accepted, rejected], [[0x123, 'aa', 0], [0x123, 'bb', 0]]),
      ([accepted, rejected, rejected], [[0x123, 'aa', 0]]),
    ):
      with self.subTest(receipts=receipts, wire=wire):
        backend, events, _ = self.sequence_hardware(receipts, wire)
        with patch('opendbc.safety.tests.hardware.time.sleep'), self.assertRaises(AssertionError):
          backend.steering_sequence(events)
        self.assertEqual(backend.evidence, {'tx_physical': 0, 'tx_rejected': 0})

  def test_sequence_rejects_stale_sources_invalid_plans_and_failed_timing(self):
    for scenario in ('stale', 'stream', 'duplicate', 'unordered', 'rx_event', 'incomplete', 'mode', 'overflow', 'invalid_rx'):
      with self.subTest(scenario=scenario):
        backend, events, report = self.sequence_hardware(baseline=[[0x123, 'aa', 128]] if scenario == 'stale' else None)
        if scenario == 'stream':
          backend._station_streams['bad'] = [[0x123, 'aa', 0]]
        elif scenario == 'duplicate':
          events[1]['frame'] = [0x123, 'aa', 0]
        elif scenario == 'unordered':
          events.reverse()
        elif scenario == 'rx_event':
          events[0]['op'] = 'rx'
        elif scenario == 'incomplete':
          report['complete'] = False
        elif scenario == 'mode':
          report['health_after']['safety_mode'] = 2
        elif scenario == 'overflow':
          report['health_after']['tx_buffer_overflow'] = 1
        elif scenario == 'invalid_rx':
          report['health_before']['safety_rx_invalid'] = 1
        with patch('opendbc.safety.tests.hardware.time.sleep'), self.assertRaises((AssertionError, ValueError)):
          backend.steering_sequence(events)
        self.assertEqual(backend.evidence, {'tx_physical': 0, 'tx_rejected': 0})
        if scenario in ('stale', 'stream', 'duplicate', 'unordered', 'rx_event'):
          self.assertFalse(any(call.args[0] == 'sequence' for call in backend.dut.call.call_args_list))

  def test_simulated_sequence_uses_c_verdicts_and_virtual_input_time(self):
    from opendbc.safety.tests.simulated import SimulatedBackend
    from opendbc.safety.tests.vehicle import get_vehicle_profile

    backend = SimulatedBackend()
    vehicle = get_vehicle_profile('hyundai')
    backend.reset(vehicle.safety_model, vehicle.safety_param)
    backend.streams(vehicle.streams())
    backend.wait(1.1)
    for changes in vehicle.engage_sequence():
      backend.streams(vehicle.update(**changes))
      backend.wait(.15)
    start = backend.now
    counter = backend.active_streams['driver_torque']['index']
    events = [{'offset_us': i * 6000, 'frame': vehicle.torque(torque)} for i, torque in enumerate((1, 1000, 0))]
    result = backend.steering_sequence(events)
    self.assertEqual(result['verdicts'], [True, False, True])
    self.assertFalse(result['timing']['physical'])
    self.assertNotIn('send_start_ns', result['timing'])
    self.assertEqual(backend.now, start + 32000)
    self.assertGreater(backend.active_streams['driver_torque']['index'], counter)
    self.assertEqual(backend.health()['safety_rx_invalid'], 0)

  def test_restart_lifecycle_and_bounded_rpc_timeout(self):
    from opendbc.safety.tests.simulated import SimulatedBackend

    backend, _, _ = self.sequence_hardware()
    backend.dut.call.side_effect = None
    backend.dut.call.return_value = {'firmware_version': 'unchanged'}
    backend.station.call.side_effect = None
    backend._station_streams['old'] = [[1, '00', 0]]
    self.assertEqual(backend.restart(), {'firmware_version': 'unchanged'})
    backend.dut.call.assert_called_once_with('restart', rpc_timeout=30)
    self.assertEqual(backend._station_streams, {})
    self.assertFalse(hasattr(backend, '_overflow_baseline'))
    self.assertFalse(hasattr(backend, '_expected_rx_invalid'))
    self.assertIsNone(backend.mode)
    self.assertEqual(backend.station.call.call_args_list, [unittest.mock.call('clear'), unittest.mock.call('clear')])
    simulated = SimulatedBackend()
    simulated.rx([0x1f1, '0200000000000000', 0])
    simulated.safety.ignition_can_cnt = 2
    simulated.restart()
    self.assertFalse(simulated.health()['ignition_can'])
    self.assertEqual(simulated.safety.ignition_can_cnt, 0)
    self.assertEqual(simulated.safety.get_current_safety_mode(), 0)

  def test_per_stream_station_pause_replacement_global_resume_and_injection(self):
    service = self.stream_station()
    measurement = {'name': 'measurement', 'frames': [[0x123, '01', 0], [0x123, '02', 0]], 'period': .01}
    other = {'name': 'other', 'frames': [[0x124, 'ff', 0]], 'period': .02}
    with patch('opendbc.safety.tests.hardware_service.time.monotonic', return_value=10):
      service.dispatch({'op': 'streams', 'streams': [measurement, other]})
      service.dispatch({'op': 'pause_stream', 'name': 'measurement'})
    with patch('opendbc.safety.tests.hardware_service.time.monotonic', return_value=20):
      service.dispatch({'op': 'pause'})
      service.dispatch({'op': 'resume'})
      self.run_one_iteration(service)
      self.assertEqual(service.streams['measurement']['index'], 0)
      self.assertEqual(service.streams['other']['index'], 1)
      self.assertTrue(service.streams['measurement']['paused'])
      self.assertEqual(service.dispatch({'op': 'rx_stream', 'name': 'measurement'}), measurement['frames'][0])
      self.assertEqual(service.streams['measurement']['index'], 1)
      self.assertEqual(service.streams['measurement']['next'], 20.01)
      replacement = dict(measurement, period=.02)
      self.assertEqual(service.dispatch({'op': 'streams', 'streams': [replacement]}), [])
      self.assertTrue(service.streams['measurement']['paused'])
      self.assertEqual(service.streams['measurement']['index'], 1)
      self.assertEqual(service.streams['measurement']['next'], 20.01)
    with patch('opendbc.safety.tests.hardware_service.time.monotonic', return_value=21):
      self.run_one_iteration(service)
      self.assertEqual(service.streams['measurement']['index'], 1)
      service.dispatch({'op': 'resume_stream', 'name': 'measurement'})
      self.assertEqual(service.streams['measurement']['next'], 21)
      self.run_one_iteration(service)
      self.run_one_iteration(service)
    packets = [call.args[0] for call in service.sockets[0].send.call_args_list]
    self.assertEqual([struct.unpack_from('=I', packet)[0] for packet in packets], [0x124, 0x123, 0x124, 0x123])
    self.assertEqual([packet[8] for packet in packets], [255, 1, 255, 2])
    self.assertEqual(service.streams['measurement']['index'], 2)
    self.assertEqual(service.streams['measurement']['next'], 21.02)

  def test_paused_simulated_measurement_window_has_no_incidental_c_samples(self):
    from opendbc.car.structs import CarParams
    from opendbc.safety.tests.simulated import SimulatedBackend

    backend = SimulatedBackend()
    backend.reset(CarParams.SafetyModel.toyota, 100)

    def measurement(torque):
      data = bytearray(8)
      data[5:7] = torque.to_bytes(2, 'big', signed=True)
      data[7] = (0x62 + len(data) + sum(data[:7])) & 0xff
      return [0x260, data.hex(), 0]

    self.assertTrue(backend.rx(measurement(600)))
    stream = {'name': 'measurement', 'frames': [measurement(0)], 'period': .02}
    backend.streams([stream, {'name': 'other', 'frames': [[0x123, '00', 0]], 'period': .01}])
    backend.pause_stream('measurement')
    # Four explicit samples plus the installation sample leave the original
    # extreme in the sixth C window slot. Other live traffic cannot evict it.
    for _ in range(4):
      self.assertTrue(backend.rx_stream('measurement'))
      backend.wait(.02)
    self.assertEqual(backend.safety.get_torque_meas_max(), 601)
    previous = dict(backend.active_streams['measurement'])
    backend.streams([dict(stream, period=.01)])
    self.assertTrue(backend.active_streams['measurement']['paused'])
    self.assertEqual(backend.active_streams['measurement']['index'], previous['index'])
    self.assertEqual(backend.active_streams['measurement']['next'], previous['next'])
    other_index = backend.active_streams['other']['index']
    backend.wait(.02)
    self.assertGreater(backend.active_streams['other']['index'], other_index)
    self.assertEqual(backend.safety.get_torque_meas_max(), 601)
    self.assertTrue(backend.rx_stream('measurement'))
    self.assertEqual(backend.safety.get_torque_meas_max(), 1)
    backend.wait(.05)
    index = backend.active_streams['measurement']['index']
    backend.resume_stream('measurement')
    self.assertEqual(backend.active_streams['measurement']['next'], backend.now)
    backend.wait(0)
    self.assertEqual(backend.active_streams['measurement']['index'], index + 1)
    self.assertEqual(backend.active_streams['measurement']['next'], backend.now + 10000)

  def test_paused_measurement_watchdog_fails_tx_health_guard(self):
    from opendbc.safety.tests.simulated import SimulatedBackend
    from opendbc.safety.tests.vehicle import get_vehicle_profile

    backend = SimulatedBackend()
    vehicle = get_vehicle_profile('toyota')
    backend.reset(vehicle.safety_model, vehicle.safety_param)
    backend.streams(vehicle.streams())
    backend.wait(1.1)
    for changes in vehicle.engage_sequence():
      backend.streams(vehicle.update(**changes))
      backend.wait(.15)
    backend.pause_stream('torque')
    other_index = backend.active_streams['speed']['index']
    # Stock safety_tick uses max(10 periods, 1 second), checked at 1 Hz.
    backend.wait(2.1)
    self.assertGreater(backend.active_streams['speed']['index'], other_index)
    self.assertTrue(backend.health()['safety_rx_checks_invalid'])
    with self.assertRaises(AssertionError):
      backend.tx(vehicle.torque(1000))

  def test_hardware_stream_pause_rpc_and_restart_timeout_transport(self):
    from opendbc.safety.tests.hardware import RPC

    backend, _, _ = self.sequence_hardware()
    backend.station.call.side_effect = None
    backend.pause_stream('measurement')
    backend.resume_stream('measurement')
    self.assertEqual(backend.station.call.call_args_list,
                     [unittest.mock.call('pause_stream', name='measurement'), unittest.mock.call('resume_stream', name='measurement')])
    rpc = RPC.__new__(RPC)
    rpc.lock = threading.Lock()
    rpc.process, rpc.responses, rpc.timeout = Mock(), Mock(), 10
    rpc.responses.get.return_value = {'ok': True, 'result': 'restarted'}
    self.assertEqual(rpc.call('restart', rpc_timeout=30), 'restarted')
    rpc.responses.get.assert_called_once_with(timeout=30)
    self.assertNotIn('rpc_timeout', rpc.process.stdin.write.call_args.args[0])
    self.assertEqual(rpc.timeout, 10)

  def test_hardware_negative_quiet_window_cannot_hide_rx_timeout(self):
    backend, frame, _ = self.forwarding_hardware(delta=0)
    backend._command_lock = threading.Lock()
    backend.evidence.update(tx_physical=0, tx_rejected=0)
    base = {'safety_mode': 1, 'safety_param': 0, 'controls_allowed': False, 'safety_rx_checks_invalid': False,
            'safety_rx_invalid': 0, 'rx_buffer_overflow': 0, 'tx_buffer_overflow': 0, 'faults': 0}
    healths = iter([base, dict(base, safety_rx_checks_invalid=True)])
    reads = iter([[], [[frame[0], frame[1], 192]]])
    backend.dut.call.side_effect = lambda op, **kwargs: next(healths) if op == 'health' else next(reads, []) if op == 'recv' else None
    backend.station.call.side_effect = None
    backend.station.call.return_value = []
    clock = iter(i * .005 for i in range(100))
    with patch('opendbc.safety.tests.hardware.time.sleep'), \
         patch('opendbc.safety.tests.hardware.time.monotonic', side_effect=lambda: next(clock)), \
         self.assertRaises(AssertionError):
      backend.tx(frame, controls=False)
    self.assertEqual(backend.evidence['tx_rejected'], 0)

  @staticmethod
  def scan_hardware(kind, corruption=None, accepted_setup=False):
    backend, _, _ = TestHardwareTransport.sequence_hardware()
    backend.scan_observations = []
    backend.evidence.update(rx_physical=0, forward_physical=0, forward_blocked=0)
    health = {'safety_mode': 1, 'safety_param': 0, 'controls_allowed': accepted_setup, 'safety_rx_checks_invalid': not accepted_setup,
              'safety_rx_invalid': 0, 'rx_buffer_overflow': 0, 'tx_buffer_overflow': 0, 'faults': 0 if accepted_setup else 1}
    dut_queue, wire_queue, calls = [], [], []

    def run_sequence(events, **kwargs):
      calls.append(events)
      dut_frames, wire_frames = [], []
      before = dict(health)
      for event in events:
        addr, data, bus = event['frame']
        if kind == 'tx':
          accepted = accepted_setup or addr % 2 == 0
          dut_frames.append([addr, data, (128 if accepted else 192) + bus])
          if accepted:
            wire_frames.append([addr, data, bus])
        else:
          dut_frames.append([addr, data, bus])
          health['safety_rx_invalid'] += 1
          if addr % 2 == 0:
            wire_frames.append([addr, data, 2])
      if corruption == 'missing':
        dut_frames.pop()
      elif corruption == 'duplicate':
        dut_frames.append(list(dut_frames[0]))
      elif corruption == 'extra_wire':
        wire_frames.append([0x7ff, 'ff', 1])
      elif corruption == 'missing_wire':
        wire_frames.clear()
      elif corruption == 'duplicate_wire':
        wire_frames.append(list(wire_frames[0]))
      elif corruption == 'source_echo':
        wire_frames.append(list(events[0]['frame']))
      elif corruption == 'cross_sources':
        wire_frames[:] = [[events[0]['frame'][0], events[0]['frame'][1], 1],
                          [events[1]['frame'][0], events[1]['frame'][1], 2]]
      elif corruption == 'invalid_delta':
        health['safety_rx_invalid'] += 1
      elif corruption == 'overflow':
        health['rx_buffer_overflow'] += 1
      elif corruption == 'mode':
        health['safety_mode'] = 2
      dut_queue.extend(dut_frames)
      wire_queue.extend(wire_frames)
      return {'complete': corruption != 'incomplete', 'health_before': before, 'health_after': dict(health), 'events': events}

    def call(role, op, **kwargs):
      if op == 'sequence':
        return run_sequence(**kwargs)
      if op == 'health':
        return dict(health)
      if op == 'recv':
        queued = dut_queue if role == 'dut' else wire_queue
        result = list(queued)
        queued.clear()
        return result
      return None

    backend.dut.call.side_effect = lambda op, **kwargs: call('dut', op, **kwargs)
    backend.station.call.side_effect = lambda op, **kwargs: call('station', op, **kwargs)
    return backend, calls

  def test_rx_scan_preserves_entire_domain_chunks_and_counts_invalid_inputs(self):
    backend, batches = self.scan_hardware('rx')
    frames = [[addr, 'aA', 0] for addr in range(257)]
    frames.append(frames[-1])
    with patch('opendbc.safety.tests.hardware.time.sleep'):
      health = backend.rx_scan(frames)
    self.assertEqual([len(batch) for batch in batches], [256, 1, 1])
    self.assertEqual([event['frame'] for batch in batches for event in batch], [[a, 'aa', b] for a, _, b in frames])
    self.assertTrue(all(event['offset_us'] == i * 10000 for batch in batches for i, event in enumerate(batch)))
    self.assertEqual(health['safety_rx_invalid'], 258)
    self.assertEqual(health['faults'], 1)
    self.assertEqual(backend._expected_rx_invalid, 258)
    self.assertEqual(backend.evidence['rx_physical'], 258)
    self.assertEqual(len(health['scan_evidence']), 3)
    self.assertEqual(backend.station.call.call_args_list[0], unittest.mock.call('pause'))
    self.assertEqual(backend.station.call.call_args_list[-1], unittest.mock.call('resume'))
    self.assertFalse(any(call.args[0] == 'mode' for call in backend.dut.call.call_args_list))

  def test_scan_exact_rx_forward_and_tx_evidence_failures(self):
    frames = [[0x100, '00', 0], [0x101, '00', 0]]
    for kind, corruptions in (
      ('rx', ('missing', 'duplicate', 'invalid_delta', 'overflow', 'mode', 'incomplete')),
      ('forward', ('missing', 'duplicate', 'extra_wire', 'missing_wire', 'source_echo', 'incomplete')),
      ('tx', ('missing', 'duplicate', 'extra_wire', 'missing_wire', 'source_echo', 'invalid_delta', 'incomplete')),
    ):
      for corruption in corruptions:
        with self.subTest(kind=kind, corruption=corruption):
          backend, _ = self.scan_hardware(kind, corruption)
          with patch('opendbc.safety.tests.hardware.time.sleep'), self.assertRaises(AssertionError):
            if kind == 'forward':
              backend.forward_scan([(frames[0], 2), (frames[1], None)])
            else:
              getattr(backend, kind + '_scan')(frames)
          self.assertEqual(backend.station.call.call_args_list[-1], unittest.mock.call('resume'))
          self.assertIn('received', backend.scan_observations[0])
          self.assertIn('timing', backend.scan_observations[0])
          self.assertEqual(backend.evidence['rx_physical'], 0)
          self.assertEqual(backend.evidence['tx_physical'], 0)
          self.assertEqual(backend.evidence['forward_physical'], 0)

  def test_forward_and_tx_scan_use_explicit_targets_and_receipt_verdicts(self):
    frames = [[0x100, '00', 0], [0x101, '00', 0]]
    for kind in ('forward', 'tx'):
      backend, batches = self.scan_hardware(kind)
      with patch('opendbc.safety.tests.hardware.time.sleep'):
        if kind == 'forward':
          self.assertTrue(backend.forward_scan([(frames[0], 2), (frames[1], None)]))
          self.assertEqual(backend.evidence['forward_physical'], 1)
          self.assertEqual(backend.evidence['forward_blocked'], 1)
        else:
          self.assertEqual(backend.tx_scan(frames), [True, False])
          self.assertEqual(backend.evidence['tx_physical'], 1)
          self.assertEqual(backend.evidence['tx_rejected'], 1)
      sender = backend.dut if kind == 'tx' else backend.station
      sender.call.assert_any_call('sequence', events=batches[0], exclusive_commands=True,
                                  scheduling='paced_scan', max_duration_us=8000000)

  def test_forward_scan_keeps_target_that_is_another_inputs_source(self):
    backend, _ = self.scan_hardware('forward', 'cross_sources')
    # Bus 1 is both an injection source and a legitimate forwarding destination.
    # Removing all source identities from station evidence would hide this frame.
    with patch('opendbc.safety.tests.hardware.time.sleep'):
      self.assertTrue(backend.forward_scan([([0x100, '00', 0], 1), ([0x100, '00', 1], 2)]))
    self.assertEqual(backend.evidence['forward_physical'], 2)

  def test_simulated_scans_use_real_hooks_and_pause_inputs_without_resets(self):
    from opendbc.car.structs import CarParams
    from opendbc.safety.tests.simulated import SimulatedBackend

    backend = SimulatedBackend()
    backend.reset(CarParams.SafetyModel.toyota, 100)
    backend.streams([{'name': 'background', 'frames': [[0x555, '00', 0]], 'period': .001}])
    before = dict(backend.active_streams['background'])
    # The first frame must route before its RX hook latches the relay fault.
    self.assertTrue(backend.forward_scan([([0x2e4, '0000000000000000', 0], 2), ([0x123, '00', 0], None)]))
    self.assertTrue(backend.health()['faults'])
    self.assertEqual(backend.active_streams['background']['index'], before['index'])
    self.assertFalse(backend.streams_paused)
    self.assertEqual(backend.tx_scan([[0x2e4, '0000000000000000', 0], [0x123, '00', 0]]), [False, False])
    self.assertTrue(backend.health()['faults'])
    backend.reset(CarParams.SafetyModel.noOutput)
    health = backend.rx_scan([[0x1f1, '0200000000000000', 0], [0x123, '00', 0]])
    self.assertTrue(health['ignition_can'])
    self.assertEqual(health['safety_mode'], CarParams.SafetyModel.noOutput)
    self.assertFalse(health['scan_evidence'][0]['physical'])
    self.assertEqual(backend.now, 30000)

  def test_simulated_scan_audit_survives_mode_resets(self):
    from opendbc.car.structs import CarParams
    from opendbc.safety.tests.simulated import SimulatedBackend

    backend = SimulatedBackend()
    backend.reset(CarParams.SafetyModel.noOutput)
    result = backend.rx_scan([[0x123, '00', 0]])
    history = backend.scan_observations
    first = history[0]
    self.assertIs(first, result['scan_evidence'][0])
    backend.reset(CarParams.SafetyModel.toyota, 100)
    self.assertIs(backend.scan_observations, history)
    self.assertIs(history[0], first)
    self.assertEqual(first['health_after']['safety_mode'], CarParams.SafetyModel.noOutput)
    self.assertTrue(first['complete'])
    self.assertEqual(backend.tx_scan([[0x123, '00', 0]]), [False])
    self.assertEqual(len(history), 2)
    self.assertEqual(history[1]['health_after']['safety_mode'], CarParams.SafetyModel.toyota)
    self.assertEqual(first['events'][0]['frame'], [0x123, '00', 0])

  def test_hardware_mode_reset_retains_scan_reports_and_evidence_counters(self):
    backend, _ = self.scan_hardware('rx')
    with patch('opendbc.safety.tests.hardware.time.sleep'):
      result = backend.rx_scan([[0x100, '00', 0], [0x101, '00', 0]])
    history, counts = backend.scan_observations, dict(backend.evidence)
    first = history[0]
    backend.sequence_observations.append({'timing': {'complete': False, 'error': 'retained partial run'}})
    sequence_history = backend.sequence_observations
    health = dict(result, safety_mode=2)
    health.pop('scan_evidence')
    backend.dut.call.side_effect = lambda op, **kwargs: dict(health) if op == 'health' else None
    backend.station.call.side_effect = None
    with patch('opendbc.safety.tests.hardware.time.sleep'):
      backend.reset(2)
    self.assertEqual(backend.mode, 2)
    self.assertIs(backend.scan_observations, history)
    self.assertIs(history[0], first)
    self.assertEqual(first['health_after']['safety_mode'], 1)
    self.assertEqual(backend.evidence, counts)
    self.assertEqual(backend.evidence['rx_physical'], 2)
    self.assertIs(backend.sequence_observations, sequence_history)
    self.assertEqual(sequence_history[0]['timing']['error'], 'retained partial run')

  def test_accepted_ramp_pipelines_all_frames_with_live_streams(self):
    backend, batches = self.scan_hardware('tx', accepted_setup=True)
    frames = [[addr, 'aA', 0] for addr in range(258)]
    with patch('opendbc.safety.tests.hardware.time.sleep') as sleep:
      result = backend.accepted_ramp(frames, 10000)
    self.assertEqual(result['verdicts'], [True] * 258)
    self.assertEqual([len(batch) for batch in batches], [256, 2])
    self.assertEqual([event['frame'] for batch in batches for event in batch], [[a, 'aa', b] for a, _, b in frames])
    self.assertTrue(all(event['offset_us'] == i * 10000 for batch in batches for i, event in enumerate(batch)))
    self.assertEqual(backend.evidence['tx_physical'], 258)
    self.assertEqual(len(result['timing']), 2)
    self.assertIs(result['timing'][0], backend.sequence_observations[0]['timing'])
    sleep.assert_any_call(.01)
    self.assertTrue(all(call.kwargs['scheduling'] == 'paced_scan' for call in backend.dut.call.call_args_list if call.args[0] == 'sequence'))
    self.assertTrue(all(call.args[0] == 'recv' for call in backend.station.call.call_args_list))

  def test_accepted_ramp_rejects_unaccepted_or_ambiguous_evidence(self):
    for corruption in ('missing', 'duplicate', 'duplicate_wire', 'missing_wire', 'incomplete', 'overflow', 'invalid_delta', 'mode'):
      with self.subTest(corruption=corruption):
        backend, _ = self.scan_hardware('tx', corruption, accepted_setup=True)
        frames = [[0x100, '00', 0]]
        with patch('opendbc.safety.tests.hardware.time.sleep'), self.assertRaises(AssertionError):
          backend.accepted_ramp(frames, 10000)
    backend, _, _ = self.sequence_hardware()
    with patch('opendbc.safety.tests.hardware.time.sleep'), self.assertRaisesRegex(AssertionError, 'ramp command rejected'):
      backend.accepted_ramp([[0x123, 'aa', 0], [0x123, 'bb', 0]], 10000)

  def test_accepted_ramp_admission_and_sources_checked_before_commands(self):
    for scenario in ('spacing', 'empty', 'late_bad_frame', 'stream_collision'):
      with self.subTest(scenario=scenario):
        backend, batches = self.scan_hardware('tx', accepted_setup=True)
        frames, spacing = [[0x100, '00', 0]], 10000
        if scenario == 'spacing':
          spacing = 9999
        elif scenario == 'empty':
          frames = []
        elif scenario == 'late_bad_frame':
          frames = [[addr, '00', 0] for addr in range(257)] + [[0x20000000, '00', 0]]
        else:
          frames = [[addr, '00', 0] for addr in range(257)]
          backend._station_streams['bad'] = [frames[-1]]
        with self.assertRaises((ValueError, AssertionError)):
          backend.accepted_ramp(frames, spacing)
        self.assertEqual(batches, [])

  def test_simulated_accepted_ramp_real_c_with_live_inputs(self):
    from opendbc.safety.tests.simulated import SimulatedBackend
    from opendbc.safety.tests.vehicle import get_vehicle_profile

    backend = SimulatedBackend()
    vehicle = get_vehicle_profile('hyundai')
    backend.reset(vehicle.safety_model, vehicle.safety_param)
    backend.streams(vehicle.streams())
    backend.wait(1.1)
    for changes in vehicle.engage_sequence():
      backend.streams(vehicle.update(**changes))
      backend.wait(.15)
    before = backend.active_streams['driver_torque']['index']
    frames = [vehicle.torque(torque) for torque in range(1, 16)]
    result = backend.accepted_ramp(frames, 10000)
    self.assertEqual(result['verdicts'], [True] * len(frames))
    self.assertGreater(backend.active_streams['driver_torque']['index'], before)
    self.assertFalse(result['timing'][0]['physical'])
    self.assertEqual(result['timing'][0]['scheduling'], 'paced_scan')
    self.assertEqual([event['frame'] for event in result['timing'][0]['events']], frames)
    with self.assertRaisesRegex(AssertionError, 'ramp command rejected'):
      backend.accepted_ramp([vehicle.torque(1000)], 10000)

  def test_queue_overflow_stays_fatal(self):
    service = station()
    service.lock = threading.RLock()
    service.stopped = threading.Event()
    service.streams = {}
    service.streams_paused = False
    service.queue = collections.deque([None] * 100000)
    service.receive = Mock(return_value=[[1, '', 0]])
    service.run()
    self.assertEqual(service.error, 'Physical receive queue overflow')


if __name__ == '__main__':
  unittest.main()
