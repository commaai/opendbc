"""Stock Panda / SocketCAN JSON-line service for physical safety scenarios."""
import argparse
import collections
import contextlib
import json
import inspect
from pathlib import Path
import socket
import struct
import sys
import threading
import time

# Linux asm-generic/socket.h; Python does not expose this on every platform.
SO_RXQ_OVFL = getattr(socket, 'SO_RXQ_OVFL', 40)
CAN_MTU = 16
CANFD_MTU = 72
CANFD_LENGTHS = (*range(9), 12, 16, 20, 24, 32, 48, 64)
CAN_RAW_FD_FRAMES = getattr(socket, 'CAN_RAW_FD_FRAMES', 5)


class Service:
  def __init__(self, role, serial=None, interfaces='can2,can3,can0', fd_buses=()):
    self.role = role
    self.fd_buses = frozenset(fd_buses)
    self.configuration = {}
    self.interfaces = interfaces.split(',')
    if not self.fd_buses <= set(range(len(self.interfaces))):
      raise ValueError('FD bus index outside station interfaces')
    self.lock = threading.RLock()
    self.queue = collections.deque()
    self.streams = {}
    self.streams_paused = False
    self.error = None
    self.stopped = threading.Event()
    if role == 'dut':
      from panda import Panda
      if serial is None:
        pandas = Panda.list()
        if len(pandas) != 1:
          raise RuntimeError('Specify --serial when exactly one Panda is not available')
        serial = pandas[0]
      self.panda = Panda(serial, disable_checks=False, cli=False)
      self.panda.set_power_save(False)
      if self.panda.health().get('heartbeat_disabled', False):
        raise RuntimeError('Firmware heartbeat checks are disabled; reset the MCU')
    else:
      self.sockets = []
      self.net_stats = {}
      for bus, interface in enumerate(self.interfaces):
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
        if bus in self.fd_buses:
          sock.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
        sock.setsockopt(socket.SOL_SOCKET, SO_RXQ_OVFL, 1)
        sock.bind((interface,))
        sock.setblocking(False)
        self.sockets.append(sock)
        for counter in ('rx_errors', 'rx_dropped', 'tx_errors', 'tx_dropped'):
          path = Path('/sys/class/net') / interface / 'statistics' / counter
          self.net_stats[path] = int(path.read_text())
    self.worker = threading.Thread(target=self.run, daemon=True)
    self.worker.start()

  def check_transport(self):
    if self.role == 'station':
      for path, baseline in self.net_stats.items():
        value = int(path.read_text())
        if value != baseline:
          raise RuntimeError(f'Station transport loss: {path.parent.parent.name} {path.name} changed {baseline} -> {value}')

  def send(self, frame):
    addr, data, bus = frame
    data = bytes.fromhex(data)
    if type(addr) is not int or not 0 <= addr <= 0x1FFFFFFF:
      raise ValueError('Invalid CAN address')
    if type(bus) is not int or not 0 <= bus < (3 if self.role == 'dut' else len(self.sockets)):
      raise ValueError('Invalid CAN bus')
    if len(data) not in CANFD_LENGTHS:
      raise ValueError('Payload length is not representable on CAN/CAN FD')
    if self.role == 'dut':
      if len(data) > 8 or bus in self.fd_buses:
        if 'fd' not in inspect.signature(self.panda.can_send).parameters:
          raise ValueError('Installed stock Panda API lacks explicit CAN FD send')
        self.panda.can_send(addr, data, bus, fd=True)
      else:
        self.panda.can_send(addr, data, bus)
    else:
      fd = bus in self.fd_buses
      if len(data) > 8 and not fd:
        raise ValueError('CAN FD requires explicit station bus opt-in')
      addr |= 0x80000000 if addr >= 0x800 else 0
      packet = struct.pack('=IBB2x64s', addr, len(data), 0, data) if fd else struct.pack('=IB3x8s', addr, len(data), data)
      if self.sockets[bus].send(packet) != len(packet):
        raise RuntimeError('Short SocketCAN write')

  def receive(self):
    if self.role == 'dut':
      return [[a, bytes(d).hex(), b] for a, d, b in self.panda.can_recv()]
    frames = []
    for bus, sock in enumerate(self.sockets):
      while True:
        try:
          packet, ancillary, flags, _ = sock.recvmsg(CANFD_MTU, socket.CMSG_SPACE(4))
          if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
            raise RuntimeError('Truncated SocketCAN receive evidence')
          for level, kind, value in ancillary:
            if level == socket.SOL_SOCKET and kind == SO_RXQ_OVFL and struct.unpack('=I', value)[0]:
              raise RuntimeError('SocketCAN kernel receive queue overflow')
          # MSG_DONTROUTE marks locally generated traffic, including echoes
          # from other sockets. MSG_CONFIRM additionally identifies own TX.
          if flags & (socket.MSG_DONTROUTE | socket.MSG_CONFIRM):
            continue
          if len(packet) == CAN_MTU:
            addr, length, data = struct.unpack('=IB3x8s', packet)
            if length > 8:
              raise RuntimeError('Invalid classic CAN payload length')
          elif len(packet) == CANFD_MTU and bus in self.fd_buses:
            addr, length, fd_flags, data = struct.unpack('=IBB2x64s', packet)
            if length not in CANFD_LENGTHS or fd_flags & ~7:
              raise RuntimeError('Invalid CAN FD receive evidence')
          else:
            raise RuntimeError('Unexpected SocketCAN frame size or CAN FD bus')
          if addr & 0x60000000:
            raise RuntimeError('Remote/error CAN frame is not physical data evidence')
          frames.append([addr & 0x1FFFFFFF, data[:length].hex(), bus])
        except BlockingIOError:
          break
    return frames

  def _send_stream(self, stream):
    self.send(stream['frames'][stream['index'] % len(stream['frames'])])
    stream['index'] += 1
    stream['next'] = time.monotonic() + stream['period']

  def run(self):
    heartbeat = 0
    try:
      while not self.stopped.is_set():
        with self.lock:
          now = time.monotonic()
          if self.role == 'dut' and now >= heartbeat:
            health = self.panda.health()
            if health.get('heartbeat_disabled', False):
              raise RuntimeError('Firmware heartbeat checks disabled')
            self.panda.send_heartbeat(bool(health['controls_allowed']))
            heartbeat = now + .1
          # Pace even overdue streams: scheduler jitter must not turn separate
          # periodic inputs into a burst that overruns the receiving controller.
          stream = min((s for s in self.streams.values() if not s.get('paused', False)), key=lambda s: s['next'], default=None)
          if not self.streams_paused and stream is not None and now >= stream['next']:
            self._send_stream(stream)
          frames = self.receive()
          if len(self.queue) + len(frames) > 100000:
            raise RuntimeError('Physical receive queue overflow')
          self.queue.extend(frames)
        self.stopped.wait(.001)
    except Exception as exc:
      self.error = str(exc)

  def dispatch(self, req):
    if req['op'] == 'rx_group':
      if __package__:
        from .hardware_sequence import execute_rx_group
      else:
        from hardware_sequence import execute_rx_group
      return execute_rx_group(self, req['events'])
    if req['op'] == 'sequence':
      # Do not hold the dispatch RLock across waits: the worker must continue
      # draining Panda RX and servicing its heartbeat throughout the batch.
      if __package__:
        from .hardware_sequence import execute_sequence
      else:
        from hardware_sequence import execute_sequence
      return execute_sequence(self, req['events'], exclusive_commands=req.get('exclusive_commands', False),
                              max_lateness_us=req.get('max_lateness_us', 2000),
                              scheduling=req.get('scheduling', 'strict'), max_duration_us=req.get('max_duration_us', 8000000),
                              receipt_timeout_us=req.get('receipt_timeout_us', 1000000))
    with self.lock:
      if self.error:
        raise RuntimeError('CAN worker failed: ' + self.error)
      op = req['op']
      reset_stats = op == 'clear' and req.get('reset_stats', False) and self.role == 'station'
      if not reset_stats:
        self.check_transport()
      if op == 'configure' and self.role == 'dut':
        if req['setting'] != 'alternative_experience':
          raise ValueError('Unsupported Panda setting: ' + req['setting'])
        value = req['value']
        if type(value) is not int or not 0 <= value <= 65535:
          raise ValueError('Invalid alternative experience')
        fn = getattr(self.panda, 'set_alternative_experience', None)
        if not callable(fn):
          raise ValueError('Installed stock Panda API lacks set_alternative_experience')
        fn(value)
        self.configuration['alternative_experience'] = {'value': value}
        return
      if op == 'recv':
        frames = list(self.queue)
        self.queue.clear()
        self.check_transport()
        return frames
      if op == 'send':
        self.send(req['frame'])
      elif op == 'rx_stream' and self.role == 'station':
        stream = self.streams[req['name']]
        if not self.streams_paused and not stream.get('paused', False):
          raise ValueError('Isolated stream injection requires paused streams')
        frame = stream['frames'][stream['index'] % len(stream['frames'])]
        self.send(frame)
        stream['index'] += 1
        stream['next'] = time.monotonic() + stream['period']
        return frame
      elif op == 'streams' and self.role == 'station':
        emitted = []
        start = time.monotonic()
        for phase, spec in enumerate(req['streams']):
          if not spec['frames'] or spec['period'] <= 0:
            raise ValueError('Streams require frames and a positive period')
          name = spec['name']
          previous = self.streams.get(name)
          index = previous['index'] if previous else 0
          frame = spec['frames'][index % len(spec['frames'])]
          if previous is None:
            emitted.append(frame)
          self.streams[name] = dict(spec, index=index, next=previous['next'] if previous else start + phase * .001,
                                    paused=previous.get('paused', False) if previous else False)
        return emitted
      elif op == 'pause_stream' and self.role == 'station':
        self.streams[req['name']]['paused'] = True
      elif op == 'resume_stream' and self.role == 'station':
        stream = self.streams[req['name']]
        stream['paused'] = False
        stream['next'] = max(time.monotonic(), stream['next'])
      elif op == 'drop' and self.role == 'station':
        del self.streams[req['name']]
      elif op == 'pause' and self.role == 'station':
        if not self.streams_paused:
          # Give already-due inputs progress before the caller's baseline drain.
          # Snapshot once: held/future inputs and missed-period catch-up stay out.
          now = time.monotonic()
          due = sorted((s for s in self.streams.values() if not s.get('paused', False) and s['next'] <= now),
                       key=lambda s: s['next'])
          for i, stream in enumerate(due):
            if i:
              time.sleep(.001)  # Match the worker's inter-frame pacing.
            self._send_stream(stream)
        self.streams_paused = True
      elif op == 'resume' and self.role == 'station':
        # Preserve overdue priority across repeated pauses; the worker still
        # sends only one frame per iteration and schedules from its send time.
        self.streams_paused = False
      elif op == 'clear':
        if self.role == 'station':
          self.streams.clear()
          self.streams_paused = False
        else:
          for bus in range(3):
            self.panda.can_clear(bus)
          self.panda.can_clear(0xFFFF)
        self.receive()
        self.queue.clear()
        if reset_stats:
          self.net_stats = {path: int(path.read_text()) for path in self.net_stats}
      elif op == 'health' and self.role == 'dut':
        return self.panda.health()
      elif op == 'restart' and self.role == 'dut':
        version = self.panda.get_version()
        self.panda.reset(enter_bootstub=False, enter_bootloader=False, reconnect=True)
        self.panda.set_power_save(False)
        if self.panda.get_version() != version:
          raise RuntimeError('Firmware changed across normal MCU restart')
        self.queue.clear()
        self.configuration.clear()
      elif op == 'provenance' and self.role == 'station':
        return {'transport': 'socketcan', 'interfaces': self.interfaces, 'fd_buses': sorted(self.fd_buses),
                'fd_tx_flags': 0, 'link_configuration': 'external', 'local_tx_evidence': 'excluded'}
      elif op == 'provenance' and self.role == 'dut':
        return {'firmware_version': self.panda.get_version(), 'serial': self.panda.get_serial()[0],
                'hardware_type': self.panda.get_type().hex(), 'transport': 'spi' if self.panda.is_connected_spi() else 'usb',
                'configuration_applied': dict(self.configuration)}
      elif op == 'mode' and self.role == 'dut':
        self.panda.set_safety_mode(req['mode'], req.get('param', 0))
      else:
        raise ValueError('Unsupported operation: ' + op)

  def close(self):
    self.stopped.set()
    self.worker.join(timeout=2)
    if self.role == 'dut':
      self.panda.set_safety_mode(0)
      self.panda.close()
    else:
      for sock in self.sockets:
        sock.close()


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('role', choices=['dut', 'station'])
  parser.add_argument('--serial')
  parser.add_argument('--interfaces', default='can2,can3,can0', help='Station interfaces for DUT buses 0,1,2')
  parser.add_argument('--fd-buses', default='',
                      help='Bus indices using CAN FD, including short frames; Panda payloads over 8 bytes also set fd=True')
  args = parser.parse_args()
  with contextlib.redirect_stdout(sys.stderr):
    service = Service(args.role, args.serial, args.interfaces, tuple(int(bus) for bus in args.fd_buses.split(',') if bus))
  try:
    for line in sys.stdin:
      try:
        with contextlib.redirect_stdout(sys.stderr):
          result = service.dispatch(json.loads(line))
        response = {'ok': True, 'result': result}
      except Exception as exc:
        response = {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
      print(json.dumps(response), flush=True)
  finally:
    service.close()


if __name__ == '__main__':
  main()
