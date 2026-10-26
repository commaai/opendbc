"""Stock Panda / SocketCAN JSON-line service for physical safety scenarios."""
import argparse
import collections
import contextlib
import json
from pathlib import Path
import socket
import struct
import sys
import threading
import time

# Linux asm-generic/socket.h; Python does not expose this on every platform.
SO_RXQ_OVFL = getattr(socket, 'SO_RXQ_OVFL', 40)


class Service:
  def __init__(self, role, serial=None, interfaces='can2,can3,can0'):
    self.role = role
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
      for interface in interfaces.split(','):
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
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
    if self.role == 'dut':
      self.panda.can_send(addr, data, bus)
    else:
      if len(data) > 8:
        raise ValueError('Station transport supports classic CAN only')
      addr |= 0x80000000 if addr >= 0x800 else 0
      self.sockets[bus].send(struct.pack('=IB3x8s', addr, len(data), data))

  def receive(self):
    if self.role == 'dut':
      return [[a, bytes(d).hex(), b] for a, d, b in self.panda.can_recv()]
    frames = []
    for bus, sock in enumerate(self.sockets):
      while True:
        try:
          packet, ancillary, flags, _ = sock.recvmsg(16, socket.CMSG_SPACE(4))
          if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
            raise RuntimeError('Truncated SocketCAN receive evidence')
          for level, kind, value in ancillary:
            if level == socket.SOL_SOCKET and kind == SO_RXQ_OVFL and struct.unpack('=I', value)[0]:
              raise RuntimeError('SocketCAN kernel receive queue overflow')
          addr, length, data = struct.unpack('=IB3x8s', packet)
          frames.append([addr & 0x1FFFFFFF, data[:length].hex(), bus])
        except BlockingIOError:
          break
    return frames

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
          stream = min(self.streams.values(), key=lambda s: s['next'], default=None)
          if not self.streams_paused and stream is not None and now >= stream['next']:
            self.send(stream['frames'][stream['index'] % len(stream['frames'])])
            stream['index'] += 1
            stream['next'] = time.monotonic() + stream['period']
          frames = self.receive()
          if len(self.queue) + len(frames) > 100000:
            raise RuntimeError('Physical receive queue overflow')
          self.queue.extend(frames)
        self.stopped.wait(.001)
    except Exception as exc:
      self.error = str(exc)

  def dispatch(self, req):
    with self.lock:
      if self.error:
        raise RuntimeError('CAN worker failed: ' + self.error)
      op = req['op']
      reset_stats = op == 'clear' and req.get('reset_stats', False) and self.role == 'station'
      if not reset_stats:
        self.check_transport()
      if op == 'recv':
        frames = list(self.queue)
        self.queue.clear()
        self.check_transport()
        return frames
      if op == 'send':
        self.send(req['frame'])
      elif op == 'streams' and self.role == 'station':
        emitted = []
        start = time.monotonic()
        for phase, spec in enumerate(req['streams']):
          if not spec['frames'] or spec['period'] <= 0:
            raise ValueError('Streams require frames and a positive period')
          name = spec['name']
          index = self.streams.get(name, {}).get('index', 0)
          frame = spec['frames'][index % len(spec['frames'])]
          emitted.append(frame)
          self.streams[name] = dict(spec, index=index, next=start + phase * .001)
        return emitted
      elif op == 'drop' and self.role == 'station':
        del self.streams[req['name']]
      elif op == 'pause' and self.role == 'station':
        self.streams_paused = True
      elif op == 'resume' and self.role == 'station':
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
      elif op == 'provenance' and self.role == 'dut':
        return {'firmware_version': self.panda.get_version(), 'serial': self.panda.get_serial()[0],
                'hardware_type': self.panda.get_type().hex(), 'transport': 'spi' if self.panda.is_connected_spi() else 'usb'}
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
  args = parser.parse_args()
  with contextlib.redirect_stdout(sys.stderr):
    service = Service(args.role, args.serial, args.interfaces)
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
