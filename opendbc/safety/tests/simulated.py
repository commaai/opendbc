"""Virtual-time transport exercising the same C hooks used by Panda firmware.

Time and health adaptation live here; observable scenarios never inspect private
safety state or manipulate the firmware clock.
"""
from opendbc.safety.tests.libsafety import libsafety_py


class SimulatedBackend:
  def __init__(self):
    self.safety = libsafety_py.libsafety
    self.reset(0)

  def reset(self, mode, param=0):
    assert self.safety.set_safety_hooks(mode, param) == 0
    self.safety.init_tests()
    self.now = 0
    self.next_tick = 1_000_000
    self.active_streams = {}
    self.invalid = 0

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
      index = self.active_streams.get(spec['name'], {}).get('index', 0)
      assert self.rx(spec['frames'][index % len(spec['frames'])]), spec['name']
      self.active_streams[spec['name']] = dict(spec, index=index + 1,
                                             next=self.now + round(spec['period'] * 1_000_000))

  def drop_stream(self, name):
    del self.active_streams[name]

  def rx(self, frame):
    accepted = bool(self.safety.safety_rx_hook(self._packet(frame)))
    self.invalid += not accepted
    return accepted

  def health(self):
    return {'controls_allowed': bool(self.safety.get_controls_allowed()),
            'safety_rx_checks_invalid': bool(self.safety.safety_rx_checks_invalid),
            'safety_rx_invalid': self.invalid, 'faults': int(self.safety.get_relay_malfunction())}

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

  def forward(self, frame, expected_bus, controls=None, healthy=True):
    self._guard(controls, healthy)
    assert self.rx(frame)
    actual = self.safety.safety_fwd_hook(frame[2], frame[0])
    self._guard(controls, healthy)
    return actual == (-1 if expected_bus is None else expected_bus)

  def wait(self, seconds):
    end = self.now + round(seconds * 1_000_000)
    while True:
      event = min([self.next_tick] + [s['next'] for s in self.active_streams.values()])
      if event > end:
        break
      self._clock(event)
      for stream in self.active_streams.values():
        if stream['next'] <= event:
          assert self.rx(stream['frames'][stream['index'] % len(stream['frames'])]), stream['name']
          stream['index'] += 1
          stream['next'] += round(stream['period'] * 1_000_000)
      if self.next_tick <= event:
        self.safety.safety_tick()
        self.next_tick += 1_000_000
    self._clock(end)

  def close(self):
    self.active_streams.clear()
