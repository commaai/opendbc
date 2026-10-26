"""Bridge legacy fixture encoders to continuous, test-owned RX inputs.

Only encoder arguments are captured; firmware state is never inspected. The
backend owns timing and counter phase, including explicit stream injections.
"""
from collections import deque
from collections.abc import Callable
from contextlib import contextmanager
from dataclasses import dataclass
from math import lcm
from functools import partial, wraps

from opendbc.can import CANPacker
from opendbc.car.structs import CarParams


class UnsupportedStreamProfile(ValueError):
  pass


def _raw(packet):
  p = packet[0]
  from opendbc.safety import DLC_TO_LEN
  return [int(p.addr), bytes(p.data[0:DLC_TO_LEN[p.data_len_code]]).hex(), int(p.bus)]


@dataclass(frozen=True)
class _Descriptor:
  name: str
  method: str | Callable
  args: tuple
  frequency: int
  modulus: int = 1
  counters: tuple = ()
  quality_faults: tuple = ()
  quality_values: tuple = ()


class _RecordingPacker:
  """Proxy preserving the fixture's packet and counter behavior."""
  def __init__(self, packer):
    self.packer = packer
    self.pending = deque(maxlen=512)
    self.encoder_depth = 0
    self.serials = {}

  def __getattr__(self, name):
    return getattr(self.packer, name)

  def make_can_msg_safety(self, name_or_addr, bus, values, fix_checksum=None):
    packet = self.packer.make_can_msg_safety(name_or_addr, bus, values, fix_checksum=fix_checksum)
    frame = _raw(packet)
    key = (frame[0], frame[2])
    serial = self.serials.get(key, 0) + 1
    self.serials[key] = serial
    self.pending.append((packet, frame, dict(values), bool(self.encoder_depth), serial))
    return packet


class _TemplatePacker:
  def make_can_msg_safety(self, name, bus, values, fix_checksum=None):
    return name, bus, dict(values), fix_checksum


class _Bridge:
  def __init__(self, case, descriptors):
    self.case = case
    # Class counters must never advance the real fixture or another test class.
    encoder_type = type(f'{type(case).__name__}StreamEncoder', (type(case),), {})
    self.encoder = encoder_type(methodName='runTest')
    self.encoder.packer = _TemplatePacker()
    for key in dir(encoder_type):
      if key.startswith('cnt_') and isinstance(getattr(encoder_type, key), int):
        setattr(encoder_type, key, 0)
    self.templates = {}
    self.values = {}
    self.descriptors = {}
    self.keys = {}
    self.last_injected = {}
    self.suspended = set()
    self.cycles = {}
    self.invalidated = set()
    for descriptor in descriptors:
      builder = (getattr(self.encoder, descriptor.method) if isinstance(descriptor.method, str)
                 else partial(descriptor.method, self.encoder))
      templates = [builder(*descriptor.args) for _ in range(descriptor.modulus)]
      name, bus, values, checksum = templates[0]
      address, data, bus = CANPacker(case.DBC).make_can_msg(name, bus, values)
      key = (address, bus)
      if key in self.keys:
        raise UnsupportedStreamProfile(f'duplicate required RX descriptor: {key}')
      self.keys[key] = descriptor.name
      self.templates[descriptor.name] = templates
      self.values[descriptor.name] = {k: v for k, v in values.items() if k not in descriptor.counters}
      self.descriptors[descriptor.name] = descriptor
    packer = case.packer
    if isinstance(packer, _RecordingPacker):
      packer = packer.packer
    case.packer = _RecordingPacker(packer)
    _install_drop_hook(case)
    methods = {d.method if isinstance(d.method, str) else d.method.__name__ for d in descriptors}
    if case.SAFETY_MODEL == CarParams.SafetyModel.toyota:
      methods.update(('_user_gas_msg', '_angle_meas_msg'))
    methods.update(_alternate_encoders(case))
    for method in methods:
      if getattr(getattr(case, method), '_fixture_stream_encoder', False):
        continue
      original = getattr(type(case), method).__get__(case, type(case))
      setattr(case, method, self._capture_encoder(original))

  def _capture_encoder(self, original):
    case = self.case

    @wraps(original)
    def encode(*args, **kwargs):
      case.packer.encoder_depth += 1
      try:
        packet = original(*args, **kwargs)
        if case.SAFETY_MODEL == CarParams.SafetyModel.mg and original.__name__ == '_user_gas_msg':
          # This fixture adds only its omitted DBC counter after packing. Seal
          # that documented transformation, never arbitrary post-pack mutation.
          for index, entry in enumerate(case.packer.pending):
            if entry[0] is packet:
              expected = bytearray.fromhex(entry[1][1])
              expected[5] = (expected[5] & 0x0f) | (((case.gas_counter - 1) % 16) << 4)
              frame = _raw(packet)
              if frame == [entry[1][0], expected.hex(), entry[1][2]]:
                case.packer.pending[index] = (entry[0], frame, *entry[2:])
              break
        return packet
      finally:
        case.packer.encoder_depth -= 1
    encode._fixture_stream_encoder = True
    return encode

  def spec(self, name):
    descriptor = self.descriptors[name]
    packer = CANPacker(self.case.DBC)
    frames = []
    templates = self.templates[name]
    samples = self.cycles.get(name, [{}])
    for index in range(lcm(len(templates), len(samples))):
      message, bus, template, checksum = templates[index % len(templates)]
      values = template | self.values[name] | samples[index % len(samples)]
      frame = packer.make_can_msg(message, bus, values)
      if checksum is not None:
        frame = checksum(frame)
      address, data, bus = frame
      frames.append([address, bytes(data).hex(), bus])
    return {'name': name, 'period': 1 / descriptor.frequency, 'frames': frames}

  def prepare_rx(self, packet):
    frame = self.case._raw_frame(packet)
    name = self.keys.get((frame[0], frame[2]))
    if name is None or name in self.suspended:
      return frame, [], None
    captured = next((entry for entry in reversed(self.case.packer.pending) if entry[0] is packet), None)
    if captured is None or captured[1] != frame or not captured[3]:
      # Manual packets and post-encoding corruption are deliberate raw injections.
      return frame, [], None
    assert name not in self.cycles, f'Incidental capture during periodic speed window: {name}'
    # A packet is a one-shot encoder event. Replaying the same object must
    # exercise a stuck counter, rather than acquire a fresh stream counter.
    self.case.packer.pending.remove(captured)
    serial = captured[4]
    previous = self.last_injected.get(name, 0)
    self.last_injected[name] = serial
    if self.descriptors[name].modulus > 1:
      if serial != previous + 1:
        # Discarded encoder calls intentionally skip counters in legacy tests.
        return frame, [], None
    values = captured[2]
    # Quality-fault encoders must exercise the original invalid frame, never turn
    # their fault into a permanently invalid background stream.
    descriptor = self.descriptors[name]
    if (any(values.get(field, 0) for field in descriptor.quality_faults) or
        any(field in values and values[field] not in allowed for field, allowed in descriptor.quality_values)):
      return frame, [], None
    changes = {key: value for key, value in values.items() if key not in descriptor.counters}
    updates = []
    if name in self.invalidated or any(key not in self.values[name] or self.values[name][key] != value
                                      for key, value in changes.items()):
      self.values[name].update(changes)
      self.invalidated.discard(name)
      updates.append(self.spec(name))
    if self.case.SAFETY_MODEL == CarParams.SafetyModel.hyundai and name == 'gas' and 'ems' in self.descriptors:
      # Classic FCEV requires both pedal and EMS traffic. Stock safety parses
      # gas from both, so neutral EMS16 would undo a pressed FCEV pedal on its
      # next periodic frame. Mirror only this explicit test-owned input; each
      # stream keeps its own checksum/counter cycle and backend phase.
      pressed = bool(self.values[name]['ACCELERATOR_PEDAL'])
      if 'ems' not in self.suspended and (self.values['ems']['CF_Ems_AclAct'] != pressed or 'ems' in self.invalidated):
        self.values['ems']['CF_Ems_AclAct'] = pressed
        self.invalidated.discard('ems')
        updates.append(self.spec('ems'))
    return frame, updates, name


def _install_drop_hook(case):
  if getattr(case._drop_required_stream, '_fixture_stream_drop_hook', False):
    return
  original = case._drop_required_stream

  @wraps(original)
  def drop(name):
    original(name)
    bridge = getattr(case, '_fixture_stream_bridge', None)
    if bridge is not None and name in bridge.descriptors:
      bridge.invalidated.add(name)
  drop._fixture_stream_drop_hook = True
  case._drop_required_stream = drop


def _alternate_encoders(case):
  modes = CarParams.SafetyModel
  return {
    modes.gm: ('_pcm_status_msg',),
    modes.mazda: ('_user_gas_msg', '_torque_meas_msg'),
    modes.volkswagenPq: ('_user_brake_msg', '_pcm_status_msg'),
    modes.volkswagenMlb: ('_motor_03_msg', '_user_brake_msg', '_pcm_status_msg'),
    modes.rivian: ('_speed_msg_2',),
    modes.byd: ('_user_gas_msg',),
    modes.ford: ('_user_brake_msg',),
    modes.tesla: ('_vehicle_moving_msg',),
    modes.hondaNidec: ('_pcm_status_msg', '_user_gas_msg', '_user_brake_msg', '_acc_state_msg'),
    modes.hondaBosch: ('_pcm_status_msg', '_user_gas_msg', '_user_brake_msg'),
  }.get(case.SAFETY_MODEL, ())


def _delegated_descriptors(case):
  from dataclasses import replace
  from opendbc.safety.tests import fixture_streams_angle, fixture_streams_torque
  modes = CarParams.SafetyModel
  if case.SAFETY_MODEL in (modes.hondaNidec, modes.hondaBosch):
    from opendbc.safety.tests.fixture_streams_honda import honda_descriptors
    descriptors = honda_descriptors(case)
  else:
    descriptors = fixture_streams_torque.get_descriptors(case)
    if descriptors is None:
      descriptors = fixture_streams_angle.get_descriptors(case)
  if descriptors is None:
    return None
  quality = {
    modes.rivian: {'speed': (('ESP_Vehicle_Speed_Q', (1,)),),
                   'gas_speed': (('VDM_VehicleSpeedQ', (1,)),),
                   'brake': (('iBESP2_BrakePedalApplied_Q', (1,)),)},
    modes.ford: {'speed': (('VehVActlBrk_D_Qf', (3,)),),
                'speed_2': (('VehVActlEng_D_Qf', (3,)),),
                'yaw': (('VehYawWActl_D_Qf', (3,)),)},
    modes.tesla: {'speed_2': (('ESP_wheelSpeedsQF', (True,)),),
                 'brake': (('ESP_driverBrakeApply', (1, 2)),)},
  }.get(case.SAFETY_MODEL, {})
  return [replace(d, quality_values=quality.get(d.name, ())) for d in descriptors]


# Bounded station cadences validated against stock RX watchdogs. Nominal CAN
# rates overload some physical RX stations; both backends use these same rates.
def _descriptors(case):
  if getattr(case, 'IGNITION_ONLY', False):
    return []
  model = case.SAFETY_MODEL
  if model == CarParams.SafetyModel.elm327:
    return []
  if model == CarParams.SafetyModel.toyota:
    from opendbc.car.toyota.values import ToyotaSafetyFlags
    descriptors = [_Descriptor('speed', '_speed_msg', (0,), 25,
                               quality_faults=tuple(f'WHEEL_SPEED_{wheel}_FAULT' for wheel in ('FR', 'FL', 'RR', 'RL'))),
                   _Descriptor('torque', '_torque_meas_msg', (0,), 50,
                               quality_faults=('STEER_ANGLE_INITIALIZING',)),
                   _Descriptor('pcm', '_pcm_status_msg', (False,), 25),
                   _Descriptor('brake', '_user_brake_msg', (False,), 25)]
    if case.SAFETY_PARAM & ToyotaSafetyFlags.SECOC:
      descriptors.append(_Descriptor('gas', '_user_gas_msg', (False,), 25))
    return descriptors
  if model == CarParams.SafetyModel.hyundaiCanfd:
    from opendbc.car.hyundai.values import HyundaiSafetyFlags
    flags = case.SAFETY_PARAM
    # CANPacker owns these counters. A complete 8-bit cycle is essential: a
    # shorter repeated array creates a counter discontinuity at every wrap.
    descriptors = [_Descriptor('gas', '_user_gas_msg', (False,), 25, 256, ('COUNTER',)),
                   _Descriptor('driver_torque', '_torque_driver_msg', (0,), 50, 256, ('COUNTER',)),
                   _Descriptor('speed', '_speed_msg', (0,), 25, 256, ('COUNTER',)),
                   _Descriptor('brake', '_user_brake_msg', (False,), 25, 256, ('COUNTER',)),
                   _Descriptor('buttons', '_button_msg', (0,), 25,
                               256 if flags & HyundaiSafetyFlags.CANFD_ALT_BUTTONS else 16, ('COUNTER',))]
    if not flags & HyundaiSafetyFlags.LONG:
      descriptors.append(_Descriptor('pcm', '_pcm_status_msg', (False,), 25, 256, ('COUNTER',)))
    return descriptors
  if model in (CarParams.SafetyModel.hyundai, CarParams.SafetyModel.hyundaiLegacy):
    from opendbc.car.hyundai.values import HyundaiSafetyFlags
    flags = case.SAFETY_PARAM
    descriptors = [_Descriptor('gas', '_user_gas_msg', (False,), 25, 4, ('AliveCounter',)),
                   _Descriptor('speed', '_speed_msg', (0,), 25, 16,
                               ('WHL_SPD_AliveCounter_LSB', 'WHL_SPD_AliveCounter_MSB')),
                   _Descriptor('brake', '_user_brake_msg', (False,), 25, 8, ('AliveCounterTCS',)),
                   _Descriptor('driver_torque', '_torque_driver_msg', (0,), 50),
                   _Descriptor('buttons', '_button_msg', (0,), 25, 16, ('CF_Clu_AliveCnt1',))]
    if not flags & HyundaiSafetyFlags.LONG:
      descriptors.append(_Descriptor('pcm', '_pcm_status_msg', (False,), 25, 16, ('CR_VSM_Alive',)))
    if flags & (HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.FCEV_GAS):
      descriptors[0] = _Descriptor('gas', '_user_gas_msg', (False,), 25)
    if flags & HyundaiSafetyFlags.FCEV_GAS:
      # FCEV adds a gas check; it does not replace the EMS16/E_EMS11 group.
      from opendbc.safety.tests.test_hyundai import TestHyundaiSafety
      descriptors.append(_Descriptor('ems', TestHyundaiSafety._user_gas_msg, (False,), 25, 4, ('AliveCounter',)))
    return descriptors
  descriptors = _delegated_descriptors(case)
  if descriptors is not None:
    return descriptors
  raise UnsupportedStreamProfile(f'{type(case).__name__}: no required RX descriptors for safety model {model}')


def required_streams(case):
  """Reset test-owned inputs and return every required RX counter/checksum cycle."""
  descriptors = _descriptors(case)
  case._fixture_streams_declared_empty = not descriptors
  if not descriptors:
    case._fixture_stream_bridge = None
    return []
  bridge = _Bridge(case, descriptors)
  if case.SAFETY_MODEL == CarParams.SafetyModel.toyota:
    case._gas_pressed = False
    case._cruise_on = False
  case._fixture_stream_bridge = bridge
  if getattr(case, 'DEFER_REQUIRED_STREAMS', False):
    bridge.invalidated.update(bridge.descriptors)
    return []
  return [bridge.spec(name) for name in bridge.descriptors]


def prepare_rx(case, packet, *, raw=False):
  """Return (original frame, replacement specs, stream to inject).

  Install replacement specs without emitting them, then use backend.rx_stream
  for a named injection. For None, inject the original frame unchanged. The
  backend must retain the schedule and consume exactly one counter slot.
  raw=True preserves intentional counter/checksum injections from helper calls.
  """
  bridge = getattr(case, '_fixture_stream_bridge', None)
  if raw or getattr(case, '_fixture_streams_declared_empty', False):
    frame = case._raw_frame(packet)
    if bridge is not None:
      captured = next((entry for entry in case.packer.pending if entry[0] is packet), None)
      if captured is not None:
        case.packer.pending.remove(captured)
    return frame, [], None
  if bridge is None:
    raise RuntimeError('required_streams(case) must run before prepare_rx')
  return bridge.prepare_rx(packet)


def suspend_required_streams(case, *names):
  """Opt into exact raw RX for a deliberate checksum/counter/timeout scenario.

  Omitting names suspends all required inputs. A backend reset reinstalls neutral
  streams. This is explicit because continuous valid traffic can heal faults.
  """
  bridge = getattr(case, '_fixture_stream_bridge', None)
  if bridge is None:
    if getattr(case, '_fixture_streams_declared_empty', False):
      return
    raise RuntimeError('required_streams(case) must run before suspension')
  names = names or tuple(bridge.descriptors)
  unknown = set(names) - bridge.descriptors.keys()
  if unknown:
    raise ValueError(f'Unknown required streams: {sorted(unknown)}')
  for name in names:
    if name not in bridge.suspended:
      if name not in bridge.invalidated:
        case._drop_required_stream(name)
      bridge.suspended.add(name)


@contextmanager
def periodic_speed_window(case, builders, values):
  """Rivian-only ordered payload evidence, not per-frame timing or full-cycle loss proof."""
  assert case.SAFETY_MODEL == CarParams.SafetyModel.rivian and len(values) == 6
  bridge, backend = case._fixture_stream_bridge, case.backend
  assert not bridge.cycles
  names = []
  try:
    for builder in builders:
      captures = []
      for value in values:
        packet = builder(value)
        entry = next(entry for entry in reversed(case.packer.pending) if entry[0] is packet)
        case.packer.pending.remove(entry)
        frame, fields = entry[1:3]
        name = bridge.keys[(frame[0], frame[2])]
        descriptor = bridge.descriptors[name]
        assert all(fields.get(field) in allowed for field, allowed in descriptor.quality_values)
        captures.append({key: value for key, value in fields.items() if key not in descriptor.counters})
      assert name not in names
      names.append(name)
      varying = {key for key in captures[0] if any(sample[key] != captures[0][key] for sample in captures)}
      bridge.cycles[name] = [{key: sample[key] for key in varying} for sample in captures]
    specs = [bridge.spec(name) for name in names]
    cursor = len(backend.transport_observations)
    phases, counts = {}, dict.fromkeys(names, 0)
    cycles = {spec['name']: [tuple(frame) for frame in spec['frames']] for spec in specs}
    assert all(len(set(cycle)) == len(cycle) for cycle in cycles.values()), 'Ambiguous Rivian cycle'
    old = {}
    # Existing ordinary payloads may already be queued when replacement starts.
    for name in names:
      samples = bridge.cycles.pop(name)
      old[name] = [tuple(frame) for frame in bridge.spec(name)['frames']]
      bridge.cycles[name] = samples
    backend.streams(specs)

    def checkpoint():
      nonlocal cursor
      backend.wait(0)
      for record in backend.transport_observations[cursor:]:
        if record['endpoint'] != 'dut':
          continue
        for raw in record.get('frames', []):
          frame = tuple(raw)
          for name in names:
            cycle = cycles[name]
            if (frame[0], frame[2]) != (cycle[0][0], cycle[0][2]):
              continue
            if name not in phases and frame in old[name]:
              continue
            assert frame in cycle, f'Unexpected periodic speed payload: {name}'
            phase = cycle.index(frame)
            assert name not in phases or phase == (phases[name] + 1) % len(cycle), f'Periodic speed discontinuity: {name}'
            phases[name] = phase
            counts[name] += 1
      cursor = len(backend.transport_observations)

    for _ in range(50):
      checkpoint()
      if min(counts.values()) >= 6:
        break
      backend.wait(.02)
    assert min(counts.values()) >= 6, 'Periodic speed activation missing six ordered deliveries'
    activated = counts.copy()
    try:
      yield checkpoint
    finally:
      checkpoint()
      assert all(counts[name] > activated[name] for name in names), 'Periodic speed made no progress'
  finally:
    for name in names:
      bridge.cycles.pop(name, None)
    if names:
      backend.streams([bridge.spec(name) for name in names])
