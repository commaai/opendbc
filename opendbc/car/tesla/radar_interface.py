"""Tesla radar: native Continental ARS4-B on bus 1, or BrownPanda's converted
stream on NGP10/EOP10's party bus 0 when no factory radar tap is present.

BrownPanda places Tesla ARS4-B-compatible classical CAN frames on logical party
bus 0 because BrownPanda exposes only two comma-facing channels. Signal layout
comes from the official ``tesla_radar_continental`` OpenDBC. See
``docs/BROWNPANDA_RADAR.md``.
"""
from __future__ import annotations

from collections.abc import Callable, Iterator
from time import monotonic
from typing import TypedDict

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.tesla.values import CANBUS, CAR, DBC

RADAR_START_ADDR = 0x410
RADAR_MSG_COUNT = 80  # 40 points * 2 messages each

BROWNPANDA_RADAR_CARS = frozenset((CAR.TESLA_MODEL_3, CAR.TESLA_MODEL_Y))

_STATUS_ID = 0x401
_OBJECT_A_BASE = 0x410
_OBJECT_B_BASE = 0x411
_NUM_SLOTS = 40
_TRIGGER_ID = _OBJECT_B_BASE + (_NUM_SLOTS - 1) * 2  # 0x45F
_STATUS_TIMEOUT_S = 0.2
_FAULT_REPORT_PERIOD_S = 0.1


def get_radar_can_parser(CP):
  if CP.carFingerprint not in DBC or Bus.radar not in DBC[CP.carFingerprint]:
    return None

  messages = [('RadarStatus', 16)]
  for i in range(RADAR_MSG_COUNT // 2):
    messages.extend([
      (f'RadarPoint{i}_A', 16),
      (f'RadarPoint{i}_B', 16),
    ])

  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class _NativeRadarInterface:
  """Factory Continental radar wired to physical bus 1, decoded via the DBC."""

  def __init__(self, CP):
    self.updated_messages = set()
    self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1
    self.rcp = get_radar_can_parser(CP)
    self.pts: dict[int, structs.RadarData.RadarPoint] = {}
    self.track_id = 0

  def update(self, can_strings) -> structs.RadarDataT | None:
    if self.rcp is None:
      return None

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    ret = self._update(self.updated_messages)
    self.updated_messages.clear()
    return ret

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    radar_status = self.rcp.vl['RadarStatus']
    if radar_status['shortTermUnavailable']:
      ret.errors.radarUnavailableTemporary = True
    if radar_status['sensorBlocked'] or radar_status['vehDynamicsError']:
      ret.errors.radarFault = True

    for i in range(RADAR_MSG_COUNT // 2):
      msg_a = self.rcp.vl[f'RadarPoint{i}_A']
      msg_b = self.rcp.vl[f'RadarPoint{i}_B']

      # Make sure msg A and B are together
      if msg_a['Index'] != msg_b['Index2']:
        continue

      if not msg_a['Tracked']:
        if i in self.pts:
          del self.pts[i]
        continue

      if i not in self.pts:
        self.pts[i] = structs.RadarData.RadarPoint()
        self.pts[i].trackId = self.track_id
        self.track_id += 1

      self.pts[i].dRel = msg_a['LongDist']
      self.pts[i].yRel = msg_a['LatDist']
      self.pts[i].vRel = msg_a['LongSpeed']

    ret.points = list(self.pts.values())
    return ret


class _ObjectA(TypedDict):
  tracked: bool
  valid: bool
  measured: bool
  index: int
  dRel: float
  vRel: float
  yRel: float


class _ObjectB(TypedDict):
  index2: int


def _read_le(data: bytes, start_bit: int, size: int, scale: float = 1.0, offset: float = 0.0) -> float:
  """Read an Intel signal using the official Continental DBC bit layout."""
  value = 0
  for i in range(size):
    bit = start_bit + i
    byte_index = bit >> 3
    if byte_index >= len(data):
      return 0.0
    if data[byte_index] & (1 << (bit & 7)):
      value |= 1 << i
  return value * scale + offset


def _messages(can_packets: list[tuple[int, list[CanData]] | list[CanData] | CanData]) -> Iterator[CanData]:
  for packet in can_packets:
    # Card supplies timestamped packet groups. Accept a plain message list too
    # so focused tests and replay tools can exercise the parser directly.
    if isinstance(packet, CanData):
      yield packet
    else:
      messages = packet[1] if isinstance(packet, tuple) else packet
      yield from messages


class _BrownPandaRadarInterface:
  """Parse BrownPanda's complete 40-pair stream from party bus 0."""

  def __init__(self, CP: structs.CarParams, time_fn: Callable[[], float] = monotonic):
    self._time_fn = time_fn
    self._object_a: dict[int, _ObjectA] = {}
    self._object_b: dict[int, _ObjectB] = {}
    self._sensor_fault = False
    self._sensor_unavailable = False
    self._have_status = False
    self._status_time = self._time_fn()
    self._last_fault_time = self._status_time - _FAULT_REPORT_PERIOD_S
    self.pts: dict[int, structs.RadarData.RadarPoint] = {}

  def update(self, can_packets: list[tuple[int, list[CanData]] | list[CanData] | CanData]) -> structs.RadarDataT | None:
    triggered = False

    for message in _messages(can_packets):
      if message.src != CANBUS.party:
        continue

      address = message.address
      data = bytes(message.dat)
      if address == _STATUS_ID and len(data) == 8:
        # Status starts a new set. Never combine an old half-pair after the
        # one-bit pair index wraps.
        self._object_a.clear()
        self._object_b.clear()
        self._sensor_unavailable = bool(_read_le(data, 23, 1) or _read_le(data, 26, 1))
        self._sensor_fault = bool(_read_le(data, 27, 1))
        self._status_time = self._time_fn()
        self._have_status = True

      elif _OBJECT_A_BASE <= address < _OBJECT_A_BASE + _NUM_SLOTS * 2 and (address & 1) == 0:
        if len(data) == 8:
          slot = (address - _OBJECT_A_BASE) // 2
          self._object_a[slot] = {
            "tracked": bool(_read_le(data, 62, 1)),
            "valid": bool(_read_le(data, 55, 1)),
            "measured": bool(_read_le(data, 61, 1)),
            "index": int(_read_le(data, 63, 1)),
            "dRel": _read_le(data, 0, 12, scale=0.0625),
            "vRel": _read_le(data, 12, 12, scale=0.0625, offset=-128.0),
            "yRel": _read_le(data, 24, 11, scale=0.125, offset=-128.0),
          }

      elif _OBJECT_B_BASE <= address < _OBJECT_B_BASE + _NUM_SLOTS * 2 and (address & 1) == 1:
        if len(data) == 8:
          slot = (address - _OBJECT_B_BASE) // 2
          self._object_b[slot] = {"index2": int(_read_le(data, 63, 1))}
          if address == _TRIGGER_ID:
            triggered = True

    now = self._time_fn()
    if not triggered:
      if now - self._status_time <= _STATUS_TIMEOUT_S:
        return None
      if now - self._last_fault_time < _FAULT_REPORT_PERIOD_S:
        return None
      self._last_fault_time = now
      return self._fault_result(unavailable=True)

    status_fresh = self._have_status and now - self._status_time <= _STATUS_TIMEOUT_S
    if not status_fresh or self._sensor_unavailable or self._sensor_fault:
      return self._fault_result(unavailable=not self._sensor_fault)

    expected_slots = set(range(_NUM_SLOTS))
    trigger_index = self._object_b[_NUM_SLOTS - 1]["index2"]
    set_complete = self._object_a.keys() == expected_slots and self._object_b.keys() == expected_slots
    indexes_coherent = set_complete and all(
      self._object_a[slot]["index"] == trigger_index and self._object_b[slot]["index2"] == trigger_index
      for slot in expected_slots
    )
    if not indexes_coherent:
      return self._fault_result(unavailable=True)

    current_tracks: set[int] = set()
    for slot in range(_NUM_SLOTS):
      track_id = slot + 1
      point_a = self._object_a.get(slot)
      point_b = self._object_b.get(slot)
      if (point_a is None or point_b is None
          or not point_a["tracked"] or not point_a["valid"] or not point_a["measured"]
          or point_a["index"] != point_b["index2"]):
        continue

      current_tracks.add(track_id)
      if track_id not in self.pts:
        self.pts[track_id] = structs.RadarData.RadarPoint()
        self.pts[track_id].trackId = track_id

      point = self.pts[track_id]
      point.dRel = point_a["dRel"]
      point.yRel = point_a["yRel"]
      point.vRel = point_a["vRel"]
      # BrownPanda reserves these unproven BYD motion fields on the wire.
      # NaN is OpenDBC's canonical marker for an unavailable optional field.
      point.aRel = float("nan")
      point.yvRel = float("nan")
      point.measured = True

    for track_id in list(self.pts):
      if track_id not in current_tracks:
        del self.pts[track_id]

    result = structs.RadarData()
    result.points = list(self.pts.values())
    self._clear_pending()
    return result

  def _fault_result(self, unavailable: bool) -> structs.RadarDataT:
    self.pts.clear()
    self._clear_pending()
    result = structs.RadarData()
    if unavailable:
      result.errors.radarUnavailableTemporary = True
    else:
      result.errors.radarFault = True
    result.points = []
    return result

  def _clear_pending(self) -> None:
    self._object_a.clear()
    self._object_b.clear()


class RadarInterface(RadarInterfaceBase):
  """Dispatches to whichever radar source is actually wired for this car.

  Factory-radar Model 3/Y have a DBC ``Bus.radar`` entry and are tapped to
  physical bus 1; BrownPanda-gatewayed cars have no bus-1 tap and instead
  advertise the same Continental ARS4-B layout on logical party bus 0. Both
  sub-parsers are self-gating (they simply never trigger if their bus has no
  traffic), so running both and taking whichever produces a result handles
  either hardware configuration without the caller having to know which one
  is present.
  """

  def __init__(self, CP: structs.CarParams, time_fn: Callable[[], float] = monotonic):
    super().__init__(CP)
    self._native: _NativeRadarInterface | None = None
    self._brownpanda: _BrownPandaRadarInterface | None = None
    if not CP.radarUnavailable:
      self._native = _NativeRadarInterface(CP)
      if CP.carFingerprint in BROWNPANDA_RADAR_CARS:
        self._brownpanda = _BrownPandaRadarInterface(CP, time_fn=time_fn)

  def update(self, can_packets) -> structs.RadarDataT | None:
    self.frame += 1
    if self._native is None and self._brownpanda is None:
      return super().update(None)

    native_result = self._native.update(can_packets) if self._native is not None else None
    if native_result is not None:
      return native_result

    if self._brownpanda is not None:
      return self._brownpanda.update(can_packets)

    return None
