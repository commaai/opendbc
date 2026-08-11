import math

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.radar_interface import RadarInterface
from opendbc.car.tesla.values import CANBUS, CAR


def _set_le(data: bytearray, start_bit: int, size: int, raw: int) -> None:
  for i in range(size):
    if raw & (1 << i):
      bit = start_bit + i
      data[bit >> 3] |= 1 << (bit & 7)


def _message(address: int, data: bytes, bus: int = 0) -> CanData:
  return CanData(address=address, dat=data, src=bus)


def _packet(*messages: CanData):
  return [(0, list(messages))]


def _point_pair(slot: int, index: int = 1, measured: bool = True,
                wire_a_rel: float = 0.0, wire_yv_rel: float = 0.0):
  point_a = bytearray(8)
  point_b = bytearray(8)
  _set_le(point_a, 0, 12, round(42.5 / 0.0625))
  _set_le(point_a, 12, 12, round((-3.0 + 128.0) / 0.0625))
  _set_le(point_a, 24, 11, round((2.25 + 128.0) / 0.125))
  _set_le(point_a, 40, 10, round((wire_a_rel + 16.0) / 0.03125))
  _set_le(point_a, 55, 1, 1)
  _set_le(point_a, 61, 1, int(measured))
  _set_le(point_a, 62, 1, 1)
  _set_le(point_a, 63, 1, index)
  _set_le(point_b, 0, 10, round((wire_yv_rel + 64.0) / 0.125))
  _set_le(point_b, 63, 1, index)
  return _message(0x410 + slot * 2, bytes(point_a)), _message(0x411 + slot * 2, bytes(point_b))


def _complete_set(selected_slot: int | None = None, **selected_kwargs):
  messages = []
  for slot in range(40):
    if slot == selected_slot:
      messages.extend(_point_pair(slot, **selected_kwargs))
    else:
      messages.extend((_message(0x410 + slot * 2, bytes.fromhex("0000000000000080")),
                       _message(0x411 + slot * 2, bytes.fromhex("0000000000000080"))))
  return messages


def _car_params(party_fingerprint):
  fingerprint = gen_empty_fingerprint()
  fingerprint[CANBUS.party] = party_fingerprint
  return CarInterface._get_params(structs.CarParams(), CAR.TESLA_MODEL_3, fingerprint, [], False, False, False)


def test_brownpanda_radar_capability_fingerprint():
  assert _car_params({}).radarUnavailable
  assert _car_params({0x401: 8}).radarUnavailable
  assert _car_params({0x401: 8, 0x45F: 7}).radarUnavailable
  assert not _car_params({0x401: 8, 0x45F: 8}).radarUnavailable


def test_party_bus_point_and_reserved_motion_fields():
  radar = RadarInterface(structs.CarParams(carFingerprint=CAR.TESLA_MODEL_3))
  assert radar.update(_packet(_message(0x401, bytes(8)))) is None
  messages = _complete_set(7, wire_a_rel=7.0, wire_yv_rel=-8.0)
  result = radar.update(_packet(*messages))

  assert result is not None
  assert not any(result.errors.to_dict().values())
  assert len(result.points) == 1
  point = result.points[0]
  assert point.trackId == 8
  assert abs(point.dRel - 42.5) < 1e-6
  assert abs(point.yRel - 2.25) < 1e-6
  assert abs(point.vRel + 3.0) < 1e-6
  assert math.isnan(point.deprecated.aRel)
  assert math.isnan(point.deprecated.yvRel)
  assert point.deprecated.measured


def test_wrong_bus_unavailable_status_and_stale_status_fail_closed():
  now = [100.0]
  radar = RadarInterface(structs.CarParams(carFingerprint=CAR.TESLA_MODEL_3), time_fn=lambda: now[0])
  assert radar.update(_packet(_message(0x401, bytes(8), bus=1))) is None

  unavailable = bytearray(8)
  _set_le(unavailable, 23, 1, 1)
  assert radar.update(_packet(_message(0x401, bytes(unavailable)))) is None
  result = radar.update(_packet(*_complete_set(0)))
  assert result is not None
  assert result.errors.radarUnavailableTemporary
  assert not result.points

  assert radar.update(_packet(_message(0x401, bytes(8)))) is None
  now[0] += 0.201
  result = radar.update(_packet(*_complete_set(0)))
  assert result is not None
  assert result.errors.radarUnavailableTemporary
  assert not result.points


def test_incomplete_unmeasured_and_short_trigger_do_not_publish():
  radar = RadarInterface(structs.CarParams(carFingerprint=CAR.TESLA_MODEL_3))
  point_a, point_b = _point_pair(0)
  assert radar.update(_packet(_message(0x401, bytes(8)), point_a)) is None
  result = radar.update(_packet(_message(0x401, bytes(8)), point_b, _message(0x45F, bytes(8))))
  assert result is not None
  assert result.errors.radarUnavailableTemporary
  assert not radar._brownpanda.pts

  assert radar.update(_packet(_message(0x401, bytes(8)))) is None
  point_a, point_b = _point_pair(0)
  assert radar.update(_packet(point_a, point_b, _message(0x45F, bytes(7)))) is None

  assert radar.update(_packet(_message(0x401, bytes(8)))) is None
  result = radar.update(_packet(*_complete_set(0, measured=False)))
  assert result is not None
  assert not result.points


def test_mixed_cycle_index_and_vehicle_dynamics_fault_fail_closed():
  radar = RadarInterface(structs.CarParams(carFingerprint=CAR.TESLA_MODEL_3))
  assert radar.update(_packet(_message(0x401, bytes(8)))) is None
  result = radar.update(_packet(*_complete_set(0, index=0)))
  assert result is not None
  assert result.errors.radarUnavailableTemporary
  assert not result.points

  vehicle_dynamics_fault = bytearray(8)
  _set_le(vehicle_dynamics_fault, 27, 1, 1)
  assert radar.update(_packet(_message(0x401, bytes(vehicle_dynamics_fault)))) is None
  result = radar.update(_packet(*_complete_set(0)))
  assert result is not None
  assert result.errors.radarFault
  assert not result.points
