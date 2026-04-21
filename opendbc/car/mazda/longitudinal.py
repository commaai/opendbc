from __future__ import annotations

from enum import Enum

from opendbc.can.dbc import DBC
from opendbc.can.packer import set_value
from opendbc.car import make_tester_present_msg, uds
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery


MAZDA_LONG_DBC = DBC("mazda_2017")

RADAR_ADDR = 0x764
RADAR_BUS = 0

CRZ_INFO_ADDR = 0x21B
CRZ_CTRL_ADDR = 0x21C

CRZ_INFO_TEMPLATE = bytes.fromhex("01ffe20006800000")

LONG_COMMAND_STEP = 2
TESTER_PRESENT_STEP = 50

ACCEL_CMD_MAX = 2000.0
ACCEL_CMD_MIN = -2000.0
HOLD_BRAKE_CMD_TARGET = -1024.0
HOLD_LATCHED_CMD_TARGET = -1.0
NEAR_STOP_BRAKE_CMD_TARGET = -750.0
NEAR_STOP_ENTRY_SPEED = 1.0
ACTIVE_STOP_CHECKSUM_BIAS = 0x04

# Stock Mazda longitudinal is not using one global raw-command scale across all
# speeds. Keep more authority at low/mid speed, and soften the map at highway
# speed where the single-scale version feels jerky.
ACCEL_SCALE_UP_BP = (0.0, 4.2, 11.1, 22.2)
ACCEL_SCALE_UP_V = (1000.0, 1000.0, 950.0, 800.0)
ACCEL_SCALE_DOWN_BP = (0.0, 1.4, 5.6, 22.2)
ACCEL_SCALE_DOWN_V = (1200.0, 1000.0, 925.0, 950.0)


class MazdaLongitudinalProfile(str, Enum):
  STANDBY = "standby"
  ENGAGED_CRUISE = "engaged_cruise"
  ENGAGED_FOLLOW = "engaged_follow"
  STOP_GO_HOLD = "stop_go_hold"
  STOP_GO_HOLD_LATCHED = "stop_go_hold_latched"


CRZ_CTRL_TEMPLATES: dict[MazdaLongitudinalProfile, bytes] = {
  MazdaLongitudinalProfile.STANDBY: bytes.fromhex("02010b0000000000"),
  MazdaLongitudinalProfile.ENGAGED_CRUISE: bytes.fromhex("0a018b2000001000"),
  MazdaLongitudinalProfile.ENGAGED_FOLLOW: bytes.fromhex("0a018b4000001000"),
  MazdaLongitudinalProfile.STOP_GO_HOLD: bytes.fromhex("0a018b6000001000"),
  MazdaLongitudinalProfile.STOP_GO_HOLD_LATCHED: bytes.fromhex("0a018b8000001000"),
}


def _get_signal(message_name: str, signal_name: str):
  return MAZDA_LONG_DBC.name_to_msg[message_name].sigs[signal_name]


def _patch_signal(message_name: str, raw: bytes, signal_name: str, value: float) -> bytes:
  sig = _get_signal(message_name, signal_name)
  encoded = int(round((value - sig.offset) / sig.factor))
  if encoded < 0:
    encoded = (1 << sig.size) + encoded

  dat = bytearray(raw)
  set_value(dat, sig, encoded)
  return bytes(dat)


def _compute_inverted_sum_checksum(raw: bytes, checksum_index: int = 7) -> int:
  return (0xFF - (sum(raw[i] for i in range(len(raw)) if i != checksum_index) & 0xFF)) & 0xFF


def _update_crz_info_checksum(raw: bytes, bias: int = 0) -> bytes:
  dat = bytearray(raw)
  dat[7] = (_compute_inverted_sum_checksum(dat) + bias) & 0xFF
  return bytes(dat)


def clip(value: float, lower: float, upper: float) -> float:
  return min(max(value, lower), upper)


def _interp_scale(v_ego: float, bp: tuple[float, ...], values: tuple[float, ...]) -> float:
  if v_ego <= bp[0]:
    return values[0]
  if v_ego >= bp[-1]:
    return values[-1]

  for i in range(1, len(bp)):
    if v_ego <= bp[i]:
      x0, x1 = bp[i - 1], bp[i]
      y0, y1 = values[i - 1], values[i]
      ratio = (v_ego - x0) / (x1 - x0)
      return y0 + (y1 - y0) * ratio

  return values[-1]


def accel_to_accel_cmd(accel: float, v_ego: float) -> int:
  scale = _interp_scale(v_ego, ACCEL_SCALE_UP_BP, ACCEL_SCALE_UP_V) if accel >= 0.0 else _interp_scale(v_ego, ACCEL_SCALE_DOWN_BP, ACCEL_SCALE_DOWN_V)
  return int(round(clip(accel * scale, ACCEL_CMD_MIN, ACCEL_CMD_MAX)))


def hold_brake_accel() -> float:
  # Stock HOLD keeps a strong negative CRZ_INFO command alive through the
  # active stop/hold phase until the chassis hold latch takes over.
  # Keep the raw target approximately constant as scales change.
  return HOLD_BRAKE_CMD_TARGET / ACCEL_SCALE_DOWN_V[0]


def hold_latched_accel() -> float:
  # Once the chassis hold latch takes over, stock CRZ_INFO.ACCEL_CMD relaxes
  # back near zero and the stop bits clear.
  return HOLD_LATCHED_CMD_TARGET / ACCEL_SCALE_DOWN_V[0]


def near_stop_brake_accel(v_ego: float) -> float:
  # Stock stop-to-hold ramps into the final HOLD brake command before true
  # standstill, rather than waiting until the speed bit drops to zero.
  ratio = clip(v_ego / NEAR_STOP_ENTRY_SPEED, 0.0, 1.0)
  target = HOLD_BRAKE_CMD_TARGET + (NEAR_STOP_BRAKE_CMD_TARGET - HOLD_BRAKE_CMD_TARGET) * ratio
  return target / ACCEL_SCALE_DOWN_V[0]


def build_crz_info(accel: float, counter: int, long_active: bool, hold_request: bool, v_ego: float,
                   hold_latched: bool = False, acc_set_allowed: bool = True,
                   resume_unlatching: bool = False) -> bytes:
  stopping_active = hold_request and not hold_latched
  raw = _patch_signal("CRZ_INFO", CRZ_INFO_TEMPLATE, "ACCEL_CMD", accel_to_accel_cmd(accel, v_ego))
  raw = _patch_signal("CRZ_INFO", raw, "ACC_ACTIVE", int(long_active))
  raw = _patch_signal("CRZ_INFO", raw, "ACC_SET_ALLOWED", int(acc_set_allowed))
  raw = _patch_signal("CRZ_INFO", raw, "CRZ_ENDED", 0)
  raw = _patch_signal("CRZ_INFO", raw, "STOPPING_MAYBE", int(stopping_active))
  raw = _patch_signal("CRZ_INFO", raw, "STOPPING_MAYBE2", int(stopping_active))
  raw = _patch_signal("CRZ_INFO", raw, "RESUME_UNLATCHING_MAYBE", int(resume_unlatching))
  raw = _patch_signal("CRZ_INFO", raw, "CTR1", counter % 16)
  checksum_bias = ACTIVE_STOP_CHECKSUM_BIAS if stopping_active else 0
  return _update_crz_info_checksum(raw, bias=checksum_bias)


def select_profile(long_active: bool, lead_visible: bool, hold_request: bool,
                   crz_hold_latched: bool) -> MazdaLongitudinalProfile:
  if not long_active:
    return MazdaLongitudinalProfile.STANDBY
  if hold_request and crz_hold_latched:
    return MazdaLongitudinalProfile.STOP_GO_HOLD_LATCHED
  if hold_request:
    return MazdaLongitudinalProfile.STOP_GO_HOLD
  if lead_visible:
    return MazdaLongitudinalProfile.ENGAGED_FOLLOW
  return MazdaLongitudinalProfile.ENGAGED_CRUISE


def build_crz_ctrl(long_active: bool, lead_visible: bool, hold_request: bool, hold_latched: bool,
                   crz_hold_latched: bool = False, crz_hold_passive: bool = False,
                   crz_resume_active: bool = False) -> bytes:
  # Stock stop-and-go progresses through multiple CRZ_CTRL stop phases. Mirror
  # that sequence so the synthetic path keeps the same latch states as stock.
  lead_visible = lead_visible or hold_request or hold_latched or crz_hold_latched or crz_hold_passive
  raw = CRZ_CTRL_TEMPLATES[select_profile(long_active, lead_visible, hold_request, crz_hold_latched)]
  raw = _patch_signal("CRZ_CTRL", raw, "CRZ_ACTIVE", int(long_active))
  raw = _patch_signal("CRZ_CTRL", raw, "ACC_ACTIVE_2", int(long_active and not crz_hold_passive))
  raw = _patch_signal("CRZ_CTRL", raw, "DISABLE_TIMER_1", 0)
  raw = _patch_signal("CRZ_CTRL", raw, "DISABLE_TIMER_2", 0)
  raw = _patch_signal("CRZ_CTRL", raw, "RADAR_HAS_LEAD", int(lead_visible))
  # Stock resume transitions rely on more than the coarse 0x21c templates. The
  # live radar path preserves these fields automatically, but the synthetic path
  # has to set them explicitly to match passive hold (distance 4), active
  # stop-go / resume (distance 3 + ACC_GAS_MAYBE2), and follow (distance 2).
  if crz_hold_passive or crz_hold_latched:
    raw = _patch_signal("CRZ_CTRL", raw, "RADAR_LEAD_RELATIVE_DISTANCE", 4)
    raw = _patch_signal("CRZ_CTRL", raw, "ACC_GAS_MAYBE2", 0)
  elif hold_request or crz_resume_active:
    raw = _patch_signal("CRZ_CTRL", raw, "RADAR_LEAD_RELATIVE_DISTANCE", 3)
    raw = _patch_signal("CRZ_CTRL", raw, "ACC_GAS_MAYBE2", 1)
  return raw


def create_longitudinal_messages(bus: int, accel: float, counter: int, long_active: bool,
                                 lead_visible: bool, standstill: bool, *, hold_request: bool = False,
                                 crz_ctrl_hold_request: bool | None = None,
                                 hold_latched: bool = False, crz_hold_latched: bool = False,
                                 crz_hold_passive: bool = False,
                                 crz_resume_active: bool = False,
                                 crz_info_resume_unlatching: bool = False,
                                 v_ego: float = 0.0) -> list[CanData]:
  if crz_ctrl_hold_request is None:
    crz_ctrl_hold_request = hold_request

  return [
    CanData(CRZ_INFO_ADDR, build_crz_info(accel, counter, long_active, hold_request, v_ego,
                                          hold_latched=hold_latched,
                                          resume_unlatching=crz_info_resume_unlatching), bus),
    CanData(CRZ_CTRL_ADDR, build_crz_ctrl(long_active, lead_visible, crz_ctrl_hold_request, hold_latched,
                                          crz_hold_latched=crz_hold_latched,
                                          crz_hold_passive=crz_hold_passive,
                                          crz_resume_active=crz_resume_active), bus),
  ]


def create_radar_tester_present(bus: int = RADAR_BUS) -> CanData:
  return make_tester_present_msg(RADAR_ADDR, bus, suppress_response=True)

def _uds_request(can_recv, can_send, bus: int, addr: int, request: bytes, response: bytes,
                 *, timeout: float = 0.1) -> bool:
  query = IsoTpParallelQuery(can_send, can_recv, bus, [(addr, None)], [request], [response])
  return len(query.get_data(timeout)) > 0


def enter_radar_programming_session(can_recv, can_send, bus: int = RADAR_BUS, addr: int = RADAR_ADDR,
                                    retry: int = 5) -> bool:
  request = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.PROGRAMMING])
  response = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, uds.SESSION_TYPE.PROGRAMMING])

  for attempt in range(retry):
    try:
      if _uds_request(can_recv, can_send, bus, addr, request, response):
        carlog.warning(f"mazda radar programming session enabled on {hex(addr)}")
        return True
    except Exception:
      carlog.exception("mazda radar programming session exception")
    carlog.error(f"mazda radar programming session retry ({attempt + 1})")

  carlog.error("mazda radar programming session failed")
  return False


def request_radar_default_session(can_recv, can_send, bus: int = RADAR_BUS, addr: int = RADAR_ADDR) -> bool:
  request = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.DEFAULT])
  response = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, uds.SESSION_TYPE.DEFAULT])

  try:
    return _uds_request(can_recv, can_send, bus, addr, request, response)
  except Exception:
    carlog.exception("mazda radar default session exception")
    return False
