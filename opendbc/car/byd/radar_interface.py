from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import RadarData
from opendbc.car.byd.values import DBC
from opendbc.car.interfaces import RadarInterfaceBase

N_SLOTS = 10
TRIGGER_MSG = 0x289
EMPTY_TRACK_ID = 255
IDLE_FRAMES = 20
IDLE_PUBLISH_DIV = 5
MIN_CONF = 0.30
MAX_LONG_M = 200.0
BUMPER_TO_RADAR_ORIGIN_M = 3.60
CUTIN_MAX_LONG_M = 20.0
CUTIN_MAX_LAT_M = 3.5
CUTIN_MIN_CYCLES = 2
CUTIN_MIN_VLEAD_MS = 1.0


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages = set()
    self.last_track_frame = 0
    self.v_ego = 0.0
    self.rcp = None
    self.cp_ego = None
    self.seen = {}
    if not CP.radarUnavailable:
      messages = [(f"RADAR_TRACK_{i:02d}", 13) for i in range(N_SLOTS)]
      self.rcp = CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)
      self.cp_ego = CANParser(DBC[CP.carFingerprint][Bus.pt], [("WHEELSPEED_CLEAN", 50)], 0)

  def update(self, can_packets):
    if self.rcp is None:
      return super().update(None)

    self.cp_ego.update(can_packets)
    if self.cp_ego.can_valid:
      self.v_ego = float(self.cp_ego.vl["WHEELSPEED_CLEAN"]["WHEELSPEED_CLEAN"]) * CV.KPH_TO_MS

    self.updated_messages.update(self.rcp.update(can_packets))
    self.frame += 1
    if TRIGGER_MSG not in self.updated_messages:
      if (self.frame - self.last_track_frame) > IDLE_FRAMES and (self.frame % IDLE_PUBLISH_DIV) == 0:
        self.pts.clear()
        self.seen.clear()
        return RadarData()
      return None
    self.updated_messages.clear()
    self.last_track_frame = self.frame

    for slot in range(N_SLOTS):
      sig = self.rcp.vl[f"RADAR_TRACK_{slot:02d}"]
      raw_tid = sig["TRACK_ID"]
      tid = EMPTY_TRACK_ID if raw_tid is None else int(raw_tid)
      raw_long = float(sig["LONG_DIST"] or 0.0)
      long_dist = raw_long - BUMPER_TO_RADAR_ORIGIN_M
      conf = float(sig["CONFIDENCE"] or 0.0)
      lat_dist = float(sig["LAT_DIST"] or 0.0)
      v_lead = float(sig["VLEAD"] or 0.0)
      if tid == EMPTY_TRACK_ID:
        self.seen.pop(slot, None)
        self.pts.pop(slot, None)
        continue
      prev = self.seen.get(slot)
      seen = prev[1] + 1 if prev is not None and prev[0] == tid else 1
      self.seen[slot] = (tid, seen)
      conf_ok = conf >= MIN_CONF or (long_dist < CUTIN_MAX_LONG_M
                                     and abs(lat_dist) <= CUTIN_MAX_LAT_M
                                     and seen >= CUTIN_MIN_CYCLES
                                     and v_lead > CUTIN_MIN_VLEAD_MS)
      if not conf_ok or not (0.0 < long_dist <= MAX_LONG_M):
        self.pts.pop(slot, None)
        continue
      pt = RadarData.RadarPoint()
      pt.trackId = tid
      pt.dRel = long_dist
      pt.yRel = lat_dist
      pt.vRel = v_lead - self.v_ego
      self.pts[slot] = pt

    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True
    ret.points = list(self.pts.values())
    return ret
