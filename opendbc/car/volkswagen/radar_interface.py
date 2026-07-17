import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.volkswagen.values import DBC, VolkswagenFlags, CanBus

RADAR_ADDR = 0x24F
NO_OBJECT_ID = 0
LANE_TYPES = ("Same_Lane", "Left_Lane", "Right_Lane")
SIGNAL_SETS = tuple(
  (
    f"{prefix}_ObjectID",
    f"{prefix}_Long_Distance",
    f"{prefix}_Lat_Distance",
    f"{prefix}_Rel_Velo",
  )
  for lane in LANE_TYPES
  for idx in (1, 2)
  for prefix in (f"{lane}_0{idx}",)
)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)

    self.updated_messages: set[int] = set()
    self.trigger_msg: int = RADAR_ADDR

    # With the MEB gateway harness, we do not have access to the raw points from the radar.
    # However, the camera publishes decent, albeit filtered, tracks. Two for each lane, left center and right.
    self.rcp: CANParser | None = None
    if CP.flags & VolkswagenFlags.MEB and not self.CP.radarUnavailable:
      self.rcp = CANParser(DBC[CP.carFingerprint][Bus.radar], [("MEB_Distance_01", 25)], CanBus(CP).cam)

    self._pts = self.pts
    self.track_id: int = 0

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    radar_data = self._process_radar_frame()
    self.updated_messages.clear()
    return radar_data

  def _process_radar_frame(self):
    ret = structs.RadarData()

    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True
      return ret

    msg = self.rcp.vl["MEB_Distance_01"]

    # Can be triggered by obstructing radar sensor
    if msg["Distance_Status"] != 0:
      ret.errors.radarUnavailableTemporary = True

    seen_ids = set()
    for obj_id_sig, long_sig, lat_sig, vel_sig in SIGNAL_SETS:
      obj_id = int(msg[obj_id_sig])
      if obj_id == NO_OBJECT_ID:
        continue

      # We shouldn't see duplicate track ids
      if obj_id in seen_ids:
        ret.errors.radarFault = True
        return ret

      seen_ids.add(obj_id)

      if obj_id not in self._pts:
        pt = structs.RadarData.RadarPoint()
        pt.trackId = self.track_id
        self.track_id += 1
        self._pts[obj_id] = pt
      else:
        pt = self._pts[obj_id]

      pt.measured = True
      pt.dRel = msg[long_sig]
      pt.yRel = msg[lat_sig]
      pt.vRel = msg[vel_sig]
      pt.aRel = math.nan
      pt.yvRel = math.nan

    inactive_ids = self._pts.keys() - seen_ids
    for obj_id in inactive_ids:
      self._pts.pop(obj_id, None)

    ret.points = list(self._pts.values())
    return ret
