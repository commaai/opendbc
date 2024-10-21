import math

from opendbc.can.parser import CANParser
from opendbc.car import structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.values import DBC, VolkswagenFlags
from collections import defaultdict

RADAR_ADDR = 0x24F
NO_OBJECT  = -5
LANE_TYPES = ['Same_Lane', 'Left_Lane', 'Right_Lane']

# info: distance signals can move without physical distance change ...

def get_radar_can_parser(CP):
  if CP.flags & VolkswagenFlags.MEB:
    messages = [("MEB_Distance_01", 25)]
  else:
    return None

  return CANParser(DBC[CP.carFingerprint]['radar'], messages, 2)
  

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages = set()
    self.trigger_msg = RADAR_ADDR
    self.track_id = 0
    self.previous_offsets = defaultdict(lambda: NO_OBJECT)

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = structs.RadarData()

    if self.rcp is None or not self.rcp.can_valid:
      ret.errors = ["canError"]
      return ret

    msg = self.rcp.vl["MEB_Distance_01"]

    # Iterate over lane types and dynamic signal parts (01, 02)
    for lane_type in LANE_TYPES:
      for idx in range(1, 3):
        signal_part = f'{lane_type}_0{idx}'
        long_distance = f'{signal_part}_Long_Distance'
        ld_offset = f'{signal_part}_LD_Offset'
        lat_distance = f'{signal_part}_Lat_Distance'
        rel_velo = f'{signal_part}_Rel_Velo'

        current_offset = msg[ld_offset]
            
        if signal_part not in self.pts:
          self.pts[signal_part] = structs.RadarData.RadarPoint()
          self.pts[signal_part].trackId = self.track_id
          self.track_id += 1

        # offset changes occur when another object is detected
        # this skips a frame of data
        if current_offset != NO_OBJECT and current_offset == self.previous_offsets[signal_part]:            
          self.pts[signal_part].measured = True
          self.pts[signal_part].dRel = msg[long_distance] + current_offset
          self.pts[signal_part].yRel = msg[lat_distance]
          self.pts[signal_part].vRel = msg[rel_velo] * CV.KPH_TO_MS
          self.pts[signal_part].aRel = float('nan')
          self.pts[signal_part].yvRel = float('nan')
        else:
          self.pts.pop(signal_part, None)
            
        self.previous_offsets[signal_part] = current_offset

    ret.points = list(self.pts.values())
    return ret
