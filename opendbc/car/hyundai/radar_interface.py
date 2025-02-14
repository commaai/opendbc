from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC, HyundaiFlags

RADAR_START_ADDR_CAN = 0x500
RADAR_START_ADDR_CANFD = 0x3A5
RADAR_MSG_COUNT = 32

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/

def get_radar_can_parser(CP, RADAR_START_ADDR):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.RADAR_START_ADDR = RADAR_START_ADDR_CANFD if CP.flags & HyundaiFlags.CANFD else RADAR_START_ADDR_CAN
    self.updated_messages = set()
    self.trigger_msg = self.RADAR_START_ADDR + RADAR_MSG_COUNT - 1
    self.track_id = 0

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP, self.RADAR_START_ADDR)

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
    if self.rcp is None:
      return ret

    errors = []

    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    for addr in range(self.RADAR_START_ADDR, self.RADAR_START_ADDR + RADAR_MSG_COUNT):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      valid = msg['STATE'] in (3, 4)
      if valid:
        # todo: gate for ccnc and undo changes
        self.pts[addr].measured = True
        self.pts[addr].dRel = msg['LONG_DIST']
        self.pts[addr].yRel = msg['LAT_DIST'] * 0.25
        self.pts[addr].vRel = msg['REL_SPEED']
        self.pts[addr].aRel = msg['REL_ACCEL']
        self.pts[addr].yvRel = float('nan')

      else:
        del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret
