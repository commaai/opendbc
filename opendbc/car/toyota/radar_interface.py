#!/usr/bin/env python3
from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.structs import RadarData
from opendbc.car.toyota.values import DBC, ToyotaFlags
from opendbc.car.interfaces import RadarInterfaceBase


def _create_radar_can_parser(CP, use_ars510=False):
  if use_ars510:
    RADAR_A_MSGS = [0x235]
    RADAR_B_MSGS = [0x237]
  elif CP.flags & ToyotaFlags.TSS2:
    RADAR_A_MSGS = list(range(0x180, 0x190))
    RADAR_B_MSGS = list(range(0x190, 0x1a0))
  else:
    RADAR_A_MSGS = list(range(0x210, 0x220))
    RADAR_B_MSGS = list(range(0x220, 0x230))

  msg_a_n = len(RADAR_A_MSGS)
  msg_b_n = len(RADAR_B_MSGS)

  if use_ars510:
    messages = list(zip(RADAR_A_MSGS + RADAR_B_MSGS, [50] * (msg_a_n + msg_b_n), strict=True))
    messages.append(('ACC_TARGET_3', 50))
  else:
    messages = list(zip(RADAR_A_MSGS + RADAR_B_MSGS, [20] * (msg_a_n + msg_b_n), strict=True))
    messages.append(('STATUS_MSG', 10))

  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    # radar-ACC cars (RAV4 2022-25, C-HR, Yaris) have a Continental ARS510: its ACC target becomes the one point
    self.use_ars510 = CP.flags & ToyotaFlags.RADAR_ACC
    if self.use_ars510:
      self.RADAR_A_MSGS = [0x235]
      self.RADAR_B_MSGS = [0x237]
    elif CP.flags & ToyotaFlags.TSS2:
      self.RADAR_A_MSGS = list(range(0x180, 0x190))
      self.RADAR_B_MSGS = list(range(0x190, 0x1a0))
    else:
      self.RADAR_A_MSGS = list(range(0x210, 0x220))
      self.RADAR_B_MSGS = list(range(0x220, 0x230))

    self.valid_cnt = {key: 0 for key in self.RADAR_A_MSGS}

    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP, self.use_ars510)
    self.trigger_msg = self.RADAR_B_MSGS[-1]
    self.ars510_cycle = 0
    self.updated_messages = set()

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    if self.use_ars510:
      # the ACC target frames repeat at 50 Hz; publish once per radar cycle (16.7 Hz), when the radar's clock advances
      cycle = self.rcp.vl['ACC_TARGET_3']['TIMESTAMP']
      if cycle == self.ars510_cycle:
        return None
      self.ars510_cycle = cycle

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.use_ars510:
      target, position = self.rcp.vl["ACC_TARGET_1"], self.rcp.vl["ACC_TARGET_2"]
      if self.rcp.can_valid and target["STATE"] == 5 and position["PRESENT"]:  # 1 = idle, with every field zero
        pt = RadarData.RadarPoint()
        pt.trackId = int(target["TARGET_ID"])
        pt.dRel = position["LONG_DIST"]
        pt.yRel = position["LAT_DIST"]
        pt.vRel = target["REL_SPEED"]
        ret.points = [pt]
      return ret

    if self.rcp.vl['STATUS_MSG']['RADAR_STATUS'] != 1 or self.rcp.vl['STATUS_MSG']['RADAR_PRE_FAULT'] != 0:
      ret.errors.radarUnavailableTemporary = True

    for ii in sorted(updated_messages):
      if ii in self.RADAR_A_MSGS:
        cpt = self.rcp.vl[ii]

        if cpt['LONG_DIST'] >= 255 or cpt['NEW_TRACK']:
          self.valid_cnt[ii] = 0    # reset counter
        if cpt['VALID'] and cpt['LONG_DIST'] < 255:
          self.valid_cnt[ii] += 1
        else:
          self.valid_cnt[ii] = max(self.valid_cnt[ii] - 1, 0)

        score = self.rcp.vl[ii+16]['SCORE']
        # print ii, self.valid_cnt[ii], score, cpt['VALID'], cpt['LONG_DIST'], cpt['LAT_DIST']

        # radar point only valid if it's a valid measurement and score is above 50
        if cpt['VALID'] or (score > 50 and cpt['LONG_DIST'] < 255 and self.valid_cnt[ii] > 0):
          if ii not in self.pts or cpt['NEW_TRACK']:
            self.pts[ii] = RadarData.RadarPoint()
            self.pts[ii].trackId = self.track_id
            self.track_id += 1
          self.pts[ii].dRel = cpt['LONG_DIST']  # from front of car
          self.pts[ii].yRel = -cpt['LAT_DIST']  # in car frame's y axis, left is positive
          self.pts[ii].vRel = cpt['REL_SPEED']
        else:
          if ii in self.pts:
            del self.pts[ii]

    ret.points = list(self.pts.values())
    return ret
