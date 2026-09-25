#!/usr/bin/env python3
from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.structs import RadarData
from opendbc.car.toyota.values import DBC, ToyotaFlags
from opendbc.car.interfaces import RadarInterfaceBase

TSS3_GEOMETRY_MSGS = [0x180, 0x181, 0x182]
TSS3_MOTION_MSGS = [0x183, 0x184, 0x185]
TSS3_SLOTS = 8
TSS3_DIST_INVALID = 0xFFF8 * 0.005


def _create_radar_can_parser(CP):
  if CP.flags & ToyotaFlags.TSS3:
    messages = [(msg, 20) for msg in TSS3_GEOMETRY_MSGS + TSS3_MOTION_MSGS]
    return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)

  if CP.flags & ToyotaFlags.TSS2:
    RADAR_A_MSGS = list(range(0x180, 0x190))
    RADAR_B_MSGS = list(range(0x190, 0x1a0))
  else:
    RADAR_A_MSGS = list(range(0x210, 0x220))
    RADAR_B_MSGS = list(range(0x220, 0x230))

  msg_a_n = len(RADAR_A_MSGS)
  msg_b_n = len(RADAR_B_MSGS)
  messages = list(zip(RADAR_A_MSGS + RADAR_B_MSGS, [20] * (msg_a_n + msg_b_n), strict=True))

  messages.append(('STATUS_MSG', 10))

  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    if CP.flags & ToyotaFlags.TSS3:
      self.trigger_msg = TSS3_MOTION_MSGS[-1]
    else:
      if CP.flags & ToyotaFlags.TSS2:
        self.RADAR_A_MSGS = list(range(0x180, 0x190))
        self.RADAR_B_MSGS = list(range(0x190, 0x1a0))
      else:
        self.RADAR_A_MSGS = list(range(0x210, 0x220))
        self.RADAR_B_MSGS = list(range(0x220, 0x230))

      self.valid_cnt = {key: 0 for key in self.RADAR_A_MSGS}
      self.trigger_msg = self.RADAR_B_MSGS[-1]

    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP)
    self.updated_messages = set()

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.CP.flags & ToyotaFlags.TSS3:
      self._update_tss3_points(ret.errors.canError)
    else:
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

  def _update_tss3_points(self, can_error):
    if can_error:
      self.pts.clear()
      return

    for bank, (geometry_addr, motion_addr) in enumerate(zip(TSS3_GEOMETRY_MSGS, TSS3_MOTION_MSGS, strict=True)):
      geometry, motion = self.rcp.vl[geometry_addr], self.rcp.vl[motion_addr]
      for slot in range(TSS3_SLOTS):
        key = bank * TSS3_SLOTS + slot
        new, ended = motion[f'NEW_TRACK_{slot}'], motion[f'TRACK_ENDED_{slot}']
        dist = geometry[f'DIST_{slot}']
        valid = motion[f'TRACK_STATE_{slot}'] != 0 and 0 < dist < TSS3_DIST_INVALID and (new or not ended)

        if new or ended or not valid:
          self.pts.pop(key, None)
        if not valid:
          continue

        if key not in self.pts:
          self.pts[key] = RadarData.RadarPoint()
          self.pts[key].trackId = self.track_id
          self.track_id += 1
        self.pts[key].dRel = dist
        self.pts[key].yRel = geometry[f'LAT_{slot}']
        self.pts[key].vRel = motion[f'VREL_{slot}']
