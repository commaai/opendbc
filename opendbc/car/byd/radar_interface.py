#!/usr/bin/env python3
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.byd.values import DBC, CanBus

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)

    if CP.radarUnavailable:
      self.rcp = None
    else:
      messages = [('RADAR_MRR', 60)]
      self.rcp = CANParser(DBC[CP.carFingerprint][Bus.pt], messages, CanBus.MPC)
      self.trigger_msg = 0x374

    self.updated_messages = set()

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    values = self.rcp.update_strings(can_strings)
    self.updated_messages.update(values)

    if self.trigger_msg not in self.updated_messages:
      return None

    ret = structs.RadarData()

    #if not self.rcp.can_valid:
    #  ret.errors.canError = True

    msg_mrr = self.rcp.vl['RADAR_MRR']
    msg_id = msg_mrr['TargetID'] #1:left, 2:front, 3:right
    if msg_id == 2:
      longdist = msg_mrr['LongDist']
      isvalid = longdist > 0 or bool(msg_mrr['IsValid'])

      if(msg_id not in self.pts):
        self.pts[msg_id] = structs.RadarData.RadarPoint()
        self.pts[msg_id].trackId = msg_id

      self.pts[msg_id].dRel = longdist if isvalid else 255
      self.pts[msg_id].yRel =  msg_mrr['LatDist'] if isvalid else 0
      self.pts[msg_id].vRel = float('nan')
      self.pts[msg_id].aRel = float('nan')
      self.pts[msg_id].yvRel = float('nan')
      self.pts[msg_id].measured = True

    ret.points = list(self.pts.values())
    self.updated_messages.clear()
    return ret
