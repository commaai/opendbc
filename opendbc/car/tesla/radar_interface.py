#!/usr/bin/env python3
from cereal import car
from opendbc.can.parser import CANParser
from opendbc.car import Bus
from opendbc.car.tesla.values import DBC, CANBUS, PLATFORM_3Y
from opendbc.car.interfaces import RadarInterfaceBase


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP

    if self.CP.carFingerprint in PLATFORM_3Y:
      return

    messages = [('RadarStatus', 16)]
    self.num_points = 40

    for i in range(self.num_points):
      messages.extend([
        (f'RadarPoint{i}_A', 16),
        (f'RadarPoint{i}_B', 16),
      ])

    self.rcp = CANParser(DBC[CP.carFingerprint][Bus.radar], messages, CANBUS.radar)
    self.updated_messages = set()
    self.track_id = 0
    self.trigger_msg = 1119

  def update(self, can_strings):
    if self.CP.carFingerprint in PLATFORM_3Y:
      return None

    if self.rcp is None:
      return super().update(None)

    values = self.rcp.update_strings(can_strings)
    self.updated_messages.update(values)

    if self.trigger_msg not in self.updated_messages:
      return None

    ret = car.RadarData.new_message()

    # Errors
    errors = []
    if not self.rcp.can_valid:
      errors.append('canError')

    radar_status = self.rcp.vl['RadarStatus']
    if radar_status['sensorBlocked'] or radar_status['shortTermUnavailable'] or radar_status['vehDynamicsError']:
      errors.append('fault')

    ret.errors = errors

    # Radar tracks
    for i in range(self.num_points):
      msg_a = self.rcp.vl[f'RadarPoint{i}_A']
      msg_b = self.rcp.vl[f'RadarPoint{i}_B']

      # Make sure msg A and B are together
      if msg_a['Index'] != msg_b['Index2']:
        continue

      # Check if it's a valid track
      if not msg_a['Tracked']:
        if i in self.pts:
          del self.pts[i]
        continue

      # New track!
      if i not in self.pts:
        self.pts[i] = car.RadarData.RadarPoint.new_message()
        self.pts[i].trackId = self.track_id
        self.track_id += 1

      # Parse track data
      self.pts[i].dRel = msg_a['LongDist']
      self.pts[i].yRel = msg_a['LatDist']
      self.pts[i].vRel = msg_a['LongSpeed']
      self.pts[i].aRel = msg_a['LongAccel']
      self.pts[i].yvRel = msg_b['LatSpeed']
      self.pts[i].measured = bool(msg_a['Meas'])

    ret.points = list(self.pts.values())
    self.updated_messages.clear()
    return ret

