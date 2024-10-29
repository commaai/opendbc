try:
  import matplotlib

  matplotlib.use('Qt5Agg')  # Use the Qt5Agg backend
  matplotlib.rcParams['figure.raise_window'] = False
  import matplotlib.pyplot as plt
except Exception:
  plt = None

import math
import copy
import numpy as np

from dataclasses import dataclass
from math import cos, sin
from opendbc.can.parser import CANParser
from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import DBC, RADAR
from opendbc.car.interfaces import RadarInterfaceBase

DELPHI_ESR_RADAR_MSGS = list(range(0x500, 0x540))

DELPHI_MRR_RADAR_START_ADDR = 0x120
DELPHI_MRR_RADAR_HEADER_ADDR = 0x174  # MRR_Header_SensorCoverage
DELPHI_MRR_RADAR_MSG_COUNT = 64

DELPHI_MRR_RADAR_RANGE_COVERAGE = {0: 42, 1: 164, 2: 45, 3: 175}  # scan index to detection range (m)
MIN_LONG_RANGE_DIST = 30  # meters

PLOT = False


@dataclass
class RadarPoint:
  dRel: float = 0.0
  yRel: float = 0.0
  vRel: float = 0.0
  trackId: int = 0


def _create_delphi_esr_radar_can_parser(CP) -> CANParser:
  msg_n = len(DELPHI_ESR_RADAR_MSGS)
  messages = list(zip(DELPHI_ESR_RADAR_MSGS, [20] * msg_n, strict=True))

  return CANParser(RADAR.DELPHI_ESR, messages, CanBus(CP).radar)


def _create_delphi_mrr_radar_can_parser(CP) -> CANParser:
  messages = [
    ("MRR_Header_InformationDetections", 33),
    ("MRR_Header_SensorCoverage", 33),
  ]

  for i in range(1, DELPHI_MRR_RADAR_MSG_COUNT + 1):
    msg = f"MRR_Detection_{i:03d}"
    messages += [(msg, 33)]

  return CANParser(RADAR.DELPHI_MRR, messages, CanBus(CP).radar)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)

    if PLOT:
      self.fig, self.ax = plt.subplots()
      self.cmap = plt.cm.get_cmap('tab20', 20)  # 'tab20' colormap with 20 colors

    self.updated_messages = set()
    self.track_id = 0
    self.radar = DBC[CP.carFingerprint]['radar']
    self.temp_pts = {}
    self.prev_pts = {}
    if CP.radarUnavailable:
      self.rcp = None
    elif self.radar == RADAR.DELPHI_ESR:
      self.rcp = _create_delphi_esr_radar_can_parser(CP)
      self.trigger_msg = DELPHI_ESR_RADAR_MSGS[-1]
      self.valid_cnt = {key: 0 for key in DELPHI_ESR_RADAR_MSGS}
    elif self.radar == RADAR.DELPHI_MRR:
      self.rcp = _create_delphi_mrr_radar_can_parser(CP)
      self.trigger_msg = DELPHI_MRR_RADAR_HEADER_ADDR
    else:
      raise ValueError(f"Unsupported radar: {self.radar}")

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    ret = structs.RadarData()
    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")

    if self.radar == RADAR.DELPHI_ESR:
      self._update_delphi_esr()
    elif self.radar == RADAR.DELPHI_MRR:
      errors.extend(self._update_delphi_mrr())

    ret.points = list(self.pts.values())
    ret.errors = errors
    self.updated_messages.clear()
    return ret

  def _update_delphi_esr(self):
    for ii in sorted(self.updated_messages):
      cpt = self.rcp.vl[ii]

      if cpt['X_Rel'] > 0.00001:
        self.valid_cnt[ii] = 0    # reset counter

      if cpt['X_Rel'] > 0.00001:
        self.valid_cnt[ii] += 1
      else:
        self.valid_cnt[ii] = max(self.valid_cnt[ii] - 1, 0)
      #print ii, self.valid_cnt[ii], cpt['VALID'], cpt['X_Rel'], cpt['Angle']

      # radar point only valid if there have been enough valid measurements
      if self.valid_cnt[ii] > 0:
        if ii not in self.pts:
          self.pts[ii] = structs.RadarData.RadarPoint()
          self.pts[ii].trackId = self.track_id
          self.track_id += 1
        self.pts[ii].dRel = cpt['X_Rel']  # from front of car
        self.pts[ii].yRel = cpt['X_Rel'] * cpt['Angle'] * CV.DEG_TO_RAD  # in car frame's y axis, left is positive
        self.pts[ii].vRel = cpt['V_Rel']
        self.pts[ii].aRel = float('nan')
        self.pts[ii].yvRel = float('nan')
        self.pts[ii].measured = True
      else:
        if ii in self.pts:
          del self.pts[ii]

  def _update_delphi_mrr(self):
    headerScanIndex = int(self.rcp.vl["MRR_Header_InformationDetections"]['CAN_SCAN_INDEX']) & 0b11

    errors = []
    if DELPHI_MRR_RADAR_RANGE_COVERAGE[headerScanIndex] != int(self.rcp.vl["MRR_Header_SensorCoverage"]["CAN_RANGE_COVERAGE"]):
      errors.append("wrongConfig")

    for ii in range(1, DELPHI_MRR_RADAR_MSG_COUNT + 1):
      msg = self.rcp.vl[f"MRR_Detection_{ii:03d}"]

      # SCAN_INDEX rotates through 0..3 on each message for different measurement modes
      # Indexes 0 and 2 have a max range of ~40m, 1 and 3 are ~170m (MRR_Header_SensorCoverage->CAN_RANGE_COVERAGE)
      # Indexes 0 and 1 have a Doppler coverage of +-71 m/s, 2 and 3 have +-60 m/s
      # TODO: can we group into 2 groups?
      scanIndex = msg[f"CAN_SCAN_INDEX_2LSB_{ii:02d}"]
      i = (ii - 1) * 4 + scanIndex

      # Throw out old measurements. Very unlikely to happen, but is proper behavior
      if scanIndex != headerScanIndex:
        continue

      valid = bool(msg[f"CAN_DET_VALID_LEVEL_{ii:02d}"])

      # Long range measurement mode is more sensitive and can detect the road surface
      dist = msg[f"CAN_DET_RANGE_{ii:02d}"]  # m [0|255.984]
      if scanIndex in (1, 3) and dist < MIN_LONG_RANGE_DIST:
        valid = False

      if valid:
        azimuth = msg[f"CAN_DET_AZIMUTH_{ii:02d}"]              # rad [-3.1416|3.13964]
        distRate = msg[f"CAN_DET_RANGE_RATE_{ii:02d}"]          # m/s [-128|127.984]
        dRel = cos(azimuth) * dist                              # m from front of car
        yRel = -sin(azimuth) * dist                             # in car frame's y axis, left is positive

        # try to find similar previous track id from last full update cycle (self.pts)
        closest_dst = None
        closest_track_id = None
        # TODO: just use self.pts, and build capnp object at the end
        for pt in self.temp_pts.values():
          dst = (pt.dRel - dRel) ** 2 + (pt.yRel - yRel) ** 2 + (pt.vRel - distRate) ** 2
          if dst < (5 ** 2) and (closest_dst is None or dst < closest_dst):
            closest_track_id = pt.trackId
            closest_dst = dst

        track_id = closest_track_id if closest_track_id is not None else self.track_id

        if i not in self.temp_pts or True:
          # self.temp_pts[i] = structs.RadarData.RadarPoint()
          self.temp_pts[i] = RadarPoint()
          self.temp_pts[i].trackId = track_id
          # self.temp_pts[i].aRel = float('nan')
          # self.temp_pts[i].yvRel = float('nan')
          if closest_track_id is None:
            self.track_id += 1

        elif abs(self.temp_pts[i].vRel - distRate) > 2 or abs(self.temp_pts[i].dRel - dRel) > 5:
          # delphi doesn't notify of track switches, so do it manually
          # TODO: refactor this to radard if more radars behave this way
          assert False
          self.temp_pts[i].trackId = track_id
          self.track_id += 1
        else:
          assert False

        self.temp_pts[i].dRel = dRel
        self.temp_pts[i].yRel = yRel
        self.temp_pts[i].vRel = distRate

        self.temp_pts[i].measured = True

      else:
        if i in self.temp_pts:
          assert False
          del self.temp_pts[i]

    if headerScanIndex == 3:
      pts_by_track_id = {}
      for pt in self.temp_pts.values():
        # print(pt)
        pts_by_track_id.setdefault(pt.trackId, []).append(pt)

      new_pts = {}
      for idx, pts in enumerate(pts_by_track_id.values()):
        new_pts[idx] = min(pts, key=lambda pt: pt.dRel)
         # new_pts.append(min(pts, key=lambda pt: pt.dRel))

      # self.pts = copy.deepcopy(self.temp_pts)
      # self.pts = copy.deepcopy(new_pts)
      self.prev_pts = new_pts
      self.pts = {i: structs.RadarData.RadarPoint(dRel=pt.dRel, yRel=pt.yRel,
                                                  vRel=pt.vRel, trackId=pt.trackId, measured=True,
                                                  aRel=float('nan'), yvRel=float('nan')) for i, pt in new_pts.items()}

      if PLOT:
        self.ax.clear()

        colors = [self.cmap(pt.trackId % 20) for pt in self.pts.values()]
        colors_pts = [self.cmap(c.trackId % 20) for c in self.temp_pts.values()]

        # self.ax.set_title(f'clusters: {len(self.clusters)}')
        self.ax.scatter([pt.dRel for pt in self.pts.values()], [pt.yRel for pt in self.pts.values()], s=80, label='points', c=colors)
        # self.ax.scatter([c.closestDRel for c in self.clusters], [c.yRel for c in self.clusters], s=80, label='clusters', c=colors)
        self.ax.scatter([p.dRel for p in self.temp_pts.values()], [p.yRel for p in self.temp_pts.values()], s=10, label='points', color='red')  # c=colors_pts)
        # text above each point with its dRel and vRel:
        # for p in self.temp_pts.values():
        #   self.ax.text(p.dRel, p.yRel, f'{p.dRel:.1f}, {p.vRel:.1f}', fontsize=8)
        # for c in self.clusters:
        #   self.ax.text(c.closestDRel, c.yRel, f'{c.dRel:.1f}, {c.yRel:.1f}, {c.vRel:.1f}, {c.cluster_id}', fontsize=8)
        self.ax.legend()
        self.ax.set_xlim(0, 180)
        self.ax.set_ylim(-30, 30)
        plt.pause(1 / 15)

    if headerScanIndex ==  3:
      self.temp_pts.clear()


    print(self.track_id)

    return errors
