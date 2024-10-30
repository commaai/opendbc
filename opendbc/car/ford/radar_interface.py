try:
  import matplotlib
  matplotlib.use('Qt5Agg')  # Use the Qt5Agg backend
  matplotlib.rcParams['figure.raise_window'] = False
  import matplotlib.pyplot as plt
  from sklearn.cluster import DBSCAN
except:
  plt = None

from collections import defaultdict
import math
import copy
import numpy as np
from math import cos, sin
from dataclasses import dataclass
from opendbc.can.parser import CANParser
from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import DBC, RADAR
from opendbc.car.interfaces import RadarInterfaceBase
# from scipy.cluster.hierarchy import dendrogram, linkage
# from scipy.optimize import linear_sum_assignment

try:
  profile
except:
  profile = lambda x: x

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


@dataclass
class Cluster2:
  dRel: float = 0.0
  dRelClosest: float = 0.0
  yRel: float = 0.0
  vRel: float = 0.0
  trackId: int = 0


class Cluster:
  def __init__(self, pts: list[RadarPoint], cluster_id: int):
    self.n_pts = len(pts)
    self.cluster_id = cluster_id

    self._dRel = sum([p.dRel for p in pts]) / self.n_pts
    self._closestDRel = min([p.dRel for p in pts])
    self._yRel = sum([p.yRel for p in pts]) / self.n_pts
    self._vRel = sum([p.vRel for p in pts]) / self.n_pts

  @property
  def dRel(self):
    return self._dRel

  @property
  def closestDRel(self):
    return self._closestDRel

  @property
  def yRel(self):
    return self._yRel

  @property
  def vRel(self):
    return self._vRel


@profile
def cluster_points(pts: list[list[float]], pts2: list[list[float]], max_dist: float) -> list[int]:
  """
  Clusters a collection of points based on another collection of points. This is useful for correlating clusters of points through time.
  Points in pts2 not close enough to any point in pts are assigned -1.
  Args:
    pts: List of points to base the new clusters on
    pts2: List of points to cluster using pts
    max_dist: Max distance from cluster center to candidate point

  Returns:
    List of cluster indices for pts2 that correspond to pts
  """

  if not len(pts2):
    return []

  if not len(pts):
    return [-1] * len(pts2)

  max_dist_sq = max_dist ** 2
  pts = np.array(pts)
  pts2 = np.array(pts2)

  # Compute squared norms
  pts_norm_sq = np.sum(pts ** 2, axis=1)
  pts2_norm_sq = np.sum(pts2 ** 2, axis=1)

  # Compute squared Euclidean distances using the identity
  # dist_sq[i, j] = ||pts2[i]||^2 + ||pts[j]||^2 - 2 * pts2[i] . pts[j]
  dist_sq = pts2_norm_sq[:, np.newaxis] + pts_norm_sq[np.newaxis, :] - 2 * np.dot(pts2, pts.T)

  # Ensure no negative distances due to floating-point errors
  dist_sq = np.maximum(dist_sq, 0.0)

  # Find the closest cluster for each point and assign its index
  closest_clusters = np.argmin(dist_sq, axis=1)
  closest_dist_sq = dist_sq[np.arange(len(pts2)), closest_clusters]
  cluster_idxs = np.where(closest_dist_sq < max_dist_sq, closest_clusters, -1)

  return cluster_idxs.tolist()


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

    self.clusters: list[Cluster] = []
    self.cluster_id = 0

    self.frame = 0
    self.clusters2 = []

    # TODO: 2.5 good enough?
    # TODO: write simple cluster function
    # self.dbscan = DBSCAN(eps=5, min_samples=1)

    if PLOT:
      self.fig, self.ax = plt.subplots()
      self.cmap = plt.cm.get_cmap('tab20', 20)  # 'tab20' colormap with 20 colors

    self.temp_pts = {}

    self.updated_messages = set()
    self.track_id = 0
    self.radar = DBC[CP.carFingerprint]['radar']
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

  @profile
  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None
    self.updated_messages.clear()

    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")

    if self.radar == RADAR.DELPHI_ESR:
      self._update_delphi_esr()
    elif self.radar == RADAR.DELPHI_MRR:
      _update, _errors = self._update_delphi_mrr()
      errors.extend(_errors)
      if not _update:
        return None

    ret = structs.RadarData()
    # ret.points = list(self.pts.values())
    ret.points = [structs.RadarData.RadarPoint(dRel=pt.dRelClosest, yRel=pt.yRel, vRel=pt.vRel, trackId=pt.trackId,
                                               measured=True, aRel=float('nan'), yvRel=float('nan'))
                  for pt in self.clusters2]
    ret.errors = errors
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

  @profile
  def _update_delphi_mrr(self):
    headerScanIndex = int(self.rcp.vl["MRR_Header_InformationDetections"]['CAN_SCAN_INDEX']) & 0b11

    # Use points with Doppler coverage of +-60 m/s, reduces similar points
    if headerScanIndex in (0, 1):
      return False, []

    errors = []
    if DELPHI_MRR_RADAR_RANGE_COVERAGE[headerScanIndex] != int(self.rcp.vl["MRR_Header_SensorCoverage"]["CAN_RANGE_COVERAGE"]):
      errors.append("wrongConfig")

    # points = []
    for ii in range(1, DELPHI_MRR_RADAR_MSG_COUNT + 1):
      msg = self.rcp.vl[f"MRR_Detection_{ii:03d}"]

      # SCAN_INDEX rotates through 0..3 on each message for different measurement modes
      # Indexes 0 and 2 have a max range of ~40m, 1 and 3 are ~170m (MRR_Header_SensorCoverage->CAN_RANGE_COVERAGE)
      # Indexes 0 and 1 have a Doppler coverage of +-71 m/s, 2 and 3 have +-60 m/s
      scanIndex = msg[f"CAN_SCAN_INDEX_2LSB_{ii:02d}"]
      i = (ii - 1) * 2 + (scanIndex - 2)
      # if scanIndex not in (2, 3):
      #   continue

      # Throw out old measurements. Very unlikely to happen, but is proper behavior
      if scanIndex != headerScanIndex:
        continue

      valid = bool(msg[f"CAN_DET_VALID_LEVEL_{ii:02d}"])

      # Long range measurement mode is more sensitive and can detect the road surface
      dist = msg[f"CAN_DET_RANGE_{ii:02d}"]  # m [0|255.984]
      if scanIndex in (1, 3) and dist < MIN_LONG_RANGE_DIST:
        valid = False

      # valid = valid and dist < 50

      if valid:
        azimuth = msg[f"CAN_DET_AZIMUTH_{ii:02d}"]              # rad [-3.1416|3.13964]
        distRate = msg[f"CAN_DET_RANGE_RATE_{ii:02d}"]          # m/s [-128|127.984]
        yRel = -sin(azimuth) * dist                             # in car frame's y axis, left is positive
        dRel = cos(azimuth) * dist                              # m from front of car

        # TODO: multiply yRel by 2
        # points.append([dRel, yRel * 2])

        # if i not in self.temp_pts:
        #   self.temp_pts[i] = RadarPoint()
        #   self.temp_pts[i].trackId = self.track_id
        #   self.temp_pts[i].aRel = float('nan')
        #   self.temp_pts[i].yvRel = float('nan')
        #   self.track_id += 1
        #
        # elif abs(self.temp_pts[i].vRel - distRate) > 2 or abs(self.temp_pts[i].dRel - dRel) > 5:
        #   # delphi doesn't notify of track switches, so do it manually
        #   # TODO: refactor this to radard if more radars behave this way
        #   self.temp_pts[i].trackId = self.track_id
        #   self.track_id += 1

        # self.temp_pts[i] = RadarPoint(dRel=dRel, yRel=yRel, vRel=distRate)
        self.temp_pts[i] = [dRel, yRel * 2, distRate * 2]

        # self.temp_pts[i].dRel = dRel
        # self.temp_pts[i].yRel = yRel
        # self.temp_pts[i].vRel = distRate

        # self.temp_pts[i].measured = True
      # else:
      #   if i in self.temp_pts:
      #     del self.temp_pts[i]

    # Update once we've cycled through all 4 scan modes
    if headerScanIndex != 3:
      return False, []

    temp_points_list = list(self.temp_pts.values())
    points_list = list(self.clusters2)
    keys = temp_points_list
    prev_keys = [[p.dRel, p.yRel * 2, p.vRel * 2] for p in points_list]
    # labels = self.dbscan.fit_predict(keys)
    labels = cluster_points(prev_keys, keys, 5)
    # TODO: can be empty
    # print(prev_keys, keys, labels)
    # clusters = [[] for _ in range(max(labels) + 1)]
    clusters_by_track_id = defaultdict(list)

    for i, label in enumerate(labels):
      if label != -1:
        clusters_by_track_id[points_list[label].trackId].append(temp_points_list[i])
        # raise Exception("DBSCAN should not return -1")
      else:
        clusters_by_track_id[self.track_id].append(temp_points_list[i])
        self.track_id += 1

      # clusters[label].append(temp_points_list[i])

    # find closest previous clusters (2.5 max diff)

    # print(clusters_by_track_id)
    # self.pts.clear()
    self.clusters2 = []
    for track_id, pts in clusters_by_track_id.items():
      if len(pts) == 0:
        # assert False
        continue

      dRel = [p[0] for p in pts]
      min_dRel = min(dRel)
      dRel = sum(dRel) / len(dRel)

      yRel = [p[1] for p in pts]
      yRel = sum(yRel) / len(yRel) / 2

      vRel = [p[2] for p in pts]
      vRel = sum(vRel) / len(vRel) / 2

      # self.pts[track_id] = RadarPoint(dRel=min_dRel, yRel=yRel, vRel=vRel, trackId=track_id)
      self.clusters2.append(Cluster2(dRel=dRel, dRelClosest=min_dRel, yRel=yRel, vRel=vRel, trackId=track_id))

    if PLOT:
      self.ax.clear()

      colors = [self.cmap(c.trackId % 20) for c in self.clusters2]
      # colors_pts = [self.cmap(c.trackId % 20) for c in self.temp_pts.values()]

      self.ax.set_title(f'clusters: {len(self.clusters2)}')
      self.ax.scatter([c.dRelClosest for c in self.clusters2], [c.yRel for c in self.clusters2], s=80, label='clusters', c=colors)
      self.ax.scatter([p[0] for p in self.temp_pts.values()], [p[1] / 2 for p in self.temp_pts.values()], s=10, label='points', color='red')  # c=colors_pts)
      # text above each point with its dRel and vRel:
      # for p in self.temp_pts.values():
      #   self.ax.text(p.dRel, p.yRel, f'{p.dRel:.1f}, {p.vRel:.1f}', fontsize=8)
      for c in self.clusters2:
        self.ax.text(c.dRelClosest, c.yRel, f'{c.dRel:.1f}, {c.yRel:.1f}, {c.vRel:.1f}, {c.trackId}', fontsize=8)
      self.ax.legend()
      self.ax.set_xlim(0, 180)
      self.ax.set_ylim(-30, 30)
      plt.pause(1/15)

    self.temp_pts.clear()

    return True, errors
