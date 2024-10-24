from math import cos, sin
from opendbc.can.parser import CANParser
from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import DBC, RADAR
from opendbc.car.interfaces import RadarInterfaceBase

DELPHI_ESR_RADAR_MSGS = list(range(0x500, 0x540))

DELPHI_MRR_RADAR_START_ADDR = 0x120
DELPHI_MRR_RADAR_HEADER_ADDR = 0x170  # MRR_Header_InformationDetections
DELPHI_MRR_RADAR_MSG_COUNT = 64


def _create_delphi_esr_radar_can_parser(CP) -> CANParser:
  msg_n = len(DELPHI_ESR_RADAR_MSGS)
  messages = list(zip(DELPHI_ESR_RADAR_MSGS, [20] * msg_n, strict=True))

  return CANParser(RADAR.DELPHI_ESR, messages, CanBus(CP).radar)


def _create_delphi_mrr_radar_can_parser(CP) -> CANParser:
  messages = [("MRR_Header_InformationDetections", 33)]

  for i in range(1, DELPHI_MRR_RADAR_MSG_COUNT + 1):
    msg = f"MRR_Detection_{i:03d}"
    messages += [(msg, 33)]

  return CANParser(RADAR.DELPHI_MRR, messages, CanBus(CP).radar)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)

    self.frame = 0

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
      print('trigger_msg:', self.trigger_msg)
    else:
      raise ValueError(f"Unsupported radar: {self.radar}")

  def update(self, can_strings):
    self.frame += 1
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    # print('updated_messages:', self.updated_messages)
    if self.trigger_msg not in self.updated_messages:
      return None

    ret = structs.RadarData()
    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    if self.radar == RADAR.DELPHI_ESR:
      self._update_delphi_esr()
    elif self.radar == RADAR.DELPHI_MRR:
      self._update_delphi_mrr()
      # print('pts', len(self.pts), self.rcp.vl['MRR_Header_InformationDetections']['CAN_NUMBER_OF_DET'])
      # if len(self.pts) != self.rcp.vl['MRR_Header_InformationDetections']['CAN_NUMBER_OF_DET'] and self.frame > 10:
      #   print('mismatch!')
      #   raise Exception

    ret.points = list(self.pts.values())
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

    print()
    header = self.rcp.vl['MRR_Header_InformationDetections']
    scan_index = int(header['CAN_SCAN_INDEX'])
    look_index = int(header['CAN_LOOK_INDEX']) & 0b11  # or CAN_LOOK_ID?
    look_id = int(header['CAN_LOOK_ID'])
    # print('scan_index', int(scan_index) & 0b11, scan_index)

    if self.frame > 10:
      assert self.rcp.vl['MRR_Header_InformationDetections']['CAN_SCAN_INDEX'] == self.rcp.vl['MRR_Header_InformationDetections']['CAN_LOOK_INDEX']
      if look_id == 0:
        assert look_index == 1
      elif look_id == 1:
        assert look_index == 3
      elif look_id == 2:
        assert look_index == 0
      elif look_id == 3:
        assert look_index == 2

    print('updated msgs', len(self.updated_messages))
    if self.frame > 10:
      assert len(self.updated_messages) == DELPHI_MRR_RADAR_MSG_COUNT + 1, len(self.updated_messages)

    for ii in range(1, DELPHI_MRR_RADAR_MSG_COUNT + 1):
      # if look_index != 0:
      #   continue
      msg = self.rcp.vl[f"MRR_Detection_{ii:03d}"]

      # SCAN_INDEX rotates through 0..3 on each message
      # treat these as separate points
      # Indexes 0 and 2 have a max range of ~40m, 1 and 3 are ~170m (MRR_Header_SensorCoverage->CAN_RANGE_COVERAGE)
      # TODO: filter out close range index 1 and 3 points, contain false positives
      # TODO: can we group into 2 groups?
      scanIndex = msg[f"CAN_SCAN_INDEX_2LSB_{ii:02d}"]
      # i = (ii - 1) * 4 + scanIndex
      i = (ii - 1) * 2 + (1 if scanIndex in (0, 2) else 0)
      # print('pt scanIndex', scanIndex)
      if scanIndex != int(scan_index) & 0b11 and self.frame > 10:
        print('doesn\'t match!', scanIndex, int(scan_index) & 0b11, scan_index)
        raise Exception

      # Throw out old measurements. Very unlikely to happen, but is proper behavior
      if scanIndex != headerScanIndex:
        continue

      if i not in self.pts:
        self.pts[i] = structs.RadarData.RadarPoint()
        self.pts[i].trackId = self.track_id
        self.pts[i].aRel = float('nan')
        self.pts[i].yvRel = float('nan')
        self.track_id += 1

      valid = bool(msg[f"CAN_DET_VALID_LEVEL_{ii:02d}"])

      if valid:
        azimuth = msg[f"CAN_DET_AZIMUTH_{ii:02d}"]              # rad [-3.1416|3.13964]
        dist = msg[f"CAN_DET_RANGE_{ii:02d}"]                   # m [0|255.984]
        distRate = msg[f"CAN_DET_RANGE_RATE_{ii:02d}"]          # m/s [-128|127.984]
        dRel = cos(azimuth) * dist                              # m from front of car
        yRel = -sin(azimuth) * dist                             # in car frame's y axis, left is positive
        super_res = msg[f"CAN_DET_SUPER_RES_TARGET_{ii:02d}"]   # bool

        # delphi doesn't notify of track switches, so do it manually
        # TODO: refactor this to radard if more radars behave this way
        if abs(self.pts[i].vRel - distRate) > 2 or abs(self.pts[i].dRel - dRel) > 5:
          self.track_id += 1
          self.pts[i].trackId = self.track_id

        self.pts[i].dRel = dRel
        self.pts[i].yRel = yRel
        self.pts[i].vRel = distRate
        self.pts[i].flags = look_index
        self.pts[i].flags2 = scanIndex
        self.pts[i].flags3 = super_res

        self.pts[i].measured = True

      else:
        del self.pts[i]
