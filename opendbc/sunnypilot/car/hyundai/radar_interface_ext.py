from opendbc.can.parser import CANParser
from opendbc.car import structs, Bus
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import DBC, HyundaiFlags

from opendbc.sunnypilot.car.hyundai.escc import EsccRadarInterfaceBase


class RadarInterfaceExt(EsccRadarInterfaceBase):
  msg_src: str
  trigger_msg: int
  rcp: CANParser
  pts: dict[int, structs.RadarData.RadarPoint]

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    EsccRadarInterfaceBase.__init__(self, CP, CP_SP)
    self.CP = CP
    self.CP_SP = CP_SP

    self.track_id = 0

  @property
  def use_radar_interface_ext(self) -> bool:
    return self.use_escc or self.CP.flags & (HyundaiFlags.CAMERA_SCC | HyundaiFlags.CANFD_CAMERA_SCC)

  def get_msg_src(self) -> str | None:
    if self.use_escc:
      return "ESCC"
    if self.CP.flags & (HyundaiFlags.CAMERA_SCC | HyundaiFlags.CANFD_CAMERA_SCC):
      return "SCC_CONTROL" if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else "SCC11"

  def get_radar_ext_can_parser(self) -> CANParser:
    if self.ESCC.enabled:
      lead_src, bus = "ESCC", 0
    elif self.CP.flags & (HyundaiFlags.CAMERA_SCC | HyundaiFlags.CANFD_CAMERA_SCC):
      lead_src = "SCC_CONTROL" if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else "SCC11"
      bus = CanBus(self.CP).CAM if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else 2
    else:
      return None

    messages = [(lead_src, 50)]
    return CANParser(DBC[self.CP.carFingerprint][Bus.pt], messages, bus)

  def get_trigger_msg(self, default_trigger_msg) -> int:
    if self.ESCC.enabled:
      return self.ESCC.trigger_msg
    if self.CP.flags & (HyundaiFlags.CAMERA_SCC | HyundaiFlags.CANFD_CAMERA_SCC):
      return 0x1A0 if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else 0x420
    return default_trigger_msg

  def initialize_radar_ext(self, default_trigger_msg) -> None:
    if self.ESCC.enabled:
      self.use_escc = True

    self.rcp = self.get_radar_ext_can_parser()
    self.trigger_msg = self.get_trigger_msg(default_trigger_msg)

  def update_ext(self, ret: structs.RadarData) -> structs.RadarData:
    if not self.rcp.can_valid:
      ret.errors.canError = True
      return ret

    for ii in range(1):
      msg_src = self.get_msg_src()
      msg = self.rcp.vl[msg_src]

      if ii not in self.pts:
        self.pts[ii] = structs.RadarData.RadarPoint()
        self.pts[ii].trackId = self.track_id
        self.track_id += 1

      valid = msg['ACC_ObjDist'] < 204.6 if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else msg['ACC_ObjStatus']
      if valid:
        self.pts[ii].measured = True
        self.pts[ii].dRel = msg['ACC_ObjDist']
        self.pts[ii].yRel = float('nan')  # FIXME-SP: Only some cars have lateral position from SCC
        self.pts[ii].vRel = msg['ACC_ObjRelSpd']
        self.pts[ii].aRel = float('nan')  # TODO-SP: calculate from ACC_ObjRelSpd and with timestep 50Hz (needs to modify in interfaces.py)
        self.pts[ii].yvRel = float('nan')

      else:
        del self.pts[ii]

    ret.points = list(self.pts.values())
    return ret
