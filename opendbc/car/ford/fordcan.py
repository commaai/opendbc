from opendbc.car import CanBusBase, structs

HUDControl = structs.CarControl.HUDControl


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)

  @property
  def main(self) -> int:
    return self.offset

  @property
  def radar(self) -> int:
    return self.offset + 1

  @property
  def camera(self) -> int:
    return self.offset + 2


def calculate_lat_ctl2_checksum(mode: int, counter: int, dat: bytearray) -> int:
  curvature = (dat[2] << 3) | ((dat[3]) >> 5)
  curvature_rate = (dat[6] << 3) | ((dat[7]) >> 5)
  path_angle = ((dat[3] & 0x1F) << 6) | ((dat[4]) >> 2)
  path_offset = ((dat[4] & 0x3) << 8) | dat[5]

  checksum = mode + counter
  for sig_val in (curvature, curvature_rate, path_angle, path_offset):
    checksum += sig_val + (sig_val >> 8)

  return 0xFF - (checksum & 0xFF)


def create_lka_msg(packer, CAN: CanBus):
  """
  Creates an empty CAN message for the Ford LKA Command.
  """
  return packer.make_can_msg("Lane_Assist_Data1", CAN.main, {})


def create_lat_ctl_msg(packer, CAN: CanBus, lat_active: bool, path_offset: float, path_angle: float, curvature: float,
                       curvature_rate: float):
  """
  Creates a CAN message for the Ford TJA/LCA Command.
  """
  values = {
    "LatCtlRng_L_Max": 0,
    "HandsOffCnfm_B_Rq": 0,
    "LatCtl_D_Rq": 1 if lat_active else 0,
    "LatCtlRampType_D_Rq": 0,
    "LatCtlPrecision_D_Rq": 1,
    "LatCtlPathOffst_L_Actl": path_offset,
    "LatCtlPath_An_Actl": path_angle,
    "LatCtlCurv_NoRate_Actl": curvature_rate,
    "LatCtlCurv_No_Actl": curvature,
  }
  return packer.make_can_msg("LateralMotionControl", CAN.main, values)


def create_lat_ctl2_msg(packer, CAN: CanBus, mode: int, path_offset: float, path_angle: float, curvature: float,
                        curvature_rate: float, counter: int):
  """
  Create a CAN message for the new Ford Lane Centering command.
  """
  values = {
    "LatCtl_D2_Rq": mode,
    "LatCtlPathOffst_L2_Actl": path_offset,
    "LatCtlPath_An2_Actl": path_angle,
    "LatCtlCurv_No2_Actl": curvature,
    "LatCtlCurv_NoRate2_Actl": curvature_rate,
    "LatCtlMsgCount2": counter,
  }
  
  # TRON platform specific checksum calculation
  dat = bytearray(8)
  dat[0] = mode
  dat[1] = counter
  dat[2] = int(curvature * 100) >> 8
  dat[3] = int(curvature * 100) & 0xFF
  dat[4] = int(path_angle * 100) >> 8
  dat[5] = int(path_angle * 100) & 0xFF
  dat[6] = int(path_offset * 100) >> 8
  dat[7] = int(path_offset * 100) & 0xFF
  
  values["LatCtlChecksum2"] = calculate_lat_ctl2_checksum(mode, counter, dat)
  
  return packer.make_can_msg("LateralMotionControl2", CAN.main, values)


def create_accel_command(packer, CAN: CanBus, accel: float, pcm_cancel: bool, standstill_req: bool, lead: bool):
  """
  Create a CAN message for Ford longitudinal acceleration control (TRON platform).
  """
  values = {
    "Accel_Req": accel,
    "Cancel_Req": 1 if pcm_cancel else 0,
    "Standstill_Req": 1 if standstill_req else 0,
    "Lead_Veh": 1 if lead else 0,
  }
  return packer.make_can_msg("LongitudinalControl", CAN.main, values)