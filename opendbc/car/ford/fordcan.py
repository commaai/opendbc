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

  This command can apply "Lane Keeping Aid" maneuvers, which are subject to the PSCM lockout.

  Frequency is 33Hz.
  """

  return packer.make_can_msg("Lane_Assist_Data1", CAN.main, {})


def create_lat_ctl_msg(packer, CAN: CanBus, lat_active: bool, path_offset: float, path_angle: float, curvature: float,
                       curvature_rate: float):
  """
  Creates a CAN message for the Ford TJA/LCA Command.

  This command can apply "Lane Centering" maneuvers: continuous lane centering for traffic jam assist and highway
  driving. It is not subject to the PSCM lockout.

  Ford lane centering command uses a third order polynomial to describe the road centerline. The polynomial is defined
  by the following coefficients:
    c0: lateral offset between the vehicle and the centerline (positive is right)
    c1: heading angle between the vehicle and the centerline (positive is right)
    c2: curvature of the centerline (positive is left)
    c3: rate of change of curvature of the centerline
  As the PSCM combines this information with other sensor data, such as the vehicle's yaw rate and speed, the steering
  angle cannot be easily controlled.

  The PSCM should be configured to accept TJA/LCA commands before these commands will be processed. This can be done
  using tools such as Forscan.

  Frequency is 20Hz.
  """

  values = {
    "LatCtlRng_L_Max": 0,                       # Unknown [0|126] meter
    "HandsOffCnfm_B_Rq": 0,                     # Unknown: 0=Inactive, 1=Active [0|1]
    "LatCtl_D_Rq": 1 if lat_active else 0,      # Mode: 0=None, 1=ContinuousPathFollowing, 2=InterventionLeft,
                                                #       3=InterventionRight, 4-7=NotUsed [0|7]
    "LatCtlRampType_D_Rq": 0,                   # Ramp speed: 0=Slow, 1=Medium, 2=Fast, 3=Immediate [0|3]
                                                #             Makes no difference with curvature control
    "LatCtlPrecision_D_Rq": 1,                  # Precision: 0=Comfortable, 1=Precise, 2/3=NotUsed [0|3]
                                                #            The stock system always uses comfortable
    "LatCtlPathOffst_L_Actl": path_offset,      # Path offset [-5.12|5.11] meter
    "LatCtlPath_An_Actl": path_angle,           # Path angle [-0.5|0.5235] radians
    "LatCtlCurv_NoRate_Actl": curvature_rate,   # Curvature rate [-0.001024|0.00102375] 1/meter^2
    "LatCtlCurv_No_Actl": curvature,            # Curvature [-0.02|0.02094] 1/meter
  }
  return packer.make_can_msg("LateralMotionControl", CAN.main, values)


def create_lat_ctl2_msg(packer, CAN: CanBus, mode: int, path_offset: float, path_angle: float, curvature: float,
                        curvature_rate: float, counter: int):
  """
  Create a CAN message for the new Ford Lane Centering command.

  This message is used on the CAN FD platform and replaces the old LateralMotionControl message. It is similar but has
  additional signals for a counter and checksum.

  Frequency is 20Hz.
  """

  values = {
    "LatCtl_D2_Rq": mode,                       # Mode: 0=None, 1=PathFollowingLimitedMode, 2=PathFollowingFullMode
    "LatCtlPathOffst_L2_Actl": path_offset,     # Path offset [-5.12|5.11] meter
    "LatCtlPath_An2_Actl": path_angle,          # Path angle [-0.5|0.5235] radians
    "LatCtlCurv_No2_Actl": curvature,           # Curvature [-0.02|0.02094] 1/meter
    "LatCtlCurv_NoRate2_Actl": curvature_rate,  # Curvature rate [-0.001024|0.00102375] 1/meter^2
    "LatCtlMsgCount2_Cnt": counter,             # Counter [0|15]
    "LatCtlChecksum2_No": 0,                    # Checksum placeholder
  }

  dat = packer.make_can_msg("LateralMotionControl2", CAN.main, values)[2]
  values["LatCtlChecksum2_No"] = calculate_lat_ctl2_checksum(mode, counter, dat)

  return packer.make_can_msg("LateralMotionControl2", CAN.main, values)


def create_tron_lat_ctl_msg(packer, CAN: CanBus, lat_active: bool, path_offset: float, path_angle: float, curvature: float,
                           curvature_rate: float, counter: int):
  """
  Create a CAN message for TRON platform lateral control.
  Combines features from both legacy and new Ford systems.
  """
  values = {
    "LatCtl_D_Rq": 1 if lat_active else 0,
    "LatCtlPathOffst_L_Actl": path_offset,
    "LatCtlPath_An_Actl": path_angle,
    "LatCtlCurv_No_Actl": curvature,
    "LatCtlCurv_NoRate_Actl": curvature_rate,
    "LatCtlMsgCount_Cnt": counter,
    "LatCtlChecksum_No": 0,
  }

  dat = packer.make_can_msg("TRON_LateralControl", CAN.main, values)[2]
  values["LatCtlChecksum_No"] = calculate_lat_ctl2_checksum(1 if lat_active else 0, counter, dat)

  return packer.make_can_msg("TRON_LateralControl", CAN.main, values)