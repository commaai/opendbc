def create_lat_ctl2_msg(packer, CAN: CanBus, mode: int, path_offset: float, path_angle: float, curvature: float,
                        curvature_rate: float, counter: int):
  """
  Create a CAN message for the new Ford Lane Centering command.

  This message is used on the CAN FD platform and replaces the old LateralMotionControl message. It is similar but has
  additional signals for a counter and checksum.

  Frequency is 20Hz.
  """

  values = {
    "LatCtl_D2_Rq": mode,                       # Mode: 0=None, 1=PathFollowingLimitedMode, 2=PathFollowEnhancedMode [0|3]
    "LatCtlPath_An_Actl": path_angle,           # Path angle [-0.5|0.5235] radians
    "LatCtlPathOffst_L_Actl": path_offset,      # Path offset [-5.12|5.11] meter
    "LatCtlCurv_No_Actl": curvature,            # Curvature [-0.02|0.02094] 1/meter
    "LatCtlCurv_NoRate_Actl": curvature_rate,   # Curvature rate [-0.001024|0.00102375] 1/meter^2
    "LatCtlMsgCount": counter,                  # Message counter [0|15]
  }
  
  # Calculate checksum for TRON platform
  dat = packer.make_can_msg("LateralMotionControl2", CAN.main, values)[2]
  values["LatCtlChecksum"] = calculate_lat_ctl2_checksum(mode, counter, dat)
  
  return packer.make_can_msg("LateralMotionControl2", CAN.main, values)