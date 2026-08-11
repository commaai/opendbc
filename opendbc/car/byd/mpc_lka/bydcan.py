from opendbc.car.byd.values import CANBUS


def mpc_lka_checksum(byte_key: int, dat: bytes | bytearray) -> int:
  first_bytes_sum = sum(byte >> 4 for byte in dat)
  second_bytes_sum = sum(byte & 0xF for byte in dat)
  remainder = second_bytes_sum >> 4
  second_bytes_sum += byte_key >> 4
  first_bytes_sum += byte_key & 0xF
  first_part = ((-first_bytes_sum + 0x9) & 0xF)
  second_part = ((-second_bytes_sum + 0x9) & 0xF)
  return (((first_part + (-remainder + 5)) << 4) + second_part) & 0xFF


def create_steering_control(packer, cam_msg: dict, req_torque: int, req_prepare: int, active: int, hud_control, counter: int):
  """MPC -> Panda -> EPS: inject ACC_MPC_STATE (790) on ESC bus with LKAS torque."""
  values = {s: cam_msg[s] for s in [
    "AutoFullBeamState",
    "LeftLaneState",
    "LKAS_Config",
    "SETME2_0x1",
    "MPC_State",
    "AutoFullBeam_OnOff",
    "LKAS_Output",
    "LKAS_Active",
    "SETME3_0x0",
    "TrafficSignRecognition_OnOff",
    "SETME4_0x0",
    "SETME5_0x1",
    "RightLaneState",
    "LKAS_State",
    "TrafficSignRecognition_Result",
    "LKAS_AlarmType",
    "SETME7_0x3",
  ]}

  values["ReqHandsOnSteeringWheel"] = 0
  values["LKAS_ReqPrepare"] = req_prepare
  values["Counter"] = counter

  if active:
    mpc_state = values["MPC_State"]
    values.update({
      "LKAS_Output": req_torque,
      "LKAS_Active": 1,
      "LKAS_State": 4 if mpc_state == 2 else 2,
      "LeftLaneState": 3 if hud_control.leftLaneDepart else int(hud_control.leftLaneVisible) + 1,
      "RightLaneState": 3 if hud_control.rightLaneDepart else int(hud_control.rightLaneVisible) + 1,
    })
  else:
    values.update({
      "LKAS_Output": 0,
      "LKAS_Active": 0,
    })

  data = packer.make_can_msg("ACC_MPC_STATE", CANBUS.main_bus, values)[1]
  values["CheckSum"] = mpc_lka_checksum(0xAF, data)
  return packer.make_can_msg("ACC_MPC_STATE", CANBUS.main_bus, values)


def create_fake_318(packer, esc_msg: dict, faketorque: float, lkas_reqprepare: bool, lkas_active: bool, enabled: bool, counter: int):
  """Inject ACC_EPS_STATE (792) on MPC bus so stock MPC stays satisfied."""
  values = {s: esc_msg[s] for s in [
    "LKAS_Prepared",
    "CruiseActivated",
    "TorqueFailed",
    "SETME1_0x1",
    "SteerWarning",
    "SteerErrorCode",
    "MainTorque",
    "SETME3_0x1",
    "SETME4_0x3",
    "SteerDriverTorque",
    "SETME5_0xFF",
    "SETME6_0xFFF",
  ]}

  values["ReportHandsNotOnSteeringWheel"] = 0
  values["Counter"] = counter

  if enabled:
    if lkas_active:
      values.update({
        "LKAS_Prepared": 0,
        "CruiseActivated": 1,
        "MainTorque": faketorque,
      })
    elif lkas_reqprepare:
      values.update({
        "LKAS_Prepared": 1,
        "CruiseActivated": 0,
        "MainTorque": 0,
      })
    else:
      values.update({
        "LKAS_Prepared": 0,
        "CruiseActivated": 0,
        "MainTorque": 0,
      })

  data = packer.make_can_msg("ACC_EPS_STATE", CANBUS.cam_bus, values)[1]
  values["CheckSum"] = mpc_lka_checksum(0xAF, data)
  return packer.make_can_msg("ACC_EPS_STATE", CANBUS.cam_bus, values)
