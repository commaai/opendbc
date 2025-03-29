import numpy as np
from opendbc.car import structs
from opendbc.car.byd.values import  CanBus, CarControllerParams

GearShifter = structs.CarState.GearShifter
VisualAlert = structs.CarControl.HUDControl.VisualAlert

def byd_checksum(byte_key, dat):
    first_bytes_sum = sum(byte >> 4 for byte in dat)
    second_bytes_sum = sum(byte & 0xF for byte in dat)
    remainder = second_bytes_sum >> 4
    second_bytes_sum += byte_key >> 4
    first_bytes_sum += byte_key & 0xF
    first_part = ((-first_bytes_sum + 0x9) & 0xF)
    second_part = ((-second_bytes_sum + 0x9) & 0xF)
    return (((first_part + (-remainder + 5)) << 4) + second_part) & 0xFF

# MPC -> Panda -> EPS
def create_steering_control(packer, CP, cam_msg: dict, req_torque, req_prepare, active, hud_control, counter):
    values = {}
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
        mpc_state = values["MPC_State"] #2: Cancelling lkas control
        values.update({
            "LKAS_Output" : req_torque,
            "LKAS_Active" : 1,
            "LKAS_State" : 4 if (mpc_state == 2) else 2,
            "LeftLaneState":  3 if hud_control.leftLaneDepart  else int(hud_control.leftLaneVisible) + 1,
            "RightLaneState": 3 if hud_control.rightLaneDepart else int(hud_control.rightLaneVisible) + 1,
        })
    else: # Note: This disables the stock AEB feature: turn steering wheel while close impacting obstacles in front.
        values.update({
            "LKAS_Output" : 0,
            "LKAS_Active" : 0,
        })

    data = packer.make_can_msg("ACC_MPC_STATE", CanBus.ESC, values)[1]
    values["CheckSum"] = byd_checksum(0xAF, data)
    return packer.make_can_msg("ACC_MPC_STATE", CanBus.ESC, values)

# op long control
def acc_cmd(packer, CP, cam_msg: dict, mrr_leaddist, accel, rfss, sss, longActive):
    values = {}

    values = {s: cam_msg[s] for s in [
        "AccelCmd",
        "ComfortBandUpper",
        "ComfortBandLower",
        "JerkUpperLimit",
        "SETME1_0x1",
        "JerkLowerLimit",
        "ResumeFromStandstill",
        "StandstillState",
        "BrakeBehaviour",
        "AccReqNotStandstill",
        "AccControlActive",
        "AccOverrideOrStandstill",
        "EspBehaviour",
        "Counter",
        "SETME2_0xF",
    ]}

    jerk_base_upper = np.interp(mrr_leaddist, CarControllerParams.K_jerk_xp, CarControllerParams.K_jerk_base_upper_fp)
    jerk_base_lower = np.interp(mrr_leaddist, CarControllerParams.K_jerk_xp, CarControllerParams.K_jerk_base_lower_fp)

    if (accel < 0): #use lower factor
        jerk_upper = jerk_base_upper
        jerk_lower = jerk_base_lower + accel * CarControllerParams.K_accel_jerk_lower
    else:
        jerk_upper = jerk_base_upper + accel * CarControllerParams.K_accel_jerk_upper
        jerk_lower = jerk_base_lower

    if longActive :
        values.update({
            "AccelCmd" : accel,
            "ComfortBandUpper" : 0.05 if mrr_leaddist > 50 else 0.10,
            "ComfortBandLower" : 0.05 if mrr_leaddist > 50 else 0.10,
            "JerkUpperLimit" : jerk_upper,
            "JerkLowerLimit" : jerk_lower,
            "ResumeFromStandstill" : rfss,
            "StandstillState" : sss,
        })

    data = packer.make_can_msg("ACC_CMD", CanBus.ESC, values)[1]
    values["CheckSum"] = byd_checksum(0xAF, data)
    return packer.make_can_msg("ACC_CMD", CanBus.ESC, values)

# send fake torque feedback from eps to trick MPC, preventing DTC, so that safety features such as AEB still working
def create_fake_318(packer, CP, esc_msg: dict, faketorque, laks_reqprepare, laks_active , enabled, counter):
    values = {}

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

    if enabled :
        if laks_active:
            values.update({
                "LKAS_Prepared" : 0,
                "CruiseActivated" : 1,
                "MainTorque" : faketorque,
            })
        elif laks_reqprepare:
            values.update({
                "LKAS_Prepared" : 1,
                "CruiseActivated" : 0,
                "MainTorque" : 0,
            })
        else:
            values.update({
                "LKAS_Prepared" : 0,
                "CruiseActivated" : 0,
                "MainTorque" : 0,
            })


    data = packer.make_can_msg("ACC_EPS_STATE", CanBus.MPC, values)[1]
    values["CheckSum"] = byd_checksum(0xAF, data)
    return packer.make_can_msg("ACC_EPS_STATE", CanBus.MPC, values)
