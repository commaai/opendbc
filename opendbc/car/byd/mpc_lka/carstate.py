import copy

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC, CANBUS, CAR, BydFlags, HUD_MULTIPLIER, LKASConfig

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["DRIVE_STATE"]["Gear"]

    self.lkas_prepared = False
    self.acc_state = 0
    self.steering_rate_deg_abs = 0.0
    self.acc_mpc_state_counter = 0
    self.eps_state_counter = 0
    self.mpc_lkas_output = 0.0
    self.mpc_lkas_reqprepare = False
    self.mpc_lkas_active = False
    self.cam_lkas = {}
    self.esc_eps = {}
    self.btn_acc_cancel = 0
    self.btn_acc_set_reset = 0
    self.btn_acc_dist_inc = 0
    self.btn_acc_dist_dec = 0

  def update(self, can_parsers):
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()
    ret.buttonEvents = []

    self.lkas_prepared = bool(cp.vl["ACC_EPS_STATE"]["LKAS_Prepared"])
    mpc_lkas_config = int(cp_cam.vl["ACC_MPC_STATE"]["LKAS_Config"])
    lkas_config_is_acc_on = mpc_lkas_config != LKASConfig.DISABLE
    lkas_main_sw_on = bool(cp.vl["PCM_BUTTONS"]["BTN_TOGGLE_ACC_OnOff"])

    acc_parser = cp if self.CP.flags & BydFlags.ACC_ON_ESC else cp_cam
    lkas_hud_acc_on1 = bool(acc_parser.vl["ACC_HUD_ADAS"]["AccOn1"])
    self.acc_state = int(acc_parser.vl["ACC_HUD_ADAS"]["AccState"])

    prev_btn_acc_cancel = self.btn_acc_cancel
    prev_btn_acc_set_reset = self.btn_acc_set_reset
    prev_btn_acc_dist_inc = self.btn_acc_dist_inc
    prev_btn_acc_dist_dec = self.btn_acc_dist_dec

    self.btn_acc_cancel = int(cp.vl["PCM_BUTTONS"]["BTN_AccCancel"])
    self.btn_acc_set_reset = int(cp.vl["PCM_BUTTONS"]["BTN_AccUpDown_Cmd"])
    self.btn_acc_dist_inc = int(cp.vl["PCM_BUTTONS"]["BTN_AccDistanceIncrease"])
    self.btn_acc_dist_dec = int(cp.vl["PCM_BUTTONS"]["BTN_AccDistanceDecrease"])

    # Song Plus flipped harness: wheel speed on bus 0 (byd_general_pt layout), not Han CARSPEED.
    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_FL"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_FR"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_BL"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_BR"],
    )
    ret.vEgoCluster = ret.vEgo * HUD_MULTIPLIER
    ret.standstill = ret.vEgoRaw < 0.01

    can_gear = int(cp.vl["DRIVE_STATE"]["Gear"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.genericToggle = bool(cp.vl["STALKS"]["HeadLight"])
    ret.leftBlinker = bool(cp.vl["STALKS"]["LeftIndicator"])
    ret.rightBlinker = bool(cp.vl["STALKS"]["RightIndicator"])

    if self.CP.enableBsm:
      ret.leftBlindspot = bool(cp.vl["BSD_RADAR"]["LEFT_APPROACH"])
      ret.rightBlindspot = bool(cp.vl["BSD_RADAR"]["RIGHT_APPROACH"])

    ret.steeringAngleDeg = float(cp.vl["EPS"]["SteeringAngle"])
    self.steering_rate_deg_abs = float(cp.vl["EPS"]["SteeringAngleRate"])
    ret.steeringRateDeg = self.steering_rate_deg_abs
    ret.steeringTorque = float(cp.vl["ACC_EPS_STATE"]["SteerDriverTorque"])
    ret.steeringTorqueEps = float(cp.vl["ACC_EPS_STATE"]["MainTorque"])
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 59, 5)

    ret.parkingBrake = bool(cp.vl["EPB"]["EPB_ActiveFlag"] == 1)
    ret.deprecated.brake = float(cp.vl["PEDAL"]["BrakePedal"])
    ret.brakePressed = ret.deprecated.brake != 0 or bool(cp.vl["DRIVE_STATE"]["BrakePressed"])
    ret.gasPressed = float(cp.vl["PEDAL"]["AcceleratorPedal"]) != 0

    ret.doorOpen = any([
      cp.vl["BCM"]["FrontLeftDoor"],
      cp.vl["BCM"]["FrontRightDoor"],
      cp.vl["BCM"]["RearLeftDoor"],
      cp.vl["BCM"]["RearRightDoor"],
    ])
    ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]["SEATBELT_DRIVER"] == 0

    ret.cruiseState.available = lkas_main_sw_on and lkas_config_is_acc_on and lkas_hud_acc_on1
    ret.cruiseState.enabled = self.acc_state in (3, 5)
    ret.cruiseState.standstill = ret.standstill
    ret.cruiseState.speed = float(acc_parser.vl["ACC_HUD_ADAS"]["SetSpeed"]) * CV.KPH_TO_MS
    ret.cruiseState.speedCluster = ret.cruiseState.speed
    ret.cruiseState.nonAdaptive = False

    ret.steerFaultTemporary = bool((self.acc_state == 7) or cp.vl["ACC_EPS_STATE"]["SteerWarning"])
    ret.steerFaultPermanent = bool(cp.vl["ACC_EPS_STATE"]["TorqueFailed"])

    self.mpc_lkas_output = float(cp_cam.vl["ACC_MPC_STATE"]["LKAS_Output"])
    self.mpc_lkas_reqprepare = bool(cp_cam.vl["ACC_MPC_STATE"]["LKAS_ReqPrepare"])
    self.mpc_lkas_active = bool(cp_cam.vl["ACC_MPC_STATE"]["LKAS_Active"])
    self.acc_mpc_state_counter = int(cp_cam.vl["ACC_MPC_STATE"]["Counter"])
    self.eps_state_counter = int(cp.vl["ACC_EPS_STATE"]["Counter"])
    self.cam_lkas = copy.copy(cp_cam.vl["ACC_MPC_STATE"])
    self.esc_eps = copy.copy(cp.vl["ACC_EPS_STATE"])

    ret.buttonEvents = [
      *create_button_events(self.btn_acc_cancel, prev_btn_acc_cancel, {1: ButtonType.cancel}),
      *create_button_events(self.btn_acc_set_reset, prev_btn_acc_set_reset, {1: ButtonType.decelCruise, 3: ButtonType.accelCruise}),
      *create_button_events(self.btn_acc_dist_inc, prev_btn_acc_dist_inc, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.btn_acc_dist_dec, prev_btn_acc_dist_dec, {1: ButtonType.gapAdjustCruise}),
    ]

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CarState.get_can_parser(CP),
      Bus.cam: CarState.get_cam_can_parser(CP),
    }

  @staticmethod
  def get_can_parser(CP):
    signals = [
      ("EPS", 100),
      ("WHEEL_SPEED", 50),
      ("PEDAL", 50),
      ("EPB", 1),
      ("ACC_EPS_STATE", 50),
      ("DRIVE_STATE", 50),
      ("STALKS", 20),
      ("BCM", 20),
      ("METER_CLUSTER", 20),
      ("PCM_BUTTONS", 20),
    ]

    if CP.flags & BydFlags.ACC_ON_ESC:
      signals += [
        ("ACC_HUD_ADAS", 50),
        ("ACC_CMD", 50),
      ]

    if CP.enableBsm:
      signals.append(("BSD_RADAR", 20))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, CANBUS.main_bus)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      ("ACC_MPC_STATE", 50),
    ]

    if not (CP.flags & BydFlags.ACC_ON_ESC):
      signals += [
        ("ACC_HUD_ADAS", 50),
        ("ACC_CMD", 50),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, CANBUS.cam_bus)
