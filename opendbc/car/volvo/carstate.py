import copy
from cereal import car
from opendbc.can import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.volvo.values import CarControllerParams, DBC, CANBUS
from opendbc.car import Bus, structs

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.cruiseState_enabled_prev = False
    self.eps_torque_timer = 0
    self.frame = 0
    self._cruise_speed_prev_kph = 0
    self._pending_delta = 0
    self._icbm_suppress_frames = 0
    self._custom_acc_step = 1  # set by carcontroller each frame; default 1 km/h

  def update(self, can_parsers) -> structs.CarState:
    pt_cp = can_parsers[Bus.pt]
    cam_cp = can_parsers[Bus.cam]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # car speed
    ret.vEgoRaw = pt_cp.vl["VehicleSpeed1"]["VehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    # gas pedal
    ret.gasPressed = pt_cp.vl["AccPedal"]["AccPedal"] >= 10

    # brake pedal
    ret.brakePressed = pt_cp.vl["Brake_Info"]["BrakePedal"] == 2

    # steering
    ret.steeringAngleDeg = pt_cp.vl["PSCM1"]["SteeringAngleServo"]
    ret.steeringRateDeg = pt_cp.vl["SAS0"]["SteeringRateOfChange"]
    self.steeringDirection = pt_cp.vl["SAS0"]["SteeringDirection"]
    ret.steeringTorque = pt_cp.vl["PSCM1"]["EPSTorque"]
    if self.steeringDirection:
      ret.steeringTorque = -abs(ret.steeringTorque)
    ret.steeringTorqueEps = pt_cp.vl["PSCM1"]["LKATorque"]
    ret.steeringPressed = abs(ret.steeringTorque) > 50

    # cruise state
    ret.cruiseState.speed = pt_cp.vl["ACC_Speed"]["ACC_Speed"] * CV.KPH_TO_MS
    ret.cruiseState.speedCluster = ret.cruiseState.speed
    ret.cruiseState.available = bool(cam_cp.vl["FSM0"]["ACC_Available"])
    ret.cruiseState.enabled = bool(cam_cp.vl["FSM0"]["ACC_Enabled"])
    # standstill hold; SNG uses this to time the resume blast (hard-cancel without it, drive 38)
    ret.cruiseState.standstill = bool(cam_cp.vl["FSM3"]["ACC_Standstill"])
    ret.cruiseState.nonAdaptive = False
    ret.accFaulted = False
    self.acc_distance = cam_cp.vl["FSM1"]["ACC_Distance"]

    # Check if servo stops responding when ACC is active
    if ret.cruiseState.enabled and ret.vEgo > self.CP.minSteerSpeed:
      if not self.cruiseState_enabled_prev:
        self.eps_torque_timer = 0

      if ret.steeringTorqueEps == 0:
        self.eps_torque_timer += 1
      else:
        self.eps_torque_timer = 0

      ret.steerFaultTemporary = self.eps_torque_timer >= CarControllerParams.STEER_TIMEOUT
    else:
      ret.steerFaultTemporary = False

    self.cruiseState_enabled_prev = ret.cruiseState.enabled

    # gear
    ret.gearShifter = car.CarState.GearShifter.drive

    # safety
    ret.stockFcw = False
    ret.stockAeb = False

    # button presses
    ret.leftBlinker = pt_cp.vl["MiscCarInfo"]["TurnSignal"] == 1
    ret.rightBlinker = pt_cp.vl["MiscCarInfo"]["TurnSignal"] == 3

    # Synthesize one accelCruise/decelCruise per frame from pending delta.
    # Suppressed after ICBM sends a button to avoid feedback into v_cruise_kph.
    cruise_kph_now = round(ret.cruiseState.speed * CV.MS_TO_KPH)
    if self._icbm_suppress_frames > 0:
      self._cruise_speed_prev_kph = cruise_kph_now
      self._icbm_suppress_frames -= 1
    elif ret.cruiseState.enabled and self.cruiseState_enabled_prev:
      self._pending_delta += cruise_kph_now - self._cruise_speed_prev_kph
    self._cruise_speed_prev_kph = cruise_kph_now

    # Emit one event per physical ACC step; trigger on delta >= 1 to handle Volvo's
    # sub-step snaps (e.g. 41→45 km/h) that would never reach a larger threshold.
    step = max(1, self._custom_acc_step)
    if self._pending_delta >= 1:
      ret.buttonEvents = [structs.CarState.ButtonEvent(pressed=False, type=ButtonType.accelCruise)]
      self._pending_delta = max(0, self._pending_delta - step)
    elif self._pending_delta <= -1:
      ret.buttonEvents = [structs.CarState.ButtonEvent(pressed=False, type=ButtonType.decelCruise)]
      self._pending_delta = min(0, self._pending_delta + step)

    # lock info
    ret.doorOpen = not all([pt_cp.vl["Doors"]["DriverDoorClosed"], pt_cp.vl["Doors"]["PassengerDoorClosed"]])
    ret.seatbeltUnlatched = False

    self.pscm_stock_values = pt_cp.vl["PSCM1"]
    self.stock_FSM1 = copy.copy(cam_cp.vl["FSM1"])
    self.stock_FSM3 = copy.copy(cam_cp.vl["FSM3"])
    self.ACC_Check = cam_cp.vl["FSM3"]["ACC_Check"]

    # TSR speed limit from camera (0 = no sign)
    tsr_raw = cam_cp.vl["FSM5"]["TSR_Speed"]
    ret_sp.speedLimit = tsr_raw * CV.KPH_TO_MS if tsr_raw > 0 else 0.0

    self.frame += 1
    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = [
      ("VehicleSpeed1", 50),
      ("AccPedal", 100),
      ("BrakePedal", 50),
      ("Brake_Info", 50),
      ("PSCM1", 50),
      ("ACC_Speed", 50),
      ("MiscCarInfo", 25),
      ("Doors", 20),
      ("SAS0", 100)
    ]

    cam_messages = [
      ("FSM0", 100),
      ("FSM1", 50),
      ("FSM3", 50),
      ("FSM5", 10),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CANBUS.pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CANBUS.cam),
    }
