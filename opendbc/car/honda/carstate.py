from collections import defaultdict

from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car import create_button_events, structs
from openpilot.selfdrive.car.common.conversions import Conversions as CV
from openpilot.selfdrive.car.common.numpy_fast import interp
from openpilot.selfdrive.car.honda.hondacan import CanBus, get_cruise_speed_conversion
from openpilot.selfdrive.car.honda.values import CAR, DBC, STEER_THRESHOLD, HONDA_BOSCH, \
                                                 HONDA_NIDEC_ALT_SCM_MESSAGES, HONDA_BOSCH_RADARLESS, \
                                                 HondaFlags, CruiseButtons, CruiseSettings
from openpilot.selfdrive.car.interfaces import CarStateBase

TransmissionType = structs.CarParams.TransmissionType
ButtonType = structs.CarState.ButtonEvent.Type

BUTTONS_DICT = {CruiseButtons.RES_ACCEL: ButtonType.accelCruise, CruiseButtons.DECEL_SET: ButtonType.decelCruise,
                CruiseButtons.MAIN: ButtonType.altButton3, CruiseButtons.CANCEL: ButtonType.cancel}
SETTINGS_BUTTONS_DICT = {CruiseSettings.DISTANCE: ButtonType.gapAdjustCruise, CruiseSettings.LKAS: ButtonType.altButton1}


def get_can_messages(CP, gearbox_msg):
  messages = [
    ("ENGINE_DATA", 100),
    ("WHEEL_SPEEDS", 50),
    ("STEERING_SENSORS", 100),
    ("SEATBELT_STATUS", 10),
    ("CRUISE", 10),
    ("POWERTRAIN_DATA", 100),
    ("CAR_SPEED", 10),
    ("VSA_STATUS", 50),
    ("STEER_STATUS", 100),
    ("STEER_MOTOR_TORQUE", 0),  # TODO: not on every car
  ]

  if CP.carFingerprint == CAR.HONDA_ODYSSEY_CHN:
    messages += [
      ("SCM_FEEDBACK", 25),
      ("SCM_BUTTONS", 50),
    ]
  else:
    messages += [
      ("SCM_FEEDBACK", 10),
      ("SCM_BUTTONS", 25),
    ]

  if CP.carFingerprint in (CAR.HONDA_CRV_HYBRID, CAR.HONDA_CIVIC_BOSCH_DIESEL, CAR.ACURA_RDX_3G, CAR.HONDA_E):
    messages.append((gearbox_msg, 50))
  else:
    messages.append((gearbox_msg, 100))

  if CP.flags & HondaFlags.BOSCH_ALT_BRAKE:
    messages.append(("BRAKE_MODULE", 50))

  if CP.carFingerprint in (HONDA_BOSCH | {CAR.HONDA_CIVIC, CAR.HONDA_ODYSSEY, CAR.HONDA_ODYSSEY_CHN}):
    messages.append(("EPB_STATUS", 50))

  if CP.carFingerprint in HONDA_BOSCH:
    # these messages are on camera bus on radarless cars
    if not CP.openpilotLongitudinalControl and CP.carFingerprint not in HONDA_BOSCH_RADARLESS:
      messages += [
        ("ACC_HUD", 10),
        ("ACC_CONTROL", 50),
      ]
  else:  # Nidec signals
    if CP.carFingerprint == CAR.HONDA_ODYSSEY_CHN:
      messages.append(("CRUISE_PARAMS", 10))
    else:
      messages.append(("CRUISE_PARAMS", 50))

  # TODO: clean this up
  if CP.carFingerprint in (CAR.HONDA_ACCORD, CAR.HONDA_CIVIC_BOSCH, CAR.HONDA_CIVIC_BOSCH_DIESEL, CAR.HONDA_CRV_HYBRID, CAR.HONDA_INSIGHT,
                           CAR.ACURA_RDX_3G, CAR.HONDA_E, CAR.HONDA_CIVIC_2022, CAR.HONDA_HRV_3G):
    pass
  elif CP.carFingerprint in (CAR.HONDA_ODYSSEY_CHN, CAR.HONDA_FREED, CAR.HONDA_HRV):
    pass
  else:
    messages.append(("DOORS_STATUS", 3))

  if CP.carFingerprint in HONDA_BOSCH_RADARLESS:
    messages.append(("CRUISE_FAULT_STATUS", 50))
  elif CP.openpilotLongitudinalControl:
    messages.append(("STANDSTILL", 50))

  return messages


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.gearbox_msg = "GEARBOX"
    if CP.carFingerprint == CAR.HONDA_ACCORD and CP.transmissionType == TransmissionType.cvt:
      self.gearbox_msg = "GEARBOX_15T"

    self.main_on_sig_msg = "SCM_FEEDBACK"
    if CP.carFingerprint in HONDA_NIDEC_ALT_SCM_MESSAGES:
      self.main_on_sig_msg = "SCM_BUTTONS"

    self.shifter_values = can_define.dv[self.gearbox_msg]["GEAR_SHIFTER"]
    self.steer_status_values = defaultdict(lambda: "UNKNOWN", can_define.dv["STEER_STATUS"]["STEER_STATUS"])

    self.brake_switch_prev = False
    self.brake_switch_active = False
    self.cruise_setting = 0
    self.v_cruise_pcm_prev = 0

    # When available we use cp.vl["CAR_SPEED"]["ROUGH_CAR_SPEED_2"] to populate vEgoCluster
    # However, on cars without a digital speedometer this is not always present (HRV, FIT, CRV 2016, ILX and RDX)
    self.dash_speed_seen = False

  def update(self, cp, cp_cam, _, cp_body, __) -> structs.CarState:
    ret = structs.CarState()

    # car params
    v_weight_v = [0., 1.]  # don't trust smooth speed at low values to avoid premature zero snapping
    v_weight_bp = [1., 6.]   # smooth blending, below ~0.6m/s the smooth speed snaps to zero

    # update prevs, update must run once per loop
    prev_cruise_buttons = self.cruise_buttons
    prev_cruise_setting = self.cruise_setting
    self.cruise_setting = cp.vl["SCM_BUTTONS"]["CRUISE_SETTING"]
    self.cruise_buttons = cp.vl["SCM_BUTTONS"]["CRUISE_BUTTONS"]

    # used for car hud message
    self.is_metric = not cp.vl["CAR_SPEED"]["IMPERIAL_UNIT"]

    # ******************* parse out can *******************
    # STANDSTILL->WHEELS_MOVING bit can be noisy around zero, so use XMISSION_SPEED
    # panda checks if the signal is non-zero
    ret.standstill = cp.vl["ENGINE_DATA"]["XMISSION_SPEED"] < 1e-5
    # TODO: find a common signal across all cars
    if self.CP.carFingerprint in (CAR.HONDA_ACCORD, CAR.HONDA_CIVIC_BOSCH, CAR.HONDA_CIVIC_BOSCH_DIESEL, CAR.HONDA_CRV_HYBRID, CAR.HONDA_INSIGHT,
                                  CAR.ACURA_RDX_3G, CAR.HONDA_E, CAR.HONDA_CIVIC_2022, CAR.HONDA_HRV_3G):
      ret.doorOpen = bool(cp.vl["SCM_FEEDBACK"]["DRIVERS_DOOR_OPEN"])
    elif self.CP.carFingerprint in (CAR.HONDA_ODYSSEY_CHN, CAR.HONDA_FREED, CAR.HONDA_HRV):
      ret.doorOpen = bool(cp.vl["SCM_BUTTONS"]["DRIVERS_DOOR_OPEN"])
    else:
      ret.doorOpen = any([cp.vl["DOORS_STATUS"]["DOOR_OPEN_FL"], cp.vl["DOORS_STATUS"]["DOOR_OPEN_FR"],
                          cp.vl["DOORS_STATUS"]["DOOR_OPEN_RL"], cp.vl["DOORS_STATUS"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = bool(cp.vl["SEATBELT_STATUS"]["SEATBELT_DRIVER_LAMP"] or not cp.vl["SEATBELT_STATUS"]["SEATBELT_DRIVER_LATCHED"])

    steer_status = self.steer_status_values[cp.vl["STEER_STATUS"]["STEER_STATUS"]]
    ret.steerFaultPermanent = steer_status not in ("NORMAL", "NO_TORQUE_ALERT_1", "NO_TORQUE_ALERT_2", "LOW_SPEED_LOCKOUT", "TMP_FAULT")
    # LOW_SPEED_LOCKOUT is not worth a warning
    # NO_TORQUE_ALERT_2 can be caused by bump or steering nudge from driver
    ret.steerFaultTemporary = steer_status not in ("NORMAL", "LOW_SPEED_LOCKOUT", "NO_TORQUE_ALERT_2")

    if self.CP.carFingerprint in HONDA_BOSCH_RADARLESS:
      ret.accFaulted = bool(cp.vl["CRUISE_FAULT_STATUS"]["CRUISE_FAULT"])
    else:
      # On some cars, these two signals are always 1, this flag is masking a bug in release
      # FIXME: find and set the ACC faulted signals on more platforms
      if self.CP.openpilotLongitudinalControl:
        ret.accFaulted = bool(cp.vl["STANDSTILL"]["BRAKE_ERROR_1"] or cp.vl["STANDSTILL"]["BRAKE_ERROR_2"])

      # Log non-critical stock ACC/LKAS faults if Nidec (camera)
      if self.CP.carFingerprint not in HONDA_BOSCH:
        ret.carFaultedNonCritical = bool(cp_cam.vl["ACC_HUD"]["ACC_PROBLEM"] or cp_cam.vl["LKAS_HUD"]["LKAS_PROBLEM"])

    ret.espDisabled = cp.vl["VSA_STATUS"]["ESP_DISABLED"] != 0

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    v_wheel = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.0

    # blend in transmission speed at low speed, since it has more low speed accuracy
    v_weight = interp(v_wheel, v_weight_bp, v_weight_v)
    ret.vEgoRaw = (1. - v_weight) * cp.vl["ENGINE_DATA"]["XMISSION_SPEED"] * CV.KPH_TO_MS * self.CP.wheelSpeedFactor + v_weight * v_wheel
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    self.dash_speed_seen = self.dash_speed_seen or cp.vl["CAR_SPEED"]["ROUGH_CAR_SPEED_2"] > 1e-3
    if self.dash_speed_seen:
      conversion = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS
      ret.vEgoCluster = cp.vl["CAR_SPEED"]["ROUGH_CAR_SPEED_2"] * conversion

    ret.steeringAngleDeg = cp.vl["STEERING_SENSORS"]["STEER_ANGLE"]
    ret.steeringRateDeg = cp.vl["STEERING_SENSORS"]["STEER_ANGLE_RATE"]

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(
      250, cp.vl["SCM_FEEDBACK"]["LEFT_BLINKER"], cp.vl["SCM_FEEDBACK"]["RIGHT_BLINKER"])
    ret.brakeHoldActive = cp.vl["VSA_STATUS"]["BRAKE_HOLD_ACTIVE"] == 1

    # TODO: set for all cars
    if self.CP.carFingerprint in (HONDA_BOSCH | {CAR.HONDA_CIVIC, CAR.HONDA_ODYSSEY, CAR.HONDA_ODYSSEY_CHN}):
      ret.parkingBrake = cp.vl["EPB_STATUS"]["EPB_STATE"] != 0

    gear = int(cp.vl[self.gearbox_msg]["GEAR_SHIFTER"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear, None))

    ret.gas = cp.vl["POWERTRAIN_DATA"]["PEDAL_GAS"]
    ret.gasPressed = ret.gas > 1e-5

    ret.steeringTorque = cp.vl["STEER_STATUS"]["STEER_TORQUE_SENSOR"]
    ret.steeringTorqueEps = cp.vl["STEER_MOTOR_TORQUE"]["MOTOR_TORQUE"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD.get(self.CP.carFingerprint, 1200)  # type: ignore[call-overload]

    if self.CP.carFingerprint in HONDA_BOSCH:
      # The PCM always manages its own cruise control state, but doesn't publish it
      if self.CP.carFingerprint in HONDA_BOSCH_RADARLESS:
        ret.cruiseState.nonAdaptive = cp_cam.vl["ACC_HUD"]["CRUISE_CONTROL_LABEL"] != 0

      if not self.CP.openpilotLongitudinalControl:
        # ACC_HUD is on camera bus on radarless cars
        acc_hud = cp_cam.vl["ACC_HUD"] if self.CP.carFingerprint in HONDA_BOSCH_RADARLESS else cp.vl["ACC_HUD"]
        ret.cruiseState.nonAdaptive = acc_hud["CRUISE_CONTROL_LABEL"] != 0
        ret.cruiseState.standstill = acc_hud["CRUISE_SPEED"] == 252.

        conversion = get_cruise_speed_conversion(self.CP.carFingerprint, self.is_metric)
        # On set, cruise set speed pulses between 254~255 and the set speed prev is set to avoid this.
        ret.cruiseState.speed = self.v_cruise_pcm_prev if acc_hud["CRUISE_SPEED"] > 160.0 else acc_hud["CRUISE_SPEED"] * conversion
        self.v_cruise_pcm_prev = ret.cruiseState.speed
    else:
      ret.cruiseState.speed = cp.vl["CRUISE"]["CRUISE_SPEED_PCM"] * CV.KPH_TO_MS

    if self.CP.flags & HondaFlags.BOSCH_ALT_BRAKE:
      ret.brakePressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0
    else:
      # brake switch has shown some single time step noise, so only considered when
      # switch is on for at least 2 consecutive CAN samples
      # brake switch rises earlier than brake pressed but is never 1 when in park
      brake_switch_vals = cp.vl_all["POWERTRAIN_DATA"]["BRAKE_SWITCH"]
      if len(brake_switch_vals):
        brake_switch = cp.vl["POWERTRAIN_DATA"]["BRAKE_SWITCH"] != 0
        if len(brake_switch_vals) > 1:
          self.brake_switch_prev = brake_switch_vals[-2] != 0
        self.brake_switch_active = brake_switch and self.brake_switch_prev
        self.brake_switch_prev = brake_switch
      ret.brakePressed = (cp.vl["POWERTRAIN_DATA"]["BRAKE_PRESSED"] != 0) or self.brake_switch_active

    ret.brake = cp.vl["VSA_STATUS"]["USER_BRAKE"]
    ret.cruiseState.enabled = cp.vl["POWERTRAIN_DATA"]["ACC_STATUS"] != 0
    ret.cruiseState.available = bool(cp.vl[self.main_on_sig_msg]["MAIN_ON"])

    # Gets rid of Pedal Grinding noise when brake is pressed at slow speeds for some models
    if self.CP.carFingerprint in (CAR.HONDA_PILOT, CAR.HONDA_RIDGELINE):
      if ret.brake > 0.1:
        ret.brakePressed = True

    if self.CP.carFingerprint in HONDA_BOSCH:
      # TODO: find the radarless AEB_STATUS bit and make sure ACCEL_COMMAND is correct to enable AEB alerts
      if self.CP.carFingerprint not in HONDA_BOSCH_RADARLESS:
        ret.stockAeb = (not self.CP.openpilotLongitudinalControl) and bool(cp.vl["ACC_CONTROL"]["AEB_STATUS"] and cp.vl["ACC_CONTROL"]["ACCEL_COMMAND"] < -1e-5)
    else:
      ret.stockAeb = bool(cp_cam.vl["BRAKE_COMMAND"]["AEB_REQ_1"] and cp_cam.vl["BRAKE_COMMAND"]["COMPUTER_BRAKE"] > 1e-5)

    self.acc_hud = False
    self.lkas_hud = False
    if self.CP.carFingerprint not in HONDA_BOSCH:
      ret.stockFcw = cp_cam.vl["BRAKE_COMMAND"]["FCW"] != 0
      self.acc_hud = cp_cam.vl["ACC_HUD"]
      self.stock_brake = cp_cam.vl["BRAKE_COMMAND"]
    if self.CP.carFingerprint in HONDA_BOSCH_RADARLESS:
      self.lkas_hud = cp_cam.vl["LKAS_HUD"]

    if self.CP.enableBsm:
      # BSM messages are on B-CAN, requires a panda forwarding B-CAN messages to CAN 0
      # more info here: https://github.com/commaai/openpilot/pull/1867
      ret.leftBlindspot = cp_body.vl["BSM_STATUS_LEFT"]["BSM_ALERT"] == 1
      ret.rightBlindspot = cp_body.vl["BSM_STATUS_RIGHT"]["BSM_ALERT"] == 1

    ret.buttonEvents = [
      *create_button_events(self.cruise_buttons, prev_cruise_buttons, BUTTONS_DICT),
      *create_button_events(self.cruise_setting, prev_cruise_setting, SETTINGS_BUTTONS_DICT),
    ]

    return ret

  def get_can_parser(self, CP):
    messages = get_can_messages(CP, self.gearbox_msg)
    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).pt)

  @staticmethod
  def get_cam_can_parser(CP):
    messages = [
      ("STEERING_CONTROL", 100),
    ]

    if CP.carFingerprint in HONDA_BOSCH_RADARLESS:
      messages += [
        ("ACC_HUD", 10),
        ("LKAS_HUD", 10),
      ]

    elif CP.carFingerprint not in HONDA_BOSCH:
      messages += [
        ("ACC_HUD", 10),
        ("LKAS_HUD", 10),
        ("BRAKE_COMMAND", 50),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).camera)

  @staticmethod
  def get_body_can_parser(CP):
    if CP.enableBsm:
      messages = [
        ("BSM_STATUS_LEFT", 3),
        ("BSM_STATUS_RIGHT", 3),
      ]
      bus_body = CanBus(CP).radar # B-CAN is forwarded to ACC-CAN radar side (CAN 0 on fake ethernet port)
      return CANParser(DBC[CP.carFingerprint]["body"], messages, bus_body)
    return None
