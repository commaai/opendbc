from collections import namedtuple
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
# from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.demo.values import DBC, CanBus, CarControllerParams

Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])
ButtonType = structs.CarState.ButtonEvent.Type

BUTTONS = [
  Button(structs.CarState.ButtonEvent.Type.setCruise, "ACC_BUTTONS", "BUTTON_SET", [1]),
  Button(structs.CarState.ButtonEvent.Type.resumeCruise, "ACC_BUTTONS", "BUTTON_RESUME", [1]),
  Button(structs.CarState.ButtonEvent.Type.accelCruise, "ACC_BUTTONS", "BUTTON_ACCEL", [1]),
  Button(structs.CarState.ButtonEvent.Type.decelCruise, "ACC_BUTTONS", "BUTTON_DECEL", [1]),
  Button(structs.CarState.ButtonEvent.Type.cancel, "ACC_BUTTONS", "BUTTON_CANCEL", [1]),
  Button(structs.CarState.ButtonEvent.Type.gapAdjustCruise, "ACC_BUTTONS", "BUTTON_GAP_ADJUST", [3]),
]


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CCP = CarControllerParams(CP)
    self.button_states = {button.event_type: False for button in BUTTONS}

    self.frame = 0
    self.acc_buttons_counter = 0

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])  # noqa: F841
    # self.shifter_values = can_define.dv["TRANSMISSION"]["GEAR_POSITION"]

  def create_button_events(self, pt_cp, buttons):
    button_events = []

    for button in buttons:
      state = pt_cp.vl[button.can_addr][button.can_msg] in button.values
      if self.button_states[button.event_type] != state:
        event = structs.CarState.ButtonEvent()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)
      self.button_states[button.event_type] = state

    return button_events

  def update(self, can_parsers) -> structs.CarState:
    """

    For each CarState entity below, find the CAN message/signal, add a DBC entry, and update the code as necessary.

    * When adding a gear position signal, also update the shifter_values dict in __init__
    * When adding ACC button signals, update the BUTTONS namedtuple above
    * If unit conversions are necessary (likely for ACC speed setpoint) uncomment the Conversions import

    """

    pt_cp = can_parsers[Bus.pt]  # noqa: F841
    cam_cp = can_parsers[Bus.cam]  # noqa: F841
    ret = structs.CarState()

    # **** Signals required for a minimum viable car port ******************* #

    # self.parse_wheel_speeds(ret,
    #   pt_cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
    #   pt_cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
    #   pt_cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
    #   pt_cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    # )
    # ret.standstill = ret.vEgoRaw == 0

    # ret.brakePressed = pt_cp.vl["ABS_1"]["DRIVER_BRAKE_PRESSED"]
    # ret.parkingBrake = pt_cp.vl["ABS_2"]["PARKING_BRAKE_SET"]

    # ret.gasPressed = pt_cp.vl["MOTOR_1"]["ACCELERATOR_POSITION"] > 0
    # ret.espDisabled = bool(pt_cp.vl["ABS_3"]["STABILITY_CONTROL_ENABLED"])
    # ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["TRANSMISSION"]["GEAR_POSITION"], None))

    # ret.steeringAngleDeg = pt_cp.vl["EPS_SENSORS"]["STEERING_ANGLE"]
    # ret.steeringTorque = pt_cp.vl["EPS_SENSORS"]["DRIVER_STEERING_TORQUE"]
    # ret.steeringPressed = abs(ret.steeringTorque) > self.CCP.STEER_DRIVER_ALLOWANCE
    # ret.steeringFaultTemporary = bool(pt_cp.vl["EPS_STATE"]["LKAS_TEMP_FAULT"])
    # ret.steeringFaultPermanent = bool(pt_cp.vl["EPS_STATE"]["LKAS_PERM_FAULT"])

    # ret.cruiseState.available = bool(pt_cp.vl["CRUISE_STATE"]["MAIN_SWITCH_ON"])
    # ret.cruiseState.enabled = bool(pt_cp.vl["CRUISE_STATE"]["ACC_ENGAGED"])
    # ret.cruiseState.speed = pt_cp.vl["ACC_HUD"]["SET_SPEED"] * CV.KPH_TO_MS
    # ret.cruiseState.nonAdaptive = not bool(pt_cp.vl["CRUISE_STATE"]["ADAPTIVE"])  # If applicable
    # ret.accFaulted = bool(pt_cp.vl["ACC_HUD"]["ACC_FAULT"])

    # ret.buttonEvents = self.create_button_events(pt_cp, BUTTONS)
    # self.acc_buttons_counter = pt_cp.vl["ACC_BUTTONS"]["COUNTER"]

    # NOTE: if necessary for the signal source, wrap these with update_blinker_from_stalk() or update_blinker_from_lamp()
    # ret.leftBlinker = bool(pt_cp.vl["BCM_1"]["TURN_SIGNAL_LEFT"])
    # ret.rightBlinker = bool(pt_cp.vl["BCM_2"]["TURN_SIGNAL_RIGHT"])

    # ret.seatbeltUnlatched = not bool(pt_cp.vl["SEATBELT"]["SEATBELT_LATCHED_DRIVER"])
    # NOTE: Get all door states if possible, but some cars only broadcast the driver's door state on their ADAS bus
    # ret.doorOpen = any([pt_cp.vl["DOOR_STATES"]["DOOR_OPEN_FL"],
    #                     pt_cp.vl["DOOR_STATES"]["DOOR_OPEN_FR"],
    #                     pt_cp.vl["DOOR_STATES"]["DOOR_OPEN_RL"],
    #                     pt_cp.vl["DOOR_STATES"]["DOOR_OPEN_RR"]])

    # **** Desirable signals, if available ********************************** #

    # if self.CP.enableBsm:
    #   ret.leftBlindspot = bool(pt_cp.vl["BSM"]["BLINDSPOT_WARNING_LEFT"])
    #   ret.rightBlindspot = bool(pt_cp.vl["BSM"]["BLINDSPOT_WARNING_RIGHT"])

    # ret.espActive = bool(pt_cp.vl["ABS_3"]["STABILITY_CONTROL_ACTIVE"])
    # ret.stockFcw = bool(pt_cp.vl["FCW_AEB_HUD"]["FCW_WARNING"])
    # ret.stockAeb = bool(pt_cp.vl["FCW_AEB_HUD"]["AEB_WARNING"])

    # NOTE: Use these signals only if the driver-displayed vehicle and cruise set speeds are offset from true wheel speeds
    #   ret.vEgoCluster = [ read or calculate as necessary ]
    #   ret.vCruiseCluster = [ read or calculate as necessary ]

    self.frame += 1
    return ret

  @staticmethod
  def get_can_parsers(CP):
    # NOTE: Message list entries are only needed for messages with variable frequencies that can't be auto-detected
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus(CP).pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus(CP).cam),
    }
