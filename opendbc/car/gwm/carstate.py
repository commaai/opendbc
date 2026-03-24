from opendbc.car import Bus, CanBusBase, create_button_events, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.gwm.values import DBC
import copy

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    # Store original message to copy it later in carcontroller
    self.steer_and_ap_stalk_msg = {}
    self.eps_stock_values = {}
    self.camera_stock_values = {}
    self.longitudinal_stock_values = {}

    self.is_activation_lever_pulled = False
    self.prev_activation_lever_pulled = False
    self.main_on = False
    self.steer_fault_temporary_counter = 0
    self.distance_button = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.main]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    prev_distance_button = self.distance_button
    self.distance_button = cp.vl["STEER_AND_AP_STALK"]["AP_REDUCE_DISTANCE_COMMAND"] or \
                           cp.vl["STEER_AND_AP_STALK"]["AP_INCREASE_DISTANCE_COMMAND"]

    # Store the original message to reuse in carcontroller
    self.steer_and_ap_stalk_msg = copy.copy(cp.vl["STEER_AND_AP_STALK"])
    self.eps_stock_values = copy.copy(cp.vl["RX_STEER_RELATED"])
    self.camera_stock_values = copy.copy(cp_cam.vl["STEER_CMD"])
    self.longitudinal_stock_values = copy.copy(cp_cam.vl["ACC_CMD"])

    # car speed
    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FRONT_LEFT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["FRONT_RIGHT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["REAR_LEFT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["REAR_RIGHT_WHEEL_SPEED"]
    )

    ret.accFaulted = bool(cp_cam.vl["ACC"]["CRUISE_STATE_2"] == 0)
    ret.cruiseState.speed = cp_cam.vl["ACC"]["ACC_SPEED_SELECTION"]  * CV.KPH_TO_MS
    if not self.CP.openpilotLongitudinalControl:
      ret.cruiseState.speed = -1

    ret.standstill = abs(ret.vEgoRaw) < 1e-3
    ret.gasPressed = cp.vl["CAR_OVERALL_SIGNALS2"]["GAS_POSITION"] > 0
    ret.brakePressed = cp.vl["BRAKE2"]["PEDAL_BRAKE_PRESSED"] != 0
    ret.brake = cp.vl["BRAKE"]["BRAKE_PRESSURE"] if not ret.brakePressed else 0
    ret.parkingBrake = cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"] == 0

    ret.gearShifter = GearShifter.drive if int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"]) == 1 else \
                      GearShifter.neutral if int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"]) == 2 else \
                      GearShifter.reverse if int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"]) == 3 else \
                      GearShifter.park

    ret.steeringAngleDeg = cp.vl["STEER_AND_AP_STALK"]["STEERING_ANGLE"] * (-1 if cp.vl["STEER_AND_AP_STALK"]["STEERING_DIRECTION"] else 1)
    ret.steeringRateDeg = cp.vl["STEER_AND_AP_STALK"]["STEERING_RATE"] * (-1 if (cp.vl["STEER_AND_AP_STALK"]["RATE_DIRECTION"] > 0) else 1)

    # Since loopback was throwing a CanError even though the logic expected cp_loopback.vl_vall > 0, I moved the detection to interface.py against lat_active.
    ret.steerFaultTemporary = False # (bool(cp_loopback.vl["STEER_CMD"]["STEER_REQUEST"]) and bool(cp.vl["RX_STEER_RELATED"]["A_RX_STEER_REQUESTED"] != 1))
    self.steer_fault_temporary_counter = (self.steer_fault_temporary_counter + 1) if (cp.vl["RX_STEER_RELATED"]["EPS_FAULT_PERMANENT"] == 1) else 0
    ret.steerFaultTemporary |= self.steer_fault_temporary_counter > 100
    ret.steerFaultPermanent = False #self.steer_fault_permanent_counter > 500

    ret.steeringTorque = cp.vl["RX_STEER_RELATED"]["B_RX_DRIVER_TORQUE"]
    ret.steeringTorqueEps = cp.vl["RX_STEER_RELATED"]["B_RX_EPS_TORQUE"]
    ret.steeringPressed = abs(ret.steeringTorque) > 50

    ret.doorOpen = any([cp.vl["DOOR_DRIVER"]["DOOR_REAR_RIGHT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_FRONT_RIGHT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_REAR_LEFT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_DRIVER_OPEN"]])
    ret.seatbeltUnlatched = bool(cp.vl["SEATBELT"]["SEAT_BELT_DRIVER_STATE"])
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["LIGHTS"]["LEFT_TURN_SIGNAL"],
                                                                      cp.vl["LIGHTS"]["RIGHT_TURN_SIGNAL"])

    if cp.vl["STEER_AND_AP_STALK"]["AP_CANCEL_COMMAND"] or ret.brakePressed:
      self.main_on = False
    self.is_activation_lever_pulled = bool(cp.vl["STEER_AND_AP_STALK"]["AP_ENABLE_COMMAND"])
    if self.is_activation_lever_pulled and not self.prev_activation_lever_pulled and not self.main_on:
      self.main_on = True
    self.prev_activation_lever_pulled = self.is_activation_lever_pulled

    ret.cruiseState.available = self.main_on
    ret.cruiseState.enabled = self.main_on

    ret.buttonEvents = create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})

    return ret

  @staticmethod
  def get_can_parsers(CP):
    # Compute bus offset from number of safetyConfigs so multipanda setups
    # (internal + external pandas) map DBCs to the correct physical bus.
    can_base = CanBusBase(CP, None)
    main_bus = can_base.offset
    adas_bus = can_base.offset + 1
    cam_bus = can_base.offset + 2

    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], [], main_bus),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [], adas_bus),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], cam_bus),
    }
