import numpy as np
from opendbc.car import structs
from opendbc.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.car.gwm.values import DBC, CANBUS, CarControllerParams


GearShifter = structs.CarState.GearShifter


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.frame = 0
    self.CCP = CarControllerParams(CP)
    self.button_states = {button.event_type: False for button in self.CCP.BUTTONS}

  def create_button_events(self, cp, buttons):
    button_events = []
    for button in buttons:
      state = cp.vl[button.can_addr][button.can_msg] in button.values
      if self.button_states[button.event_type] != state:
        event = structs.CarState.ButtonEvent()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)
      self.button_states[button.event_type] = state
    return button_events

  def update(self, cp, cam_cp, _, __, loopback_cp) -> structs.CarState:
    ret = structs.CarState()

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["FRONT_LEFT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["FRONT_RIGHT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["REAR_LEFT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["REAR_RIGHT_WHEEL_SPEED"],
    )
    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = abs(ret.vEgoRaw) < 1e-3

    ret.steeringAngleDeg = cp.vl["STEER_AND_AP_STALK"]["STEERING_ANGLE"] * (-1 if cp.vl["STEER_AND_AP_STALK"]["STEERING_DIRECTION"] else 1)
    ret.steeringRateDeg = 0 # TODO
    ret.steeringTorque = cp.vl["STEER_AND_AP_STALK"]["STEERING_TORQUE"] * (-1 if cp.vl["STEER_AND_AP_STALK"]["STEERING_DIRECTION"] else 1) * 73
    ret.steeringPressed = abs(ret.steeringTorque) > 10
    # ret.yawRate = NOT ABSOLUTE NECESSARY
    # ret.steerFaultTemporary, ret.steerFaultPermanent = CRITICAL SAFETY TODO, CRITICAL SAFETY TODO

    ret.gas = cp.vl["CAR_OVERALL_SIGNALS2"]["GAS_POSITION"]
    ret.gasPressed = ret.gas > 0
    ret.brake = cp.vl["BRAKE"]["BRAKE_PRESSURE"]
    ret.brakePressed = cp.vl["BRAKE"]["BRAKE_PRESSURE"] > 0
    ret.parkingBrake = cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"] == 0

    # begin toyota brakePressed TODO clean-after-port
    # ret.brakePressed = cp.vl["COROLLA_BRAKE_MODULE"]["BRAKE_PRESSED"] != 0
    # end TODO clean-after-port

    ret.gearShifter = GearShifter.drive if int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"]) == 1 else \
                      GearShifter.neutral if int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"]) == 2 else \
                      GearShifter.reverse if int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"]) == 3 else \
                      GearShifter.park

    ret.doorOpen = any([cp.vl["DOOR_DRIVER"]["DOOR_REAR_RIGHT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_FRONT_RIGHT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_REAR_LEFT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_DRIVER_OPEN"]])
    ret.seatbeltUnlatched = cp.vl["SEATBELT"]["SEAT_BELT_DRIVER_STATE"] != 1

    ret.cruiseState.available = True #CRITICAL SAFETY TODO
    # ret.cruiseState.enabled = TODO
    # ret.cruiseState.speed = TODO

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["LIGHTS"]["LEFT_TURN_SIGNAL"],
                                                                      cp.vl["LIGHTS"]["RIGHT_TURN_SIGNAL"])
    ret.buttonEvents = self.create_button_events(cp, self.CCP.BUTTONS)
    # ret.espDisabled = TODO

    self.frame += 1
    return ret


  @staticmethod
  def get_can_parser(CP):
    messages = [
      # COROLLA:
      # ("COROLLA_BRAKE_MODULE", 40),

      # HAVAL:
      ("BRAKE", 50),
      ("CAR_OVERALL_SIGNALS", 50),
      ("CAR_OVERALL_SIGNALS2", 100),
      ("DOOR_DRIVER", 20),
      ("LIGHTS", 20),
      ("SEATBELT", 2),
      ("STEER_AND_AP_STALK", 100),
      ("WHEEL_SPEEDS", 50),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.pt)


  @staticmethod
  def get_cam_can_parser(CP):
    messages = []

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.cam)