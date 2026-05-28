from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.renault.values import DBC, GEAR_MAP
from opendbc.car.common.conversions import Conversions as CV

GearShifter = structs.CarState.GearShifter


class CarState(CarStateBase):
  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_body = can_parsers[Bus.body]
    ret = structs.CarState()

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS_226"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS_226"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS_226"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS_226"]["WHEEL_SPEED_RR"],
    )
    ret.standstill = ret.vEgoRaw < 0.01

    ret.gasPressed = cp.vl["ACC_PEDAL_8C"]["ACCEL_PEDAL"] > 0.1
    ret.brake = cp_body.vl["PEDAL_AND_GEAR_224"]["BRAKE_PRESSURE"] / 255.0
    ret.brakePressed = cp_body.vl["STEERING_SENSOR_112"]["BRAKE_PRESSED"] == 1

    ret.steeringAngleDeg = cp.vl["STEERING_CTRL_1AB"]["STEER_ANGLE"]
    ret.steeringRateDeg = cp.vl["STEERING_CTRL_1AB"]["STEER_RATE"]
    ret.steeringTorque = cp.vl["EPS_17C"]["DRIVER_TORQUE"]
    ret.steeringTorqueEps = cp.vl["STEERING_CTRL_1AB"]["LKAS_TORQUE_CMD"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 150, 5)

    if cp.vl["GEAR_12F"]["PARK_ACTIVE"] == 1:
      ret.gearShifter = GearShifter.park
    else:
      ret.gearShifter = GEAR_MAP.get(int(cp.vl["GEAR_12F"]["GEAR_STATE"]), GearShifter.unknown)

    # R5 splits turn signals across different messages/buses: left is on 0x112, right on 0x58B
    ret.leftBlinker = cp_body.vl["STEERING_SENSOR_112"]["TURN_SIGNAL_LEFT"] == 1
    ret.rightBlinker = cp.vl["TURN_SIGNAL_RIGHT_58B"]["TURN_SIGNAL_RIGHT"] == 1

    # 0x57A is the clean ADAS state source. State 0 is active controlling;
    # state 8 is armed/standby; state 16 is driver override; state 41 is fault.
    adas_state = cp_body.vl["ADAS_STATE_57A"]["ADAS_STATE"]
    set_speed_kph = cp.vl["ACC_SETPOINT_5EF"]["SET_SPEED"]
    active_target_kph = cp_body.vl["ADAS_TARGET_517"]["ADAS_ACTIVE_TARGET"]
    has_target = 0 < active_target_kph < 250
    ret.cruiseState.available = adas_state in (0, 8, 16) and (set_speed_kph > 0 or has_target)
    ret.cruiseState.enabled = adas_state == 0 and set_speed_kph > 0
    ret.cruiseState.standstill = False
    # Driver-selected ACC/limiter setpoint; 0x517 tracks the active sign-adapted
    # target and is kept in the DBC, but isn't the driver's set speed.
    ret.cruiseState.speed = set_speed_kph * CV.KPH_TO_MS if set_speed_kph > 0 else 0.0

    ret.steerFaultTemporary = cp.vl["STEERING_CTRL_1AB"]["LKAS_STATE"] == 3

    ret.doorOpen = cp.vl["BELTS_DOORS_588"]["DOOR_DRIVER_OPEN_CANDIDATE"] == 1
    ret.seatbeltUnlatched = cp.vl["BELTS_DOORS_588"]["BELT_UNBUCKLED_CANDIDATE"] == 1

    return ret

  @staticmethod
  def get_can_parsers(CP):
    pt_messages = [
      ("ACC_PEDAL_8C", 100),
      ("GEAR_12F", 100),
      ("EPS_17C", 100),
      ("STEERING_CTRL_1AB", 100),
      ("WHEEL_SPEEDS_226", 50),
      ("VEHICLE_SPEED_3FA", 10),
      ("TURN_SIGNAL_RIGHT_58B", 10),
      ("BELTS_DOORS_588", 10),
      ("ACC_SETPOINT_5EF", 1),
    ]
    body_messages = [
      ("STEERING_SENSOR_112", 100),
      ("PEDAL_AND_GEAR_224", 50),
      ("EPS_STATE_4B5", 10),
      ("ADAS_TARGET_517", 2),
      ("ADAS_STATE_57A", 2),
    ]
    return {
      Bus.pt:   CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages,   0),
      Bus.body: CANParser(DBC[CP.carFingerprint][Bus.pt], body_messages, 1),
      Bus.cam:  CANParser(DBC[CP.carFingerprint][Bus.pt], [],            2),
    }
