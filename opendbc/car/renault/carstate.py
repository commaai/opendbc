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

    # ADAS_ENGAGED is a multi-state enum and currently only used as availability.
    # Keep enabled False until an ACC-active state/bit is identified.
    adas_engaged = cp_body.vl["EPS_STATE_4B5"]["ADAS_ENGAGED"]
    ret.cruiseState.available = adas_engaged != 0
    ret.cruiseState.enabled = False
    ret.cruiseState.standstill = False
    # Raw 200 km/h is the UNSET sentinel when cruise has no target
    set_speed_kph = cp.vl["VEHICLE_SPEED_3FA"]["CRUISE_SET_SPEED"]
    ret.cruiseState.speed = 0.0 if set_speed_kph >= 200 else set_speed_kph * CV.KPH_TO_MS

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
    ]
    body_messages = [
      ("STEERING_SENSOR_112", 100),
      ("PEDAL_AND_GEAR_224", 50),
      ("EPS_STATE_4B5", 10),
    ]
    return {
      Bus.pt:   CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages,   0),
      Bus.body: CANParser(DBC[CP.carFingerprint][Bus.pt], body_messages, 1),
      Bus.cam:  CANParser(DBC[CP.carFingerprint][Bus.pt], [],            2),
    }
