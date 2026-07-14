from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.renault.values import DBC, GEAR_MAP

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

    # 0x57A byte 5 alone cannot distinguish Active Assist from ACC/limiter:
    # state 16 occurs in all longitudinal-only modes and during steering
    # override. Byte 6 carries the mode context needed for lateral availability.
    adas_mode_state = int(cp_body.vl["ADAS_STATE_57A"]["ADAS_MODE_STATE"])
    ret.cruiseState.available = adas_mode_state in (0x02, 0x41, 0x91)
    ret.cruiseState.enabled = adas_mode_state in (0x02, 0x91)
    ret.cruiseState.standstill = False
    # The driver-selected setpoint is not present on the two intercepted CAN
    # pairs. 0x517 is the camera-detected road limit and 0x5EF stayed at 16
    # while the displayed setpoint moved between 60 and 123 km/h in the
    # 2026-07-11 capture. Reporting zero is safer than publishing either as the
    # driver's set speed.
    ret.cruiseState.speed = 0.0

    # No steering-fault signal has been isolated. LKAS_STATE 3 was previously
    # called unavailable, but today's drive only exercised states 1 and 2 and
    # does not justify exposing state 3 as a CarState fault.

    # Door and belt candidates on 0x588 did not remain asserted through the
    # controlled 2026-07-11 door/belt states and also toggled together. Leave
    # the CarState fields at their safe defaults until isolated signals exist.

    return ret

  @staticmethod
  def get_can_parsers(CP):
    pt_messages = [
      ("ACC_PEDAL_8C", 100),
      ("GEAR_12F", 100),
      ("EPS_17C", 100),
      ("STEERING_CTRL_1AB", 100),
      ("WHEEL_SPEEDS_226", 50),
      ("TURN_SIGNAL_RIGHT_58B", 10),
    ]
    body_messages = [
      ("STEERING_SENSOR_112", 100),
      ("PEDAL_AND_GEAR_224", 50),
      ("ADAS_STATE_57A", 2),
    ]
    return {
      Bus.pt:   CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages,   0),
      Bus.body: CANParser(DBC[CP.carFingerprint][Bus.pt], body_messages, 1),
      Bus.cam:  CANParser(DBC[CP.carFingerprint][Bus.pt], [],            2),
    }
