from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.leapmotor.values import DBC, CarControllerParams

GearShifter = structs.CarState.GearShifter
GEARS = {0: GearShifter.park, 1: GearShifter.reverse, 2: GearShifter.neutral, 3: GearShifter.drive}


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.steering_angle_last = 0.0
    self.adas_lka: dict | None = None
    self.lka_received_seen = False
    self.driver_override_frames = 0
    self.driver_override = False
    self.powertrain_state: dict | None = None

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    # speed
    self.parse_wheel_speeds(ret,
      cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_FL'],
      cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_FR'],
      cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_RL'],
      cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_RR'],
    )
    ret.vEgoCluster = cp.vl['CLUSTER']['DISPLAY_SPEED'] * CV.KPH_TO_MS
    ret.standstill = ret.vEgoRaw < 0.01
    ret.yawRate = cp.vl['IMU']['YAW_RATE'] * CV.DEG_TO_RAD

    # pedals
    ret.gasPressed = cp.vl['VEHICLE_STATE']['ACCEL_PEDAL'] > 0
    ret.brakePressed = bool(cp.vl['VEHICLE_STATE']['BRAKE_PRESSED'])

    # steering: the EPS speed field has no sign, so take the sign from the angle change
    angle = cp.vl['EPS_STEERING']['STEER_ANGLE']
    sign = 1.0 if angle > self.steering_angle_last else (-1.0 if angle < self.steering_angle_last else 0.0)
    ret.steeringAngleDeg = angle
    ret.steeringRateDeg = sign * cp.vl['EPS_STEERING']['STEER_SPEED_ABS']
    self.steering_angle_last = angle
    ret.steeringTorque = cp.vl['EPS_STEERING']['STEER_TORQUE_DRIVER']
    ret.steeringTorqueEps = cp.vl['EPS_STEERING']['STEER_TORQUE_EPS']
    # the torque reading is coarse: during an LCC override it only reached 2-3, but the EPS flagged the override itself.
    # The EPS also sets that flag whenever the car is above about 121 km/h, with or without the driver (99% of frames
    # above 121 km/h on a stock drive, and openpilot's own drive at 125 km/h), so above 118 km/h only the torque counts.
    eps_override = cp.vl['EPS_STEERING']['DRIVER_OVERRIDE'] and ret.vEgoRaw < 118 * CV.KPH_TO_MS
    self.driver_override_frames = self.driver_override_frames + 1 if eps_override else 0
    torque_pressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    ret.steeringPressed = torque_pressed or self.driver_override_frames >= 2
    # When to let go of the wheel, like stock LCC: on the EPS's own flag. The torque reading follows the EPS's own effort
    # (about 1 per 14 of EPS torque with hands on the wheel), so it read 5-7 whenever the EPS pushed 65-95 in a curve:
    # letting go on it dropped the EPS to 0 and back every 0.7 s (41 times in 14 min). Above 118 km/h the flag is
    # always set, so only the torque counts there, where the EPS torque stays small.
    self.driver_override = self.driver_override_frames >= 2 or (torque_pressed and ret.vEgoRaw >= 118 * CV.KPH_TO_MS)
    # The EPS reports whether it receives ADAS_LKA (0x83): losing it after it was received means our steering frame is
    # missing or rejected (camera-unplug test). Not set before the camera's first frame after ignition.
    lka_received = bool(cp.vl['EPS_STEERING']['ADAS_LKA_RECEIVED'])
    self.lka_received_seen = self.lka_received_seen or lka_received
    ret.steerFaultTemporary = self.lka_received_seen and not lka_received

    # gear
    ret.gearShifter = GEARS.get(int(cp.vl['VEHICLE_STATE']['GEAR']), GearShifter.unknown)

    # body
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl['BODY_CONTROL']['BLINKER_LEFT'] == 1,
                                                                      cp.vl['BODY_CONTROL']['BLINKER_RIGHT'] == 1)
    ret.doorOpen = bool(cp.vl['BODY_CONTROL']['DRIVER_DOOR_OPEN'])
    ret.seatbeltUnlatched = bool(cp.vl['SEATBELT']['DRIVER_SEATBELT_UNLATCHED'])

    # blind-spot radars: silent when stopped, so they are not timeout-checked (see get_can_parsers)
    ret.leftBlindspot = bool(cp.vl['BSD_LEFT']['OBJECT_DETECTED'])
    ret.rightBlindspot = bool(cp.vl['BSD_RIGHT']['OBJECT_DETECTED'])

    # stock ACC / LCC, read from the camera side
    ret.cruiseState.enabled = bool(cp.vl['POWERTRAIN_STATE']['ASSIST_ENGAGED'])
    # latest VCU frame, copied by the controller to cancel ACC; None until received
    self.powertrain_state = dict(cp.vl['POWERTRAIN_STATE']) if cp.ts_nanos['POWERTRAIN_STATE']['COUNTER'] > 0 else None
    # ACC_STATE: 1 not ready (Park, brake, accelerator override), 2 ready, 3 engaged. Engaged also counts as
    # available so that an accelerator override (state 1 while engaged) does not raise wrongCarMode
    ret.cruiseState.available = ret.cruiseState.enabled or int(cp_cam.vl['ADAS_LKA']['ACC_STATE']) in (2, 3)
    ret.cruiseState.speed = cp_cam.vl['ADAS_LKA']['ACC_SET_SPEED'] * CV.KPH_TO_MS
    ret.cruiseState.speedCluster = ret.cruiseState.speed
    ret.cruiseState.standstill = ret.cruiseState.enabled and ret.standstill
    # latest camera frame, rebuilt by the controller; None until the camera's 0x83 has been received
    self.adas_lka = dict(cp_cam.vl['ADAS_LKA']) if cp_cam.ts_nanos['ADAS_LKA']['COUNTER'] > 0 else None

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [("BSD_LEFT", float('nan')), ("BSD_RIGHT", float('nan'))], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
