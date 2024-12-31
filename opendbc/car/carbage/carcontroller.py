import math
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, rate_limit, apply_meas_steer_torque_limits, common_fault_avoidance, \
                        DT_CTRL, structs
from opendbc.car.common.pid import PIDController
from opendbc.car.common.numpy_fast import clip, interp
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.carbage.values import CarControllerParams

LongCtrlState = structs.CarControl.Actuators.LongControlState

# LKA limits
# EPS faults if you apply torque while the steering rate is above 100 deg/s for too long
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 18  # tx control frames needed before torque can be cut
MAX_USER_TORQUE = 500

# ACCEL limits
ACCEL_WINDUP_LIMIT = 4.0 * DT_CTRL  # m/s^2 / frame
ACCEL_WINDDOWN_LIMIT = -4.0 * DT_CTRL  # m/s^2 / frame

# Servo limits
SERVO_MIN = 0
SERVO_MAX = 60

# Brake limits
BRAKE_MIN = 0
BRAKE_MAX = 6

def toyota_checksum(addr, dat):
  ret = len(dat)
  ret += (addr & 0xFF) + ((addr >> 8) & 0xFF)
  ret += sum(dat)
  return ret & 0xFF

def tesla_checksum(msg_id, dat):
  ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
  ret += sum(dat)
  return ret & 0xFF

def create_steer_command(packer, steer, steer_req, counter):
  values = {
    "STEER_REQUEST": steer_req,
    "STEER_TORQUE_CMD": steer,
    "SET_ME_1": 1,
    "COUNTER": (counter % 64),
  }
  data = packer.make_can_msg("STEERING_LKA", 0, values)[1]
  values["CHECKSUM"] = toyota_checksum(0x2e4, data)
  return packer.make_can_msg("STEERING_LKA", 0, values)

def create_servo_command(packer, servo_val, counter):
  assert 0 <= servo_val <= 100, "servo_val out of range"
  values = {
    "SERVO": servo_val,
    "COUNTER": (counter % 16),
  }
  data = packer.make_can_msg("ServoControl", 0, values)[1]
  values["CHECKSUM"] = tesla_checksum(0x200, data)
  return packer.make_can_msg("ServoControl", 0, values)

def create_brake_command(packer, brake_mm, counter):
  values = {
    "BRAKE": brake_mm,
    "COUNTER": (counter % 16),
  }
  data = packer.make_can_msg("BrakeControl", 0, values)[1]
  values["CHECKSUM"] = tesla_checksum(0x201, data)
  return packer.make_can_msg("BrakeControl", 0, values)

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(self.CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.steer_rate_counter = 0
    self.last_steer = 0

    kiBP = [0.]
    kdBP = [0.]
    kdV = [0.]
    kiBP = [0., 5., 35.]
    kiV = [3.6, 2.4, 1.5]

    self.long_pid = PIDController(0.0, (kiBP, kiV), k_f=1.0, k_d=(kdBP, kdV),
                       pos_limit=self.params.ACCEL_MAX, neg_limit=self.params.ACCEL_MIN,
                       rate=1 / DT_CTRL)
    self.aego = FirstOrderFilter(0.0, 0.25, DT_CTRL)
    self.pitch = FirstOrderFilter(0, 0.5, DT_CTRL)
    self.prev_accel = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    if len(CC.orientationNED) == 3:
      self.pitch.update(CC.orientationNED[1])

    # Lateral
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_meas_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params)
    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, lat_active,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)
    self.last_steer = apply_steer
    can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req, self.frame))

    # Longitudinal
    pcm_accel_cmd = actuators.accel
    if CC.longActive:
      pcm_accel_cmd = rate_limit(pcm_accel_cmd, self.prev_accel, ACCEL_WINDDOWN_LIMIT, ACCEL_WINDUP_LIMIT)
    self.prev_accel = pcm_accel_cmd

    # wind down integral when approaching target for step changes and smooth ramps to reduce overshoot
    prev_aego = self.aego.x
    self.aego.update(CS.out.aEgo)
    j_ego = (self.aego.x - prev_aego) / (DT_CTRL * 3)
    a_ego_future = CS.out.aEgo + j_ego * 0.5

    if actuators.longControlState == LongCtrlState.pid:
      error = pcm_accel_cmd - CS.out.aEgo
      self.error_rate.update((error - self.prev_error) / (DT_CTRL * 3))
      self.prev_error = error

      error_future = pcm_accel_cmd - a_ego_future
      pcm_accel_cmd = self.long_pid.update(error_future, error_rate=self.error_rate.x,
                                            speed=CS.out.vEgo,
                                            feedforward=pcm_accel_cmd)
    else:
      self.long_pid.reset()
      self.error_rate.x = 0.0
      self.prev_error = 0.0

    pcm_accel_cmd = clip(pcm_accel_cmd, self.params.ACCEL_MIN, self.params.ACCEL_MAX)

    servo_val = int(interp(max(0, pcm_accel_cmd), [0, self.params.ACCEL_MAX], [SERVO_MIN, SERVO_MAX]))
    brake_val = int(interp(min(pcm_accel_cmd, 0), [self.params.ACCEL_MIN, 0], [BRAKE_MAX, BRAKE_MIN]))

    can_sends.append(create_servo_command(self.packer, servo_val, self.frame))
    can_sends.append(create_brake_command(self.packer, brake_val, self.frame))

    new_actuators = actuators.as_builder()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.accel = pcm_accel_cmd

    self.frame += 1
    return new_actuators, can_sends
