from opendbc.car import DT_CTRL
from opendbc.car.perodua.peroduacan import create_can_steer_command, \
                                             perodua_create_accel_command, \
                                             perodua_create_brake_command, perodua_create_hud, \
                                             perodua_buttons
from opendbc.car.perodua.values import ACC_CAR, DBC, BRAKE_SCALE, GAS_SCALE
from opendbc.can.packer import CANPacker
from opendbc.car.common.numpy_fast import clip, interp

from bisect import bisect_left

BRAKE_THRESHOLD = 0.01
BRAKE_MAG = [BRAKE_THRESHOLD,.32,.46,.61,.76,.90,1.06,1.21,1.35,1.51,4.0]
PUMP_VALS = [0, .1, .2, .3, .4, .5, .6, .7, .8, .9, 1.0]
PUMP_RESET_INTERVAL = 1.5
PUMP_RESET_DURATION = 0.1

class BrakingStatus:
  STANDSTILL_INIT = 0
  BRAKE_HOLD = 1
  PUMP_RESET = 2

def apply_perodua_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, blinkerOn, LIMITS):

  # limits due to driver torque and lane change
  reduced_torque_mult = 10 if blinkerOn else 1.5
  driver_max_torque = 255 + driver_torque * reduced_torque_mult
  driver_min_torque = -255 - driver_torque * reduced_torque_mult
  max_steer_allowed = max(min(255, driver_max_torque), 0)
  min_steer_allowed = min(max(-255, driver_min_torque), 0)
  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)

  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque, apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque)))

def apply_acttr_steer_torque_limits(apply_torque, apply_torque_last, LIMITS):
  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque, apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque)))

def compute_gb(accel):
  gb = float(accel) / 4.0
  return clip(gb, 0.0, 1.0), clip(-gb, 0.0, 1.0)

# reset pump every PUMP_RESET_INTERVAL seconds for. Reset to zero for PUMP_RESET_DURATION
def standstill_brake(min_accel, ts_last, ts_now, prev_status):
  brake = min_accel
  status = prev_status

  dt = ts_now - ts_last
  if prev_status == BrakingStatus.PUMP_RESET and dt > PUMP_RESET_DURATION:
    status = BrakingStatus.BRAKE_HOLD
    ts_last = ts_now

  if prev_status == BrakingStatus.BRAKE_HOLD and dt > PUMP_RESET_INTERVAL:
    status = BrakingStatus.PUMP_RESET
    ts_last = ts_now

  if prev_status == BrakingStatus.STANDSTILL_INIT and dt > PUMP_RESET_INTERVAL:
    status = BrakingStatus.PUMP_RESET
    ts_last = ts_now

  if status == BrakingStatus.PUMP_RESET:
    brake = 0

  return brake, status, ts_last

def psd_brake(apply_brake, last_pump):
  # reversed engineered from Ativa stock braking
  # this is necessary for noiseless pump braking
  pump = PUMP_VALS[bisect_left(BRAKE_MAG, apply_brake)]

  # make sure the pump value decrease and increases within 0.1
  # to prevent brake bleeding.
  if abs(pump - last_pump) > 0.1:
    pump = last_pump + clip(pump - last_pump, -0.1, 0.1)
  last_pump = pump

  if apply_brake >= BRAKE_THRESHOLD:
    brake_req = 1
  else:
    brake_req = 0

  return pump, brake_req, last_pump

class CarControllerParams:
  def __init__(self, CP):

    self.STEER_BP = CP.lateralParams.torqueBP
    self.STEER_LIM_TORQ = CP.lateralParams.torqueV

    # for torque limit calculation
    self.STEER_DELTA_UP = 10
    self.STEER_DELTA_DOWN = 30

    self.STEER_REDUCE_FACTOR = 1000                 # how much to divide the steer when reducing fighting torque
    self.GAS_MAX = 2600                             # KommuActuator dac gas value
    self.GAS_STEP = 2                               # how often we update the longitudinal cmd
    self.BRAKE_ALERT_PERCENT = 60                   # percentage of brake to sound stock AEB alert
    self.ADAS_STEP = 5                              # 100/5 approx ASA frequency of 20 hz

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.steer_rate_limited = False
    self.steering_direction = False
    self.brake_pressed = False
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.brake = 0
    self.brake_scale = BRAKE_SCALE[CP.carFingerprint]
    self.gas_scale = GAS_SCALE[CP.carFingerprint]

    self.need_clear_engine = True

    self.last_pump = 0

    # standstill globals
    self.prev_ts = 0.
    self.standstill_status = BrakingStatus.STANDSTILL_INIT
    self.min_standstill_accel = 0

    self.stockLdw = False
    self.using_stock_acc = False
    self.force_use_stock_acc = False

  def update(self, enabled, CS, frame, actuators, lead_visible, rlane_visible, llane_visible, pcm_cancel, ldw, laneActive):
    can_sends = []

    # steer
    steer_max_interp = interp(CS.out.vEgo, self.params.STEER_BP, self.params.STEER_LIM_TORQ)
    new_steer = int(round(actuators.steer * steer_max_interp))
    apply_steer = apply_acttr_steer_torque_limits(new_steer, self.last_steer, self.params)

    if CS.CP.carFingerprint in ACC_CAR:
      isBlinkerOn = CS.out.leftBlinker != CS.out.rightBlinker
      apply_steer = apply_perodua_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, isBlinkerOn, self.params)

    self.steer_rate_limited = (new_steer != apply_steer) and (apply_steer != 0)
    self.steer_rate_limited &= not CS.out.steeringPressed

    # gas, brake
    apply_gas, apply_brake = compute_gb(actuators.accel)
    apply_brake *= self.brake_scale
    apply_brake = clip(apply_brake, 0., 1.56)

    if self.using_stock_acc:
      apply_brake = max(CS.stock_brake_mag * 0.85, apply_brake)

    if CS.out.gasPressed:
      apply_brake = 0
    apply_gas *= self.gas_scale

    ts = frame * DT_CTRL

    if self.need_clear_engine or frame < 1000:
      can_sends.append(self.packer.make_can_msg(2015, b'\x01\x04\x00\x00\x00\x00\x00\x00', 0))

    # CAN controlled lateral
    if (frame % 2) == 0:

      # allow stock LDP passthrough
      self.stockLdw = CS.out.stockAdas.laneDepartureHUD
      if self.stockLdw:
          apply_steer = -CS.out.stockAdas.ldpSteerV

      steer_req = (enabled or self.stockLdw) and CS.lkas_latch
      can_sends.append(create_can_steer_command(self.packer, apply_steer, steer_req and laneActive, (frame/2) % 16))

    # CAN controlled longitudinal
    if (frame % 5) == 0 and CS.CP.openpilotLongitudinalControl:

      # check if need to revert to stock acc
      if enabled and CS.out.vEgo > 10: # 36kmh
        if CS.stock_acc_engaged and self.force_use_stock_acc:
          self.using_stock_acc = True
      else:
        if enabled:
          # spam engage until stock ACC engages
          can_sends.append(perodua_buttons(self.packer, 0, 1, (frame/5) % 16))

      # check if need to revert to openpilot acc
      if CS.out.vEgo < 8.3: # 30kmh
        self.using_stock_acc = False

      # set stock acc follow speed
      if enabled and self.using_stock_acc:
        if CS.out.cruiseState.speedCluster - (CS.stock_acc_set_speed // 3.6) > 0.3:
          can_sends.append(perodua_buttons(self.packer, 0, 1, (frame/5) % 16))
        if (CS.stock_acc_set_speed // 3.6) - CS.out.cruiseState.speedCluster > 0.3:
          can_sends.append(perodua_buttons(self.packer, 1, 0, (frame/5) % 16))

      # standstill logic
      if enabled and apply_brake > 0 and CS.out.standstill:
        if self.standstill_status == BrakingStatus.STANDSTILL_INIT:
          self.min_standstill_accel = apply_brake + 0.2
        apply_brake, self.standstill_status, self.prev_ts = standstill_brake(self.min_standstill_accel, self.prev_ts, ts, self.standstill_status)
      else:
        self.standstill_status = BrakingStatus.STANDSTILL_INIT
        self.prev_ts = ts

      # PSD brake logic
      pump, brake_req, self.last_pump = psd_brake(apply_brake, self.last_pump)

      # the accel is too high at lower speed below 5kmh
      boost = interp(CS.out.vEgo, [0.2, 0.5], [0., 1.0])
      des_speed = actuators.speed + min((actuators.accel * boost), 1.0)

      if self.using_stock_acc:
        combined_cmd = (CS.stock_acc_cmd / 3.6 + des_speed)/2 if CS.stock_acc_cmd > 0 else des_speed
        des_speed = min(combined_cmd, des_speed)
        can_sends.append(perodua_create_accel_command(self.packer, CS.out.cruiseState.speedCluster,
                                                      CS.out.cruiseState.available, enabled, lead_visible,
                                                      des_speed, apply_brake, pump, CS.out.cruiseState.setDistance))
      else:
        can_sends.append(perodua_create_accel_command(self.packer, CS.out.cruiseState.speedCluster,
                                                      CS.out.cruiseState.available, enabled, lead_visible,
                                                      des_speed, apply_brake, pump, CS.out.cruiseState.setDistance))

      # Let stock AEB kick in only when system not engaged
      aeb = not enabled and CS.out.stockAdas.aebV
      can_sends.append(perodua_create_brake_command(self.packer, enabled, brake_req, pump, apply_brake, aeb, (frame/5) % 8))
      can_sends.append(perodua_create_hud(
        self.packer, CS.out.cruiseState.available and CS.lkas_latch,
        enabled, llane_visible, rlane_visible, self.stockLdw, CS.out.stockFcw,
        CS.out.stockAeb, CS.out.stockAdas.frontDepartureHUD, CS.stock_lkc_off,
        CS.stock_fcw_off))

    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / steer_max_interp

    return new_actuators, can_sends
