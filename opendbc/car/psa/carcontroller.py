from opendbc.car import structs
from opendbc.can.packer import CANPacker
from opendbc.car.common.numpy_fast import clip
from opendbc.car import apply_std_steer_angle_limits, Bus
from opendbc.car.interfaces  import CarControllerBase
from opendbc.car.psa import psacan
from opendbc.car.psa.values import CarControllerParams

GearShifter = structs.CarState.GearShifter

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    self.CP = CP
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.frame = 0

    self.lkas_max_torque = 0
    self.apply_angle_last = 0
    # torque factor ramp
    self.ramp_value = 0

    # sync to send out frames
    self.last_counter = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    actuators = CC.actuators

    # Ramp up/down logic
    if CC.latActive:
      self.ramp_value = min(self.ramp_value + 3.5, 100)  # Ramp up the torque factor
    else:
      self.ramp_value = max(self.ramp_value - 1, 0)    # Ramp down the torque factor

    ### lateral control ###
    # Generate message with 100 Hz and send out synced with car messages at 20 Hz
    if CC.latActive:
      # windup slower
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CarControllerParams)

      # Max torque from driver before EPS will give up and not apply torque
      if not bool(CS.out.steeringPressed):
        self.lkas_max_torque = CarControllerParams.LKAS_MAX_TORQUE
      else:
        # Scale max torque based on how much torque the driver is applying to the wheel
        self.lkas_max_torque = max(
          # Scale max torque down to half LKAX_MAX_TORQUE as a minimum
          CarControllerParams.LKAS_MAX_TORQUE * 0.5,
          # Start scaling torque at STEER_THRESHOLD
          CarControllerParams.LKAS_MAX_TORQUE - 0.6 * max(0, abs(CS.out.steeringTorque) - CarControllerParams.STEER_THRESHOLD)
        )

    else:
      apply_angle = CS.out.steeringAngleDeg
      self.lkas_max_torque = 0

    apply_angle = clip(apply_angle, -CarControllerParams.STEER_MAX, CarControllerParams.STEER_MAX)

    # Sync to original message
    # If sent unsynced, steering still works but driving mode is set to normal and recuperation set to low.
    # Steer quality is also slightly better.
    # TODO: test spamming at 100 Hz
    if(self.last_counter != CS.original_lka_values['COUNTER']):
      can_sends.append(psacan.create_lka_msg(self.packer, self.CP, apply_angle, self.frame, CC.latActive, self.ramp_value))
      self.last_counter = CS.original_lka_values['COUNTER']

    self.apply_angle_last = apply_angle
    ### lateral control end ###

    ### cruise buttons ###
    # TODO: find cruise buttons msg

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.steer = self.lkas_max_torque

    self.frame += 1
    return new_actuators, can_sends
