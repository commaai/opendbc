from opendbc.car import apply_driver_steer_torque_limits, common_fault_avoidance, structs
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
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.frame = 0

    self.lkas_max_torque = 0
    self.apply_steer_last = 0

    # Add a variable to track the ramp value
    self.ramp_value = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    apply_steer = 0

    actuators = CC.actuators
    driving = CS.out.vEgo > 0

    # Ramp up/down logic
    if CC.latActive:
      self.ramp_value = min(self.ramp_value + 3.5, 100)  # Ramp up the torque factor
    else:
      self.ramp_value = max(self.ramp_value - 1, 0)    # Ramp down the torque factor

    # Steering torque logic (executed every STEER_STEP frames)
    # if (self.frame % CarControllerParams.STEER_STEP) == 0:
    # Calculate new steering torque
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)

    if not CC.latActive:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    # can_sends.append(psacan.create_lka_msg_only_chks(self.packer, self.CP, CS.original_lka_values))
    # Create LKA message with ramp_value
    can_sends.append(
        psacan.create_lka_msg(
            self.packer,
            self.CP,
            apply_steer,
            CS.out.steeringAngleDeg,
            self.frame,
            CC.latActive,
            self.lkas_max_torque,
            self.ramp_value,
            driving,
            CS.original_lka_values,
        )
    )

    # Cruise buttons (placeholder)
    # TODO: Implement cruise buttons message

    new_actuators = actuators.as_builder()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer

    self.frame += 1
    return new_actuators, can_sends
