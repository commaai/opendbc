import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_meas_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.gwm import gwmcan
from opendbc.car.gwm.values import CarControllerParams

LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(self.CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.apply_torque_last = 0
    self.accel = 0.0
    self.braking = False

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < self.params.STEER_DRIVER_ALLOWANCE

    if CC.cruiseControl.cancel:
      can_sends.append(gwmcan.create_buttons_command(self.packer, CS.stalk_stock_values, cancel=True))

    if self.frame % self.params.STEER_STEP == 0:
      new_torque = int(round(actuators.torque * self.params.STEER_MAX))
      apply_torque = apply_meas_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorqueEps, self.params)
      if not lat_active:
        apply_torque = 0
      self.apply_torque_last = apply_torque
      can_sends.append(gwmcan.create_steer_command(self.packer, CS.steer_stock_values, apply_torque, lat_active))

      # the camera expects driver torque while it thinks it's steering, report at least twice our command
      driver_torque = float(np.clip(apply_torque * 2, -self.params.STEER_MAX, self.params.STEER_MAX))
      if abs(CS.out.steeringTorque) > abs(driver_torque):
        driver_torque = CS.out.steeringTorque
      can_sends.append(gwmcan.create_eps_feedback(self.packer, CS.eps_stock_values, driver_torque))

      if self.CP.openpilotLongitudinalControl:
        self.accel = float(np.clip(actuators.accel, self.params.ACCEL_MIN, self.params.ACCEL_MAX))
        if self.accel < self.params.BRAKE_ENTER_ACCEL:
          self.braking = True
        elif self.accel > self.params.BRAKE_EXIT_ACCEL or not CC.longActive:
          self.braking = False

        gas = max(self.accel * self.params.GAS_PER_ACCEL, self.params.GAS_MIN)
        brake = self.params.BRAKE_ZERO + max(-self.accel, 0.) * self.params.BRAKE_PER_ACCEL
        stopping = actuators.longControlState == LongCtrlState.stopping
        can_sends.append(gwmcan.create_longitudinal_command(self.packer, CS.acc_stock_values, gas, brake, self.braking,
                                                            CC.longActive, stopping))

    if self.frame % 5 == 0:
      can_sends.append(gwmcan.create_hud_command(self.packer, CS.hud_stock_values, CC.latActive))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends
