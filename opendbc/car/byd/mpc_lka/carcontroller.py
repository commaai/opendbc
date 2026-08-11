import numpy as np

from opendbc.can.packer import CANPacker
from opendbc.car.byd.mpc_lka.bydcan import create_fake_318, create_steering_control
from opendbc.car.byd.values import DBC, MpcLkaCarControllerParams
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_driver_steer_torque_limits


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])
    self.params = MpcLkaCarControllerParams(CP)

    self.frame = 0
    self.last_steer_frame = 0
    self.apply_torque_last = 0

    self.mpc_lkas_counter = 0
    self.eps_fake318_counter = 0

    self.lkas_req_prepare = 0
    self.lkas_active = 0
    self.lat_safeoff = 0
    self.steer_softstart_limit = 0
    self.steer_rate_lim_active = False
    self.steer_rate_lim = 1.0
    self.first_start = True

  def update(self, CC, CS, now_nanos):
    del now_nanos
    can_sends = []

    if (self.frame - self.last_steer_frame) >= self.params.STEER_STEP:
      if self.first_start:
        self.mpc_lkas_counter = int(CS.acc_mpc_state_counter + 1) & 0xF
        self.eps_fake318_counter = int(CS.eps_state_counter + 1) & 0xF
        self.first_start = False

      apply_torque = 0

      if CC.latActive:
        if self.lkas_active:
          steer_desire = CC.actuators.torque

          if self.params.USE_STEERING_SPEED_LIMITER:
            rate_limit = np.interp(CS.out.aEgo, [8.3, 27.8], [132, 64])
            delta_rate = CS.steering_rate_deg_abs - rate_limit

            if delta_rate < 0:
              self.steer_rate_lim -= 0.005 * delta_rate
              if delta_rate < -0.05:
                self.steer_rate_lim_active = False
              if self.steer_rate_lim > 1.0:
                self.steer_rate_lim = 1.0
                self.steer_rate_lim_active = False
            else:
              if self.steer_rate_lim_active:
                self.steer_rate_lim -= 0.005 * delta_rate
              else:
                self.steer_rate_lim = steer_desire
                self.steer_rate_lim_active = True
              if self.steer_rate_lim < 0:
                self.steer_rate_lim = 0

            new_steer_pu = np.clip(steer_desire, -self.steer_rate_lim, self.steer_rate_lim)
          else:
            new_steer_pu = steer_desire

          new_steer = int(round(new_steer_pu * self.params.STEER_MAX))

          if self.steer_softstart_limit < self.params.STEER_MAX:
            self.steer_softstart_limit += self.params.STEER_SOFTSTART_STEP
            new_steer = int(np.clip(new_steer, -self.steer_softstart_limit, self.steer_softstart_limit))

          apply_torque = apply_driver_steer_torque_limits(
            new_steer, self.apply_torque_last, CS.out.steeringTorque, self.params)

        elif CS.lkas_prepared:
          self.lkas_active = 1
          self.steer_rate_lim_active = False
          self.steer_rate_lim = 1.0
          self.lkas_req_prepare = 0
          self.steer_softstart_limit = 0
          self.lat_safeoff = 1
        else:
          self.lkas_req_prepare = 1

      elif self.lat_safeoff:
        if self.apply_torque_last == 0:
          self.lat_safeoff = 0
        apply_torque = apply_driver_steer_torque_limits(
          0, self.apply_torque_last, CS.out.steeringTorque, self.params)
      else:
        self.lkas_req_prepare = 0
        self.steer_rate_lim_active = False
        self.steer_rate_lim = 1.0
        self.lkas_active = 0
        self.steer_softstart_limit = 0

      self.apply_torque_last = apply_torque
      self.mpc_lkas_counter = int(self.mpc_lkas_counter + 1) & 0xF
      self.eps_fake318_counter = int(self.eps_fake318_counter + 1) & 0xF
      self.last_steer_frame = self.frame

      can_sends.append(create_steering_control(
        self.packer,
        CS.cam_lkas,
        self.apply_torque_last,
        self.lkas_req_prepare,
        self.lkas_active,
        CC.hudControl,
        self.mpc_lkas_counter,
      ))

      can_sends.append(create_fake_318(
        self.packer,
        CS.esc_eps,
        CS.mpc_lkas_output,
        bool(CS.mpc_lkas_reqprepare),
        bool(CS.mpc_lkas_active),
        True,
        self.eps_fake318_counter,
      ))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.steeringAngleDeg = float(CS.out.steeringAngleDeg)

    self.frame += 1
    return new_actuators, can_sends
