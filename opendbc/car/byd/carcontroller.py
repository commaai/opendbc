import numpy as np

from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.byd import bydcan
from opendbc.car.byd.carstate import STEER_TEMPLATE_DEFAULT
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CP = CP
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = CarControllerParams(CP)

    self.apply_angle_last = 0.0
    self.acc_idx = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel

    can_sends = []

    # === STEERING ===
    # The EPS is a position servo: STEER_ANGLE is an absolute wheel angle target.
    # The command must therefore stay anchored to the measured angle at all times —
    # a limiter that only tracks its own previous output can ratchet away from the
    # wheel, saturate at the clamp and get every frame rejected by panda.
    if self.frame % self.params.STEER_STEP == 0:
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last,
                                                 CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                 CC.latActive, self.params.ANGLE_LIMITS)

      # Hand control back to the driver rather than fighting them. DRIVER_EPS_TORQUE is
      # an unsigned magnitude from the column sensor, so this is a pure override test.
      if CS.out.steeringTorque > self.params.STEER_DRIVER_ALLOWANCE:
        apply_angle = CS.out.steeringAngleDeg

      # Windup guard: never let the command drift outside a fixed window around the
      # measured angle. Makes the saturation failure mode structurally impossible.
      apply_angle = float(np.clip(apply_angle,
                                  CS.out.steeringAngleDeg - self.params.MAX_ANGLE_ERROR,
                                  CS.out.steeringAngleDeg + self.params.MAX_ANGLE_ERROR))

      self.apply_angle_last = apply_angle

      # Transmit only while actually steering. Whenever we go quiet, panda stops
      # blocking the camera's command after ~150ms and the stock LKAS takes the wheel
      # back — so there is never a window with nobody driving the EPS. It also lets the
      # camera keep refreshing the frame template we copy.
      if CC.latActive:
        # Everything in the frame other than the angle is held at the constant the
        # camera itself sends while steering (bytes 0-2 = 2b 55 eb, at every speed).
        # It must not be varied: walking these fields across their range is what made
        # the car drop its whole ADAS mid-drive. Use the camera's own latched value
        # when we have seen it steer, and the captured constant otherwise — a blocked
        # lens means the camera never steers and never gives us one.
        template = CS.steer_template or STEER_TEMPLATE_DEFAULT

        can_sends.append(bydcan.create_steering_control(self.packer, apply_angle,
                                                        template,
                                                        self.frame // self.params.STEER_STEP))

        # Tell the cluster openpilot has the wheel. The camera drops its own LKAS
        # within seconds of us blocking its steering command, so without this the
        # LKAS indicator goes dark while openpilot is still steering. Sent on the
        # same cadence as the steering command, so panda hands 0x316 back to the
        # camera at the same moment it hands back 0x1E2.
        can_sends.append(bydcan.create_lkas_hud(self.packer, CS.lkas_hud,
                                                self.frame // self.params.STEER_STEP))

    # === LONGITUDINAL ===
    if self.CP.openpilotLongitudinalControl:
      if self.frame % self.params.STEER_STEP == 0:
        accel = float(np.clip(actuators.accel, self.params.ACCEL_MIN, self.params.ACCEL_MAX))
        if not CC.longActive or pcm_cancel_cmd:
          accel = 0.0

        can_sends.append(bydcan.create_acc_control(self.packer, accel,
                                                   CC.longActive and not pcm_cancel_cmd, self.acc_idx))

        set_speed = hud_control.setSpeed if hud_control.setSpeed > 0 else CS.out.cruiseState.speed
        can_sends.append(bydcan.create_acc_hud(self.packer, CC.enabled, set_speed * 3.6,
                                               hud_control.leadVisible, self.acc_idx))
        self.acc_idx += 1

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
