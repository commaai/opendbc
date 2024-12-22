from opendbc.can.packer import CANPacker
from opendbc.car.common.numpy_fast import clip
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import DBC, CarControllerParams
from opendbc.car.interfaces import CarControllerBase


def apply_byd_steer_angle_limits(apply_angle, actual_angle, v_ego, LIMITS):
    # pick angle rate limits based on wind up/down
    steer_up = actual_angle * \
        apply_angle >= 0. and abs(apply_angle) > abs(actual_angle)
    rate_limits = LIMITS.ANGLE_RATE_LIMIT_UP if steer_up else LIMITS.ANGLE_RATE_LIMIT_DOWN

    return clip(apply_angle, actual_angle - rate_limits, actual_angle + rate_limits)


class CarController(CarControllerBase):
    def __init__(self, dbc_name, CP):
        super().__init__(dbc_name, CP)
        self.params = CarControllerParams(self.CP)
        self.packer = CANPacker(DBC[self.CP.carFingerprint]['pt'])
        self.steer_rate_limited = False
        self.lka_active = False
        self.send_resume = False
        self.resume_counter = 0

    def update(self, CC, CS, now_nanos):
        # Send CAN Commands
        can_sends = []

        actuators = CC.actuators

        # steer
        apply_angle = apply_byd_steer_angle_limits(
            actuators.steeringAngleDeg, CS.out.steeringAngleDeg, CS.out.vEgo, self.params)
        self.steer_rate_limited = (
            abs(apply_angle - CS.out.steeringAngleDeg) > 2.5)

        # BYD CAN controlled lateral running at 50hz
        if (self.frame % 2) == 0:

            # logic to activate and deactivate lane keep, cannot tie to the lka_on state because it will occasionally deactivate itself
            if CS.lka_on:
                self.lka_active = True
            if not CS.lka_on and CS.lkas_rdy_btn:
                self.lka_active = False

            if CS.out.steeringTorqueEps > 15:
                apply_angle = CS.out.steeringAngleDeg

            lat_active = CC.enabled and abs(
                CS.out.steeringAngleDeg) < 90 and self.lka_active and not CS.out.standstill
            # temporary hardcode 60 because if 90 degrees it will fault
            # brake_hold = False
            can_sends.append(bydcan.create_can_steer_command(
                self.packer, apply_angle, lat_active, CS.out.standstill, (self.frame/2) % 16))
#      can_sends.append(create_accel_command(self.packer, actuators.accel, enabled, brake_hold, (frame/2) % 16))
            can_sends.append(bydcan.create_lkas_hud(self.packer, CC.enabled, CS.lss_state, CS.lss_alert, CS.tsr, CS.abh, CS.passthrough, CS.HMA, CS.pt2, CS.pt3,
                                                    CS.pt4, CS.pt5, self.lka_active, self.frame % 16))

        # frequency doesn't matter (original 20hz), but the counter must match + 1 else it will fault
        if (CS.out.standstill or CS.out.cruiseState.standstill) and CC.enabled and (self.frame % 50 == 0):
            self.send_resume = True

        # send 3 consecutive resume command
        if (self.frame % 10) == 0 and self.send_resume:
            if self.resume_counter >= 2:
                self.send_resume = False
                self.resume_counter = 0
            can_sends.append(bydcan.send_buttons(
                self.packer, 1, (CS.counter_pcm_buttons + 1) % 16))
            self.resume_counter += 1

        new_actuators = actuators.as_builder()
        new_actuators.steeringAngleDeg = apply_angle

        return new_actuators, can_sends
