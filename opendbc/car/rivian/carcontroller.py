from opendbc.can.packer import CANPacker
from opendbc.car import Bus, apply_driver_steer_torque_limits, common_fault_avoidance
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.rivian.riviancan import create_lka_steering, create_longitudinal, create_wheel_touch, create_adas_status
from opendbc.car.rivian.values import CarControllerParams

MAX_STEER_RATE = 90  # deg/s
MAX_STEER_RATE_FRAMES = 10  # tx control frames needed before torque can be cut


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])

    self.steer_rate_counter = 0
    self.cancel_frames = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # >100 degree/sec steering fault prevention
    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, CC.latActive,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    apply_torque = 0
    if CC.latActive:
      new_torque = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, CarControllerParams)

    # send steering command
    self.apply_torque_last = apply_torque
    can_sends.append(create_lka_steering(self.packer, CS.acm_lka_hba_cmd, apply_torque, apply_steer_req))

    if self.frame % 5 == 0:
      can_sends.append(create_wheel_touch(self.packer, CS.sccm_wheel_touch, CC.enabled))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      can_sends.append(create_longitudinal(self.packer, self.frame % 15, actuators.accel, CC.enabled))
    else:
      interface_status = None
      if CC.cruiseControl.cancel:
        # if there is a noEntry, we need to send a status of "available" before the ACM will accept "unavailable"
        # send "available" right away as the VDM itself takes a few frames to acknowledge
        interface_status = 1 if self.cancel_frames < 5 else 0
        self.cancel_frames += 1
      else:
        self.cancel_frames = 0

      can_sends.append(create_adas_status(self.packer, CS.vdm_adas_status, interface_status))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends
