from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.demo import democan
from opendbc.car.demo.values import CanBus, CarControllerParams as CCP


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CAN = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])

    self.apply_torque_last = 0
    self.acc_buttons_counter_last = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    """

    For each message to be sent below, find the CAN message/signal, add a DBC entry, and add support in democan.py.
    Example code is provided for the common case of cars with torque control APIs. For cars with angle control, see
    the existing angle control ports like Tesla or Nissan for examples.

    """

    if self.frame % CCP.STEER_STEP == 0:
      apply_torque = 0
      if CC.latActive:
        new_torque = int(round(CC.actuators.torque * CCP.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, CCP)
      can_sends.append(democan.create_steering_control(self.packer_pt, self.CAN.pt, apply_torque, CC.latActive))

    if self.frame % CCP.LKA_HUD_STEP == 0:
      can_sends.append(democan.create_lka_hud_control(self.packer_pt, self.CAN.pt, CC.latActive, CC.hudControl))

    button_send_ready = CS.acc_buttons_counter != self.acc_buttons_counter_last
    if button_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(democan.create_acc_buttons_control(self.packer_pt, self.CAN.ext, CS.acc_buttons_counter,
                                                          cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.acc_buttons_counter_last = CS.acc_buttons_counter
    self.frame += 1
    return new_actuators, can_sends
