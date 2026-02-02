from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.gwm import gwmcan
# DEBUG
from openpilot.common.params import Params
# DEBUG


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_angle_last = 0
    self.status = 2
    self.CAN = gwmcan.CanBus(CP)
    self.fake_torque = False

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    # Try to satisfy steer nudge requests
    # DEBUG
    self.fake_torque = Params().get_bool("AleSato_DebugButton2")
    # DEBUG

    # can_sends.append(gwmcan.create_steer_and_ap_stalk(
    #   self.packer,
    #   self.CAN,
    #   CS.steer_and_ap_stalk_msg,
    #   self.fake_torque,
    # ))

    # Test to try understand EPS communication
    ea_simulated_torque = CS.out.steeringTorque
    if self.fake_torque and abs(ea_simulated_torque) < 50:
      ea_simulated_torque = 50.0 * (1 if ea_simulated_torque >= 0 else -1)
    can_sends.append(gwmcan.create_eps_update(
      self.packer,
      self.CAN,
      eps_stock_values=CS.eps_stock_values,
      ea_simulated_torque=ea_simulated_torque,
    ))

    new_actuators = actuators.as_builder()
    self.frame += 1
    return new_actuators, can_sends
