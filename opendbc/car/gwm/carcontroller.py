from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_meas_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.gwm import gwmcan
from opendbc.car.gwm.values import CarControllerParams
# DEBUG
from openpilot.common.params import Params
# DEBUG

MAX_USER_TORQUE = 100  # 1.0 Nm
HELLO = 20


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(self.CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_torque_last = 0
    self.CAN = gwmcan.CanBus(CP)
    self.fake_torque = False

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

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
    if self.frame % 2 == 0: # 50 Hz
      # Try to satisfy steer nudge requests
      ea_simulated_torque = CS.out.steeringTorque
      if self.fake_torque and abs(ea_simulated_torque) < HELLO:
        ea_simulated_torque = ea_simulated_torque + (HELLO if ea_simulated_torque >= 0 else -HELLO)
      can_sends.append(gwmcan.create_eps_update(
        self.packer,
        self.CAN,
        eps_stock_values=CS.eps_stock_values,
        ea_simulated_torque=ea_simulated_torque,
      ))

      # Steer command
      new_torque = int(round(actuators.torque * self.params.STEER_MAX))
      apply_torque = apply_meas_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorqueEps, self.params)
      if not lat_active:
        apply_torque = 0
      self.apply_torque_last = apply_torque
      if not Params().get_bool("AleSato_DebugButton1"):
        can_sends.append(gwmcan.bypass_steer_cmd(
          self.packer,
          self.CAN,
          camera_stock_values=CS.camera_stock_values,
        ))
      else:
        can_sends.append(gwmcan.create_steer_command(
          self.packer,
          self.CAN,
          camera_stock_values=CS.camera_stock_values,
          steer=apply_torque,
          steer_req=lat_active,
        ))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    self.frame += 1
    return new_actuators, can_sends
