from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.gwm import gwmcan
from opendbc.car.gwm.values import CarControllerParams
import numpy as np
# DEBUG
from openpilot.common.params import Params
# DEBUG

MAX_USER_TORQUE = 100  # 1.0 Nm
EPS_MAX_TORQUE = 200
HELLO = 20


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(self.CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_angle_last = 0
    self.status = 2
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
      apply_steer = np.clip(new_torque, -EPS_MAX_TORQUE, EPS_MAX_TORQUE)
      if not lat_active:
        apply_steer = 0
      if not Params().get_bool("AleSato_DebugButton1"):
        apply_steer = CS.camera_stock_values["TORQUE_CMD"]
        lat_active = CS.camera_stock_values["STEER_REQUEST"]
      can_sends.append(gwmcan.create_steer_command(
        self.packer,
        self.CAN,
        camera_stock_values=CS.camera_stock_values,
        steer=apply_steer,
        steer_req=lat_active,
      ))

    new_actuators = actuators.as_builder()
    self.frame += 1
    return new_actuators, can_sends
