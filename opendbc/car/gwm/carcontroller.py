from opendbc.can.packer import CANPacker
from opendbc.car import Bus, CanBusBase
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.gwm.gwmcan import create_steer_and_ap_stalk


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_angle_last = 0
    self.status = 2

    # Compute panda multipanda offset based on safetyConfigs length. This let's us
    # add the offset to all bus numbers when an external panda is present.
    can_base = CanBusBase(self.CP, None)
    self.bus_main = can_base.offset
    self.bus_cam = can_base.offset + 2  # Bus to forward the messages

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    # Forward STEER_AND_AP_STALK to bus 2, modifying only STEERING_TORQUE
    # TODO: apply_torque will be adjusted once openpilot integration is finalized
    apply_torque = int(CS.steer_and_ap_stalk_msg.get('STEERING_TORQUE', 0) * 100)
    can_sends.append(create_steer_and_ap_stalk(
        self.packer,
        CS.steer_and_ap_stalk_msg,
        apply_torque,
        bus=self.bus_cam
    ))

    new_actuators = actuators.as_builder()
    self.frame += 1
    return new_actuators, can_sends
