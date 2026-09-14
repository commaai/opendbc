from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel


def get_safety_CP():
  from opendbc.car.byd.interface import CarInterface
  return CarInterface.get_non_essential_params("BYD_ATTO_3")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.apply_angle_last = 0.0
    self.not_accepted_frames = 0

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    hud_control = CC.hudControl

    if self.frame % 2:
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                          CS.out.steeringAngleDeg, CC.latActive, CarControllerParams, self.VM)

      cntr = (self.frame // 2) % 16

      # 0x1E2 runs for the whole engagement, not just while steering. STEER_REQ=0 carries
      # the measured angle, which keeps the EPS fed and panda's angle reference synced, so
      # the first frame after lateral comes back is not rate limited against a stale angle.
      if CC.enabled:
        can_sends.append(bydcan.create_steering_control(self.packer, self.apply_angle_last, CC.latActive, cntr))

      if CC.latActive:
        can_sends.append(bydcan.create_lkas_hud(self.packer, cntr, CS.lkas_hud, hud_control))

        # STEER_REQ=1 while EPS reports idle (LKS_PREPARED=1) for >1 s => not accepted
        not_accepted = not CS.eps_engaged
        self.not_accepted_frames = self.not_accepted_frames + 1 if not_accepted else 0
        CS.steer_not_accepted = self.not_accepted_frames > 50
      else:
        # Camera 0x316 comes back once the HUD hold expires; its 0x1E2 comes back only
        # after openpilot stops sending, i.e. once we are no longer engaged.
        self.not_accepted_frames = 0
        CS.steer_not_accepted = False

    if CC.cruiseControl.cancel and self.frame % 10 == 0:
      can_sends.append(bydcan.create_buttons(self.packer, cancel=True))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = float(self.apply_angle_last)

    self.frame += 1
    return new_actuators, can_sends
