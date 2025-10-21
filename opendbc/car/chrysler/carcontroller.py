from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.lateral import apply_meas_steer_torque_limits
from opendbc.car.chrysler import chryslercan
from opendbc.car.chrysler.values import RAM_CARS, CarControllerParams, ChryslerFlags, RAM_DT
from opendbc.car.interfaces import CarControllerBase

from opendbc.sunnypilot.car.chrysler.carcontroller_ext import CarControllerExt
from opendbc.sunnypilot.car.chrysler.icbm import IntelligentCruiseButtonManagementInterface
from opendbc.sunnypilot.car.chrysler.mads import MadsCarController
from opendbc.sunnypilot.car.chrysler.values import ChryslerFlagsSP


class CarController(CarControllerBase, MadsCarController, CarControllerExt, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    MadsCarController.__init__(self)
    CarControllerExt.__init__(self, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.apply_torque_last = 0

    self.hud_count = 0
    self.last_lkas_falling_edge = 0
    self.lkas_control_bit_prev = False
    self.last_button_frame = 0

    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = CarControllerParams(CP)

  def update(self, CC, CC_SP, CS, now_nanos):
    MadsCarController.update(self, CC, CC_SP, CS)
    can_sends = []

    lkas_active = CC.latActive and self.lkas_control_bit_prev

    # cruise buttons
    if (self.frame - self.last_button_frame) * DT_CTRL > 0.05:
      das_bus = 2 if self.CP.carFingerprint in RAM_CARS else 0

      # ACC cancellation
      if CC.cruiseControl.cancel:
        self.last_button_frame = self.frame
        can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, cancel=True))

      # ACC resume from standstill
      elif CC.cruiseControl.resume:
        self.last_button_frame = self.frame
        can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, resume=True))

    # HUD alerts
    if self.frame % 25 == 0:
      if CS.lkas_car_model != -1:
        can_sends.append(chryslercan.create_lkas_hud(self.packer, self.CP, lkas_active, CC.hudControl.visualAlert,
                                                     self.hud_count, CS.lkas_car_model, CS.auto_high_beam, self.mads))
        self.hud_count += 1

    # steering
    if self.frame % self.params.STEER_STEP == 0:

      # TODO: can we make this more sane? why is it different for all the cars?
      lkas_control_bit = self.lkas_control_bit_prev
      if self.CP_SP.flags & ChryslerFlagsSP.NO_MIN_STEERING_SPEED or self.CP.carFingerprint in RAM_DT:
        lkas_control_bit = CarControllerExt.get_lkas_control_bit(self, CS, CC, lkas_control_bit)
      elif CS.out.vEgo > self.CP.minSteerSpeed:
        lkas_control_bit = True
      elif self.CP.flags & ChryslerFlags.HIGHER_MIN_STEERING_SPEED:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 3.0):
          lkas_control_bit = False
      elif self.CP.carFingerprint in RAM_CARS:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 0.5):
          lkas_control_bit = False

      # EPS faults if LKAS re-enables too quickly
      lkas_control_bit = lkas_control_bit and (self.frame - self.last_lkas_falling_edge > 200)

      if not lkas_control_bit and self.lkas_control_bit_prev:
        self.last_lkas_falling_edge = self.frame
      self.lkas_control_bit_prev = lkas_control_bit

      # steer torque
      new_torque = int(round(CC.actuators.torque * self.params.STEER_MAX))
      apply_torque = apply_meas_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorqueEps, self.params)
      if not lkas_active or not lkas_control_bit:
        apply_torque = 0
      self.apply_torque_last = apply_torque

      can_sends.append(chryslercan.create_lkas_command(self.packer, self.CP, int(apply_torque), lkas_control_bit))

    if self.frame % 10 == 0 and self.CP.carFingerprint not in RAM_CARS:
      can_sends.append(MadsCarController.create_lkas_heartbit(self.packer, CS.lkas_heartbit, self.mads))

    # Intelligent Cruise Button Management
    can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CS, CC_SP, self.packer, self.frame, self.last_button_frame))

    self.frame += 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    return new_actuators, can_sends
