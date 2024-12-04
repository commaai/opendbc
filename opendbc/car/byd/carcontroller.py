from opendbc.can.packer import CANPacker
from opendbc.car import apply_driver_steer_torque_limits, structs
from opendbc.car.byd import bydcan
from openpilot.common.numpy_fast import clip
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.interfaces import CarControllerBase

VisualAlert = structs.CarControl.HUDControl.VisualAlert
ButtonType = structs.CarState.ButtonEvent.Type

STEER_STEP = 2  #100/2=50hz
ACC_STEP = 2    #50hz

STEER_SOFTSTART_STEP = 6 # 20ms(50Hz) * 200 / 6 = 666ms. This means the clip ceiling will be increased to 200 in 666ms

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    super().__init__(dbc_name, CP)

    self.packer = CANPacker(dbc_name)
    self.params = CarControllerParams(self.CP)

    self.last_steer_frame = 0
    self.last_acc_frame = 0

    self.apply_steer_last = 0

    self.mpc_lkas_counter = 0
    self.mpc_acc_counter = 0
    self.eps_fake318_counter = 0

    self.lkas_req_prepare = 0
    self.lkas_active = 0

    self.steer_softstart_limit = 0

    self.first_start = True


  def update(self, CC, CS, now_nanos):
    can_sends = []

    if (self.frame - self.last_steer_frame) >= STEER_STEP:

      #Resolve counter mismatch problem
      if self.first_start:
        self.mpc_lkas_counter = CS.acc_mpc_state_counter
        self.mpc_acc_counter = CS.acc_cmd_counter
        self.eps_fake318_counter = CS.eps_state_counter
        self.first_start = False

      apply_steer = 0

      if CC.latActive :
        if self.lkas_active:
          new_steer = int(round(CC.actuators.steer * CarControllerParams.STEER_MAX))

          if self.steer_softstart_limit < CarControllerParams.STEER_MAX :
            self.steer_softstart_limit = self.steer_softstart_limit + STEER_SOFTSTART_STEP
            new_steer = clip(new_steer, -self.steer_softstart_limit, self.steer_softstart_limit)

          apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last,
                                                          CS.out.steeringTorque, CarControllerParams)

        else :
          if CS.lkas_prepared:
            self.lkas_active = 1
            self.lkas_req_prepare = 0
            self.steer_softstart_limit = 0
          else:
            self.lkas_req_prepare = 1

      else :
        self.lkas_req_prepare = 0
        self.lkas_active = 0
        self.soft_start_torque_limit = 0
        self.steer_softstart_limit = 0

      self.apply_steer_last = apply_steer

      self.mpc_lkas_counter = int(self.mpc_lkas_counter + 1) & 0xF
      self.eps_fake318_counter = int(self.eps_fake318_counter + 1) & 0xF
      self.last_steer_frame = self.frame

      # send steering command, op to esc
      can_sends.append(bydcan.create_steering_control(self.packer, self.CP, CS.cam_lkas,
          self.apply_steer_last, self.lkas_req_prepare, self.lkas_active, self.mpc_lkas_counter))

      # send fake 318 from op to mpc
      can_sends.append(bydcan.create_fake_318(self.packer, self.CP, CS.esc_eps,
                                              CS.mpc_laks_output, CS.mpc_laks_reqprepare, CS.mpc_laks_active,
                                              CC.latActive, self.eps_fake318_counter))


    #handle wrap around // note not necessary for python 3 as int has no limit, good for lazy people
    #if self.frame < self.last_acc_frame:
    #  self.last_acc_frame = self.frame - 1

    accel = 0
    if (self.frame - self.last_acc_frame) >= ACC_STEP:
      accel = clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

      self.last_acc_frame = self.frame
      can_sends.append(bydcan.acc_command(self.packer, self.CP, CS.cam_acc, accel, CC.enabled))

    new_actuators = CC.actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / CarControllerParams.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    self.frame += 1
    return new_actuators, can_sends
