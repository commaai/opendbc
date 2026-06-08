from opendbc.can import CANPacker
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.values import CANBUS, CarControllerParams, SteerDirection
from opendbc.sunnypilot.car.volvo.icbm import IntelligentCruiseButtonManagementInterface


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.CP = CP
    self.CCP = CarControllerParams(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.frame = 0

    self.apply_steer_prev = 0
    self.apply_steer_dir_prev = SteerDirection.NONE

    self.latActive_prev = False
    self.steer_blocked = False
    self.steer_blocked_cnt = 0
    self.steer_dir_bf_block = SteerDirection.NONE

    # Custom ACC increment: read once at init, refreshed every 100 frames.
    # Passed to carstate so _pending_delta emits the right number of events.
    self._params = Params()
    self._custom_acc_step = max(1, int(self._params.get("CustomAccShortPressIncrement", return_default=True) or 1))

    # SNG
    self.last_resume_frame = 0
    self.distance = 0
    self.waiting = False
    self.sng_count = 0
    # FSM3 ACC_Check=1 ack window: stock FSM3 has ACC_Check=0; ECU ignores
    # OP's CCButtons resume unless FSM3 confirms it. Force 25 frames (~0.5s).
    self.sng_ack_frames = 0

    # wall-clock gate for FSM3/FSM1 TX — frame%2 unreliable due to controlsd jitter;
    # fwd_hook blocks stock FSM3/FSM1 when controls_allowed && !gas_pressed, relay at 50Hz.
    self.next_long_tx_nanos = 0
    self.LONG_TX_PERIOD_NANOS = 20_000_000  # 50Hz, matching stock FSM3/FSM1

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

    # TODO: verify if this minSteerSpeed guard is still needed
    if pcm_cancel_cmd and CS.out.vEgo > self.CP.minSteerSpeed:
      can_sends.append(volvocan.create_button_msg(self.packer_pt, cancel=True))

    # run at 50hz
    if self.frame % 2 == 0:
      if CC.latActive and CS.out.vEgo > self.CP.minSteerSpeed:
        #apply_steer = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_steer_prev, CS.out.vEgoRaw, CarControllerParams)
        apply_steer = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_steer_prev, CS.out.vEgoRaw, CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)
        apply_steer_dir = SteerDirection.LEFT if apply_steer > 0 else SteerDirection.RIGHT

        error = CS.out.steeringAngleDeg - apply_steer
        error_with_deadzone = 0 if abs(error) < CarControllerParams.DEADZONE else error

        # Update prev with desired if just enabled.
        if not self.latActive_prev:
          self.apply_steer_dir_prev = apply_steer_dir

        if self.steer_blocked:
          if (apply_steer_dir == self.steer_dir_bf_block) or (self.steer_blocked_cnt <= 0) or (error_with_deadzone == 0):
            self.steer_blocked = False
        else:
          if apply_steer_dir != self.apply_steer_dir_prev and error_with_deadzone != 0:
            self.steer_blocked = True
            self.steer_blocked_cnt = CarControllerParams.BLOCK_LEN
            self.steer_dir_bf_block = self.apply_steer_dir_prev

        if self.steer_blocked:
          self.steer_blocked_cnt -= 1
          apply_steer_dir = SteerDirection.NONE
        elif error_with_deadzone == 0:
          # Set old request when inside deadzone
          apply_steer_dir = self.apply_steer_dir_prev

      else:
        apply_steer = 0
        apply_steer_dir = SteerDirection.NONE

      can_sends.append(volvocan.create_lka_msg(self.packer_pt, apply_steer, int(apply_steer_dir)))

      self.apply_steer_prev = apply_steer
      self.apply_steer_dir_prev = apply_steer_dir
      self.latActive_prev = CC.latActive

      # Manipulate data from servo to FSM
      # Avoids faults that will stop servo from accepting steering commands.
      can_sends.append(volvocan.create_lkas_state_msg(self.packer_pt, CS.out.steeringAngleDeg, CS.pscm_stock_values))

    at_standstill = (CS.out.cruiseState.enabled and CS.out.cruiseState.standstill
                     and CS.out.vEgo < 0.01)

    # SNG
    if (self.frame - self.last_resume_frame) * DT_CTRL > 1.00:
      if at_standstill and not self.waiting:
        self.distance = CS.acc_distance
        self.waiting = True
        self.sng_count = 0

      lead_moved = CS.acc_distance > self.distance

      if at_standstill and self.waiting and lead_moved:
        can_sends.extend([volvocan.create_button_msg(self.packer_pt, resume=True)] * self.CCP.BUTTON_BURST)
        if self.sng_count == 0:
          self.sng_ack_frames = 25
        self.sng_count += 1
      # disable sending resume after 5 cycles sent or if no more in standstill
      if self.waiting and (self.sng_count >= 5 or not CS.out.cruiseState.standstill):
        self.waiting = False
        self.last_resume_frame = self.frame

    # FSM3/FSM1 at 50Hz wall-clock: relay stock values so ECU doesn't fault.
    # Override ACC_Check=1 during SNG resume so ECU acknowledges OP's CCButtons resume.
    long_tx_due = now_nanos >= self.next_long_tx_nanos
    if long_tx_due:
      next_tx = self.next_long_tx_nanos + self.LONG_TX_PERIOD_NANOS
      if self.next_long_tx_nanos == 0 or next_tx <= now_nanos:
        next_tx = now_nanos + self.LONG_TX_PERIOD_NANOS
      self.next_long_tx_nanos = next_tx

      accel = float(CS.stock_FSM3["ACC_AccelerationRequest"])
      if self.sng_ack_frames > 0:
        acc_check = 1
        self.sng_ack_frames -= 1
      else:
        acc_check = int(CS.stock_FSM3["ACC_Check"])

      can_sends.append(volvocan.create_radar(self.packer_pt, CS.stock_FSM1, False))
      can_sends.append(volvocan.create_longitudinal(self.packer_pt, CS.stock_FSM3, accel, acc_check))

    # Refresh custom ACC step every 100 frames and forward to carstate so that
    # _pending_delta emits exactly one synthetic event per physical ACC step.
    if self.frame % 100 == 0:
      self._custom_acc_step = max(1, int(self._params.get("CustomAccShortPressIncrement", return_default=True) or 1))
    CS._custom_acc_step = self._custom_acc_step

    # Intelligent Cruise Button Management
    icbm_sends = IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer_pt, self.frame, self.last_button_frame)
    if icbm_sends:
      CS._pending_delta = 0
      CS._icbm_suppress_frames = 25
    can_sends.extend(icbm_sends)

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_steer_prev

    self.frame += 1
    return new_actuators, can_sends
