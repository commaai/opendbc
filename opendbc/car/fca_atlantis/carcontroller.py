import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.fca_atlantis import fca_atlantiscan
from opendbc.car.fca_atlantis.values import CanBus, CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CCP = CarControllerParams(CP)
    self.CANBUS = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])

    self.use_friction = CP.lateralTuning.which() == 'pid'
    self.apply_torque_last = 0
    self.torque_cut = 0  # counts removed from the request (plus friction) by the torque limits
    self.steering = False  # lateral active and the EPS accepting LKA torque
    self.lat_active_last = False
    self.release_hold_frames = 0
    self.button_counter_last = 0
    self.frame = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      if CC.latActive and CS.lka_status == 2:  # "lka_active" per DBC
        # This van uses PID rather than the torque controller (see interface.py): the torque it needs tracks steering
        # angle times speed, not lateral accel, and the torque controller's fixed low-speed gain made it oscillate.
        # PID lacks the torque controller's steering friction compensation, though, and without it PID sticks at the
        # steering friction (~40 counts) and tracks small changes poorly. So add it here: push toward the desired angle
        # (from the PID controller) with up to STEER_FRICTION counts, over a band that widens at low speed (see values.py).
        # Only with PID: the other controllers don't report a desired angle.
        angle_error = actuators.steeringAngleDeg - CS.out.steeringAngleDeg if self.use_friction else 0.
        band = np.interp(CS.out.vEgo, self.CCP.STEER_FRICTION_BAND_BP, self.CCP.STEER_FRICTION_BAND_V)
        friction = np.interp(CS.out.vEgo, self.CCP.STEER_FRICTION_FADE_BP, [0., self.CCP.STEER_FRICTION]) * \
                   np.clip(angle_error / band, -1., 1.)
        new_torque = int(round(actuators.torque * self.CCP.STEER_MAX + friction))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)
        self.torque_cut = apply_torque - new_torque
      elif CS.lka_status == 2:
        # Stock LKA never cuts torque in one step: it ramps to zero, then holds LKA_ACTIVE at zero torque for a few
        # frames before releasing. An abrupt release with torque applied is suspected of latching EPS LKA_FAULT.
        # Panda safety allows this winddown after a disengage, ramping down at up to STEER_DELTA_DOWN per frame.
        torque_mag = max(abs(self.apply_torque_last) - self.CCP.STEER_DELTA_DOWN, 0)
        apply_torque = torque_mag if self.apply_torque_last > 0 else -torque_mag
      else:
        apply_torque = 0

      if CC.latActive or apply_torque != 0:
        self.release_hold_frames = self.CCP.STEER_RELEASE_HOLD_FRAMES
      lka_active = CC.latActive or self.release_hold_frames > 0
      if not CC.latActive and apply_torque == 0 and self.release_hold_frames > 0:
        self.release_hold_frames -= 1

      self.steering = CC.latActive and CS.lka_status == 2
      self.apply_torque_last = apply_torque
      # The EPS is suspected to listen only to LKA_COMMAND_2. LKA_COMMAND is a 1/4 scale copy, which panda safety
      # requires to follow LKA_COMMAND_2 in the same frame.
      # TODO: try removing LKA_COMMAND if the EPS steers without it
      can_sends.append(fca_atlantiscan.create_steering_control(self.packer_pt, self.CANBUS.pt, "LKA_COMMAND_2", apply_torque, lka_active))
      can_sends.append(fca_atlantiscan.create_steering_control(self.packer_pt, self.CANBUS.pt, "LKA_COMMAND",
                                                               fca_atlantiscan.lka_command_torque(apply_torque), lka_active))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.HUD_2_STEP == 0:
      can_sends.append(fca_atlantiscan.create_lka_hud_2_control(self.packer_pt, self.CANBUS.pt, CC.latActive))
    lat_active_rising = CC.latActive and not self.lat_active_last
    if self.frame % self.CCP.HUD_3_STEP == 0 or lat_active_rising:
      can_sends.append(fca_atlantiscan.create_lka_hud_3_control(self.packer_pt, self.CANBUS.pt, CC.latActive))
    self.lat_active_last = CC.latActive

    # **** ACC Button Controls ********************************************** #

    # the ACC ECU sees the stock buttons forwarded from the pt bus, so follow each stock message with a cancel
    if CS.button_counter != self.button_counter_last:
      self.button_counter_last = CS.button_counter
      if CC.cruiseControl.cancel:
        can_sends.append(fca_atlantiscan.create_acc_button_control(self.packer_pt, self.CANBUS.cam, CS.button_counter + 1, cancel=True))

    new_actuators = actuators.as_builder()
    if self.steering:
      # Report the request plus only what the limits cut, not the friction term: controlsd sets steer_limited_by_safety
      # (freezing PID's integrator) when this differs from the request.
      new_actuators.torque = actuators.torque + self.torque_cut / self.CCP.STEER_MAX
    else:
      new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends
