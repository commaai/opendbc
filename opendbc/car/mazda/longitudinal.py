from opendbc.car import DT_CTRL
from opendbc.car.mazda.values import CarControllerParams


RESUME_UNLATCH_LATCHED_FRAMES = int(CarControllerParams.RESUME_UNLATCH_LATCHED_T / DT_CTRL)
RESUME_REPULSE_FRAMES = int(CarControllerParams.RESUME_REPULSE_T / DT_CTRL)
RELEASE_DEBOUNCE_FRAMES = int(CarControllerParams.RELEASE_DEBOUNCE_T / DT_CTRL)
BREAKAWAY_FRAMES = int(CarControllerParams.ACCEL_BREAKAWAY_T / DT_CTRL)


class StandstillHold:
  """Hold the car until the plan or driver requests movement."""

  def __init__(self):
    self._reset()

  def _reset(self):
    self.holding = False
    self.car_has_hold = False
    self.unlatch_frames = 0
    self.release_frames = 0
    self.latched_release = False
    self.just_released = False
    self.latched_frames = 0
    self.repulsed = False

  def update(self, long_engaged: bool, stopping: bool, standstill: bool,
             plan_accel: float, brake_hold: bool, gas_pressed: bool) -> None:
    self.just_released = False
    if not long_engaged:
      self._reset()
      return

    was_holding = self.holding
    self.release_frames = self.release_frames + 1 if plan_accel > 0. else 0
    release = gas_pressed or self.release_frames >= RELEASE_DEBOUNCE_FRAMES
    self.holding = not release and (stopping or standstill)

    if self.unlatch_frames > 0:
      self.unlatch_frames -= 1
    if was_holding and not self.holding and standstill and not gas_pressed and self.unlatch_frames == 0:
      self.latched_release = self.car_has_hold
      if self.latched_release:
        self.unlatch_frames = RESUME_UNLATCH_LATCHED_FRAMES
      self.just_released = True
      self.latched_frames = 0
      self.repulsed = False

    if self.latched_release and not self.holding and standstill and brake_hold and not gas_pressed:
      self.latched_frames += 1
      if self.latched_frames >= RESUME_REPULSE_FRAMES and not self.repulsed and self.unlatch_frames == 0:
        self.unlatch_frames = RESUME_UNLATCH_LATCHED_FRAMES
        self.repulsed = True
    else:
      self.latched_frames = 0

    self.car_has_hold = self.holding and standstill and brake_hold

  @property
  def stop_bits(self) -> bool:
    return self.holding and not self.car_has_hold and self.unlatch_frames == 0

  @property
  def resume_unlatching(self) -> bool:
    return self.unlatch_frames > 0

  @property
  def acc_active_2(self) -> bool:
    return not self.car_has_hold
