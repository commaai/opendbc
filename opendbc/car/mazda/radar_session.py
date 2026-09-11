from opendbc.car import DT_CTRL, make_tester_present_msg, uds
from opendbc.car.can_definitions import CanData
from opendbc.car.mazda.values import CarControllerParams


RADAR_ADDR, RADAR_BUS = 0x764, 0
RADAR_SESSION_LIMIT_FRAMES = round(CarControllerParams.RADAR_SESSION_LIMIT_T / DT_CTRL)


def create_radar_session_msg(session_type: int) -> CanData:
  return CanData(RADAR_ADDR, bytes([2, uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, session_type, 0, 0, 0, 0, 0]), RADAR_BUS)


class RadarSessionManager:
  def __init__(self):
    self.silencing = False
    self.restoring = False
    self.replacement_active = False
    self.failed = False
    self.state_frames = 0
    self.programming_sent = False
    self.diagnostic_message: CanData | None = None

  def _transition(self, *, silencing=False, restoring=False, active=False) -> None:
    self.silencing, self.restoring, self.replacement_active = silencing, restoring, active
    self.programming_sent = False
    self.state_frames = 0

  def update(self, gate_passed: bool, stock_radar_alive: bool, stock_radar_gone: bool,
             standstill: bool, frame: int) -> None:
    self.diagnostic_message = None
    self.state_frames += 1

    if self.restoring:
      if stock_radar_alive:
        self._transition()
      elif self.state_frames >= RADAR_SESSION_LIMIT_FRAMES:
        self.failed = True
        self._transition(active=True)
      elif frame % CarControllerParams.RADAR_UDS_STEP == 0:
        self.diagnostic_message = create_radar_session_msg(uds.SESSION_TYPE.DEFAULT)
      return

    if self.replacement_active and stock_radar_alive:
      self._transition()

    if not (self.silencing or self.replacement_active or self.failed) and gate_passed:
      if stock_radar_gone:
        self._transition(active=True)
      elif standstill and stock_radar_alive:
        self._transition(silencing=True)

    if self.silencing:
      if not self.programming_sent and (not gate_passed or not standstill):
        self._transition()
      elif self.programming_sent and (not gate_passed or not standstill):
        self._transition(restoring=True, active=True)
      elif self.programming_sent and not stock_radar_alive:
        self._transition(active=True)
      elif self.state_frames >= RADAR_SESSION_LIMIT_FRAMES:
        self.failed = True
        self._transition(restoring=self.programming_sent, active=self.programming_sent)
      elif frame % CarControllerParams.RADAR_UDS_STEP == 0:
        self.diagnostic_message = create_radar_session_msg(uds.SESSION_TYPE.PROGRAMMING)
        self.programming_sent = True

    if self.replacement_active and not self.failed and frame % CarControllerParams.RADAR_UDS_STEP == 0:
      self.diagnostic_message = make_tester_present_msg(RADAR_ADDR, RADAR_BUS, suppress_response=True)
