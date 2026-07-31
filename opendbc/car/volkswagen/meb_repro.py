# On-car experiment for reproducing the MEB near-standstill TSK permanent fault.

from opendbc.car import structs

LongCtrlState = structs.CarControl.Actuators.LongControlState


class _ReproCarControl:
  # Stand-in for the immutable CarControl reader with only longControlState replaced.

  def __init__(self, CC, long_control_state):
    self.enabled = CC.enabled
    self.longActive = CC.longActive
    self.cruiseControl = CC.cruiseControl
    self.actuators = structs.CarControl.Actuators(longControlState=long_control_state)


class MebShouldStopChurnRepro:
  # REPRO ONLY, do not merge.
  #
  # Once the car reaches the recorded entry condition, replay every 50 Hz policy
  # longControlState and accel input from b6/23 frame for frame. The window begins
  # at t=30.763331160, vEgo=0.1067569 m/s, Standstill=0, Motion_State=forwards,
  # and ends at t=35.244319054, one ACC_18 frame before TSK becomes permanent fault 7.
  # Only the approach to the entry speed is synthesized. The production
  # MebLongStateMachine still converts these exact policy inputs into ACC_18.

  APPROACH = "approach"
  REPLAY = "replay"

  ARM_SPEED = 10.0
  ENTRY_SPEED_MIN = 0.10
  ENTRY_SPEED_TARGET = 0.1067569
  ENTRY_SPEED_MAX = 0.115

  MOTION_FORWARDS = 1
  MOTION_REVERSING = 2
  MOTION_STOPPED = 3

  KP = 4.0
  APPROACH_ACCEL_MIN = -1.5
  APPROACH_ACCEL_MAX = 0.12
  LAUNCH_ACCEL = 0.12

  LONG_CONTROL_STATES = (
    LongCtrlState.off,
    LongCtrlState.pid,
    LongCtrlState.stopping,
    LongCtrlState.starting,
  )

  RECORDED_POLICY_INPUTS = ((1, 0.054852768778800964),
 (1, 0.05952766537666321),
 (1, 0.07462151348590851),
 (1, 0.07704810798168182),
 (1, 0.07920491695404053),
 (2, -0.019999999552965164),
 (2, -0.03999999910593033),
 (2, -0.05000000074505806),
 (2, -0.07999999821186066),
 (2, -0.10000000149011612),
 (2, -0.11999999731779099),
 (2, -0.14000000059604645),
 (2, -0.1599999964237213),
 (2, -0.18000000715255737),
 (2, -0.20000000298023224),
 (2, -0.2199999988079071),
 (2, -0.23999999463558197),
 (2, -0.25),
 (2, -0.2800000011920929),
 (2, -0.30000001192092896),
 (1, 0.0692165195941925),
 (1, 0.07038815319538116),
 (1, 0.07038815319538116),
 (1, 0.0812048390507698),
 (1, 0.08230344951152802),
 (1, 0.08727064728736877),
 (1, 0.08824723958969116),
 (1, 0.08824723958969116),
 (1, 0.08824723958969116),
 (2, -0.05000000074505806),
 (1, 0.08631652593612671),
 (1, 0.08624789863824844),
 (1, 0.09294483065605164),
 (1, 0.09273403882980347),
 (1, 0.09255243092775345),
 (1, 0.09397073090076447),
 (1, 0.09385316073894501),
 (1, 0.09385316073894501),
 (2, -0.029999999329447746),
 (2, -0.05000000074505806),
 (2, -0.07000000029802322),
 (2, -0.09000000357627869),
 (2, 0.09586943686008453),
 (1, 0.0959901362657547),
 (1, 0.09626269340515137),
 (1, -0.009999999776482582),
 (2, -0.03999999910593033),
 (2, -0.03999999910593033),
 (2, -0.07999999821186066),
 (2, -0.10000000149011612),
 (2, -0.10999999940395355),
 (2, -0.10999999940395355),
 (2, -0.1599999964237213),
 (2, -0.18000000715255737),
 (2, -0.20000000298023224),
 (2, -0.20999999344348907),
 (2, -0.23999999463558197),
 (2, -0.25999999046325684),
 (2, -0.2800000011920929),
 (2, -0.30000001192092896),
 (2, -0.3100000023841858),
 (2, -0.3400000035762787),
 (2, 0.08227246254682541),
 (1, 0.08400736004114151),
 (1, 0.08566243201494217),
 (1, 0.0964336097240448),
 (1, 0.09796986728906631),
 (1, 0.10864084213972092),
 (1, 0.110081747174263),
 (1, 0.11076467484235764),
 (1, 0.12190763652324677),
 (1, 0.12323205173015594),
 (1, 0.1360638290643692),
 (1, 0.13739481568336487),
 (1, 0.13871726393699646),
 (1, 0.15543954074382782),
 (1, 0.1568754017353058),
 (1, 0.15779317915439606),
 (1, 0.15928010642528534),
 (1, 0.16002309322357178),
 (1, 0.16364091634750366),
 (1, 0.16517964005470276),
 (1, 0.172075554728508),
 (1, 0.17365051805973053),
 (1, 0.1752350628376007),
 (1, 0.18358595669269562),
 (1, 0.18517903983592987),
 (1, 0.1812686324119568),
 (1, 0.1827852874994278),
 (1, 0.18353617191314697),
 (1, 0.18138237297534943),
 (1, 0.1828102022409439),
 (1, 0.18577122688293457),
 (1, 0.18740132451057434),
 (1, 0.18913614749908447),
 (1, 0.19572672247886658),
 (1, 0.19763916730880737),
 (1, -0.009999999776482582),
 (2, -0.029999999329447746),
 (2, -0.03999999910593033),
 (2, -0.03999999910593033),
 (1, 0.13691295683383942),
 (1, 0.1380041390657425),
 (1, 0.14010408520698547),
 (1, 0.14206562936306),
 (1, 0.1441105455160141),
 (1, 0.14666058123111725),
 (1, -0.009999999776482582),
 (2, -0.029999999329447746),
 (2, -0.03999999910593033),
 (2, -0.05999999865889549),
 (2, -0.09000000357627869),
 (2, -0.10999999940395355),
 (2, -0.12999999523162842),
 (2, -0.15000000596046448),
 (1, 0.11425913125276566),
 (1, 0.11555542051792145),
 (1, -0.009999999776482582),
 (2, -0.029999999329447746),
 (2, -0.03999999910593033),
 (2, -0.07000000029802322),
 (2, -0.09000000357627869),
 (2, -0.10999999940395355),
 (2, -0.12999999523162842),
 (2, -0.15000000596046448),
 (2, 0.10681670904159546),
 (1, 0.108151376247406),
 (1, 0.1153978705406189),
 (1, 0.11629227548837662),
 (1, 0.11673492193222046),
 (1, 0.11673492193222046),
 (1, 0.1251402497291565),
 (1, 0.13176380097866058),
 (1, 0.13273169100284576),
 (1, 0.13370226323604584),
 (1, 0.13716048002243042),
 (1, 0.13815803825855255),
 (1, 0.1398540735244751),
 (1, 0.1408645659685135),
 (1, 0.14137104153633118),
 (1, 0.14370383322238922),
 (1, 0.14446069300174713),
 (1, 0.14792302250862122),
 (1, 0.14850716292858124),
 (1, 0.14907224476337433),
 (1, 0.14799052476882935),
 (1, 0.14825749397277832),
 (1, 0.1474616676568985),
 (1, 0.14769680798053741),
 (1, 0.1478356122970581),
 (1, 0.14689235389232635),
 (1, 0.1473684459924698),
 (1, 0.14591248333454132),
 (1, 0.14657935500144958),
 (1, 0.14733867347240448),
 (2, -0.019999999552965164),
 (2, -0.03999999910593033),
 (2, 0.11977284401655197),
 (1, 0.12072501331567764),
 (1, 0.12170600146055222),
 (1, 0.12014923989772797),
 (1, 0.12113932520151138),
 (1, 0.12113932520151138),
 (2, -0.029999999329447746),
 (2, -0.05000000074505806),
 (2, -0.07000000029802322),
 (2, -0.09000000357627869),
 (2, -0.10999999940395355),
 (2, -0.12999999523162842),
 (2, -0.15000000596046448),
 (1, 0.10922684520483017),
 (1, 0.11010091006755829),
 (1, 0.11010091006755829),
 (1, 0.11489462107419968),
 (1, 0.11578327417373657),
 (1, 0.11583836376667023),
 (1, 0.11671803891658783),
 (1, 0.11758352071046829),
 (1, 0.11846272647380829),
 (1, 0.11934220790863037),
 (1, 0.12689879536628723),
 (1, 0.12783223390579224),
 (1, 0.12783223390579224),
 (1, 0.13675877451896667),
 (1, 0.13804200291633606),
 (1, 0.13883505761623383),
 (1, 0.14065150916576385),
 (1, -0.009999999776482582),
 (2, -0.029999999329447746),
 (2, -0.05000000074505806),
 (1, 0.12004034221172333),
 (1, 0.12169719487428665),
 (1, 0.12169719487428665),
 (1, 0.12566430866718292),
 (1, 0.12696778774261475),
 (1, 0.125629261136055),
 (1, 0.12730586528778076),
 (1, -0.009999999776482582),
 (2, -0.029999999329447746),
 (2, -0.05000000074505806),
 (1, 0.1124979555606842),
 (1, 0.11333118379116058),
 (1, 0.11333118379116058),
 (1, 0.11494379490613937),
 (1, 0.11576811224222183),
 (2, -0.019999999552965164),
 (2, -0.03999999910593033),
 (2, -0.05999999865889549),
 (2, -0.07999999821186066),
 (2, -0.10000000149011612),
 (2, -0.11999999731779099),
 (2, -0.14000000059604645),
 (2, -0.14000000059604645),
 (2, -0.18000000715255737),
 (2, -0.20000000298023224),
 (2, -0.20999999344348907),
 (2, -0.23999999463558197),
 (2, -0.25999999046325684),
 (2, -0.2800000011920929),
 (2, -0.30000001192092896),
 (2, -0.3199999928474426),
 (2, -0.33000001311302185),
 (2, -0.36000001430511475),
 (2, -0.3799999952316284),
 (2, -0.4000000059604645))

  def __init__(self):
    self.blocked_until_disengage = False
    self._reset_sequence()

  def _reset_sequence(self):
    self.phase = self.APPROACH
    self.replay_index = 0

  @staticmethod
  def _replace_long_control_state(CC, long_control_state):
    return _ReproCarControl(CC, long_control_state)

  def _command(self, CC, raw_long_control_state, accel):
    long_control_state = self.LONG_CONTROL_STATES[raw_long_control_state]
    return accel, self._replace_long_control_state(CC, long_control_state)

  def _replay_next(self, CC):
    raw_long_control_state, accel = self.RECORDED_POLICY_INPUTS[self.replay_index]
    self.replay_index += 1
    if self.replay_index == len(self.RECORDED_POLICY_INPUTS):
      self._reset_sequence()
    return self._command(CC, raw_long_control_state, accel)

  def _approach(self, CS, CC):
    if CS.esp_hold_confirmation or CS.meb_motion_state == self.MOTION_STOPPED:
      return self._command(CC, LongCtrlState.pid, self.LAUNCH_ACCEL)

    speed = CS.out.vEgo
    at_recorded_entry = (CS.meb_motion_state == self.MOTION_FORWARDS and
                         self.ENTRY_SPEED_MIN <= speed <= self.ENTRY_SPEED_MAX)
    if at_recorded_entry:
      self.phase = self.REPLAY
      return self._replay_next(CC)

    accel = min(max((self.ENTRY_SPEED_TARGET - speed) * self.KP,
                    self.APPROACH_ACCEL_MIN), self.APPROACH_ACCEL_MAX)
    long_control_state = LongCtrlState.pid if accel >= 0.0 else LongCtrlState.stopping
    return accel, self._replace_long_control_state(CC, long_control_state)

  def update(self, CS, CC, accel):
    if not CC.enabled:
      self.blocked_until_disengage = False
      self._reset_sequence()
      return accel, CC

    driver_cancel = CS.out.brakePressed or CS.out.gasPressed
    reversing = CS.meb_motion_state == self.MOTION_REVERSING
    unsafe_to_run = CS.out.accFaulted or CS.out.parkingBrake or CS.out.vEgo > self.ARM_SPEED or reversing
    if driver_cancel or unsafe_to_run:
      self.blocked_until_disengage = True
      self._reset_sequence()

    if self.blocked_until_disengage:
      return accel, CC

    if not CC.longActive:
      self._reset_sequence()
      return accel, CC

    if self.phase == self.REPLAY:
      return self._replay_next(CC)
    return self._approach(CS, CC)
