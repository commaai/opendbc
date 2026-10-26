"""Required RX cycles for torque steering and body/default safety fixtures.

State runs at 25 Hz and torque measurements at 50 Hz. These rates remain
inside the stock ten-period RX watchdogs, including 100 Hz checks.

Integration must capture alternate encoders of shared messages: Mazda gas and
motor torque, GM PCM status, PQ PCM/brake, MLB gas/brake, and Rivian speed_2.
Rivian quality faults require value predicates (valid is 1), rather than the
bridge's current nonzero fault test. MG gas is modified after packing; its raw
counter must be adapted when capturing explicit injections.
"""
from opendbc.car.structs import CarParams
from opendbc.safety.tests.fixture_streams import _Descriptor


def _message(message, bus, values, capture_method):
  def encode(case):
    return case.packer.make_can_msg_safety(message, bus, values)
  encode.__name__ = capture_method
  return encode


def _mg_encoder(method):
  def encode(case, value):
    # MG fixture initialization happens after common.setUp installs streams.
    if not hasattr(case, 'counters'):
      case.counters = dict.fromkeys((0x1b6, 0x1ec, 0x23c, 0x242), 0)
    return getattr(case, method)(value)
  encode.__name__ = method
  return encode


def _mg_gas(case, gas):
  # The DBC omits the counter. Preserve the fixture's exact byte placement
  # through the template checksum callback, which runs after CAN packing.
  counter = getattr(case, '_stream_gas_counter', 0)
  case._stream_gas_counter = (counter + 1) % 16

  def counter_frame(frame):
    address, data, bus = frame
    data = bytearray(data)
    data[5] = (data[5] & 0x0f) | (counter << 4)
    return address, bytes(data), bus

  return case.packer.make_can_msg_safety('GW_HSC2_HCU_FrP00', 0,
                                       {'EPTAccelActuPosHSC2': 100 if gas else 0}, fix_checksum=counter_frame)


_mg_gas.__name__ = '_user_gas_msg'


def get_descriptors(case) -> list[_Descriptor] | None:
  """Return one descriptor per required RX group, or None for unowned modes."""
  model = case.SAFETY_MODEL
  modes = CarParams.SafetyModel
  if model in (modes.silent, modes.noOutput, modes.allOutput):
    return []
  if model in (modes.chrysler, modes.chryslerCusw):
    return [
      _Descriptor('torque', '_torque_meas_msg', (0,), 50, 16, ('COUNTER',)),
      _Descriptor('speed', '_speed_msg', (0,), 25, 16 if model == modes.chryslerCusw or case.SAFETY_PARAM else 1, ('COUNTER',)),
      _Descriptor('brake', '_user_brake_msg', (False,), 25, 16, ('COUNTER',)),
      _Descriptor('gas', '_user_gas_msg', (0,), 25, 16, ('COUNTER',)),
      _Descriptor('pcm', '_pcm_status_msg', (False,), 25, 16, ('COUNTER',)),
    ]
  if model == modes.gm:
    from opendbc.car.gm.values import GMSafetyFlags
    # Both brake messages are required, regardless of which one the mode reads.
    descriptors = [
      _Descriptor('driver_torque', '_torque_driver_msg', (0,), 50),
      _Descriptor('speed', '_speed_msg', (0,), 25),
      _Descriptor('buttons', _message('ASCMSteeringButton', 0, {'ACCButtons': 1}, '_button_msg'), (), 25),
      _Descriptor('brake_position', _message('ECMAcceleratorPos', 0, {'BrakePedalPos': 0}, '_user_brake_msg'), (), 25),
      _Descriptor('brake_switch', _message('ECMEngineStatus', 0, {'BrakePressed': False}, '_user_brake_msg'), (), 25),
      _Descriptor('gas_pcm', '_user_gas_msg', (0,), 25),
    ]
    if case.SAFETY_PARAM & GMSafetyFlags.EV:
      descriptors.append(_Descriptor('regen', '_user_regen_msg', (False,), 25))
    return descriptors
  if model == modes.mazda:
    return [
      _Descriptor('driver_torque', '_torque_driver_msg', (0,), 50),
      _Descriptor('engine', '_speed_msg', (0,), 25),
      _Descriptor('brake', '_user_brake_msg', (False,), 25),
      _Descriptor('pcm', '_pcm_status_msg', (False,), 25),
      _Descriptor('buttons', '_button_msg', (), 25),
    ]
  if model == modes.mg:
    return [
      _Descriptor('speed', _mg_encoder('_speed_msg'), (0,), 25, 16, ('VehSpdAvgAlvRCHSC2',)),
      _Descriptor('driver_torque', _mg_encoder('_torque_driver_msg'), (0,), 50, 16, ('ChLKAAlvRCHSC2',)),
      _Descriptor('brake', _mg_encoder('_user_brake_msg'), (False,), 25, 16, ('BrkPdlAppdRCHSC2',)),
      _Descriptor('pcm', _mg_encoder('_pcm_status_msg'), (False,), 25, 16, ('ACCSysAlvRlngCtr_SCSHSC2',)),
      _Descriptor('gas', _mg_gas, (0,), 25, 16),
    ]
  if model == modes.rivian:
    return [
      _Descriptor('speed', '_speed_msg', (0,), 50, 15, ('ESP_Status_Counter',)),
      _Descriptor('gas_speed', '_user_gas_msg', (0,), 50, 15, ('VDM_PropStatus_Counter',)),
      _Descriptor('brake', '_user_brake_msg', (False,), 25, 15, ('iBESP2_AliveCounter',)),
      _Descriptor('driver_torque', '_torque_driver_msg', (0,), 50, 15, ('EPAS_SystemStatus_Counter',)),
      _Descriptor('pcm', '_pcm_status_msg', (False,), 25, 15, ('ACM_Status_Counter',)),
    ]
  if model in (modes.subaru, modes.subaruPreglobal):
    cycle = 16 if model == modes.subaru else 1
    return [
      _Descriptor('driver_torque', '_torque_driver_msg', (0,), 50, cycle, ('COUNTER',)),
      _Descriptor('speed', '_speed_msg', (0,), 25, cycle, ('COUNTER',)),
      _Descriptor('brake', '_user_brake_msg', (False,), 25, cycle, ('COUNTER',)),
      _Descriptor('gas', '_user_gas_msg', (0,), 25, cycle, ('COUNTER',)),
      _Descriptor('pcm', '_pcm_status_msg', (False,), 25, cycle, ('COUNTER',)),
    ]
  if model in (modes.volkswagen, modes.volkswagenPq, modes.volkswagenMlb):
    pq = model == modes.volkswagenPq
    mlb = model == modes.volkswagenMlb
    descriptors = [
      _Descriptor('driver_torque', '_torque_driver_msg', (0,), 50, 16, ('COUNTER',)),
      _Descriptor('speed', '_speed_msg', (0,), 25, 16 if mlb else 1, ('COUNTER',)),
      _Descriptor('gas', '_user_gas_msg', (0,), 25, 1 if pq else 16, ('COUNTER',)),
    ]
    if pq:
      descriptors.extend([
        _Descriptor('brake_pcm', '_motor_2_msg', (), 25),
        _Descriptor('main', '_motor_5_msg', (), 25),
        _Descriptor('buttons', '_button_msg', (False, False, False, 0), 25, 16, ('COUNTER',)),
      ])
    else:
      descriptors.extend([
        _Descriptor('brake_pressure', '_esp_05_msg', (False,), 25, 16, ('COUNTER',)),
        _Descriptor('pcm', '_tsk_status_msg', (False,), 25, 16, ('COUNTER',)),
        _Descriptor('buttons', '_ls_01_msg' if mlb else '_gra_acc_01_msg', (0, 0, 0, 0), 25, 16, ('COUNTER',)),
      ])
      if not mlb:
        descriptors.append(_Descriptor('brake_switch', '_motor_14_msg', (False,), 25))
    return descriptors
  if model == modes.body:
    return [_Descriptor('motors', '_motors_data_msg', (0, 0), 50, 16, ('COUNTER',))]
  return None
