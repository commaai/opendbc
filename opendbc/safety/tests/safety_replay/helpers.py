from opendbc.car.toyota.values import ToyotaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py

def to_signed(d, bits):
  ret = d
  if d >= (1 << (bits - 1)):
    ret = d - (1 << bits)
  return ret

def is_steering_msg(mode, param, addr):
  ret = False
  if mode in (CarParams.SafetyModel.hondaNidec, CarParams.SafetyModel.hondaBosch):
    ret = (addr == 0xE4) or (addr == 0x194) or (addr == 0x33D) or (addr == 0x33DA) or (addr == 0x33DB)
  elif mode == CarParams.SafetyModel.toyota:
    ret = addr == (0x191 if param & ToyotaSafetyFlags.LTA else 0x2E4)
  elif mode == CarParams.SafetyModel.gm:
    ret = addr == 384
  elif mode == CarParams.SafetyModel.hyundai:
    ret = addr == 832
  elif mode == CarParams.SafetyModel.chrysler:
    ret = addr == 0x292
  elif mode == CarParams.SafetyModel.subaru:
    ret = addr == 0x122
  elif mode == CarParams.SafetyModel.ford:
    ret = addr == 0x3d3
  elif mode == CarParams.SafetyModel.nissan:
    ret = addr == 0x169
  elif mode == CarParams.SafetyModel.rivian:
    ret = addr == 0x120
  return ret

def get_steer_value(mode, param, to_send):
  torque, angle = 0, 0
  if mode in (CarParams.SafetyModel.hondaNidec, CarParams.SafetyModel.hondaBosch):
    torque = (to_send.data[0] << 8) | to_send.data[1]
    torque = to_signed(torque, 16)
  elif mode == CarParams.SafetyModel.toyota:
    if param & ToyotaSafetyFlags.LTA:
      angle = (to_send.data[1] << 8) | to_send.data[2]
      angle = to_signed(angle, 16)
    else:
      torque = (to_send.data[1] << 8) | (to_send.data[2])
      torque = to_signed(torque, 16)
  elif mode == CarParams.SafetyModel.gm:
    torque = ((to_send.data[0] & 0x7) << 8) | to_send.data[1]
    torque = to_signed(torque, 11)
  elif mode == CarParams.SafetyModel.hyundai:
    torque = (((to_send.data[3] & 0x7) << 8) | to_send.data[2]) - 1024
  elif mode == CarParams.SafetyModel.chrysler:
    torque = (((to_send.data[0] & 0x7) << 8) | to_send.data[1]) - 1024
  elif mode == CarParams.SafetyModel.subaru:
    torque = ((to_send.data[3] & 0x1F) << 8) | to_send.data[2]
    torque = -to_signed(torque, 13)
  elif mode == CarParams.SafetyModel.ford:
    angle = ((to_send.data[0] << 3) | (to_send.data[1] >> 5)) - 1000
  elif mode == CarParams.SafetyModel.nissan:
    angle = (to_send.data[0] << 10) | (to_send.data[1] << 2) | (to_send.data[2] >> 6)
    angle = -angle + (1310 * 100)
  elif mode == CarParams.SafetyModel.rivian:
    torque = ((to_send.data[2] << 3) | (to_send.data[3] >> 5)) - 1024
  return torque, angle

def package_can_msg(msg):
  return libsafety_py.make_CANPacket(msg.address, msg.src % 4, msg.dat)

def init_segment(safety, lr, mode, param):
  sendcan = (msg for msg in lr if msg.which() == 'sendcan')
  steering_msgs = (can for msg in sendcan for can in msg.sendcan if is_steering_msg(mode, param, can.address))

  msg = next(steering_msgs, None)
  if msg is None:
    # no steering msgs
    return

  to_send = package_can_msg(msg)
  torque, angle = get_steer_value(mode, param, to_send)
  if torque != 0:
    safety.set_controls_allowed(1)
    safety.set_desired_torque_last(torque)
  elif angle != 0:
    safety.set_controls_allowed(1)
    safety.set_desired_angle_last(angle)
    safety.set_angle_meas(angle, angle)
  assert safety.safety_tx_hook(to_send), "failed to initialize panda safety for segment"
