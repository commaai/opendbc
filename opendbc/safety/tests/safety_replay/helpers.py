from opendbc.safety.tests.libsafety import libsafety_py


def package_can_msg(msg):
  return libsafety_py.make_CANPacket(msg.address, msg.src % 4, msg.dat)
