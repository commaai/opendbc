import ctypes
import os

from opendbc.safety import LEN_TO_DLC

libsafety_dir = os.path.dirname(os.path.abspath(__file__))
libsafety_fn = os.path.join(libsafety_dir, "libsafety.so")


class CANPacket(ctypes.Structure):
  _pack_ = 1
  _fields_ = [
    ("fd_bus_dlc", ctypes.c_uint8),
    ("flags_addr", ctypes.c_uint32),
    ("checksum", ctypes.c_uint8),
    ("data", ctypes.c_uint8 * 64),
    ("_padding", ctypes.c_uint8 * 2),
  ]

  @property
  def fd(self) -> int:
    return self.fd_bus_dlc & 0x1

  @fd.setter
  def fd(self, value: int) -> None:
    self.fd_bus_dlc = (self.fd_bus_dlc & ~0x1) | (value & 0x1)

  @property
  def bus(self) -> int:
    return (self.fd_bus_dlc >> 1) & 0x7

  @bus.setter
  def bus(self, value: int) -> None:
    self.fd_bus_dlc = (self.fd_bus_dlc & ~0xE) | ((value & 0x7) << 1)

  @property
  def data_len_code(self) -> int:
    return (self.fd_bus_dlc >> 4) & 0xF

  @data_len_code.setter
  def data_len_code(self, value: int) -> None:
    self.fd_bus_dlc = (self.fd_bus_dlc & ~0xF0) | ((value & 0xF) << 4)

  @property
  def rejected(self) -> int:
    return self.flags_addr & 0x1

  @rejected.setter
  def rejected(self, value: int) -> None:
    self.flags_addr = (self.flags_addr & ~0x1) | (value & 0x1)

  @property
  def returned(self) -> int:
    return (self.flags_addr >> 1) & 0x1

  @returned.setter
  def returned(self, value: int) -> None:
    self.flags_addr = (self.flags_addr & ~0x2) | ((value & 0x1) << 1)

  @property
  def extended(self) -> int:
    return (self.flags_addr >> 2) & 0x1

  @extended.setter
  def extended(self, value: int) -> None:
    self.flags_addr = (self.flags_addr & ~0x4) | ((value & 0x1) << 2)

  @property
  def addr(self) -> int:
    return (self.flags_addr >> 3) & 0x1FFFFFFF

  @addr.setter
  def addr(self, value: int) -> None:
    self.flags_addr = (self.flags_addr & 0x7) | ((value & 0x1FFFFFFF) << 3)


class CANPacketHandle:
  def __init__(self):
    self._packet = CANPacket()
    self._pointer = ctypes.pointer(self._packet)

  @property
  def _as_parameter_(self):
    return self._pointer

  def __getitem__(self, index: int) -> CANPacket:
    return self._pointer[index]

  def __getattr__(self, name: str):
    return getattr(self._packet, name)


CANPacketPtr = ctypes.POINTER(CANPacket)


class LibSafety:
  pass


def _set_sig(libsafety, name: str, argtypes, restype) -> None:
  fn = getattr(libsafety, name)
  fn.argtypes = argtypes
  fn.restype = restype


def _set_sig_if_present(libsafety, name: str, argtypes, restype) -> None:
  if hasattr(libsafety, name):
    _set_sig(libsafety, name, argtypes, restype)


def _configure(libsafety):
  _set_sig(libsafety, "safety_rx_hook", [CANPacketPtr], ctypes.c_bool)
  _set_sig(libsafety, "safety_tx_hook", [CANPacketPtr], ctypes.c_bool)
  _set_sig(libsafety, "safety_fwd_hook", [ctypes.c_int, ctypes.c_int], ctypes.c_int)
  _set_sig(libsafety, "set_safety_hooks", [ctypes.c_uint16, ctypes.c_uint16], ctypes.c_int)

  _set_sig(libsafety, "set_controls_allowed", [ctypes.c_bool], None)
  _set_sig(libsafety, "get_controls_allowed", [], ctypes.c_bool)
  _set_sig(libsafety, "get_longitudinal_allowed", [], ctypes.c_bool)
  _set_sig(libsafety, "set_alternative_experience", [ctypes.c_int], None)
  _set_sig(libsafety, "get_alternative_experience", [], ctypes.c_int)
  _set_sig(libsafety, "set_relay_malfunction", [ctypes.c_bool], None)
  _set_sig(libsafety, "get_relay_malfunction", [], ctypes.c_bool)
  _set_sig(libsafety, "get_gas_pressed_prev", [], ctypes.c_bool)
  _set_sig(libsafety, "set_gas_pressed_prev", [ctypes.c_bool], None)
  _set_sig(libsafety, "get_brake_pressed_prev", [], ctypes.c_bool)
  _set_sig(libsafety, "get_regen_braking_prev", [], ctypes.c_bool)
  _set_sig(libsafety, "get_steering_disengage_prev", [], ctypes.c_bool)
  _set_sig(libsafety, "get_acc_main_on", [], ctypes.c_bool)
  _set_sig(libsafety, "get_vehicle_speed_min", [], ctypes.c_float)
  _set_sig(libsafety, "get_vehicle_speed_max", [], ctypes.c_float)
  _set_sig(libsafety, "get_current_safety_mode", [], ctypes.c_int)
  _set_sig(libsafety, "get_current_safety_param", [], ctypes.c_int)

  _set_sig(libsafety, "set_torque_meas", [ctypes.c_int, ctypes.c_int], None)
  _set_sig(libsafety, "get_torque_meas_min", [], ctypes.c_int)
  _set_sig(libsafety, "get_torque_meas_max", [], ctypes.c_int)
  _set_sig(libsafety, "set_torque_driver", [ctypes.c_int, ctypes.c_int], None)
  _set_sig(libsafety, "get_torque_driver_min", [], ctypes.c_int)
  _set_sig(libsafety, "get_torque_driver_max", [], ctypes.c_int)
  _set_sig(libsafety, "set_desired_torque_last", [ctypes.c_int], None)
  _set_sig(libsafety, "set_rt_torque_last", [ctypes.c_int], None)
  _set_sig(libsafety, "set_desired_angle_last", [ctypes.c_int], None)
  _set_sig(libsafety, "get_desired_angle_last", [], ctypes.c_int)
  _set_sig(libsafety, "set_angle_meas", [ctypes.c_int, ctypes.c_int], None)
  _set_sig(libsafety, "get_angle_meas_min", [], ctypes.c_int)
  _set_sig(libsafety, "get_angle_meas_max", [], ctypes.c_int)

  _set_sig(libsafety, "get_cruise_engaged_prev", [], ctypes.c_bool)
  _set_sig(libsafety, "set_cruise_engaged_prev", [ctypes.c_bool], None)
  _set_sig(libsafety, "get_vehicle_moving", [], ctypes.c_bool)
  _set_sig(libsafety, "set_timer", [ctypes.c_uint32], None)

  _set_sig(libsafety, "safety_tick_current_safety_config", [], None)
  _set_sig(libsafety, "safety_config_valid", [], ctypes.c_bool)
  _set_sig(libsafety, "init_tests", [], None)

  _set_sig(libsafety, "set_honda_fwd_brake", [ctypes.c_bool], None)
  _set_sig(libsafety, "get_honda_fwd_brake", [], ctypes.c_bool)
  _set_sig(libsafety, "set_honda_alt_brake_msg", [ctypes.c_bool], None)
  _set_sig(libsafety, "set_honda_bosch_long", [ctypes.c_bool], None)
  _set_sig(libsafety, "get_honda_hw", [], ctypes.c_int)

  _set_sig_if_present(libsafety, "mutation_set_active_mutant", [ctypes.c_int], None)
  _set_sig_if_present(libsafety, "mutation_get_active_mutant", [], ctypes.c_int)

  return libsafety


libsafety: LibSafety = _configure(ctypes.CDLL(libsafety_fn))


def load(path):
  global libsafety
  libsafety = _configure(ctypes.CDLL(str(path)))


def make_CANPacket(addr: int, bus: int, dat):
  ret = CANPacketHandle()
  ret[0].extended = 1 if addr >= 0x800 else 0
  ret[0].addr = addr
  ret[0].data_len_code = LEN_TO_DLC[len(dat)]
  ret[0].bus = bus
  ret[0].data[:len(dat)] = dat
  return ret
