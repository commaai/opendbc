import os
from cffi import FFI
from typing import Protocol

from opendbc.safety import LEN_TO_DLC

libsafety_dir = os.path.dirname(os.path.abspath(__file__))
libsafety_fn = os.path.join(libsafety_dir, "libsafety.so")

ffi = FFI()

ffi.cdef("""
typedef struct {
  unsigned char fd : 1;
  unsigned char bus : 3;
  unsigned char data_len_code : 4;
  unsigned char rejected : 1;
  unsigned char returned : 1;
  unsigned char extended : 1;
  unsigned int addr : 29;
  unsigned char checksum;
  unsigned char data[64];
} CANPacket_t;
""", packed=True)

ffi.cdef("""
bool safety_rx_hook(CANPacket_t *msg);
bool safety_tx_hook(CANPacket_t *msg);
int safety_fwd_hook(int bus_num, int addr);
int set_safety_hooks(uint16_t mode, uint16_t param);
""")


def setup_safety_helpers(ffi):
  ffi.cdef("""
  void set_controls_allowed(bool c);
  bool get_controls_allowed(void);
  bool get_longitudinal_allowed(void);
  void set_alternative_experience(int mode);
  int get_alternative_experience(void);
  void set_relay_malfunction(bool c);
  bool get_relay_malfunction(void);
  bool get_gas_pressed_prev(void);
  void set_gas_pressed_prev(bool);
  bool get_brake_pressed_prev(void);
  bool get_regen_braking_prev(void);
  bool get_steering_disengage_prev(void);
  bool get_acc_main_on(void);
  float get_vehicle_speed_min(void);
  float get_vehicle_speed_max(void);
  int get_current_safety_mode(void);
  int get_current_safety_param(void);

  void set_torque_meas(int min, int max);
  int get_torque_meas_min(void);
  int get_torque_meas_max(void);
  void set_torque_driver(int min, int max);
  int get_torque_driver_min(void);
  int get_torque_driver_max(void);
  void set_desired_torque_last(int t);
  void set_rt_torque_last(int t);
  void set_desired_angle_last(int t);
  int get_desired_angle_last();
  void set_angle_meas(int min, int max);
  int get_angle_meas_min(void);
  int get_angle_meas_max(void);

  bool get_cruise_engaged_prev(void);
  void set_cruise_engaged_prev(bool engaged);
  bool get_vehicle_moving(void);
  void set_timer(uint32_t t);

  void safety_tick_current_safety_config();
  bool safety_config_valid();

  void init_tests(void);

  void set_honda_fwd_brake(bool c);
  bool get_honda_fwd_brake(void);
  void set_honda_alt_brake_msg(bool c);
  void set_honda_bosch_long(bool c);
  int get_honda_hw(void);
  """)


class SafetyHelpers(Protocol):
  def set_controls_allowed(self, c: bool) -> None: ...
  def get_controls_allowed(self) -> bool: ...
  def get_longitudinal_allowed(self) -> bool: ...
  def set_alternative_experience(self, mode: int) -> None: ...
  def get_alternative_experience(self) -> int: ...
  def set_relay_malfunction(self, c: bool) -> None: ...
  def get_relay_malfunction(self) -> bool: ...
  def get_gas_pressed_prev(self) -> bool: ...
  def set_gas_pressed_prev(self, c: bool) -> None: ...
  def get_brake_pressed_prev(self) -> bool: ...
  def get_regen_braking_prev(self) -> bool: ...
  def get_steering_disengage_prev(self) -> bool: ...
  def get_acc_main_on(self) -> bool: ...
  def get_vehicle_speed_min(self) -> int: ...
  def get_vehicle_speed_max(self) -> int: ...
  def get_current_safety_mode(self) -> int: ...
  def get_current_safety_param(self) -> int: ...

  def set_torque_meas(self, min: int, max: int) -> None: ...  # noqa: A002
  def get_torque_meas_min(self) -> int: ...
  def get_torque_meas_max(self) -> int: ...
  def set_torque_driver(self, min: int, max: int) -> None: ...  # noqa: A002
  def get_torque_driver_min(self) -> int: ...
  def get_torque_driver_max(self) -> int: ...
  def set_desired_torque_last(self, t: int) -> None: ...
  def set_rt_torque_last(self, t: int) -> None: ...
  def set_desired_angle_last(self, t: int) -> None: ...
  def get_desired_angle_last(self) -> int: ...
  def set_angle_meas(self, min: int, max: int) -> None: ...  # noqa: A002
  def get_angle_meas_min(self) -> int: ...
  def get_angle_meas_max(self) -> int: ...

  def get_cruise_engaged_prev(self) -> bool: ...
  def set_cruise_engaged_prev(self, enabled: bool) -> None: ...
  def get_vehicle_moving(self) -> bool: ...
  def set_timer(self, t: int) -> None: ...

  def safety_tick_current_safety_config(self) -> None: ...
  def safety_config_valid(self) -> bool: ...

  def init_tests(self) -> None: ...

  def set_honda_fwd_brake(self, c: bool) -> None: ...
  def get_honda_fwd_brake(self) -> bool: ...
  def set_honda_alt_brake_msg(self, c: bool) -> None: ...
  def set_honda_bosch_long(self, c: bool) -> None: ...
  def get_honda_hw(self) -> int: ...


setup_safety_helpers(ffi)


class CANPacket:
  reserved: int
  bus: int
  data_len_code: int
  rejected: int
  returned: int
  extended: int
  addr: int
  data: list[int]


class LibSafety(SafetyHelpers, Protocol):
  def safety_rx_hook(self, msg: CANPacket) -> int: ...
  def safety_tx_hook(self, msg: CANPacket) -> int: ...
  def safety_fwd_hook(self, bus_num: int, addr: int) -> int: ...
  def set_safety_hooks(self, mode: int, param: int) -> int: ...


libsafety: LibSafety = ffi.dlopen(libsafety_fn)


# helpers

def make_CANPacket(addr: int, bus: int, dat):
  ret = ffi.new('CANPacket_t *')
  ret[0].extended = 1 if addr >= 0x800 else 0
  ret[0].addr = addr
  ret[0].data_len_code = LEN_TO_DLC[len(dat)]
  ret[0].bus = bus
  ret[0].data = bytes(dat)
  return ret
