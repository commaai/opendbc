import os
import subprocess
import tempfile
from pathlib import Path
from typing import Any
from cffi import FFI

from opendbc.safety import LEN_TO_DLC

libsafety_dir = os.path.dirname(os.path.abspath(__file__))


def _build_libsafety() -> str:
  """Compile libsafety.so to a temp file and return its path."""
  root = str(Path(libsafety_dir).parents[3])
  safety_c = os.path.join(libsafety_dir, "safety.c")
  safety_os = os.path.join(libsafety_dir, "safety.os")

  cflags = [
    '-Wall', '-Wextra', '-Werror', '-nostdlib', '-fno-builtin',
    '-std=gnu11', '-Wfatal-errors', '-Wno-pointer-to-int-cast',
    '-g', '-O0', '-fno-omit-frame-pointer', '-DALLOW_DEBUG',
    '-fprofile-arcs', '-ftest-coverage',
  ]
  ldflags = [
    '-fsanitize=undefined', '-fno-sanitize-recover=undefined',
    '-fprofile-arcs', '-ftest-coverage',
  ]

  fd, libsafety_so = tempfile.mkstemp(suffix='.so')
  os.close(fd)

  subprocess.check_call(['cc', '-fPIC', *cflags, '-I', root, '-c', safety_c, '-o', safety_os])
  subprocess.check_call(['cc', '-shared', safety_os, '-o', libsafety_so, *ldflags])
  return libsafety_so


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
class CANPacket:
  pass

ffi.cdef("""
bool safety_rx_hook(CANPacket_t *msg);
bool safety_tx_hook(CANPacket_t *msg);
int safety_fwd_hook(int bus_num, int addr);
int set_safety_hooks(uint16_t mode, uint16_t param);

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

void mutation_set_active_mutant(int id);
int mutation_get_active_mutant(void);
""")

# Method declarations for the CFFI-loaded C library.
# ffi.dlopen() returns an FFILibrary object whose attributes are the C functions
# declared in ffi.cdef(). We declare them here so pyrefly can type-check callers.
class LibSafety:
  def safety_rx_hook(self, msg: Any) -> bool: ...
  def safety_tx_hook(self, msg: Any) -> bool: ...
  def safety_fwd_hook(self, bus_num: int, addr: int) -> int: ...
  def set_safety_hooks(self, mode: int, param: int) -> int: ...

  def set_controls_allowed(self, c: bool | int) -> None: ...
  def get_controls_allowed(self) -> bool: ...
  def get_longitudinal_allowed(self) -> bool: ...
  def set_alternative_experience(self, mode: int) -> None: ...
  def get_alternative_experience(self) -> int: ...
  def set_relay_malfunction(self, c: bool | int) -> None: ...
  def get_relay_malfunction(self) -> bool: ...
  def get_gas_pressed_prev(self) -> bool: ...
  def set_gas_pressed_prev(self, v: bool | int) -> None: ...
  def get_brake_pressed_prev(self) -> bool: ...
  def get_regen_braking_prev(self) -> bool: ...
  def get_steering_disengage_prev(self) -> bool: ...
  def get_acc_main_on(self) -> bool: ...
  def get_vehicle_speed_min(self) -> float: ...
  def get_vehicle_speed_max(self) -> float: ...
  def get_current_safety_mode(self) -> int: ...
  def get_current_safety_param(self) -> int: ...

  def set_torque_meas(self, min_val: int, max_val: int) -> None: ...
  def get_torque_meas_min(self) -> int: ...
  def get_torque_meas_max(self) -> int: ...
  def set_torque_driver(self, min_val: int, max_val: int) -> None: ...
  def get_torque_driver_min(self) -> int: ...
  def get_torque_driver_max(self) -> int: ...
  def set_desired_torque_last(self, t: int) -> None: ...
  def set_rt_torque_last(self, t: int) -> None: ...
  def set_desired_angle_last(self, t: int) -> None: ...
  def get_desired_angle_last(self) -> int: ...
  def set_angle_meas(self, min_val: int, max_val: int) -> None: ...
  def get_angle_meas_min(self) -> int: ...
  def get_angle_meas_max(self) -> int: ...

  def get_cruise_engaged_prev(self) -> bool: ...
  def set_cruise_engaged_prev(self, engaged: bool | int) -> None: ...
  def get_vehicle_moving(self) -> bool: ...
  def set_timer(self, t: int) -> None: ...

  def safety_tick_current_safety_config(self) -> None: ...
  def safety_config_valid(self) -> bool: ...

  def init_tests(self) -> None: ...

  def set_honda_fwd_brake(self, c: bool | int) -> None: ...
  def get_honda_fwd_brake(self) -> bool: ...
  def set_honda_alt_brake_msg(self, c: bool | int) -> None: ...
  def set_honda_bosch_long(self, c: bool | int) -> None: ...
  def get_honda_hw(self) -> int: ...

  def mutation_set_active_mutant(self, id_val: int) -> None: ...
  def mutation_get_active_mutant(self) -> int: ...

# Lazy-loaded via __getattr__ -> _build_libsafety() -> load()
libsafety: LibSafety

def load(path: str) -> None:
  global libsafety
  libsafety = ffi.dlopen(str(path))  # pyrefly: ignore[bad-assignment]

def __getattr__(name: str):
  if name == "libsafety":
    load(_build_libsafety())
    return libsafety
  raise AttributeError(name)

def make_CANPacket(addr: int, bus: int, dat: bytes | bytearray) -> Any:
  ret = ffi.new('CANPacket_t *')
  ret[0].extended = 1 if addr >= 0x800 else 0
  ret[0].addr = addr
  ret[0].data_len_code = LEN_TO_DLC[len(dat)]
  ret[0].bus = bus
  ret[0].data = bytes(dat)
  return ret
