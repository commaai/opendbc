import os
import subprocess
from pathlib import Path

from cffi import FFI

from opendbc.safety import LEN_TO_DLC

libsafety_dir = os.path.dirname(os.path.abspath(__file__))


def _needs_rebuild():
  """Check if libsafety.so needs to be (re)built by comparing mtimes against source files."""
  libsafety_so = os.path.join(libsafety_dir, "libsafety.so")
  if not os.path.exists(libsafety_so):
    return True

  so_mtime = os.path.getmtime(libsafety_so)
  safety_dir = str(Path(libsafety_dir).parents[1])
  source_files = [os.path.join(libsafety_dir, "safety.c")]
  for dirpath, _, filenames in os.walk(safety_dir):
    for f in filenames:
      if f.endswith('.h'):
        source_files.append(os.path.join(dirpath, f))
  return any(os.path.getmtime(f) > so_mtime for f in source_files)


def _build_libsafety():
  """Compile libsafety.so on demand if sources have changed."""
  if not _needs_rebuild():
    return

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

  subprocess.check_call(['cc', '-fPIC', *cflags, '-I', root, '-c', safety_c, '-o', safety_os])
  libsafety_so = os.path.join(libsafety_dir, "libsafety.so")
  subprocess.check_call(['cc', '-shared', safety_os, '-o', libsafety_so, *ldflags])


_build_libsafety()

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

class LibSafety:
  pass
libsafety: LibSafety = ffi.dlopen(os.path.join(libsafety_dir, "libsafety.so"))

def load(path):
  global libsafety
  libsafety = ffi.dlopen(str(path))

def make_CANPacket(addr: int, bus: int, dat):
  ret = ffi.new('CANPacket_t *')
  ret[0].extended = 1 if addr >= 0x800 else 0
  ret[0].addr = addr
  ret[0].data_len_code = LEN_TO_DLC[len(dat)]
  ret[0].bus = bus
  ret[0].data = bytes(dat)
  return ret
