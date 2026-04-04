import importlib.util
import os
import subprocess
import sysconfig
import tempfile
from pathlib import Path

from opendbc.safety import LEN_TO_DLC

libsafety_dir = os.path.dirname(os.path.abspath(__file__))


def _build_libsafety() -> str:
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


def _build_extension():
  root = str(Path(libsafety_dir).parents[3])
  ext_suffix = sysconfig.get_config_var("EXT_SUFFIX") or ".so"
  include_flags = sysconfig.get_config_var("INCLUDEPY")
  platinclude = sysconfig.get_config_var("PLATINCLUDE")
  source = os.path.join(libsafety_dir, "_libsafety_c_ext.c")

  fd, ext_path = tempfile.mkstemp(suffix=ext_suffix)
  os.close(fd)

  cmd = ['cc', '-shared', '-fPIC', '-O3', '-I', root]
  if include_flags:
    cmd.extend(['-I', include_flags])
  if platinclude and platinclude != include_flags:
    cmd.extend(['-I', platinclude])
  cmd.extend([source, '-o', ext_path])
  subprocess.check_call(cmd)
  return ext_path


def _load_extension():
  spec = importlib.util.spec_from_file_location("_libsafety_c_ext", _build_extension())
  if spec is None or spec.loader is None:
    raise ImportError("failed to load libsafety C extension")
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


_ext = _load_extension()
CANPacket = _ext.CANPacket
LibSafety = _ext.LibSafety
libsafety: LibSafety


def load(path):
  global libsafety
  libsafety = _ext.load(str(path))


def __getattr__(name):
  if name == "libsafety":
    load(_build_libsafety())
    return libsafety
  raise AttributeError(name)


def make_CANPacket(addr: int, bus: int, dat):
  return _ext.make_CANPacket(addr, bus, LEN_TO_DLC[len(dat)], bytes(dat))
