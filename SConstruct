import os
import subprocess
import sysconfig
import numpy as np

zmq = 'zmq'
arch = subprocess.check_output(["uname", "-m"], encoding='utf8').rstrip()

cereal_dir = Dir('.')

python_path = sysconfig.get_paths()['include']
cpppath = [
  '#',
  '/usr/lib/include',
  python_path
]

AddOption('--test',
          action='store_true',
          help='build test files')

AddOption('--asan',
          action='store_true',
          help='turn on ASAN')

AddOption('--ubsan',
          action='store_true',
          help='turn on UBSan')


ccflags = []
ldflags = []
if GetOption('ubsan'):
  flags = [
    "-fsanitize=undefined",
    "-fno-sanitize-recover=undefined",
  ]
  ccflags += flags
  ldflags += flags
elif GetOption('asan'):
  ccflags += ["-fsanitize=address", "-fno-omit-frame-pointer"]
  ldflags += ["-fsanitize=address"]

env = Environment(
  ENV=os.environ,
  CC='clang',
  CXX='clang++',
  CCFLAGS=[
    "-g",
    "-fPIC",
    "-O2",
    "-Wunused",
    "-Werror",
    "-Wshadow",
  ] + ccflags,
  LDFLAGS=ldflags,
  LINKFLAGS=ldflags,
  LIBPATH=[
    "#opendbc/can/",
  ],
  CFLAGS="-std=gnu11",
  CXXFLAGS=["-std=c++1z"],
  CPPPATH=cpppath,
  CYTHONCFILESUFFIX=".cpp",
  tools=["default", "cython"]
)

common = ''
Export('env', 'zmq', 'arch', 'common')

cereal = [File('#cereal/libcereal.a')]
messaging = [File('#cereal/libmessaging.a')]
Export('cereal', 'messaging')


envCython = env.Clone()
envCython["CPPPATH"] += [np.get_include()]
envCython["CCFLAGS"] += ["-Wno-#warnings", "-Wno-shadow", "-Wno-deprecated-declarations"]

python_libs = []
if arch == "Darwin":
  envCython["LINKFLAGS"] += ["-bundle", "-undefined", "dynamic_lookup"]
elif arch == "aarch64":
  envCython["LINKFLAGS"] += ["-shared"]

  python_libs.append(os.path.basename(python_path))
else:
  envCython["LINKFLAGS"] += ["-pthread", "-shared"]

envCython["LIBS"] = python_libs

Export('envCython')


SConscript(['cereal/SConscript'])
SConscript(['opendbc/can/SConscript'])
