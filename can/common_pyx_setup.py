import os
import subprocess
import sysconfig
import platform
from distutils.core import Extension, setup  # pylint: disable=import-error,no-name-in-module

from Cython.Build import cythonize
from Cython.Distutils import build_ext


ANNOTATE = os.getenv('ANNOTATE') is not None
BASEDIR = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../"))

CROSS_COMPILATION = os.getenv("CROSS_COMPILATION") is not None
sysroot_args=[]

if CROSS_COMPILATION:
  os.environ['CC'] = 'aarch64-linux-gnu-gcc'
  os.environ['CXX'] = 'aarch64-linux-gnu-g++'
  os.environ['LDSHARED'] = 'aarch64-linux-gnu-gcc -shared'
  os.environ['LDCXXSHARED'] = 'aarch64-linux-gnu-g++ -shared'
  os.environ["LD_LIBRARY_PATH"] = "/usr/aarch64-linux-gnu/lib"
  sysroot_args=['--sysroot', '/usr/aarch64-linux-gnu']

def get_ext_filename_without_platform_suffix(filename):
  name, ext = os.path.splitext(filename)
  ext_suffix = sysconfig.get_config_var('EXT_SUFFIX')

  if ext_suffix == ext:
    return filename

  ext_suffix = ext_suffix.replace(ext, '')
  idx = name.find(ext_suffix)

  if idx == -1:
    return filename
  else:
    return name[:idx] + ext


class BuildExtWithoutPlatformSuffix(build_ext):
  def get_ext_filename(self, ext_name):
    filename = super().get_ext_filename(ext_name)
    return get_ext_filename_without_platform_suffix(filename)


extra_compile_args = ["-std=c++14"]
ARCH = subprocess.check_output(["uname", "-m"], encoding='utf8').rstrip()  # pylint: disable=unexpected-keyword-arg
if ARCH == "aarch64":
  extra_compile_args += ["-Wno-deprecated-register"]

if platform.system() == "Darwin":
  libdbc = "libdbc.dylib"
else:
  libdbc = "libdbc.so"

if CROSS_COMPILATION:
  extra_compile_args += ["-Wno-deprecated-register"]
  extra_compile_args += sysroot_args
  extra_link_args = ['-L/usr/aarch64-linux-gnu/lib']
  extra_link_args += ['--prefix=$HOME/linker_bin/']
else:
  extra_link_args = [os.path.join(BASEDIR, 'opendbc', 'can', libdbc)]
  
include_dirs = [
  BASEDIR,
  os.path.join(BASEDIR, 'phonelibs'),
]

# Build CAN Parser

setup(name='CAN parser',
      cmdclass={'build_ext': BuildExtWithoutPlatformSuffix},
      ext_modules=cythonize(
        Extension(
          "parser_pyx",
          language="c++",
          sources=['parser_pyx.pyx'],
          extra_compile_args=extra_compile_args,
          include_dirs=include_dirs,
          extra_link_args=extra_link_args,
          libraries=[':libdbc.so'],
          library_dirs=['.'],
        ),
        nthreads=4,
        annotate=ANNOTATE
      ),
)

if platform.system() == "Darwin":
  os.system("install_name_tool -change opendbc/can/libdbc.dylib " + BASEDIR + "/opendbc/can/libdbc.dylib parser_pyx.so")


# Build CAN Packer

setup(name='CAN packer',
      cmdclass={'build_ext': BuildExtWithoutPlatformSuffix},
      ext_modules=cythonize(
        Extension(
          "packer_pyx",
          language="c++",
          sources=['packer_pyx.pyx'],
          extra_compile_args=extra_compile_args,
          include_dirs=include_dirs,
          extra_link_args=extra_link_args,
        ),
        nthreads=4,
        annotate=ANNOTATE
      ),
)

if platform.system() == "Darwin":
  os.system("install_name_tool -change opendbc/can/libdbc.dylib " + BASEDIR + "/opendbc/can/libdbc.dylib packer_pyx.so")
