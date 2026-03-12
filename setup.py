import os
import sys

from setuptools import setup

ext_modules = []

# Only compile with mypyc when explicitly requested via MYPYC=1
if os.environ.get('MYPYC') == '1' and 'build_ext' in sys.argv:
  from mypyc.build import mypycify
  ext_modules = mypycify([
    'opendbc/can/_types.py',
    'opendbc/can/_checksums.py',
    'opendbc/can/packer.py',
    'opendbc/can/parser.py',
  ], opt_level='3')

setup(ext_modules=ext_modules)
