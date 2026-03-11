import glob
import os

DBC_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dbc')

# -I include path for e.g. "#include <opendbc/safety/safety.h>"
INCLUDE_PATH = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../"))

_dbc_generated = False

def ensure_dbc_generated():
  """Generate *_generated.dbc files on demand if they don't exist."""
  global _dbc_generated
  if _dbc_generated:
    return
  _dbc_generated = True
  if not glob.glob(os.path.join(DBC_PATH, '*_generated.dbc')):
    from opendbc.dbc.generator.generator import create_all
    create_all(DBC_PATH)
