import sys
import os
import dbcs
from dbc import dbc

def run_test(test):
  dbcpath = dbcs.DBC_PATH
  if test == "delphi":
    dbc_read = dbc(os.path.join(dbcpath, "delphi_esr2_64.dbc"))
    st = "0021e7cd0000864d".decode("hex")
    msg = 1280
  elif test == "nidec":
    dbc_read = dbc(os.path.join(dbcpath, "acura_ilx_2016_nidec.dbc"))
    st = "0118111100000000".decode("hex")
    msg = 0x301
  elif test == "steer_status":
    dbc_read = dbc(os.path.join(dbcpath, "acura_ilx_2016_can.dbc"))
    st = "00000000000000".decode("hex")
    msg = 0x18f

  print(st)
  print('here', dbc_read.msgs[msg])
  out = dbc_read.decode([msg, 0, st], debug=True)[1]
  print("out!!",out)
  encoded = dbc_read.encode(msg, out)
  print("compare")
  print(encoded.encode("hex"))
  print(st.encode("hex"))
  assert encoded == st

if __name__ == "__main__":
  map(run_test, ['delphi', 'nidec', 'steer_status'])

