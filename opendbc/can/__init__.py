from opendbc.can.parser_pyx import CANParser, CANDefine
assert CANParser, CANDefine

try:
  from opendbc.can.packer import CANPacker
except Exception:  # fallback to compiled version if available
  from opendbc.can.packer_pyx import CANPacker
assert CANPacker
