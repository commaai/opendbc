"""AUTOSAR E2E checksum for the TSS 3.0 camera ACC request (0x160).

Unlike the SecOC path in secoc.py (a keyed AES-CMAC over 8-byte messages), the
32-byte 0x160 on the CAN FD ADAS bus is protected by a plain, keyless E2E
CRC-16 plus a rolling counter. Parameters were recovered by brute force and
verified on 26,290 logged frames across two sessions (100%):

    CRC-16/CCITT   poly 0x1021   init 0x0000   xorout 0x0000   no reflection
    computed over  payload[2:] + DataID(0x444A, little-endian)
    stored in bytes 0-1, little-endian:  b0 = crc & 0xff, b1 = crc >> 8
    counter in byte 2, +1 per frame mod 256

Because it is keyless, openpilot can regenerate a valid 0x160 after modifying
the acceleration request -- no key extraction required. See port NOTES.md.
"""

E2E_160_DATA_ID = 0x444A   # camera ACC request  0x160
E2E_1A0_DATA_ID = 0xBEA8   # LTA steering command 0x1A0 (verified 1200/1200 frames)


def _crc16_ccitt(data: bytes) -> int:
  reg = 0x0000
  for b in data:
    reg ^= b << 8
    for _ in range(8):
      reg = ((reg << 1) ^ 0x1021) & 0xFFFF if reg & 0x8000 else (reg << 1) & 0xFFFF
  return reg & 0xFFFF


def e2e_160_checksum(payload: bytes, data_id: int = E2E_160_DATA_ID) -> int:
  """CRC over everything after the 2-byte checksum field, with DataID appended."""
  return _crc16_ccitt(bytes(payload[2:]) + bytes([data_id & 0xFF, (data_id >> 8) & 0xFF]))


def apply_e2e_160(payload: bytes, counter: int | None = None) -> bytes:
  """Return payload with byte 2 counter (if given) and bytes 0-1 CRC set."""
  buf = bytearray(payload)
  if counter is not None:
    buf[2] = counter & 0xFF
  crc = e2e_160_checksum(buf)
  buf[0] = crc & 0xFF
  buf[1] = (crc >> 8) & 0xFF
  return bytes(buf)


def e2e_160_valid(payload: bytes) -> bool:
  """Check a received frame's checksum -- used to validate the template."""
  want = (payload[1] << 8) | payload[0]
  return e2e_160_checksum(payload) == want


# ---- generic helpers (0x1A0 steering command reuses the exact same E2E scheme
#      as 0x160, only the Data ID differs) --------------------------------------
def e2e_checksum(payload: bytes, data_id: int) -> int:
  return _crc16_ccitt(bytes(payload[2:]) + bytes([data_id & 0xFF, (data_id >> 8) & 0xFF]))


def apply_e2e(payload: bytes, data_id: int, counter: int | None = None) -> bytes:
  buf = bytearray(payload)
  if counter is not None:
    buf[2] = counter & 0xFF
  crc = e2e_checksum(buf, data_id)
  buf[0] = crc & 0xFF
  buf[1] = (crc >> 8) & 0xFF
  return bytes(buf)


def e2e_valid(payload: bytes, data_id: int) -> bool:
  return e2e_checksum(payload, data_id) == ((payload[1] << 8) | payload[0])


def apply_e2e_1a0(payload: bytes, counter: int | None = None) -> bytes:
  return apply_e2e(payload, E2E_1A0_DATA_ID, counter)


def e2e_1a0_valid(payload: bytes) -> bool:
  return e2e_valid(payload, E2E_1A0_DATA_ID)
