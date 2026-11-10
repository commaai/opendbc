import unittest
from opendbc.can import CANParser, CANPacker
from opendbc.can.dbc import DBC, Signal
from opendbc.car.volkswagen.mqbcan import (
  volkswagen_mqb_meb_checksum,
  volkswagen_mqb_meb_dyn_len_checksum,
  volkswagen_meb_alt_crc_checksum,
)
from opendbc.car.mg.mgcan import mg_checksum
from opendbc.car.psa.psacan import psa_checksum


class TestDefensiveHardening(unittest.TestCase):
  """
  Deterministic regression test suite certifying defensive hardening:
  - Prevention of IndexError crashes on runt/empty CAN frames
  - Rejection of truncated frames (< state.size) to prevent sensor state corruption
  - Signal state preservation under corrupted or partial bus payloads
  - Safe bounds validation in DBC Big-Endian parsing and raw value bit shifting
  """

  def test_volkswagen_checksum_runt_safety(self):
    """Test VW MQB/MEB checksum routines on runt payloads (len < 2)"""
    # Empty bytearray
    empty = bytearray()
    self.assertEqual(volkswagen_mqb_meb_checksum(0x122, None, empty), 0)
    self.assertEqual(volkswagen_mqb_meb_dyn_len_checksum(0x122, None, empty, 8, [0] * 16), 0)
    self.assertEqual(volkswagen_meb_alt_crc_checksum(0x122, None, empty), 0)

    # 1-byte bytearray (previously crashed with IndexError on d[1])
    one_byte = bytearray([0x55])
    self.assertEqual(volkswagen_mqb_meb_checksum(0x122, None, one_byte), 0)
    self.assertEqual(volkswagen_mqb_meb_dyn_len_checksum(0x122, None, one_byte, 8, [0] * 16), 0)

  def test_mg_checksum_runt_safety(self):
    """Test MG checksum routine on runt payloads (< 7 bytes on 0x1EC)"""
    empty = bytearray()
    self.assertEqual(mg_checksum(0x1EC, None, empty), 0)

    # 6-byte payload (missing byte 6 which previously crashed with IndexError on d[6])
    six_bytes = bytearray([0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC])
    self.assertEqual(mg_checksum(0x1EC, None, six_bytes), 0)

  def test_psa_checksum_runt_safety(self):
    """Test PSA checksum routine when buffer is shorter than sig.start_bit // 8"""
    dummy_sig = Signal("CHECKSUM", start_bit=56, msb=56, lsb=56, size=4,
                       is_signed=False, factor=1.0, offset=0.0, is_little_endian=True)
    # Payload shorter than start_bit // 8 (byte 7)
    short_payload = bytearray(b"\x01\x02\x03")
    self.assertEqual(psa_checksum(0x452, dummy_sig, short_payload), 0)
    self.assertEqual(psa_checksum(0x452, dummy_sig, bytearray()), 0)

  def test_truncated_can_frame_rejection_and_state_preservation(self):
    """
    Ensure that CANParser rejects truncated frames (< state.size)
    and prevents corrupting internal signal values (vl and vl_all).
    """
    dbc_name = "honda_civic_touring_2016_can_generated"
    # HUD_SETTING has size=5 and contains IMPERIAL_UNIT without counter/checksum
    parser = CANParser(dbc_name, [("HUD_SETTING", 0)], 0)
    packer = CANPacker(dbc_name)

    # 1. Establish valid baseline state: IMPERIAL_UNIT = 1.0
    valid_msg = packer.make_can_msg("HUD_SETTING", 0, {"IMPERIAL_UNIT": 1})
    updated = parser.update([(0, [valid_msg])])
    self.assertIn(493, updated)
    self.assertEqual(parser.vl["HUD_SETTING"]["IMPERIAL_UNIT"], 1.0)
    self.assertEqual(parser.vl_all["HUD_SETTING"]["IMPERIAL_UNIT"], [1.0])

    # 2. Inject truncated frames of varying runt lengths: 0, 1, 2, 4 bytes (expected size=5)
    for runt_len in (0, 1, 2, 4):
      runt_data = bytes(range(runt_len))
      updated = parser.update([(1_000_000_000 * (runt_len + 1), [(493, runt_data, 0)])])

      # Must NOT report as updated
      self.assertEqual(len(updated), 0, f"Failed: accepted truncated frame of length {runt_len}")
      # Must PRESERVE previous valid signal value
      self.assertEqual(parser.vl["HUD_SETTING"]["IMPERIAL_UNIT"], 1.0)
      # vl_all for current step must be empty (no corrupt history)
      self.assertEqual(parser.vl_all["HUD_SETTING"]["IMPERIAL_UNIT"], [])

  def test_vw_mqb_parser_runt_resilience(self):
    """Test that VW MQB parser handles runt frames without crashing or corrupting state"""
    dbc_name = "vw_mqb"
    parser = CANParser(dbc_name, [("ACC_06", 0)], 0)

    # Empty frame (len 0) and 1-byte frame (len 1) must be dropped safely
    updated_empty = parser.update([(0, [(0x122, b"", 0)])])
    self.assertEqual(len(updated_empty), 0)

    updated_runt = parser.update([(1_000_000_000, [(0x122, b"\x01", 0)])])
    self.assertEqual(len(updated_runt), 0)

  def test_dbc_big_endian_bounds_validation(self):
    """Test that DBC parser defensively catches invalid Big-Endian bit layouts"""
    malformed_dbc = """
VERSION ""
BO_ 100 TEST_MSG: 8 Vector__XXX
 SG_ OUT_OF_BOUNDS_BE : 600|8@0+ (1,0) [0|255] "" Vector__XXX
"""
    with self.assertRaises(ValueError) as ctx:
      dbc_cls = getattr(DBC, "__wrapped__", DBC)
      dbc = dbc_cls.__new__(dbc_cls)
      dbc._parse_content("test_malformed", malformed_dbc)
    self.assertIn("Invalid Big-Endian bit layout", str(ctx.exception))

  def test_signal_raw_value_bounds_safety(self):
    """Test Signal.get_raw_value with inverted bounds (lsb > msb)"""
    inverted_sig = Signal("INV", start_bit=10, msb=5, lsb=8, size=4,
                          is_signed=False, factor=1.0, offset=0.0, is_little_endian=True)
    val = inverted_sig.get_raw_value(b"\xff\xff")
    self.assertIsInstance(val, int)
    self.assertEqual(val, 15)


if __name__ == "__main__":
  unittest.main()
