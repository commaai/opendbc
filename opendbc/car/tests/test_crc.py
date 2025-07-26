from opendbc.car.crc import _gen_crc8_table, _gen_crc16_table, CRC8H2F, CRC8J1850, CRC16_XMODEM


class TestCrc:
  def test_gen_crc8_table_structure(self):
    """Test CRC8 table generation produces correct structure"""
    poly = 0x2F
    table = _gen_crc8_table(poly)
    
    # Should have 256 entries
    assert len(table) == 256
    
    # All entries should be 8-bit values
    for entry in table:
      assert 0 <= entry <= 0xFF
      assert isinstance(entry, int)

  def test_gen_crc8_table_deterministic(self):
    """Test CRC8 table generation is deterministic"""
    poly = 0x2F
    table1 = _gen_crc8_table(poly)
    table2 = _gen_crc8_table(poly)
    
    # Should produce identical tables
    assert table1 == table2

  def test_gen_crc8_table_different_polynomials(self):
    """Test different polynomials produce different tables"""
    table1 = _gen_crc8_table(0x2F)
    table2 = _gen_crc8_table(0x1D)
    
    # Different polynomials should produce different tables
    assert table1 != table2

  def test_gen_crc8_table_known_values(self):
    """Test CRC8 table generation against known values"""
    # Test H2F polynomial (0x2F)
    table = _gen_crc8_table(0x2F)
    
    # First few entries should match expected values for this polynomial
    assert table[0] == 0x00  # CRC of 0 should be 0
    # Additional spot checks could be added with known CRC8-H2F values
    
  def test_gen_crc16_table_structure(self):
    """Test CRC16 table generation produces correct structure"""
    poly = 0x1021
    table = _gen_crc16_table(poly)
    
    # Should have 256 entries
    assert len(table) == 256
    
    # All entries should be 16-bit values
    for entry in table:
      assert 0 <= entry <= 0xFFFF
      assert isinstance(entry, int)

  def test_gen_crc16_table_deterministic(self):
    """Test CRC16 table generation is deterministic"""
    poly = 0x1021
    table1 = _gen_crc16_table(poly)
    table2 = _gen_crc16_table(poly)
    
    # Should produce identical tables
    assert table1 == table2

  def test_gen_crc16_table_known_values(self):
    """Test CRC16 table generation against known values"""
    # Test XMODEM polynomial (0x1021)
    table = _gen_crc16_table(0x1021)
    
    # First entry should be 0
    assert table[0] == 0x0000
    # XMODEM CRC16 table should have specific known values
    assert table[1] == 0x1021  # Known value for XMODEM polynomial

  def test_predefined_crc8_tables_exist(self):
    """Test predefined CRC8 tables are generated"""
    # CRC8H2F should be generated with 0x2F polynomial
    assert len(CRC8H2F) == 256
    assert CRC8H2F == _gen_crc8_table(0x2F)
    
    # CRC8J1850 should be generated with 0x1D polynomial
    assert len(CRC8J1850) == 256
    assert CRC8J1850 == _gen_crc8_table(0x1D)

  def test_predefined_crc16_tables_exist(self):
    """Test predefined CRC16 tables are generated"""
    # CRC16_XMODEM should be generated with 0x1021 polynomial
    assert len(CRC16_XMODEM) == 256
    assert CRC16_XMODEM == _gen_crc16_table(0x1021)

  def test_crc8_table_bit_operations(self):
    """Test CRC8 table generation bit manipulation logic"""
    # Test the core algorithm with a simple case
    poly = 0xFF  # All bits set for testing
    table = _gen_crc8_table(poly)
    
    # Verify the algorithm works correctly
    assert len(table) == 256
    
    # Test edge cases
    assert table[0] == 0x00  # Input 0 should give CRC 0
    assert table[0x80] != table[0x40]  # Different high bits should give different results

  def test_crc16_table_bit_operations(self):
    """Test CRC16 table generation bit manipulation logic"""
    # Test with a simple polynomial
    poly = 0xFFFF  # All bits set for testing
    table = _gen_crc16_table(poly)
    
    # Verify the algorithm works correctly
    assert len(table) == 256
    
    # Test edge cases
    assert table[0] == 0x0000  # Input 0 should give CRC 0
    assert table[0x80] != table[0x40]  # Different high bits should give different results

  def test_crc_tables_automotive_polynomials(self):
    """Test that automotive standard polynomials are correctly implemented"""
    # H2F is used in some automotive applications
    h2f_table = _gen_crc8_table(0x2F)
    assert h2f_table == CRC8H2F
    
    # J1850 is used in automotive diagnostics
    j1850_table = _gen_crc8_table(0x1D)
    assert j1850_table == CRC8J1850
    
    # XMODEM CRC16 is commonly used in automotive communications
    xmodem_table = _gen_crc16_table(0x1021)
    assert xmodem_table == CRC16_XMODEM

  def test_crc8_table_coverage_full_range(self):
    """Test CRC8 table covers full 8-bit input range"""
    table = CRC8H2F
    
    # Verify all possible 8-bit inputs are covered
    for i in range(256):
      crc_val = table[i]
      assert 0 <= crc_val <= 0xFF
    
    # Should have some variety in outputs (not all same value)
    unique_values = len(set(table))
    assert unique_values > 1  # Should not all be the same value

  def test_crc16_table_coverage_full_range(self):
    """Test CRC16 table covers full input range correctly"""
    table = CRC16_XMODEM
    
    # Verify all 256 table entries are valid 16-bit values
    for i in range(256):
      crc_val = table[i]
      assert 0 <= crc_val <= 0xFFFF
    
    # Should have variety in outputs
    unique_values = len(set(table))
    assert unique_values > 1  # Should not all be the same value