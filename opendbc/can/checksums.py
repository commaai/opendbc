# Python implementations of checksum algorithms from opendbc/can/common.cc

from typing import List, Dict, Union

# CRC table generation functions
def gen_crc_lookup_table_8(poly: int, crc_lut_list: List[int]):
    """
    Generates an 8-bit CRC lookup table.
    Equivalent to gen_crc_lookup_table<uint8_t, 8> in C++.
    """
    crc_lut_list.clear()
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ poly
            else:
                crc <<= 1
        crc_lut_list.append(crc & 0xFF)

def gen_crc_lookup_table_16(poly: int, crc_lut_list: List[int]):
    """
    Generates a 16-bit CRC lookup table.
    Equivalent to gen_crc_lookup_table<uint16_t, 16> in C++.
    """
    crc_lut_list.clear()
    for i in range(256): # C++ iterates 256 times for 16-bit LUTs as well, processing byte by byte
        crc = i << 8 # Process byte by byte, C++ shifts the input
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ poly
            else:
                crc <<= 1
        crc_lut_list.append(crc & 0xFFFF)

# Initialize CRC lookup tables
crc8_lut_8h2f: List[int] = []
gen_crc_lookup_table_8(0x2F, crc8_lut_8h2f)

crc8_lut_j1850: List[int] = []
gen_crc_lookup_table_8(0x1D, crc8_lut_j1850)

crc16_lut_xmodem: List[int] = []
gen_crc_lookup_table_16(0x1021, crc16_lut_xmodem)

# Volkswagen MQB/MEB CRC constants
volkswagen_mqb_meb_crc_constants: Dict[int, int] = {
    5: 0x00,    # SEAT_MQB_LSSIG_CHECKSUM_1
    6: 0x8D,    # SEAT_MQB_LSSIG_CHECKSUM_2
    10: 0xBC,   # VW_MQB_LSSIG_CHECKSUM
    15: 0xF9,   # VW_MQB_LSSIG_CHECKSUM_2
    23: 0x97,   # AUDI_MQB_LSSIG_CHECKSUM
    24: 0x31,   # AUDI_MQB_LSSIG_CHECKSUM_2
    30: 0xA2,   # SKODA_MQB_LSSIG_CHECKSUM
    31: 0x0E,   # SKODA_MQB_LSSIG_CHECKSUM_2
    769: 0xC4,  # VW_MEB_LSSIG_CHECKSUM
    # TODO: Add other constants if present in common.cc
    # From the provided C++ code, these seem to be the ones directly used by volkswagen_mqb_meb_checksum
    # based on the `volkswagen_mqb_meb_crc_byte_map`
}

# Checksum function implementations will follow

def honda_checksum(address: int, sig: object, d: bytes) -> int:
    s = 0
    # The C++ code iterates 4 times, which implies it's processing a 32-bit address.
    # Python's int can be arbitrarily large, so we need to handle the address bytes carefully.
    # Assuming standard CAN ID (11-bit) or extended CAN ID (29-bit),
    # it's safer to iterate based on how CAN IDs are typically handled.
    # However, the C++ code explicitly iterates 4 times.
    # Let's replicate that behavior.
    addr_temp = address
    for _ in range(4):
        s += addr_temp & 0xFF
        addr_temp >>= 8

    for byte_val in d:
        s += byte_val
    return (256 - s) & 0xFF

def toyota_checksum(address: int, sig: object, d: bytes) -> int:
    s = len(d)
    addr_temp = address
    while addr_temp > 0:
        s += addr_temp & 0xFF
        addr_temp >>= 8
    # The C++ loop is `for (size_t i = 0; i < d.size() - 1; i++)`
    # This means it iterates up to, but not including, the last byte.
    for i in range(len(d) - 1):
        s += d[i]
    return s & 0xFF

def subaru_checksum(address: int, sig: object, d: bytes) -> int:
    s = 0
    # Replicating the C++ behavior of summing 4 bytes from the address
    addr_temp = address
    for _ in range(4):
        s += addr_temp & 0xFF
        addr_temp >>= 8

    for byte_val in d:
        s += byte_val
    return s & 0xFF

def chrysler_checksum(address: int, sig: object, d: bytes) -> int:
    # address and sig are not used in the C++ implementation
    checksum = 0xFF  # seed
    poly = 0x18      # CRC-8 polynomial
    # The loop in C++ is `for (int i=0; i<len-1; i++)`, processing all but the last byte
    for byte_val in d[:-1]:
        for j in range(8):
            data_bit_is_set = (byte_val & (0x80 >> j)) != 0
            checksum_msb_is_set = (checksum & 0x80) != 0

            if data_bit_is_set ^ checksum_msb_is_set:
                checksum = (checksum << 1) ^ poly
            else:
                checksum <<= 1
            checksum &= 0xFF # Ensure checksum remains 8-bit
    return checksum

def volkswagen_mqb_meb_checksum(address: int, sig: object, d: bytes) -> int:
    # sig is not used
    try:
        crc = volkswagen_mqb_meb_crc_constants[address]
    except KeyError:
        # Fallback or error handling if address not in constants
        # The C++ code uses .at() which would throw an exception if key not found.
        # Consider raising an error or returning a default if appropriate for the application.
        # For now, let's replicate the C++ behavior of expecting the address to be in the map.
        raise ValueError(f"Volkswagen MQB/MEB checksum: Address {address} not found in CRC constants.")

    for byte_val in d:
        crc = crc8_lut_8h2f[crc ^ byte_val]
    return crc

def xor_checksum(address: int, sig: object, d: bytes) -> int:
    # address is not used
    checksum = 0
    # The sig object needs to have a 'start_bit' attribute.
    # This is used to determine which byte in the data 'd' contains the checksum itself,
    # so that byte is skipped from the XOR sum.
    # If sig is None or does not have start_bit, this will raise an AttributeError.
    # Consider adding error handling if sig structure is not guaranteed.
    checksum_byte_index = sig.start_bit // 8

    for i, byte_val in enumerate(d):
        if i == checksum_byte_index:
            continue
        checksum ^= byte_val
    return checksum

def pedal_checksum(address: int, sig: object, d: bytes) -> int:
    # address and sig are not used
    crc = 0xFF  # seed
    poly = 0xD5 # standard CRC8 polynomial (as mentioned in C++ comments)

    for byte_val in d:
        crc ^= byte_val
        for _ in range(8):
            if (crc & 0x80) != 0:
                crc = (crc << 1) ^ poly
            else:
                crc <<= 1
            crc &= 0xFF # Ensure crc remains 8-bit
    # The C++ comment states: "The output of this function is XORed with 0xFF in the CAN packer"
    # So, we return the raw CRC here. The packer should handle the final XOR.
    return crc

def hkg_can_fd_checksum(address: int, sig: object, d: bytes) -> int:
    # address and sig are not used
    crc = 0xFFFF  # seed
    # crc16_lut_xmodem should be already populated by gen_crc_lookup_table_16(0x1021, crc16_lut_xmodem)
    # at the module level.

    for byte_val in d:
        # The C++ logic: crc = (crc << 8U) ^ crc16_lut_xmodem.at((crc >> 8U) ^ x);
        # crc16_lut_xmodem in C++ is equivalent to crc_table_idx_top_byte in common.h example
        # which means index is (crc >> 8) ^ current_byte
        # The result from table is XORed with (crc << 8)
        # Python equivalent:
        idx = (crc >> 8) ^ byte_val
        crc = (crc << 8) ^ crc16_lut_xmodem[idx]
        crc &= 0xFFFF # Ensure crc remains 16-bit

    return crc

def fca_giorgio_checksum(address: int, sig: object, d: bytes) -> int:
    # address and sig are not used
    crc = 0xFF  # seed
    # crc8_lut_j1850 should be already populated by gen_crc_lookup_table_8(0x1D, crc8_lut_j1850)
    # at the module level.

    for byte_val in d:
        crc = crc8_lut_j1850[crc ^ byte_val]

    return crc ^ 0xFF # type B CRC (J1850) - final XOR with 0xFF

def tesla_checksum(address: int, sig: object, d: bytes) -> int:
    # address is not used
    checksum = 0
    # The sig object needs to have a 'start_bit' attribute.
    # This is used to determine which byte in the data 'd' contains the checksum itself,
    # so that byte is skipped from the XOR sum.
    # If sig is None or does not have start_bit, this will raise an AttributeError.
    # Consider adding error handling if sig structure is not guaranteed.
    checksum_byte_index = sig.start_bit // 8

    for i, byte_val in enumerate(d):
        if i == checksum_byte_index:
            continue
        checksum ^= byte_val
    return checksum
