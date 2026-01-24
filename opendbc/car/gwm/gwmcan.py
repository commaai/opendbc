def gwm_checksum(address: int, sig, d: bytearray) -> int:
    crc = 0x00
    poly = 0x1D
    xor_out = 0x2D

    for byte in d[1:]:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ poly
            else:
                crc = (crc << 1)
            crc &= 0xFF
    return crc ^ xor_out


def create_helloworld(packer, bus=0):
    values = {
        'STEERING_TORQUE': 2000,
    }
    return packer.make_can_msg('STEER_AND_AP_STALK', bus, values)
