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


def create_steer_and_ap_stalk(packer, steer_msg, apply_torque, bus=0):
    """
    Copy STEER_AND_AP_STALK message from bus 0 and forward to bus 2,
    modifying only the STEERING_TORQUE
    """
    values = {
        'STEERING_ANGLE': steer_msg['STEERING_ANGLE'],
        'STEERING_DIRECTION': steer_msg['STEERING_DIRECTION'],
        'STEERING_TORQUE': apply_torque,
        'EPS_ACTUATING': steer_msg['EPS_ACTUATING'],
        'AP_REDUCE_DISTANCE_COMMAND': steer_msg['AP_REDUCE_DISTANCE_COMMAND'],
        'AP_INCREASE_DISTANCE_COMMAND': steer_msg['AP_INCREASE_DISTANCE_COMMAND'],
        'AP_CANCEL_COMMAND': steer_msg['AP_CANCEL_COMMAND'],
        'AP_ENABLE_COMMAND': steer_msg['AP_ENABLE_COMMAND'],
        'AP_DECREASE_SPEED_COMMAND': steer_msg['AP_DECREASE_SPEED_COMMAND'],
        'AP_INCREASE_SPEED_COMMAND': steer_msg['AP_INCREASE_SPEED_COMMAND'],
        'COUNTER': steer_msg['COUNTER'],
    }
    return packer.make_can_msg('STEER_AND_AP_STALK', bus, values)
