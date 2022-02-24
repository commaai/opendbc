import os
import unittest

from numpy.testing import assert_almost_equal

from opendbc import DBC_PATH
from opendbc.can.dbc import dbc
from opendbc.can.packer import CANPacker


def assert_message_equal(msg1, msg2, decimal=10):
  for key in msg1[1]:
    assert_almost_equal(msg1[1][key], msg2[1][key], decimal=decimal)
  return True


class TestPythonDBC(unittest.TestCase):
  def test_toyota(self):
    dbc_test = dbc(os.path.join(DBC_PATH, 'toyota_nodsu_pt_generated.dbc'))
    packer_test = CANPacker('toyota_nodsu_pt_generated')

    msg = ('STEER_ANGLE_SENSOR', {'STEER_ANGLE': -6.0, 'STEER_RATE': 4, 'STEER_FRACTION': -0.2})
    encoded = dbc_test.encode(*msg)
    assert encoded == packer_test.make_can_msg(msg[0], 0, msg[1])[2]

    decoded = dbc_test.decode((0x25, 0, encoded))
    assert decoded == msg

    msg = ('WHEEL_SPEEDS', {'WHEEL_SPEED_FR': 100.00, 'WHEEL_SPEED_FL': 100.00,
                            'WHEEL_SPEED_RR': 166.99, 'WHEEL_SPEED_RL': 166.99})
    encoded = dbc_test.encode(*msg)
    assert encoded == packer_test.make_can_msg(msg[0], 0, msg[1])[2]

    decoded = dbc_test.decode((0xAA, 0, encoded))
    assert_message_equal(decoded, msg, decimal=10)

  def test_hyundai(self):
    # Test Little Endian
    dbc_test = dbc(os.path.join(DBC_PATH, 'hyundai_2015_ccan.dbc'))
    packer_test = CANPacker('hyundai_2015_ccan')
    decoded = dbc_test.decode((0x2b0, 0, b'\xfa\xfe\x00\x07\x12'))
    assert abs(decoded[1]['SAS_Angle'] - (-26.2)) < 0.001

    msg = ('SAS11', {'SAS_Stat': 7.0, 'MsgCount': 0.0, 'SAS_Angle': -26.200000000000003, 'SAS_Speed': 0.0, 'CheckSum': 0.0})
    encoded = dbc_test.encode(*msg)
    assert encoded == packer_test.make_can_msg(msg[0], 0, msg[1])[2]

    decoded = dbc_test.decode((0x2b0, 0, encoded))
    assert decoded == msg


if __name__ == "__main__":
  unittest.main()
