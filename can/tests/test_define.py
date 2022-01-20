#!/usr/bin/env python3
import unittest

from opendbc.can.can_define import CANDefine


class TestCADNDefine(unittest.TestCase):
  def test_civic(self):
    dbc_file = "honda_civic_touring_2016_can_generated"
    defs = CANDefine(dbc_file)

    self.assertDictEqual(defs.dv[399], defs.dv['STEER_STATUS'])
    self.assertDictEqual(defs.dv[399]['STEER_STATUS'],
                         {6: 'TMP_FAULT', 'TMP_FAULT': 6,
                          5: 'FAULT_1', 'FAULT_1': 5,
                          4: 'NO_TORQUE_ALERT_2', 'NO_TORQUE_ALERT_2': 4,
                          3: 'LOW_SPEED_LOCKOUT', 'LOW_SPEED_LOCKOUT': 3,
                          2: 'NO_TORQUE_ALERT_1', 'NO_TORQUE_ALERT_1': 2,
                          0: 'NORMAL', 'NORMAL': 0,
                          }
                         )


if __name__ == "__main__":
  unittest.main()
