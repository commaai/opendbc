"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
import unittest

from opendbc.safety.tests.common import CANPackerPanda, PandaSafetyTestBase


class GasInterceptorSafetyTest(PandaSafetyTestBase):

  INTERCEPTOR_THRESHOLD = 0

  cnt_gas_cmd = 0
  cnt_user_gas = 0

  packer: CANPackerPanda

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "GasInterceptorSafetyTest" or cls.__name__.endswith("Base"):
      cls.safety = None
      raise unittest.SkipTest

  def _interceptor_gas_cmd(self, gas: int):
    values: dict[str, float | int] = {"PEDAL_COUNTER": self.__class__.cnt_gas_cmd & 0xF}
    if gas > 0:
      values["GAS_COMMAND"] = gas * 255.
      values["GAS_COMMAND2"] = gas * 255.
    self.__class__.cnt_gas_cmd += 1
    return self.packer.make_can_msg_panda("GAS_COMMAND", 0, values)

  def _interceptor_user_gas(self, gas: int):
    values = {"INTERCEPTOR_GAS": gas, "INTERCEPTOR_GAS2": gas,
              "PEDAL_COUNTER": self.__class__.cnt_user_gas}
    self.__class__.cnt_user_gas += 1
    return self.packer.make_can_msg_panda("GAS_SENSOR", 0, values)

  # Skip non-interceptor user gas tests
  def test_prev_gas(self):
    pass

  def test_no_disengage_on_gas(self):
    pass

  def test_prev_gas_interceptor(self):
    self._rx(self._interceptor_user_gas(0x0))
    self.assertFalse(self.safety.get_gas_interceptor_prev())
    self._rx(self._interceptor_user_gas(0x1000))
    self.assertTrue(self.safety.get_gas_interceptor_prev())
    self._rx(self._interceptor_user_gas(0x0))

  def test_no_disengage_on_gas_interceptor(self):
    self.safety.set_controls_allowed(True)
    for g in range(0x1000):
      self._rx(self._interceptor_user_gas(g))
      # Test we allow lateral, but not longitudinal
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(g <= self.INTERCEPTOR_THRESHOLD, self.safety.get_longitudinal_allowed())
      # Make sure we can re-gain longitudinal actuation
      self._rx(self._interceptor_user_gas(0))
      self.assertTrue(self.safety.get_longitudinal_allowed())

  def test_allow_engage_with_gas_interceptor_pressed(self):
    self._rx(self._interceptor_user_gas(0x1000))
    self.safety.set_controls_allowed(True)
    self._rx(self._interceptor_user_gas(0x1000))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._interceptor_user_gas(0))

  def test_gas_interceptor_safety_check(self):
    for gas in np.arange(0, 4000, 100):
      for controls_allowed in [True, False]:
        self.safety.set_controls_allowed(controls_allowed)
        if controls_allowed:
          send = True
        else:
          send = gas == 0
        self.assertEqual(send, self._tx(self._interceptor_gas_cmd(gas)))
