import importlib
import unittest

from opendbc.car import DT_CTRL, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.values import PLATFORMS
from opendbc.testing import parameterized_class, fuzzy_test

ACCEL_BRANDS = ('ford', 'honda', 'hyundai', 'toyota', 'volkswagen')
EXTREME_ACCEL = 100.
EXTREME_TORQUE = 1000.

ACCEL_FUZZ_VALUES = (EXTREME_ACCEL, -EXTREME_ACCEL, EXTREME_ACCEL / 10, -EXTREME_ACCEL / 10,
                     4., -4., 3.5, -3.5, 2., -2., 1., 0., 1e-3)
TORQUE_FUZZ_VALUES = (EXTREME_TORQUE, -EXTREME_TORQUE, 10., -10., 1., -1., .5, 0.)
V_EGO_FUZZ_VALUES = (0., 5., 10., 20., 30.)
FRAMES = 30


@parameterized_class('car_model', [(c,) for c in sorted(PLATFORMS)])
class TestActuationLimits(unittest.TestCase):
  car_model: str

  @classmethod
  def setUpClass(cls):
    if 'car_model' not in cls.__dict__:
      raise unittest.SkipTest('Base class')

    CarInterface = interfaces[cls.car_model]
    cls.CP = CarInterface.get_non_essential_params(cls.car_model)

    if cls.CP.notCar:
      raise unittest.SkipTest('notCar')

    cls.car_interface = CarInterface(cls.CP)
    values = importlib.import_module(f'opendbc.car.{cls.CP.brand}.values')
    ccp = getattr(values, 'CarControllerParams', None)
    if ccp is not None:
      try:
        ccp = ccp(cls.CP)
      except TypeError:
        ccp = ccp()

    if ccp is None:
      cls.accel_limits = None
      cls.steer_max = None
    else:
      if cls.CP.brand == 'honda':
        cls.accel_limits = (ccp.BOSCH_ACCEL_MIN, ccp.BOSCH_ACCEL_MAX)
      elif hasattr(ccp, 'ACCEL_MIN'):
        cls.accel_limits = (ccp.ACCEL_MIN, ccp.ACCEL_MAX)
      else:
        cls.accel_limits = None
      # some brands limit torque by speed, so the bound is the largest allowed value
      lookup = getattr(ccp, 'STEER_MAX_LOOKUP', None)
      cls.steer_max = max(lookup[1]) if lookup is not None else getattr(ccp, 'STEER_MAX', None)
      # the normalized torque divides by the current limit, and the rate limiter carries an
      # above limit command for a few frames after the limit drops
      cls.torque_bound = max(lookup[1]) / min(lookup[1]) if lookup is not None else 1.

    cls.inactive_gas = getattr(ccp, 'INACTIVE_GAS', None) if ccp is not None else None

    cls.checks_accel = cls.CP.brand in ACCEL_BRANDS and cls.accel_limits is not None
    cls.checks_torque = cls.CP.steerControlType == 'torque' and cls.CP.brand != 'honda' and cls.steer_max is not None

  def command_extreme_accel(self, accel: float, v_ego: float) -> None:
    for frame in range(FRAMES):
      cc = structs.CarControl.new_message()
      cc.enabled = True
      cc.latActive = True
      cc.longActive = True
      cc.actuators.accel = accel
      cc.actuators.longControlState = structs.CarControl.Actuators.LongControlState.pid

      self.car_interface.update([])
      self.car_interface.CS.out.vEgo = v_ego
      self.car_interface.CS.out.vEgoRaw = v_ego
      actuators, _ = self.car_interface.apply(cc.as_reader(), now_nanos=frame * DT_CTRL * 1e9)

      accel_min, accel_max = self.accel_limits
      assert accel_min <= actuators.accel <= accel_max, \
        f"{self.car_model}: commanded accel {actuators.accel} outside [{accel_min}, {accel_max}]"

      if self.CP.brand == 'ford':
        gas = actuators.gas
        assert gas == self.inactive_gas or accel_min <= gas <= accel_max, \
          f"{self.car_model}: commanded gas {gas} outside [{accel_min}, {accel_max}]"

  def command_extreme_torque(self, torque: float, v_ego: float) -> None:
    for frame in range(FRAMES):
      cc = structs.CarControl.new_message()
      cc.enabled = True
      cc.latActive = True
      cc.actuators.torque = torque

      self.car_interface.update([])
      self.car_interface.CS.out.vEgo = v_ego
      self.car_interface.CS.out.vEgoRaw = v_ego
      actuators, _ = self.car_interface.apply(cc.as_reader(), now_nanos=frame * DT_CTRL * 1e9)

      assert -self.torque_bound <= actuators.torque <= self.torque_bound, \
        f"{self.car_model}: commanded torque {actuators.torque} outside [-{self.torque_bound}, {self.torque_bound}]"

      steer_max = self.steer_max
      assert -steer_max <= actuators.torqueOutputCan <= steer_max, \
        f"{self.car_model}: commanded torqueOutputCan {actuators.torqueOutputCan} outside [-{steer_max}, {steer_max}]"

  @fuzzy_test(max_examples=25)
  def test_accel_within_limits(self, fuzzy):
    if not self.checks_accel:
      self.skipTest('brand does not command accel')
    accel = fuzzy.choice(ACCEL_FUZZ_VALUES)
    v_ego = fuzzy.choice(V_EGO_FUZZ_VALUES)
    self.command_extreme_accel(accel, v_ego)

  @fuzzy_test(max_examples=25)
  def test_torque_within_limits(self, fuzzy):
    if not self.checks_torque:
      self.skipTest('brand does not command a normalized torque')
    torque = fuzzy.choice(TORQUE_FUZZ_VALUES)
    v_ego = fuzzy.choice(V_EGO_FUZZ_VALUES)
    self.command_extreme_torque(torque, v_ego)
