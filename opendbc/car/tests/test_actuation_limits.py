import importlib
import unittest

from opendbc.car import DT_CTRL, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.values import PLATFORMS
from opendbc.testing import parameterized_class, fuzzy_test

ACCEL_BRANDS = ('ford', 'honda', 'hyundai', 'toyota', 'volkswagen')
EXTREME_ACCEL = 100.

ACCEL_FUZZ_VALUES = (EXTREME_ACCEL, -EXTREME_ACCEL, EXTREME_ACCEL / 10, -EXTREME_ACCEL / 10,
                     4., -4., 3.5, -3.5, 2., -2., 1., 0., 1e-3)
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
    if cls.CP.brand not in ACCEL_BRANDS:
      raise unittest.SkipTest(f'{cls.CP.brand} does not command accel')

    cls.car_interface = CarInterface(cls.CP)
    values = importlib.import_module(f'opendbc.car.{cls.CP.brand}.values')
    ccp = values.CarControllerParams(cls.CP)
    if cls.CP.brand == 'honda':
      cls.accel_limits = (ccp.BOSCH_ACCEL_MIN, ccp.BOSCH_ACCEL_MAX)
    else:
      cls.accel_limits = (ccp.ACCEL_MIN, ccp.ACCEL_MAX)
    cls.inactive_gas = getattr(ccp, 'INACTIVE_GAS', None)

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

  @fuzzy_test(max_examples=25)
  def test_accel_within_limits(self, fuzzy):
    accel = fuzzy.choice(ACCEL_FUZZ_VALUES)
    v_ego = fuzzy.choice(V_EGO_FUZZ_VALUES)
    self.command_extreme_accel(accel, v_ego)
