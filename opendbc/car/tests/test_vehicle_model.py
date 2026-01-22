import pytest
import math


class TestVehicleModel:
  @pytest.fixture(autouse=True)
  def setup(self, car_lib):
    from opendbc.car.honda.values import CAR
    CP = car_lib.interfaces[CAR.HONDA_CIVIC].get_non_essential_params(CAR.HONDA_CIVIC)
    self.VM = car_lib.vehicle_model.VehicleModel(CP)

  def test_round_trip_yaw_rate(self, car_lib):
    for u in car_lib.np.linspace(1, 30, num=10):
      for roll in car_lib.np.linspace(math.radians(-20), math.radians(20), num=11):
        for sa in car_lib.np.linspace(math.radians(-20), math.radians(20), num=11):
          yr = self.VM.yaw_rate(sa, u, roll)
          new_sa = self.VM.get_steer_from_yaw_rate(yr, u, roll)
          assert sa == pytest.approx(new_sa)

  def test_dyn_ss_sol_against_yaw_rate(self, car_lib):
    for roll in car_lib.np.linspace(math.radians(-20), math.radians(20), num=11):
      for u in car_lib.np.linspace(1, 30, num=10):
        for sa in car_lib.np.linspace(math.radians(-20), math.radians(20), num=11):
          _, yr1 = car_lib.vehicle_model.dyn_ss_sol(sa, u, roll, self.VM)
          yr2 = self.VM.yaw_rate(sa, u, roll)
          assert float(yr1[0]) == pytest.approx(yr2)

  def test_syn_ss_sol_simulate(self, car_lib):
    for roll in car_lib.np.linspace(math.radians(-20), math.radians(20), num=11):
      for u in car_lib.np.linspace(1, 30, num=10):
        A, B = car_lib.vehicle_model.create_dyn_state_matrices(u, self.VM)
        dt, top = 0.01, car_lib.np.hstack((A, B))
        Md = sum([car_lib.np.linalg.matrix_power(car_lib.np.vstack((top, car_lib.np.zeros_like(top))) * dt, k) / math.factorial(k) for k in range(25)])
        Ad, Bd = Md[:A.shape[0], :A.shape[1]], Md[:A.shape[0], A.shape[1]:]

        for sa in car_lib.np.linspace(math.radians(-20), math.radians(20), num=11):
          inp = car_lib.np.array([[sa], [roll]])
          x1 = car_lib.np.zeros((2, 1))
          for _ in range(100):
            x1 = Ad @ x1 + Bd @ inp
          x2 = car_lib.vehicle_model.dyn_ss_sol(sa, u, roll, self.VM)
          car_lib.np.testing.assert_almost_equal(x1, x2, decimal=3)
