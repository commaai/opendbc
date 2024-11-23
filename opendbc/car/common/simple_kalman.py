import numpy as np


def get_kalman_gain(dt, A, C, Q, R, iterations=100):
  P = np.zeros_like(Q)
  for _ in range(iterations):
    P = A.dot(P).dot(A.T) + dt * Q
    S = C.dot(P).dot(C.T) + R
    K = P.dot(C.T).dot(np.linalg.inv(S))
    P = (np.eye(len(P)) - K.dot(C)).dot(P)
  return K


class KF1D:
  # This EKF assumes a constant covariance matrix, so calculations are simpler.
  # The Kalman gain also needs to be precomputed using the control module.

  def __init__(self, x0, A, C, K):
    # State variables
    self.x0_0 = x0[0][0]  # vEgo (velocity)
    self.x1_0 = x0[1][0]  # aEgo (acceleration)
    self.x2_0 = x0[2][0]  # jEgo (jerk) - Added

    # State transition matrix elements
    self.A0_0 = A[0][0]
    self.A0_1 = A[0][1]
    self.A0_2 = A[0][2]  # Added
    self.A1_0 = A[1][0]
    self.A1_1 = A[1][1]
    self.A1_2 = A[1][2]  # Added
    self.A2_0 = A[2][0]  # Added
    self.A2_1 = A[2][1]  # Added
    self.A2_2 = A[2][2]  # Added

    # Observation matrix elements
    self.C0 = C[0]
    self.C1 = C[1]
    self.C2 = C[2]  # Added

    # Kalman gain elements
    self.K0_0 = K[0][0]
    self.K1_0 = K[1][0]
    self.K2_0 = K[2][0]  # Added

    # Precompute (A - K * C) matrix elements
    self.A_K_0 = self.A0_0 - self.K0_0 * self.C0
    self.A_K_1 = self.A0_1 - self.K0_0 * self.C1
    self.A_K_2 = self.A0_2 - self.K0_0 * self.C2  # Added
    self.A_K_3 = self.A1_0 - self.K1_0 * self.C0
    self.A_K_4 = self.A1_1 - self.K1_0 * self.C1
    self.A_K_5 = self.A1_2 - self.K1_0 * self.C2  # Added
    self.A_K_6 = self.A2_0 - self.K2_0 * self.C0  # Added
    self.A_K_7 = self.A2_1 - self.K2_0 * self.C1  # Added
    self.A_K_8 = self.A2_2 - self.K2_0 * self.C2  # Added

    # K matrix needs to be precomputed as follows:
    # import control
    # (x, l, K) = control.dare(np.transpose(A), np.transpose(C), Q, R)
    # K = np.transpose(K)

  def update(self, meas):
    # State update equations with jEgo included
    x0_0 = (self.A_K_0 * self.x0_0 +
            self.A_K_1 * self.x1_0 +
            self.A_K_2 * self.x2_0 +  # Added
            self.K0_0 * meas)
    x1_0 = (self.A_K_3 * self.x0_0 +
            self.A_K_4 * self.x1_0 +
            self.A_K_5 * self.x2_0 +  # Added
            self.K1_0 * meas)
    x2_0 = (self.A_K_6 * self.x0_0 +
            self.A_K_7 * self.x1_0 +
            self.A_K_8 * self.x2_0 +  # Added
            self.K2_0 * meas)         # Added

    # Update state variables
    self.x0_0 = x0_0
    self.x1_0 = x1_0
    self.x2_0 = x2_0  # Added

    return [self.x0_0, self.x1_0, self.x2_0]  # Modified to include jEgo

  @property
  def x(self):
    return [[self.x0_0], [self.x1_0], [self.x2_0]]  # Modified to include jEgo

  def set_x(self, x):
    self.x0_0 = x[0][0]
    self.x1_0 = x[1][0]
    self.x2_0 = x[2][0]  # Added
