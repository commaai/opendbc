import math
import numpy as np
from opendbc.car.car_helpers import interfaces
from selfdrive.controls.lib.vehicle_model import VehicleModel
import matplotlib.pyplot as plt

car_model = 'TESLA_MODEL_Y'
CarInterface, _, _, _ = interfaces[car_model]
CP = CarInterface.get_non_essential_params(car_model)

sR = 10.3
VM = VehicleModel(CP)

r = 50  # Hz
X = np.linspace(0.1, 35, 100)

jerk_up = []
jerk_down = []
for v in X:
  sa_up = np.interp(v, [0., 5., 25.], [2.5, 2.0, .2])  # deg/frame
  sa_down = np.interp(v, [0., 5., 25.], [5., 3.0, 0.3])  # deg/frame

  jerk_up.append(VM.yaw_rate(math.radians(sa_up) * 50, v, 0) * v)
  jerk_down.append(VM.yaw_rate(math.radians(sa_down) * 50, v, 0) * v)

plt.figure(0)
plt.clf()
plt.ylim(0, 14)
plt.plot(X, jerk_up, label='jerk up')
plt.plot(X, jerk_down, label='jerk down')
plt.legend()
plt.show()
plt.pause(100000)
