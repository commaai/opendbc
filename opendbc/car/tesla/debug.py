import matplotlib.pyplot as plt
import numpy as np
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.car_helpers import interfaces
plt.ion()

CI = interfaces['TESLA_MODEL_Y']

print(CI)

CP = CI.get_non_essential_params('TESLA_MODEL_Y')
print(CP)

VM = VehicleModel(CP)


X = np.linspace(0, 35, 100)
Y_curv_VM = []
Y_curv_man = []

Y_accel_VM = []
Y_accel_man = []

for spd in X:
  a = np.radians(10)
  curvature = VM.calc_curvature(a, spd, 0)
  curvature_manual = a / CP.steerRatio / CP.wheelbase

  lat_accel = curvature * spd ** 2
  lat_accel_manual = curvature_manual * spd ** 2

  # print(f"Curvature: {curvature}, Manual Curvature: {curvature_manual}")
  Y_curv_VM.append(curvature)
  Y_curv_man.append(curvature_manual)
  Y_accel_VM.append(lat_accel)
  Y_accel_man.append(lat_accel_manual)


fig, ax = plt.subplots(2, 1, figsize=(10, 8))
ax[0].plot(X, Y_curv_VM, label='VM Curvature for 5 degrees steering angle')
ax[0].plot(X, Y_curv_man, label='Manual Curvature for 5 degrees steering angle')
ax[0].set_xlabel('Speed (m/s)')
ax[0].set_ylabel('Curvature (1/m)')
ax[0].legend()
ax[1].plot(X, Y_accel_VM, label='VM Lateral Acceleration for 5 degrees steering angle')
ax[1].plot(X, Y_accel_man, label='Manual Lateral Acceleration for 5 degrees steering angle')
plt.plot([0, 35], [3, 3], 'r--', label='ISO 11270 Limit')
ax[1].set_xlabel('Speed (m/s)')
ax[1].set_ylabel('Lateral Acceleration (m/s^2)')
ax[1].legend()
plt.legend()
plt.show()
