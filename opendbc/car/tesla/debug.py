# import matplotlib.pyplot as plt
# import numpy as np
# from opendbc.car.vehicle_model import VehicleModel, calc_slip_factor
# from opendbc.car.car_helpers import interfaces
# import math
# plt.ion()
#
# ISO_LATERAL_ACCEL = 3.0  # m/s^2
# ISO_LATERAL_JERK = 5.0  # m/s^3
#
# CI = interfaces['TESLA_MODEL_Y']
# CP = CI.get_non_essential_params('TESLA_MODEL_Y')
#
# VM = VehicleModel(CP)
#
# X = np.linspace(0, 35, 100)
# Y_curv_VM = []
# Y_curv_man = []
#
# Y_accel_VM = []
# Y_accel_man = []
#
# for spd in X:
#   calc_curvature_from_accel = ISO_LATERAL_ACCEL / spd ** 2
#   calc_angle = calc_curvature_from_accel * CP.steerRatio * CP.wheelbase / VM.curvature_factor(spd)
#
#   # a = np.radians(10)
#   curvature = VM.calc_curvature(calc_angle, spd, 0)
#   curvature_manual = calc_angle / CP.steerRatio / CP.wheelbase
#
#   lat_accel = curvature * spd ** 2
#   lat_accel_manual = curvature_manual * spd ** 2
#
#   # print(f"Curvature: {curvature}, Manual Curvature: {curvature_manual}")
#   Y_curv_VM.append(curvature)
#   Y_curv_man.append(curvature_manual)
#   Y_accel_VM.append(lat_accel)
#   Y_accel_man.append(lat_accel_manual)
#
#
# fig, ax = plt.subplots(2, 1, figsize=(10, 8))
# ax[0].plot(X, Y_curv_VM, label='VM Curvature for 3 m/s^2 lateral acceleration')
# ax[0].plot(X, Y_curv_man, label='Manual Curvature for 3 m/s^2 lateral acceleration')
# ax[0].set_xlabel('speed (m/s)')
# ax[0].set_ylabel('curvature (1/m)')
# ax[0].legend()
# ax[1].plot(X, Y_accel_VM, label='VM lateral acceleration for 3 m/s^2 lateral acceleration')
# ax[1].plot(X, Y_accel_man, label='Manual lateral acceleration for 3 m/s^2 lateral acceleration')
# plt.plot([0, 35], [3, 3], 'r--', label='ISO 11270 Limit')
# ax[1].set_xlabel('speed (m/s)')
# ax[1].set_ylabel('lateral acceleration (m/s^2)')
# ax[1].legend()
# plt.legend()
# plt.show()
