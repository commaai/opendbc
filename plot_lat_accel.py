#!/usr/bin/env python3
"""Plot max lateral accel and jerk for angle steering cars given safety params."""
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR

# Cars listed in hyundai_canfd.h as angle steering candidates
ANGLE_CARS = [
  CAR.HYUNDAI_IONIQ_5,       # proxy for IONIQ_5_PE
  CAR.HYUNDAI_IONIQ_6,
  CAR.KIA_EV6_2025,
  CAR.KIA_EV6,
  CAR.GENESIS_GV80,          # proxy for GV80_2025
  CAR.KIA_SPORTAGE_5TH_GEN,  # proxy for SPORTAGE_HEV_2026
]

# Current safety params (GV80_2025 from hyundai_canfd.h)
SAFETY_CAR = CAR.KIA_EV6_2025

ISO_LATERAL_ACCEL = 3.0  # m/s^2
ISO_LATERAL_JERK = 5.0   # m/s^3
DT = 0.01

speeds = np.linspace(3, 40, 200)  # m/s

# build safety VM
CP_safety = CarInterface.get_non_essential_params(SAFETY_CAR)
VM_safety = VehicleModel(CP_safety)

fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

for car in ANGLE_CARS:
  CP = CarInterface.get_non_essential_params(car)
  VM = VehicleModel(CP)

  lat_accels = []
  lat_jerks = []
  for v in speeds:
    # safety limits max angle using safety VM
    max_curv_safety = ISO_LATERAL_ACCEL / (v ** 2)
    max_ang_safety = math.degrees(VM_safety.get_steer_from_curvature(max_curv_safety, v, 0))

    # safety limits max angle rate using safety VM
    max_curv_rate_safety = ISO_LATERAL_JERK / (v ** 2)
    max_ang_rate_safety = math.degrees(VM_safety.get_steer_from_curvature(max_curv_rate_safety, v, 0))  # deg per sec

    # actual lat accel on this car at the safety-limited angle
    actual_curv = VM.calc_curvature(math.radians(max_ang_safety), v, 0)
    actual_lat_accel = abs(actual_curv) * v ** 2

    # actual lat jerk: if we apply max_ang_rate for one step
    actual_curv_rate = VM.calc_curvature(math.radians(max_ang_rate_safety * DT), v, 0)
    actual_lat_jerk = abs(actual_curv_rate) * v ** 2 / DT

    lat_accels.append(actual_lat_accel)
    lat_jerks.append(actual_lat_jerk)

  speeds_kph = speeds * 3.6
  name = str(car).split(".")[-1]
  axes[0].plot(speeds_kph, lat_accels, label=name)
  axes[1].plot(speeds_kph, lat_jerks, label=name)
  idx80 = np.searchsorted(speeds_kph, 80)
  print(f"{name}: SR={CP.steerRatio:.2f} WB={CP.wheelbase:.3f} accel@80={lat_accels[min(idx80, len(lat_accels)-1)]:.2f} jerk@80={lat_jerks[min(idx80, len(lat_jerks)-1)]:.2f}")

axes[0].axhline(ISO_LATERAL_ACCEL, color='red', linestyle='--', label=f'ISO {ISO_LATERAL_ACCEL} m/s²')
axes[0].set_ylabel('Max Lateral Accel (m/s²)')
axes[0].set_title(f'Actual lateral accel per car (safety params from {str(SAFETY_CAR).split(".")[-1]})')
axes[0].legend(fontsize=8)
axes[0].grid(True)

axes[1].axhline(ISO_LATERAL_JERK, color='red', linestyle='--', label=f'ISO {ISO_LATERAL_JERK} m/s³')
axes[1].set_ylabel('Max Lateral Jerk (m/s³)')
axes[1].set_xlabel('Speed (km/h)')
axes[1].set_title('Actual lateral jerk per car')
axes[1].legend(fontsize=8)
axes[1].grid(True)

plt.tight_layout()
plt.savefig('lat_accel_comparison.png', dpi=150)
plt.show()
