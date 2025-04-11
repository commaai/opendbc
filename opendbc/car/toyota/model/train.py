import matplotlib.pyplot as plt
import math
import numpy as np
from tqdm import tqdm
import tensorflow as tf
from sklearn.preprocessing import MinMaxScaler
from tensorflow import keras
from collections import deque
from matplotlib.widgets import Slider, CheckButtons
import tf2onnx
import onnx
import onnxruntime as ort
import ipywidgets as widgets
from IPython.display import display, clear_output

import streamlit as st
import plotly.express as px
import pandas as pd


from tools.lib.logreader import LogReader
from opendbc.can.parser import CANParser
from openpilot.selfdrive.pandad import can_capnp_to_list
from openpilot.common.filter_simple import FirstOrderFilter
from opendbc.car.common.basedir import BASEDIR

plt.ion()

# Corolla w/ new tune real drive
# lr1 = list(LogReader('a2bddce0b6747e10/000002ac--c04552b8af', sort_by_time=True))
# lr2 = list(LogReader('a2bddce0b6747e10/000002ab--4c8e1b86d9', sort_by_time=True))
# lr3 = list(LogReader('a2bddce0b6747e10/000002aa--9b130338b9', sort_by_time=True))
# lr4 = list(LogReader('a2bddce0b6747e10/000002ba--969fe70a70', sort_by_time=True))
# lr5 = list(LogReader('a2bddce0b6747e10/000002bb--3752fc5bba', sort_by_time=True))
# lr6 = list(LogReader('a2bddce0b6747e10/000002c8--f1df5a0b52', sort_by_time=True))
# lr7 = list(LogReader('a2bddce0b6747e10/000002c9--19a235a8d4', sort_by_time=True))

lrs = [
  (True, LogReader('d9b97c1d3b8c39b2/000000b9--1ff97b1087', sort_by_time=True)),
  (True, LogReader('d9b97c1d3b8c39b2/000000bf--4bc0673df6', sort_by_time=True)),
  (True, LogReader('d9b97c1d3b8c39b2/000000bd--1878ccda58', sort_by_time=True)),
]

# Corolla w/ new tune maneuvers
# lr8 = list(LogReader('a2bddce0b6747e10/000002a9--a207f8b605', sort_by_time=True))

accel_deque = deque([0] * 1000, maxlen=1000)
f = FirstOrderFilter(0.0, 1.0, 0.01)

X_sections = [[]]
Y_sections = [[]]
X_accels = []
X_accels_past = []
X_accels_past1 = []
X_accels_past2 = []
X_accels_past3 = []
X_accels_past4 = []
X_accels_past5 = []
X_accels_past6 = []
X_pitches = []
X_vegos = []
X_permit_braking = []

y_aegos = []
predicted_aegos = []

for stock_route, lr in tqdm(lrs):
  cp = CANParser("toyota_nodsu_pt_generated", [
    ("PCM_CRUISE", 33),
    ("CLUTCH", 15),
    ("VSC1S07", 20),
  ], 0)
  cp2 = CANParser("toyota_nodsu_pt_generated", [("ACC_CONTROL", 33)], 2)
  cp128 = CANParser("toyota_nodsu_pt_generated", [("ACC_CONTROL", 33)], 128)

  CC = None
  CS = None
  CP = None
  LP = None
  CO = None
  prev_new_accel = 0
  long_active_frames = 0

  for msg in tqdm(lr):
    if msg.which() == 'carControl':
      CC = msg.carControl

    #   if CS is None or CP is None or not cp.can_valid or not cp2.can_valid or not cp128.can_valid:
    #     continue
    #
    #   accel = CO.actuatorsOutput.accel
    #   accel_deque.append(accel)
    #   f.update(accel)
    #   scale = 1.0
    #
    #   long_active_frames = long_active_frames + 1 if (CC.longActive and not CS.cruiseState.standstill) else 0
    #
    #   # let it settle in
    #   if long_active_frames > 100:
    #     X_accels.append(accel)
    #     y_aegos.append(CS.aEgo)
    #     # y_aegos.append(LP.accelerationDevice.x)
    #     X_vegos.append(CS.vEgo)
    #     X_pitches.append(CC.orientationNED[1])
    #     X_permit_braking.append(cp128.vl['ACC_CONTROL']['PERMIT_BRAKING'])
    #
    #     X_accels_past.append(f.x)
    #     X_accels_past2.append(accel_deque[-20])
    #     X_accels_past3.append(accel_deque[-40])
    #     X_accels_past4.append(accel_deque[-80])
    #     X_accels_past5.append(accel_deque[-160])
    #
    #     # predict resulting aEgo from requested accel
    #     new_accel = accel
    #     if new_accel > 0:
    #       scale = min(max(scale - (new_accel - accel_deque[-20]) * 5, 0.8), 1.0)
    #       print(scale)
    #     # else:
    #     #   scale = 1.0
    #     scale = min(scale + 0.005, 1.0)
    #     new_accel *= scale
    #     # new_accel = accel + max(cp.vl["PCM_CRUISE"]['NEUTRAL_FORCE'] / CP.mass, 0)
    #
    #     # if accel > 0:
    #     #   new_accel = new_accel * 1.2
    #     #
    #     #  # TODO: add first order filter
    #     # rate_limit = np.interp(CS.vEgo, [0, 5], [0.01, 0.02])
    #     #
    #     # new_accel = max(new_accel, prev_new_accel - rate_limit)
    #     prev_new_accel = new_accel
    #     predicted_aegos.append(new_accel)
    #
    #   # else:
    #   #   accel_deque = deque([0.0] * 1000, maxlen=1000)

    elif msg.which() == 'carOutput':
      CO = msg.carOutput

    elif msg.which() == 'carParams':
      CP = msg.carParams

    elif msg.which() == 'livePose':
      LP = msg.livePose

    elif msg.which() == 'carState':
      CS = msg.carState

    elif msg.which() == 'can':
      lst = can_capnp_to_list([msg.as_builder().to_bytes()])
      cp.update_strings(lst)
      cp2.update_strings(lst)
      # cp128.update_strings(lst)
      # print(cp.can_valid, cp2.can_valid, cp128.can_valid)

      if not cp.can_valid or not cp2.can_valid:# or not cp128.can_valid:
        continue

      if CC is None or CS is None or not len(CC.orientationNED):
        continue

      if CS.gasPressed or CS.brakePressed or CS.cruiseState.standstill or not CS.cruiseState.enabled:
        if len(X_sections[-1]):
          # print('new section!')
          # print(X_sections[-1])
          # print(Y_sections[-1])
          # print()
          X_sections.append([])
          Y_sections.append([])
        continue

      car_pitch = math.radians(cp.vl['VSC1S07']['ASLP'])
      pcm_accel_net = cp.vl['PCM_CRUISE']['ACCEL_NET']
      clutch_accel_net = cp.vl['CLUTCH']['ACCEL_NET']

      pitch = CC.orientationNED[1]
      stock_camera_accel = cp2.vl['ACC_CONTROL']['ACCEL_CMD']
      stock_camera_permit_braking = cp2.vl['ACC_CONTROL']['PERMIT_BRAKING']
      # mini car is needed to get high val_brake_output_accuracy because it is taken into consideration
      # by camera when setting permit braking
      mini_car = cp2.vl['ACC_CONTROL']['MINI_CAR']

      # GVC does not overshoot ego acceleration when starting from stop, but still has a similar delay
      gvc = cp.vl["VSC1S07"]["GVC"]
      a_ego = np.interp(CS.vEgo, [1.0, 2.0], [gvc, CS.aEgo])

      X_sections[-1].append([CS.vEgo, a_ego, pitch, mini_car, car_pitch, pcm_accel_net, clutch_accel_net])
      Y_sections[-1].append([stock_camera_accel, stock_camera_permit_braking])
      # print(X_sections[-1][-1])
      # print(Y_sections[-1][-1])
      # print()


# keep track of sections because data is not continuous
# delay cmd (y) by 15 frames so that it roughly matches the result (x)
X, Y = [], []
for x_section, y_section in zip(X_sections, Y_sections):
  # trim off first 0.5s after engaging
  X.extend(x_section[15:][50:])
  Y.extend(y_section[:-15][50:])

X = np.array(X)
Y = np.array(Y)
assert len(X) == len(Y)
print('Samples', len(X))

def plot_data_stats():
  # scatter plot where x is aEgo and y is accel cmd
  fig, ax = plt.subplots(1)
  ax.scatter([x[1] for x in X], [y[0] for y in Y], s=1)
  ax.plot([-5, 5], [-5, 5], 'r--', label='y=x')
  ax.set_xlabel('aEgo')
  ax.set_ylabel('accel cmd')
  ax.set_title('aEgo vs accel cmd')
  ax.set_xlim(-5, 5)
  ax.set_ylim(-5, 5)
  plt.legend()
  plt.show()
  # plt.pause(100)


plot_data_stats()


# FIXME: this one is vibe coded and terrible, but cool idea
def plot_data_stats2():
  # Assumes X and Y are defined in the global scope.
  vEgo = np.array([x[0] for x in X])
  aEgo = np.array([x[1] for x in X])
  pitch = np.array([x[2] for x in X])
  accel_cmd = np.array([y[0] for y in Y])

  # --- Plot setup ---
  fig, ax = plt.subplots()
  # Adjust layout to accommodate sliders and toggles.
  plt.subplots_adjust(bottom=0.3, right=0.8)

  # Plot the initial scatter using ALL the points (before filtering).
  sc = ax.scatter(aEgo, accel_cmd, c=pitch, s=3, cmap='viridis')
  ax.set_xlabel('aEgo')
  ax.set_ylabel('accel_cmd')
  ax.set_title('aEgo vs accel_cmd colored by pitch')
  cb = plt.colorbar(sc, ax=ax)
  cb.set_label("Pitch")

  # --- Add y = x reference line ---
  # Create a reference line over the range of aEgo values
  x_line = np.linspace(np.min(aEgo), np.max(aEgo), 100)
  ax.plot(x_line, x_line, 'r--', label='y = x')
  ax.legend(loc='upper left')

  # --- Slider axes for filter centers ---
  ax_speed = plt.axes([0.15, 0.2, 0.65, 0.03])
  ax_pitch = plt.axes([0.15, 0.15, 0.65, 0.03])
  # Slider for vEgo center with its range set to that of vEgo.
  s_speed = Slider(ax_speed, 'vEgo Center', vEgo.min(), vEgo.max(), valinit=vEgo.mean())
  # Slider for pitch center (in rad) with its range set to that of pitch.
  s_pitch = Slider(ax_pitch, 'Pitch Center (rad)', pitch.min(), pitch.max(), valinit=pitch.mean())

  # --- Toggle checkboxes ---
  # Place checkboxes in an axes on the right.
  ax_check = plt.axes([0.82, 0.4, 0.15, 0.15])
  # Two toggles: one to apply the speed filter and one for the pitch filter.
  check = CheckButtons(ax_check, ['Speed Filter', 'Pitch Filter'], [True, True])

  def update(val):
    speed_center = s_speed.val
    pitch_center = s_pitch.val

    # If the speed filter is enabled, select points within ±5 m/s.
    if check.get_status()[0]:
      mask_speed = np.abs(vEgo - speed_center) <= 5.0
    else:
      mask_speed = np.ones_like(vEgo, dtype=bool)
    # If the pitch filter is enabled, select points within ±5° (in radians).
    if check.get_status()[1]:
      mask_pitch = np.abs(pitch - pitch_center) <= np.deg2rad(5)
    else:
      mask_pitch = np.ones_like(pitch, dtype=bool)

    mask = mask_speed & mask_pitch

    # Decimate the data if too many points are selected
    N_selected = np.sum(mask)
    decimation_threshold = 10000  # adjust threshold as needed
    if N_selected > decimation_threshold:
      indices = np.where(mask)[0]
      chosen_indices = np.random.choice(indices, decimation_threshold, replace=False)
      current_mask = np.zeros_like(mask, dtype=bool)
      current_mask[chosen_indices] = True
    else:
      current_mask = mask

    # Update scatter plot with filtered (and possibly decimated) data.
    offsets = np.column_stack((aEgo[current_mask], accel_cmd[current_mask]))
    sc.set_offsets(offsets)
    sc.set_array(pitch[current_mask])
    fig.canvas.draw_idle()

  s_speed.on_changed(update)
  s_pitch.on_changed(update)
  check.on_clicked(lambda label: update(None))

  update(None)  # initial draw
  plt.show()


# Call the function to run the visualization.
plot_data_stats2()

# raise Exception
# train model to simulate aEgo from requested accel

# the model
inputs = keras.layers.Input(shape=X.shape[1:])
shared = keras.layers.BatchNormalization()(inputs)  # too lazy to scale
shared = keras.layers.Dense(32, kernel_regularizer=keras.regularizers.l2(0.001))(shared)
# shared = keras.layers.BatchNormalization()(shared)
shared = keras.layers.LeakyReLU()(shared)
# shared = keras.layers.Dropout(0.2)(shared)

shared = keras.layers.Dense(16, kernel_regularizer=keras.regularizers.l2(0.001))(shared)
shared = keras.layers.LeakyReLU()(shared)
# shared = keras.layers.Dropout(0.2)(shared)

# two outputs
accel_output = keras.layers.Dense(1, name='accel_output')(shared)
brake_output = keras.layers.Dense(1, activation='sigmoid', name='brake_output')(shared)

model = keras.models.Model(inputs=inputs, outputs=[accel_output, brake_output])

model.compile(
  optimizer='adam', loss={'accel_output': 'mse', 'brake_output': 'binary_crossentropy'},
  metrics={'accel_output': ['mae'], 'brake_output': ['accuracy']},
)

model.summary()

# X = np.array([X_accels, X_accels_past2, X_accels_past3, X_accels_past4, X_accels_past5, X_pitches, X_vegos]).T
# # X = np.array([X_accels, X_accels_past, X_pitches, X_vegos]).T
# y = np.array(y_aegos)

# offset X
# X = X[:-25]
# y = y[25:]

# print('Samples', len(X))

Y_accel = Y[:, 0]  # float
Y_brake = Y[:, 1]  # binary (binary, 0/1)

split_idx = int(len(X) * 0.8)
X_train, X_test = X[:split_idx], X[split_idx:]
Y_train_accel, Y_test_accel = Y_accel[:split_idx], Y_accel[split_idx:]
Y_train_brake, Y_test_brake = Y_brake[:split_idx], Y_brake[split_idx:]

try:
  model.fit(X_train, [Y_train_accel, Y_train_brake], batch_size=256, epochs=10, shuffle=True,
            validation_data=(X_test, [Y_test_accel, Y_test_brake]))
except KeyboardInterrupt:
  pass


def plot_model_prediction():
  fig, ax = plt.subplots(1)
  speeds = np.linspace(5, 35, 4)
  for speed in speeds:
    x = np.linspace(-4, 4, 10)
    y = model.predict(np.stack([np.full_like(x, speed), x, np.zeros_like(x), np.ones_like(x)], axis=1))
    ax.plot(x, y[0].flatten(), 'o-', label=f'speed {speed:.1f} m/s')
    ax.set_xlabel('aEgo')
    ax.set_ylabel('accel cmd')
    ax.set_title('aEgo vs accel cmd')
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.legend()
  plt.show()


# plot_model_prediction()

# raise Exception

# save model
# model.output_names=['output']
print("Saving model")
spec = (tf.TensorSpec((None, *model.input_shape[1:]), tf.float32, name="input"),)
onnx_model, _ = tf2onnx.convert.from_keras(model, input_signature=spec)
onnx.save(onnx_model, BASEDIR + '/toyota/camera.onnx')

X_pred = model.predict(X)
predicted_camera_accel = X_pred[0].flatten()
predicted_permit_braking = X_pred[1].flatten()

loaded_model = ort.InferenceSession(BASEDIR + '/toyota/camera.onnx')

predicted_camera_accel_loaded = loaded_model.run(None, {'input': X.astype(np.float32)})[0].flatten()

# plot prediction
fig, ax = plt.subplots(3, 1, sharex=True)
ax[0].plot([y[0] for y in Y], label='accel cmd (output)')
ax[0].plot(predicted_camera_accel, label='predicted accel cmd (output)')
ax[0].plot([x[1] for x in X], label='aEgo (input)')

# ax[0].plot(X_accels, label='actuatorsOutput.accel')
# ax[0].plot(y_aegos, label='aEgo (ground)')
# # ax[0].plot(y_aegos, label='accelerationDevice.x (ground)')
# ax[0].plot(predicted_aegos, label='predicted aEgo')
# # ax[0].plot(predicted_aegos_nn, label='predicted aEgo (NN)')
# # ax[0].plot(predicted_aegos_nn2, label='predicted aEgo (branch NN)')
# # ax[0].plot(X_accels_past, label='past accel')
ax[0].legend()

# permit braking (output) and mini car (input)
ax[1].plot([y[1] for y in Y], label='permit braking (output)')
ax[1].plot(predicted_permit_braking, label='predicted permit braking (output)')
ax[1].plot([x[3] for x in X], label='mini car (input)')
ax[1].legend()

ax[2].plot([x[0] for x in X], label='vEgo')
ax[2].legend()

plt.show()
