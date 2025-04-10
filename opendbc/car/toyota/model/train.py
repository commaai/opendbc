import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm
import tensorflow as tf
from sklearn.preprocessing import MinMaxScaler
from tensorflow import keras
from collections import deque
import tf2onnx
import onnx
import onnxruntime as ort

from tools.lib.logreader import LogReader
from opendbc.can.parser import CANParser
from openpilot.selfdrive.pandad import can_capnp_to_list
from openpilot.common.filter_simple import FirstOrderFilter
from opendbc.car.common.basedir import BASEDIR

# Corolla w/ new tune real drive
# lr1 = list(LogReader('a2bddce0b6747e10/000002ac--c04552b8af', sort_by_time=True))
lr2 = list(LogReader('a2bddce0b6747e10/000002ab--4c8e1b86d9', sort_by_time=True))
# lr3 = list(LogReader('a2bddce0b6747e10/000002aa--9b130338b9', sort_by_time=True))
# lr4 = list(LogReader('a2bddce0b6747e10/000002ba--969fe70a70', sort_by_time=True))
# lr5 = list(LogReader('a2bddce0b6747e10/000002bb--3752fc5bba', sort_by_time=True))
# lr6 = list(LogReader('a2bddce0b6747e10/000002c8--f1df5a0b52', sort_by_time=True))
# lr7 = list(LogReader('a2bddce0b6747e10/000002c9--19a235a8d4', sort_by_time=True))


# Corolla w/ new tune maneuvers
# lr8 = list(LogReader('a2bddce0b6747e10/000002a9--a207f8b605', sort_by_time=True))

accel_deque = deque([0] * 1000, maxlen=1000)
f = FirstOrderFilter(0.0, 1.0, 0.01)

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

for lr in tqdm([lr2,]):
  cp = CANParser("toyota_nodsu_pt_generated", [("PCM_CRUISE", 33)], 0)
  cp128 = CANParser("toyota_nodsu_pt_generated", [("ACC_CONTROL", 33)], 128)

  CS = None
  CP = None
  LP = None
  CO = None
  prev_new_accel = 0
  long_active_frames = 0

  for msg in tqdm(lr):
    if msg.which() == 'carControl':
      CC = msg.carControl

      if CS is None or CP is None or not cp.can_valid or not cp128.can_valid:
        continue

      accel = CO.actuatorsOutput.accel
      accel_deque.append(accel)
      f.update(accel)
      scale = 1.0

      long_active_frames = long_active_frames + 1 if (CC.longActive and not CS.cruiseState.standstill) else 0

      # let it settle in
      if long_active_frames > 100:
        X_accels.append(accel)
        y_aegos.append(CS.aEgo)
        # y_aegos.append(LP.accelerationDevice.x)
        X_vegos.append(CS.vEgo)
        X_pitches.append(CC.orientationNED[1])
        X_permit_braking.append(cp128.vl['ACC_CONTROL']['PERMIT_BRAKING'])

        X_accels_past.append(f.x)
        X_accels_past2.append(accel_deque[-20])
        X_accels_past3.append(accel_deque[-40])
        X_accels_past4.append(accel_deque[-80])
        X_accels_past5.append(accel_deque[-160])

        # predict resulting aEgo from requested accel
        new_accel = accel
        if new_accel > 0:
          scale = min(max(scale - (new_accel - accel_deque[-20]) * 5, 0.8), 1.0)
          print(scale)
        # else:
        #   scale = 1.0
        scale = min(scale + 0.005, 1.0)
        new_accel *= scale
        # new_accel = accel + max(cp.vl["PCM_CRUISE"]['NEUTRAL_FORCE'] / CP.mass, 0)

        # if accel > 0:
        #   new_accel = new_accel * 1.2
        #
        #  # TODO: add first order filter
        # rate_limit = np.interp(CS.vEgo, [0, 5], [0.01, 0.02])
        #
        # new_accel = max(new_accel, prev_new_accel - rate_limit)
        prev_new_accel = new_accel
        predicted_aegos.append(new_accel)

      # else:
      #   accel_deque = deque([0.0] * 1000, maxlen=1000)

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
      cp128.update_strings(lst)


# train model to simulate aEgo from requested accel
model = keras.models.Sequential([
  keras.layers.Dense(8, activation=keras.layers.LeakyReLU(), input_shape=(7,)),
  # keras.layers.Dropout(0.2),
  keras.layers.Dense(8, activation=keras.layers.LeakyReLU()),
  # keras.layers.Dropout(0.1),
  keras.layers.Dense(8, activation=keras.layers.LeakyReLU()),
  # keras.layers.Dropout(0.1),
  keras.layers.Dense(1),
])

model.compile(optimizer='adam', loss='mse')

X = np.array([X_accels, X_accels_past2, X_accels_past3, X_accels_past4, X_accels_past5, X_pitches, X_vegos]).T
# X = np.array([X_accels, X_accels_past, X_pitches, X_vegos]).T
y = np.array(y_aegos)

# offset X
# X = X[:-25]
# y = y[25:]

print('Samples', len(X))

# try:
#   model.fit(X, y, batch_size=256, epochs=10, validation_split=0.3, shuffle=True)
# except KeyboardInterrupt:
#   pass

model.output_names=['output']
spec = (tf.TensorSpec((None, *model.input_shape[1:]), tf.float32, name="input"),)
onnx_model, _ = tf2onnx.convert.from_keras(model, input_signature=spec)
onnx.save(onnx_model, '/home/batman/openpilot/opendbc/car/toyota/pcm.onnx')

predicted_aegos_nn = model.predict(X)

loaded_model = ort.InferenceSession(BASEDIR + '/toyota/pcm.onnx')

predicted_aegos_nn2 = loaded_model.run(None, {'input': X.astype(np.float32)})[0].T[0].reshape(-1, 1)

fig, ax = plt.subplots(2, 1, sharex=True)
ax[0].plot(X_accels, label='actuatorsOutput.accel')
ax[0].plot(y_aegos, label='aEgo (ground)')
# ax[0].plot(y_aegos, label='accelerationDevice.x (ground)')
ax[0].plot(predicted_aegos, label='predicted aEgo')
# ax[0].plot(predicted_aegos_nn, label='predicted aEgo (NN)')
# ax[0].plot(predicted_aegos_nn2, label='predicted aEgo (branch NN)')
# ax[0].plot(X_accels_past, label='past accel')
ax[0].legend()

ax[1].plot(X_vegos, label='vEgo')
ax[1].legend()

plt.show()
