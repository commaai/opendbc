import os
import matplotlib.pyplot as plt
from tools.lib.logreader import LogReader

os.chdir('/home/batman/openpilot/selfdrive/debug')

# lr = LogReader('a2bddce0b6747e10/0000033f--bc20a12d8b/:2')
# lr_regen = LogReader('a2bddce0b6747e10_0000033f--bc20a12d8b_:2_card.zst')

lr = LogReader('a2bddce0b6747e10/00000379--298edb2ca7/6:8')
lr_regen = LogReader('a2bddce0b6747e10_00000379--298edb2ca7_6:8_card.zst')

accels = []
accels_out = []
aegos = []

accels_regen = []
accels_out_regen = []

for msg in lr:
  if msg.which() == 'carState':
    aegos.append(msg.carState.aEgo)
  elif msg.which() == 'carControl':
    accels.append(msg.carControl.actuators.accel)
  elif msg.which() == 'carOutput':
    accels_out.append(msg.carOutput.actuatorsOutput.accel)

for msg in lr_regen:
  if msg.which() == 'carControl':
    accels_regen.append(msg.carControl.actuators.accel)
  elif msg.which() == 'carOutput':
    accels_out_regen.append(msg.carOutput.actuatorsOutput.accel)

fig, ax = plt.subplots(2, sharex=True)

ax[0].plot(accels, label='accels')
ax[0].plot(accels_regen, label='accels_regen')
ax[0].plot(aegos, label='aegos')
ax[0].legend()

ax[1].plot(accels, label='accels')
ax[1].plot(accels_out_regen, label='accels_out_regen')
ax[1].plot(accels_out, label='accels_out')
# ax[1].plot(aegos, label='aegos')
ax[1].legend()

plt.show()
