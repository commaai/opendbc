#!/usr/bin/env python3
import os; os.environ['FILEREADER_CACHE'] = '1'
import numpy as np
import matplotlib.pyplot as plt
from opendbc.can import CANParser
from openpilot.tools.lib.logreader import LogReader

route = '3fc2f65852a45c13/00000043--0faaacc68d'
lr = LogReader(route, sort_by_time=True)
can_msgs = sorted([m for m in lr if m.which() == 'can'], key=lambda m: m.logMonoTime)

parser = CANParser('hyundai_canfd_generated', [('MDPS', 0), ('LFA_ALT', 0), ('WHEEL_SPEEDS', 0)], 1)

tqs, gains, spds, cmd_angs = [], [], [], []
for cm in can_msgs:
    parser.update([0, [(c.address, c.dat, c.src) for c in cm.can]])
    tqs.append(parser.vl['MDPS']['MDPS_StrTqSnsrVal'])
    gains.append(parser.vl['LFA_ALT']['ADAS_ACIAnglTqRedcGainVal'])
    spds.append((parser.vl['WHEEL_SPEEDS']['WHL_SpdFLVal'] + parser.vl['WHEEL_SPEEDS']['WHL_SpdFRVal']) / 2.0 / 3.6)
    cmd_angs.append(parser.vl['LFA_ALT']['ADAS_StrAnglReqVal'])

tqs = np.array(tqs)
gains = np.array(gains)
spds = np.array(spds)
cmd = np.array(cmd_angs)

active = gains > 0.01

# filter to steady-state: low gain rate AND low torque rate
gain_rate = np.abs(np.diff(gains))
gain_rate = np.append(gain_rate, 0)
tq_rate = np.abs(np.diff(tqs))
tq_rate = np.append(tq_rate, 0)
steady = (gain_rate < 0.005) & (tq_rate < 20)

fig, axes = plt.subplots(2, 1, figsize=(16, 10))

# Plot 1: Torque winddown curves by speed
ax = axes[0]
tq_bins = np.arange(0, 750, 25)
tq_centers = (tq_bins[:-1] + tq_bins[1:]) / 2

speed_ranges = [(0, 4, 'C0'), (4, 7, 'C1'), (7, 11, 'C2'), (11, 17, 'C3'), (17, 22, 'C4'), (22, 33, 'C5')]
for spd_lo, spd_hi, color in speed_ranges:
    mask = active & steady & (spds >= spd_lo) & (spds < spd_hi)
    if mask.sum() < 200:
        continue
    p25, medians, p75 = [], [], []
    valid_centers = []
    for i in range(len(tq_bins) - 1):
        m2 = mask & (np.abs(tqs) >= tq_bins[i]) & (np.abs(tqs) < tq_bins[i + 1])
        if m2.sum() < 20:
            continue
        g = gains[m2]
        p25.append(np.percentile(g, 25))
        medians.append(np.median(g))
        p75.append(np.percentile(g, 75))
        valid_centers.append(tq_centers[i])
    ax.plot(valid_centers, medians, 'o-', color=color, label=f'{spd_lo}-{spd_hi} m/s', markersize=3)
    ax.fill_between(valid_centers, p25, p75, alpha=0.1, color=color)

ax.set_xlabel('|User Torque| (Nm)')
ax.set_ylabel('Gain (median + 25-75th percentile)')
ax.set_title('Torque winddown by speed')
ax.legend()
ax.grid(True)

# Plot 2: Ceiling vs speed with scatter
ax = axes[1]

# scatter: ALL active points
idx = np.where(active)[0]
if len(idx) > 10000:
    idx = np.random.choice(idx, 10000, replace=False)
ax.scatter(spds[idx], gains[idx], s=2, alpha=0.4, color='C0', label='all active')

# median line
spd_bins = np.arange(0, 31, 1)
spd_centers = (spd_bins[:-1] + spd_bins[1:]) / 2
medians, p25s, p75s = [], [], []
valid_spd = []
for i in range(len(spd_bins) - 1):
    mask = active & (spds >= spd_bins[i]) & (spds < spd_bins[i + 1])
    if mask.sum() < 30:
        continue
    g = gains[mask]
    medians.append(np.median(g))
    p25s.append(np.percentile(g, 25))
    p75s.append(np.percentile(g, 75))
    valid_spd.append(spd_centers[i])

ax.plot(valid_spd, medians, 'o-', color='red', markersize=5, label='median', zorder=5)
ax.fill_between(valid_spd, p25s, p75s, alpha=0.2, color='red', label='25-75th')
ax.set_xlabel('Speed (m/s)')
ax.set_ylabel('Gain')
ax.set_title('Gain vs speed (all active)')
ax.legend()
ax.grid(True)

plt.tight_layout()
plt.savefig('gain_analysis.png', dpi=150)
plt.show()
