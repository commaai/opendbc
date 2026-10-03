# GWM (Great Wall Motors) Port

Supports the **Haval H6 Mk3 (2019–26)** — the first GWM platform in opendbc.
Lateral control is torque-based steering; longitudinal (gas/brake) is available
behind the `GwmSafetyFlags.LONG_CONTROL` alpha flag.

## Architecture

```
main bus (0)  ── car signals: wheel speeds, brake, gas, steering, EPS feedback
adas bus (1)  ── (parsed, currently unused)
cam bus  (2)  ── stock camera: STEER_CMD, ACC_CMD, LATERAL_STATE (we intercept & modify)
loopback (128 + offset) ── our own transmitted messages, echoed back by the panda
```

Key messages:

| Message | Addr | Dir | Purpose |
|---|---|---|---|
| `STEER_AND_AP_STALK` | 0xA1 | RX | steering angle/rate, AP stalk buttons, cancel |
| `CAR_OVERALL_SIGNALS2` | 0x60 | RX | gas pedal position |
| `BRAKE2` | 0x120 | RX | brake pedal state |
| `WHEEL_SPEEDS` | 0x13B | RX | wheel speeds |
| `RX_STEER_RELATED` | 0x147 | RX/TX | EPS feedback (driver + EPS torque, fault bits); forwarded to camera with simulated driver torque |
| `STEER_CMD` | 0x12B | TX | steering torque command to EPS |
| `ACC_CMD` | 0x143 | TX | longitudinal command to PCM |
| `LATERAL_STATE` | 0x23D | TX | HUD / LKAS state |

## CRC scheme

64-byte messages carry an **independent CRC8 + 4-bit counter per 8-byte block**
(poly `0x1D`, per-block `xor_out`). Known xor constants, all proven on the car
because `gwmcan.py` forges them and the ECUs accept the messages:

| Message / block | xor_out |
|---|---|
| `STEER_AND_AP_STALK` | `0x2D` |
| `WHEEL_SPEEDS` block A | `0x7F` |
| `RX_STEER_RELATED` blocks A & B | `0x61` |
| `BRAKE2` block A | `0xEE` |
| `CAR_OVERALL_SIGNALS2` (gas) blocks A/B/C | `0xF3` / `0x95` / `0xAE` |
| `STEER_CMD` block 2 | `0x9B` |
| `STEER_CMD` block 3 | `0x34` (from the author's `haval_fwd-hook` branch; not yet used here) |
| `ACC_CMD` brake / acc blocks | `0xEF` / `0x87` |
| `LATERAL_STATE` | `0x66` |

The `BRAKE2` and gas constants were derived from a public drive log
(route `075b133b6181e058/00000163`, ~9k/12k frames, 100% consistent, confirmed
on a second segment).

`STEER_CMD` additionally carries a 5-bit arithmetic checksum
(`gwm_basic_chksum_for_0x12B`).

## Safety validation status (`opendbc/safety/modes/gwm.h`)

Checksum + counter validated on RX:

- `STEER_AND_AP_STALK` (0xA1) — cruise buttons
- `WHEEL_SPEEDS` (0x13B) — vehicle speed
- `RX_STEER_RELATED` (0x147) — **block B** (bytes 8–15), where the EPS/driver
  torque used by the torque safety checks lives. CRC at byte 8 covers bytes
  9–15, counter in the low nibble of byte 15.

Checksum validated, counter ignored on RX:

- `BRAKE2` (0x120) — **block A**, CRC byte 0 over bytes 1–7 (brake-pressed bit).
- `CAR_OVERALL_SIGNALS2` (0x60) — **block B**, CRC byte 8 over bytes 9–15
  (`GAS_POSITION`). Counters on both are ignored: in logs they show a
  systematic ~1-in-15 irregularity that would trip false counter faults on
  these safety-critical messages.

The 0x147 counter check assumes the EPS increments block-B counters
monotonically at 50Hz — confirm against a road test before relying on it.

## EPS fault detection (loopback bus)

The EPS acknowledges steering requests via `A_RX_STEER_REQUESTED` in
`RX_STEER_RELATED`. To detect "we are commanding steer but the EPS is not
obeying", carstate needs to know whether a steer command was actually sent.
That comes from the **loopback bus**: the panda echoes every transmitted
message back on `bus + 128`, and carstate parses our own `STEER_CMD` there —
the same pattern GM uses (`gm/carstate.py`).

Two details make this work:

1. The loopback message is registered with `float('nan')` frequency, which
   sets `ignore_alive` — the parser stays `can_valid` even though nothing is
   echoed until openpilot starts transmitting. Without this, `canValid` drops
   before engagement and openpilot raises a CAN error (this is why the first
   loopback attempt in this port's history failed; `ignore_alive` semantics
   are guaranteed by the pure-Python CANParser, July 2025+).
2. The fault counter only advances on cycles where an echo actually arrived
   (`len(vl_all[...]) > 0` guard), since `STEER_CMD` is sent at 50Hz but
   carstate updates at 100Hz.

A fault is raised after ~1s of ignored commands (50 echoed frames), or ~1s of
`EPS_FAULT_PERMANENT` being set. Note the echoed `STEER_REQUEST` already
reflects the driver-torque cutout in carcontroller, so driver overrides do not
count toward a fault.

## Cruise state

Engagement is read from the PCM: `CRUISE_STATE_2` (3 bits) in the camera's
`ACC` message (0x2AB) — **0–2 = deactivated, >2 = active** (0 also observed as
fault/off). Semantics were validated on-car on an H6 PHEV
(otaviobonder's `haval-new-pcm-signal` branch, Feb 2026); GWM appears to share
this system across the mk3 platform. Both carstate and the safety
`pcm_cruise_check` use this signal, replacing an earlier hand-rolled stalk-lever
state machine that could desync from the car's real ACC state. Not yet
re-validated on the non-PHEV H6 — check on the first drive.

## Personality / follow-distance sync (`interface.py`)

The stock ACC cycles through **4** follow distances; openpilot has **3**
personalities (stock 3 and 4 both map to the farthest). When the driver
changes the stock setting, the interface pulses synthetic `gapAdjustCruise`
button events until openpilot's personality matches. Pulses are proper
press→release pairs (openpilot cycles on release), rate-limited to one per 25
frames so the `leadDistanceBars` feedback can round-trip before the next pulse.

**Why this lives in `interface.py`:** it needs `CC.hudControl.leadDistanceBars`
(controller side, captured in `apply()`) *and* emits `ret.buttonEvents`
(state side, in `update()`). The controller cannot emit button events and
carstate cannot see `CC`, so the interface is the only meeting point.

**Design history:** the author's `haval_match-distance-lines` branch shows the
stalk-button-edge approach (emit `gapAdjustCruise` from
`AP_REDUCE_DISTANCE_COMMAND`/`AP_INCREASE_DISTANCE_COMMAND` in carstate, like
Toyota/Hyundai) was implemented first and deliberately replaced with the
display-sync approach. The stalk buttons are consumed by the stock ACC (they
change the car's own distance setting, which openpilot forwards to the
camera), so reacting to the button *and* to the resulting display change would
double-apply. Syncing to `CAR_DISTANCE_SELECTION` keeps a single source of
truth.

## Carcontroller quirks

- A commanded torque of exactly ±1 is doubled to ±2 — repeated ±1 frames were
  observed to cause EPS faults.
- Lateral is cut when driver torque exceeds 1.0 Nm (`MAX_USER_TORQUE`), and a
  simulated driver torque is forwarded to the camera (`create_wheel_touch`) to
  satisfy the stock system's hands-on detection.
- Accel is normalized to [-1, 1] with asymmetric scaling: braking by
  `|ACCEL_MIN|` (-3.5), acceleration by `ACCEL_MAX` (2), then mapped to raw
  `BRAKE_CMD`/`GAS_CMD` ranges in `gwmcan.py`.

## Fingerprinting

Only the engine ECU responds to UDS queries, and only on the OBD port
(`GREATWALLMOTORS_RX_OFFSET = 0x6a`). No other ECUs have been reachable —
fingerprinting relies on that single firmware plus CAN fingerprints.

## Stock LKAS coexistence (experimental, not in this PR)

Stock LKAS shares the `STEER_CMD` (0x12B) message. The author's
`haval_fwd-hook` branch prototypes dynamic forwarding: the safety rx hook
detects stock LKAS engagement (rising edge of the camera's `STEER_REQUEST`
while controls are not allowed), a fwd hook then forwards the stock camera
`STEER_CMD` instead of blocking it, and the tx hook rejects openpilot steering
while stock LKAS is active. That branch also bumps the counter by +2 to avoid
a gap during the handover. Relevant when reviewing relay/forwarding behavior.

## History

An earlier port attempt (openpilot#32880 + opendbc#1086, 2024, with
@celobusana's H6 PHEV) was closed when car code moved to the opendbc repo.
The CRC8 algorithm (poly `0x1D`, xor `0x2D` for the stalk message) was first
cracked in openpilot#32877, which includes 300+ logged sample vectors for
`STEER_AND_AP_STALK`. Notes from that effort: the H6 PHEV is a distinct
variant, and the Haval H6 "GT" needs a different harness from the regular H6.
