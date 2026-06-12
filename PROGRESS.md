# PROGRESS — opendbc #2557: 100% branch coverage gate

Tracking file for the branch-coverage work. **Delete before marking PR ready.**

## Phase 0 — state (2026-06-12)
- Issue #2557 OPEN, `bounty` label, no GH assignee. Maintainer locked it to @mpurnell1 in a comment.
- Competing PRs:
  - #2944 (mpurnell1) "Enforce 100% branch coverage" — OPEN draft, 35 files, stale since 2026-03-27.
  - #3148 (mpurnell1) — CLOSED 2026-06-08 (auto-close, 60d inactivity).
  - #3436 (obiyang) — OPEN, ignition tests only.
- Decision (user): proceed anyway; treat the stale lock as abandoned.
- Draft PR opened: commaai/opendbc#3462 (head thomaskgould:branch-coverage).

## Environment (macOS local — advisory; Linux CI is authoritative)
- venv: `.venv-cov` (cffi, hypothesis==6.47.*, numpy, pycapnp, gcovr 8.6)
- `export SDKROOT="$(xcrun --show-sdk-path)"` (CLT clang needs it)
- `export PATH="$PATH:/Library/Developer/CommandLineTools/usr/bin"` (llvm-cov)
- `export PYTHONPATH=<repo>`
- Run: `cd opendbc/safety/tests && rm -f libsafety/*.gcda && python -m unittest discover -s . && gcovr -r ../ --gcov-executable "llvm-cov gcov" -e ^libsafety --txt-metric branch`

## Phase 1 — baseline (llvm-cov local, master @ 75889fd9)
- 3196 tests pass (~26s), skipped=397.
- Branch coverage 92% (1900 branches, 1765 taken, 135 missing).

## Phase 3 — triage (R=remove, W=rewrite, T=test) — DONE (6 Fable agents, cross-verified)

### Dominant patterns
- **R (whitelist-redundant guard):** rx/tx hooks only run for messages matching the RxCheck/TX whitelist (addr+bus+len, safety.h:111-126,192-199,237-246). In-hook re-checks of bus/addr that the whitelist already pins are provably dead → delete. Invariant relied on: "each addr is on exactly one bus per config."
- **W (terminal else-if over enumerated set):** counter/checksum/quality/crc dispatchers run only for enumerated addrs → final `else if (addr==X)` can't be false → collapse to `else` (also removes MISRA empty-else stub).
- **W (operand order):** `(addr==X) && flag` where X is whitelisted only when flag set → swap to `flag && (addr==X)` so both edges are coverable.
- **T:** real behavior untested — cruise/ACC engaged-states (3/4/5/6), gas-signal bit decomposition, per-wheel vehicle_moving OR, held/edge buttons, tester-present nonzero tail, etc.

### Per-line (R/W/T)
**Core**
- helpers.h:65 R (drop defensive `SAFETY_MAX(dx,0.0001)`; dx>0 by loop invariant)
- ignition.h:12,16,23,37,52,58 T (wrong-bus + wrong-len per-brand ignition tests: GM/Rivian/Tesla/Mazda/VW-MEB)
- lateral.h:19 T (sign-loop test_non_realtime_limit_down, common.py); :70 W (CLAMP→SAFETY_MIN, Rivian max_torque always >0); :363,364,375 T (Tesla angle-meas beyond ±max_angle)
- safety.h:131 T (wrong-DLC after msg_seen); :149 R (update_counter index!=-1 dead); :166 W (drop compute_checksum NULL operand); :321 R (safety_tick cfg!=NULL dead); :329+337 W (collapse 1e6 lag floor + move ≥10Hz check to CI assert); :331 T (fresh non-lagging tick); :337 T (is_msg_valid both ways); :367 T (sustained Tesla disengage); :485 T+W (unknown mode -1 test + drop init!=NULL); :503,505 R (drop SAFETY_MAX(bits,0))

**body/chrysler/ford/gm**
- body.h:6 R (only rx 0x201); :20 T (2nd magic word false + flash_msg false)
- chrysler.h:70 R (drop bus operand)
- chrysler_cusw.h:25 R (bus wrapper); :86 T (no-button-pressed case)
- ford.h:29,42,57,75 W (else-if→else in 4 getters); :100 R (bus wrapper); :145 T (CcStat==4 engaged)
- gm.h:35 R (bus wrapper); :51 T (camera buttons-dont-engage, pcm_cruise); :146 R (drop gm_pcm_cruise, tx-whitelist pins)

**honda/hyundai**
- honda.h:137 T (brake_switch 2-frame); :147 T (DISABLE_STOCK_AEB); :148 R (drop bus==2); :188,201,212,228,259 R (drop bus==bus_pt/bus_buttons; remove now-unused locals); :240 T (0x194 steer); :241 T (steer allowed when engaged); :251 T (0xE5 supplemental); :267 T (tester-present tail)
- hyundai.h:72,88 W (else-if→else); :104 W (rewrite to `(j<6)||((i%2)==0)`, clang artifact); :157 T (E_EMS11 in ICE); :159 W (swap flag&&addr); :232 T (tester-present tail); :238 T (tx buttons in long)
- hyundai_canfd.h:99,101,103 T (cross-config gas msgs); :102 T (pedal raw=1/512 bits); :119,120 T (single-wheel moving); :129 W (swap !long&&addr); :183 T (shared tester-present); :205 T (cancel raw=0,val≠0)
- hyundai_common.h:79 R (remove !long wrapper — caveat: latent guard); :80 T (steady-state engaged); :131 W (len 24/32 ternary)

**mazda/nissan/psa/rivian/subaru**
- mazda.h:20 R (rx bus); :40 T (PEDAL_GAS≥16 data[4]); :62 R (tx bus)
- nissan.h:64 T (cruise on non-selected whitelisted bus)
- psa.h:22,53 W (else-if→else); :77,84 R (inner addr dead); :106 R (single tx msg)
- rivian.h:37,48 W (else-if→else); :88 R (inner addr dead)
- subaru.h:96,104,110,121,125 R (drop bus operand, remove alt_main_bus); :116 T (per-wheel moving)
- subaru_preglobal.h:23 R (rx bus wrapper)

**tesla/toyota**
- tesla.h:27 T (UI_warning 0x311 rx); :33 W (else-if→else); :49 T+W (0x286/0x311 + final else); :60,70 R (checksum_byte!=-1 after total dispatch); :87 W (else→else); :101 W (ctrl_type ternary); :170,171,172 T (cruise states 3/4/6); :197 T (LKAS 2nd frame + controls_allowed); :312 T (autopark fwd passthrough)
- toyota.h:83 W (else-if→else); :98 R (rx bus wrapper); :123 T (angle init in Torque); :257 T (AEB data[4]); :299 T (asymmetric driver torque window)

**volkswagen**
- volkswagen_common.h:80 W (else-if→else crc)
- meb.h:37 W (else-if→else crc); :91 R (rx bus); :98 T (per-wheel moving); :121 T (ACC 4/5); :143 T (button held)
- mlb.h:49 T (LS_01 no-cancel); :70 W (collapse bus/addr); :100 T (HCA status 5)
- mqb.h:38 R (rx bus); :63 T (ACC 4/5); :79 T (stock GRA rx); :82 T (button held)
- pq.h:26 W (else-if→else counter)

### Execution log (Phase 4)
_per-commit results below._

