# BYD / Chery (Omoda, iCaur, JAECOO) car port

**Status:** lives on branch `port/upstream-bump-byd-chery-jaecoo`, not merged to
`master`. The `dev/EOP10` submodule pin (`opendbc_repo@2cde2462`) is
unchanged — bumping it is a separate, deliberate step (see "What this is not"
below).

## Where this came from

Ported from [kommuai/opendbc](https://github.com/kommuai/opendbc) (MIT
licensed, same as this fork), which carries the most complete community BYD
and Chery car support available. JAECOO is Chery's sub-brand, so JAECOO
support here is the Chery module (`opendbc/car/chery/`) — there is no
separate "JAECOO" car directory. Also pulled in was BYD's own multi-DBC,
`cam_lka`/`mpc_lka`-split port (`opendbc/car/byd/`), which is more complete
than the single-DBC BYD Atto 3 port `dev/EDP10` vendors inline.

## Platforms

- **Chery** (`opendbc/car/chery/`, `chery_general_pt.dbc`,
  `opendbc/safety/modes/chery.h`): `CHERY_JAECOO_J7_PHEV` (2024-26, marked
  "under validation" upstream), `CHERY_TIGGO_8_PRO`, `CHERY_OMODA_5`,
  `CHERY_ICAUR_03`. Lateral-only (angle control), no longitudinal control,
  no radar.
- **BYD** (`opendbc/car/byd/`, `byd_general_pt.dbc` +
  `byd_han_dmev_2020.dbc` + `byd_radar_fd.dbc`/`byd_radar_seal6_fd.dbc`,
  `opendbc/safety/modes/byd.h`): `BYD_ATTO3`, `BYD_M6`, `BYD_SEAL`,
  `BYD_SEALION7`, `BYD_SEAL6`, `BYD_SHARK` (angle control, `cam_lka`),
  `BYD_SONG_PLUS_DMI_21` (torque control, `mpc_lka`).

## `car.capnp` `SafetyModel` enum numbering

`byd @35` deliberately matches `dev/EDP10`'s own
`opendbc_repo/opendbc/car/car.capnp` numbering (EDP10 already carries a BYD
Atto 3 port) — a mismatched number would decode a recorded route's safety
model to the wrong brand if the route is ever opened on the other branch.
`chery @36` is new; EDP10 has no Chery port to match against.

## What changed versus a straight copy

This fork's core was bumped from v0.2.1 (2025-02-10, predates the
`opendbc/safety/` migration entirely) to current upstream first — byd/chery
don't import against that old core at all. On top of the bump, real API
drift between kommuai's base and this fork's newer one required fixes, not
just a file copy:

- `car.CarParams`/`car.CarState` (pre-migration `cereal` import) → `opendbc.car.structs`.
- `AngleSteeringLimits` (byd.h): dropped fields this fork's C struct doesn't
  have; verified every one was a no-op in the source config already, or (for
  `inactive_angle_is_zero`) that this fork's fixed behavior is the *stricter*
  of the two options — not a safety regression.
- `startingState`/`startAccel`/`stoppingDecelRate`/`.brake` moved into
  `car.capnp`'s `deprecated :group` in this fork's core →
  `ret.deprecated.X`. **Note:** `startingState`/`startAccel`/
  `stoppingDecelRate` are still live fields in this *project's* own
  `cereal/car.capnp` (`selfdrive/controls/lib/longcontrol.py` reads them) —
  whether the `structs.CarParams` → `cereal.car.CarParams` conversion this
  project uses actually threads the `deprecated.*` value through was not
  traced; flagged, not assumed working.
- `personality`/`lkaDisabled` (CarState): kommuai-only schema fields absent
  from both this fork's `car.capnp` and this project's own
  `cereal/car.capnp`. Dropped the assignments — BYD's distance-personality
  value still reaches the UI via the standard `buttonEvents` path; Chery's
  doesn't (no fallback existed), but neither field has a consumer in this
  project regardless.
- `chery/values.py`'s `CarControllerParams` didn't dispatch to
  `MpcLkaCarControllerParams` for BYD's torque-controlled MPC_LKA platform;
  `test_lateral_limits.py` needs it to. Added `__new__` dispatch.
- Fixed a real pre-existing bug in kommuai's own `test_chery.py`: the Omoda
  safety-param HUD-forwarding tests asserted the opposite of what
  `chery_fwd_hook`'s own comment documents ("Omoda/iCaur: leave native
  HUD"). Fixed the test to match the code.
- `dbc_dict()` / `CUSTOM_CAR_PARTS`: small kommuai-only helpers both ports
  depend on, added to `opendbc/car/__init__.py` / `docs_definitions.py`.
  Added the `CarDocs` fields kommuai's docs schema uses
  (`variant`/`acc_low_speed`/etc.) but dropped `kommu_supported` — a
  kommu.ai-branding flag with no place in this fork.
- `opendbc/car/torque_data/override.toml` and `opendbc/car/tests/routes.py`
  needed BYD/Chery entries (generic infra every registered car must have an
  entry in).

## Known gaps — do not assume complete

- **`CHERY_JAECOO_J7_PHEV` and `CHERY_TIGGO_8_PRO` are not uniquely
  identifiable by CAN fingerprint.** kommuai's own `chery/fingerprints.py`
  reuses the JAECOO capture byte-for-byte for Tiggo 8 Pro. Documented
  in-file (`opendbc/car/chery/fingerprints.py`); needs a real Tiggo 8 Pro
  capture or a distinguishing `FW_VERSIONS` entry to resolve — not
  fabricated here.
- **No BrownPanda/TC375 firmware wiring.** `byd.h`/`chery.h` exist as C
  safety-mode source here, but `~/panda/TC375_BrownPanda` (the actual
  gateway firmware, itself still bring-up status) vendors its own separate
  copy of the safety framework and wasn't touched. Nothing here makes
  BYD/Chery drivable on real hardware.
- **No generic `car_helpers.get_car()` dispatch wired into `dev/EOP10` at
  all.** EOP10 currently has a Tesla-only custom path
  (`system/socketd/vehicle/tesla/`), not the standard fingerprint-and-dispatch
  flow this port (like every other brand) relies on.

## What this is not

Not a bump of the `dev/EOP10` submodule pin, not a merge to this fork's
`master`. Both are separate, deliberate follow-ups — `.gitmodules` pins
`branch = master`, so landing an unverified core bump there is exactly what
a future `git submodule update --remote` would silently pull onto EOP10.

## Validation

Full `opendbc/` test suite after the port: 3723 passed, 1718 skipped, 21098
subtests passed. Two known subfailures are the JAECOO/Tiggo fingerprint gap
above; one (`test_misra_mutation`) is a missing-`cppcheck`-binary
environment gap unrelated to any car brand.
