# PROGRESS — opendbc #2557: 100% branch coverage gate

Tracking file for the branch-coverage work. **Delete before marking PR ready.**

## Phase 0 — state (2026-06-12)
- Issue #2557 OPEN, `bounty` label, no GH assignee. Maintainer locked it to @mpurnell1 in a comment.
- Competing PRs:
  - #2944 (mpurnell1) "Enforce 100% branch coverage" — OPEN draft, 35 files, stale since 2026-03-27.
  - #3148 (mpurnell1) — CLOSED 2026-06-08 (auto-close, 60d inactivity).
  - #3436 (obiyang) — OPEN, ignition tests only.
- Decision (user): proceed anyway; treat the stale lock as abandoned.

## Environment (macOS local — advisory; Linux CI is authoritative)
- venv: `.venv-cov` (cffi, hypothesis==6.47.*, numpy, pycapnp, gcovr 8.6)
- `export SDKROOT="$(xcrun --show-sdk-path)"` (CLT clang needs it)
- `export PATH="$PATH:/Library/Developer/CommandLineTools/usr/bin"` (llvm-cov)
- `export PYTHONPATH=<repo>`
- Run: `cd opendbc/safety/tests && rm -f libsafety/*.gcda && python -m unittest discover -s . && gcovr -r ../ --gcov-executable "llvm-cov gcov" -e ^libsafety --txt-metric branch`

## Phase 1 — baseline (llvm-cov local, master @ 75889fd9)
- 3196 tests pass (~26s), skipped=397.
- Branch coverage 92% (1900 branches, 1765 taken, 135 missing).

## Phase 3 — triage (R=remove, W=rewrite, T=test)
_To be filled per file:line._
