#!/usr/bin/env python3
import os
import glob
import shutil
import subprocess
import tempfile
import random
from concurrent.futures import ThreadPoolExecutor

random.seed(0)

HERE = os.path.abspath(os.path.dirname(__file__))
ROOT = os.path.join(HERE, "../../../../")

IGNORED_PATHS = (
  'opendbc/safety/main.c',
  'opendbc/safety/tests/',
  'opendbc/safety/board/',
)

mutations = [
  # no mutation, should pass
  (None, None, lambda s: s, False),
]

patterns = [
  ("misra-c2012-10.3", lambda s: s + "\nvoid test(float len) { for (float j = 0; j < len; j++) {;} }\n"),
  ("misra-c2012-13.3", lambda s: s + "\nvoid test(int tmp) { int tmp2 = tmp++ + 2; if (tmp2) {;}}\n"),
  ("misra-c2012-13.4", lambda s: s + "\nint test(int x, int y) { return (x=2) && (y=2); }\n"),
  ("misra-c2012-13.5", lambda s: s + "\nvoid test(int tmp) { if (true && tmp++) {;} }\n"),
  ("misra-c2012-13.6", lambda s: s + "\nvoid test(int tmp) { if (sizeof(tmp++)) {;} }\n"),
  ("misra-c2012-14.2", lambda s: s + "\nvoid test(int cnt) { for (cnt=0;;cnt++) {;} }\n"),
  ("misra-c2012-14.4", lambda s: s + "\nvoid test(int len) { if (len - 8) {;} }\n"),
  ("misra-c2012-16.4", lambda s: s + "\nvoid test(int temp) {switch (temp) { case 1: ; }}\n"),
  ("misra-c2012-20.4", lambda s: s + "\n#define auto 1\n"),
  ("misra-c2012-20.5", lambda s: s + "\n#define TEST 1\n#undef TEST\n"),
]

all_files = glob.glob('opendbc/safety/**', root_dir=ROOT, recursive=True)
files = [f for f in all_files if f.endswith(('.c', '.h')) and not f.startswith(IGNORED_PATHS)]
assert len(files) > 20, files

for p in patterns:
  mutations.append((random.choice(files), *p, True))

mutations = random.sample(mutations, 2)  # can remove this once cppcheck is faster


def _run_single_mutation(args):
  fn, rule, transform, should_fail = args
  with tempfile.TemporaryDirectory() as tmp:
    shutil.copytree(ROOT, tmp, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('.venv', 'cppcheck', '.git', '*.ctu-info', '.hypothesis'))

    # apply patch
    if fn is not None:
      with open(os.path.join(tmp, fn), 'r+') as f:
        content = f.read()
        f.seek(0)
        f.write(transform(content))

    # run test
    r = subprocess.run(f"OPENDBC_ROOT={tmp} opendbc/safety/tests/misra/test_misra.sh",
                       stdout=subprocess.PIPE, cwd=ROOT, shell=True, encoding='utf8')
    failed = r.returncode != 0
    return fn, rule, should_fail, failed, r.stdout


def test_misra_mutation():
  """Run all sampled mutations concurrently."""
  with ThreadPoolExecutor(max_workers=len(mutations)) as pool:
    results = list(pool.map(_run_single_mutation, mutations))
  for fn, rule, should_fail, failed, stdout in results:
    print(stdout)
    assert failed == should_fail, f"Mutation {fn} {rule}: expected fail={should_fail}, got {failed}"
    if should_fail:
      assert rule in stdout, f"MISRA test failed but not for the correct violation: {rule}"
