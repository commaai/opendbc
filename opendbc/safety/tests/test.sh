#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

source ../../../setup.sh

# reset coverage data and generate gcc note file
rm -f ./libsafety/*.gcda
if [ "$1" == "--ubsan" ]; then
  scons -j$(nproc) -D --ubsan
else
  scons -j$(nproc) -D
fi

# run safety tests and generate coverage data
pytest -n8 --ignore-glob=misra/*

# generate a nice HTML report
mkdir -p coverage-out/
gcovr -r ../ --html-nested coverage-out/index.html
echo "View the coverage report at: coverage-out/index.html"

# test coverage
GCOV="gcovr -s -r $DIR/../ -d --fail-under-line=100 -e ^libsafety"
if ! GCOV_OUTPUT="$($GCOV)"; then
  echo "$GCOV_OUTPUT"
  echo -e "FAILED: coverage check!"
  exit 1
else
  echo -e "$GCOV_OUTPUT"
  echo "SUCCESS: All checked files have 100% coverage!"
fi
