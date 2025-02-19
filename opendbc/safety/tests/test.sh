#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

# TODO: why doesn't uv do this?
export PYTHONPATH=../../../$DIR

# *** dependencies install ***
if ! command -v uv &>/dev/null; then
  echo "'uv' is not installed. Installing 'uv'..."
  curl -LsSf https://astral.sh/uv/install.sh | sh
fi

uv sync --all-extras
source ../../../.venv/bin/activate

# reset coverage data and generate gcc note file
rm -f ./libsafety/*.gcda
if [ "$1" == "--ubsan" ]; then
  scons -j8 -D --coverage --ubsan
else
  scons -j8 -D --coverage
fi

# run safety tests and generate coverage data
pytest test_*.py

# generate and open report
if [ "$1" == "--report" ]; then
  geninfo ./libsafety/ -o coverage.info
  genhtml coverage.info -o coverage-out
  sensible-browser coverage-out/index.html
fi

# test coverage
GCOV_OUTPUT=$(gcov -n ./libsafety/safety.c)
INCOMPLETE_COVERAGE=$(echo "$GCOV_OUTPUT" | paste -s -d' \n' | grep -E "File.*(\/safety\/safety_.*)|(safety)\.h" | grep -v "100.00%" || true)
if [ -n "$INCOMPLETE_COVERAGE" ]; then
  echo "FAILED: Some files have less than 100% coverage:"
  echo "$INCOMPLETE_COVERAGE"
  exit 1
else
  echo "SUCCESS: All checked files have 100% coverage!"
fi
