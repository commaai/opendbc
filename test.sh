#!/bin/bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

source ./setup.sh

# reset safety coverage data and generate gcc note file
rm -f ./opendbc/safety/libsafety/*.gcda

echo "hi $@"

# *** build ***
scons -j8 --coverage "$@"

# *** lint + test ***
lefthook run test

# generate and open report
if [ "$1" == "--report" ]; then
  mkdir -p coverage-out
  gcovr -r ../ --html-nested coverage-out/index.html
  sensible-browser coverage-out/index.html
fi

# test coverage
GCOV="gcovr -r ../ --fail-under-line=100 -e ^libsafety -e ^../board"
if ! GCOV_OUTPUT="$($GCOV)"; then
  echo -e "FAILED:\n$GCOV_OUTPUT"
  exit 1
else
  echo "SUCCESS: All checked files have 100% coverage!"
fi

# *** all done ***
GREEN='\033[0;32m'
NC='\033[0m'
printf "\n${GREEN}All good!${NC} Finished build, lint, and test in ${SECONDS}s\n"
