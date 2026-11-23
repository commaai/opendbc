#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

source ../../../setup.sh

# TODO: get UBSan and coverage without running the tests twice.
# Run UBSan separately so its checks do not count as coverage branches.
SAFETY_COVERAGE=0 python -m unittest discover -s .

# reset coverage data, including metadata from previous builds
rm -rf ./libsafety/debug ./libsafety/release

# run safety tests on the debug and the release build in parallel and generate coverage data
SAFETY_COVERAGE=1 SAFETY_RELEASE=1 python -m unittest discover -s . -q &
SAFETY_COVERAGE=1 python -m unittest discover -s .
wait $!

# NOTE: we accept that these tools will have slight differences,
# and in return, we get to use the stock toolchain instead of
# installing LLVM on all users' machines
if [ "$(uname)" = "Darwin" ]; then
  GCOV_EXEC="xcrun llvm-cov gcov"
else
  GCOV_EXEC="gcov"
fi

# generate and open report
if [ "$1" == "--report" ]; then
  for build in debug release; do
    mkdir -p coverage-out/$build
    gcovr -r ../ --gcov-executable "$GCOV_EXEC" --html-nested coverage-out/$build/index.html libsafety/$build
  done
  sensible-browser coverage-out/debug/index.html
fi

# test coverage of both builds. The release build is only required to be covered with SAFETY_RELEASE_COVERAGE=1,
# since it still has code that only the debug build can reach.
for build in debug release; do
  GCOV="gcovr -r $DIR/../ --gcov-executable \"$GCOV_EXEC\" -d --fail-under-line=100 --fail-under-branch=100 --txt-metric branch -e ^libsafety $DIR/libsafety/$build"
  if GCOV_OUTPUT="$(eval $GCOV)"; then
    echo "SUCCESS: All checked files have 100% line and branch coverage in the $build build!"
  elif [ "$build" == "debug" ] || [ "$SAFETY_RELEASE_COVERAGE" == "1" ]; then
    echo -e "FAILED: $build build\n$GCOV_OUTPUT"
    exit 1
  else
    echo -e "WARNING: $build build coverage is not required yet\n$GCOV_OUTPUT"
  fi
done
