#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

source $DIR/../../../setup.sh

GIT_REF="${GIT_REF:-origin/master}"
GIT_ROOT=$(git rev-parse --show-toplevel)
cat > $GIT_ROOT/mull.yml <<EOF
mutators: [cxx_increment, cxx_decrement, cxx_comparison, cxx_boundary, cxx_bitwise_assignment, cxx_bitwise, cxx_arithmetic_assignment, cxx_arithmetic, cxx_remove_negation]
timeout: 1000000
gitProjectRoot: $GIT_ROOT
EOF

scons -j4 -D

export MUTATION=1

# Create temporary test runner script
TEST_RUNNER=$(mktemp)
chmod +x $TEST_RUNNER
cat > $TEST_RUNNER <<EOF
#!/usr/bin/env bash
set -e

# run with unittest first since it has way less overhead than pytest
# toyota should catch ~75% of it
./test_toyota.py -f

# last resort, run full test suite
pytest -n8 --ignore-glob=misra/*
EOF

mull-runner-17 \
  --include-not-covered \
  --ld-search-path /lib/x86_64-linux-gnu/ \
  ./libsafety/libsafety_mutation.so \
  --workers=16 \
  --test-program=$TEST_RUNNER
  #--test-program=pytest -- -h