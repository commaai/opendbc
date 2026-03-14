#!/bin/bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
pip3 install -e .[testing,docs]  # install dependencies
scons -j8                        # build with 8 cores
pytest .                         # run the tests
pytest opendbc/can_common.py     # run specific tests for can_common
lefthook run lint                # run the linter
# *** uv lockfile check ***
uv lock --check

# *** lint + test ***
lefthook run test

# *** all done ***
GREEN='\033[0;32m'
NC='\033[0m'
printf "\n${GREEN}All good!${NC} Finished lint and test in ${SECONDS}s\n"
