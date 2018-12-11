#!/usr/bin/env bash
# shellcheck disable=SC1090
# SC1090 -> don't try to follow sourced files

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "${SCRIPT_DIR}/exit_routines.sh"

#pushd "${SCRIPT_DIR}/../../python-tests" > /dev/null 2>&1 || exit 1

#python setup.py clean --all
#check_bail_with_popd $? 

exit_with_popd 0
