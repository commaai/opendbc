#!/usr/bin/env bash
# shellcheck disable=SC1090
# SC1090 -> don't try to follow sourced files

IP_ADDRESS=${1:?"The Loop server IP Address is expected as the first command line argument in order to run integration tests against the Loop server"}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "${SCRIPT_DIR}/exit_routines.sh"

OUTPUT_DIR="${SCRIPT_DIR}/output/run-integration-tests"

pushd "${SCRIPT_DIR}/../.." > /dev/null 2>&1 || exit 1
check_bail $?

mkdir -p "${OUTPUT_DIR}"
check_bail_with_popd $?

pip3 install --upgrade loop_python_client
check_bail_with_popd $?

pip3 install -r python/requirements.txt
check_bail_with_popd $?

"${SCRIPT_DIR}"/clean.sh
check_bail_with_popd $?

export LOOPSERVERIPADDR="${IP_ADDRESS}"
export TESTCANADAPTER="can0"
python3 -m nose --with-xunit --xunit-file="${OUTPUT_DIR}/${IP_ADDRESS}-integration-nosetests.xml" -w python
check_bail_with_popd $?

echo "Integration test results placed in ${OUTPUT_DIR}..."
exit_with_popd 0
