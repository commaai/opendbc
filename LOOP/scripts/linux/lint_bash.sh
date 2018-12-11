#!/usr/bin/env bash
# shellcheck disable=SC1090
# SC1090 -> don't try to follow sourced files

# lint_bash [directory to lint, defaults tocurrent]

function do_lint()
{
    if ! command -v shellcheck > /dev/null 2>&1 ; then
        echo "!!!INSTALL shellcheck for Bash Lint!!!"
        return 2
    fi

    shellcheck --version

    local COUNT=0
    while IFS= read -r line
    do
         shellcheck "${line}"  || COUNT+=1
    done < <(grep -Irl --exclude-dir=.git --exclude-dir=node_modules  -e '^#!/usr/bin/env bash' -e '^#!/bin/bash' .)

    [[ COUNT -eq 0 ]] || return 1

    return 0
}

TARGET_DIR="${1:-.}"
pushd "${TARGET_DIR}" > /dev/null 2>&1 || exit 1

do_lint 
RETURN_CODE=$?

popd > /dev/null 2>&1 || exit 1

exit ${RETURN_CODE}
