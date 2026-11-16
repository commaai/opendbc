#!/usr/bin/env bash
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd $DIR

source ../../../../setup.sh

GREEN="\e[1;32m"
RED="\e[1;31m"
NC='\033[0m'

OPENDBC_ROOT=${OPENDBC_ROOT:-$BASEDIR}

misra() {
  # exits with 1 on findings and 2 on analysis or usage errors
  # --inline-suppr honors the cppcheck-suppress comments in the sources, misrable reads cppcheck's syntax
  if ! misrable -I $OPENDBC_ROOT --inline-suppr \
         --suppressions-list $DIR/suppressions.txt \
         --external-api $DIR/external_api.txt \
         "$@" $OPENDBC_ROOT/opendbc/safety/tests/misra/main.c; then
    printf "${RED}** FAILED: MISRA violations or analysis errors!${NC}\n"
    exit 1
  fi
}

printf "\n${GREEN}** Safety **${NC}\n"
misra
printf "\n${GREEN}** Safety with ALLOW_DEBUG **${NC}\n"
misra -DALLOW_DEBUG

printf "\n${GREEN}Success!${NC} took $SECONDS seconds\n"
