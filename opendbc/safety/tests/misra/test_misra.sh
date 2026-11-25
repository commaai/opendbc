#!/usr/bin/env bash
set -e

# Checks the safety code for MISRA C:2012 violations with misrable, and runs cppcheck's generic checks.
#
# Scope: main.c includes all of the safety code (safety.h and ignition.h). Both builds, release and ALLOW_DEBUG,
# are checked with the compiler flags that change how panda's firmware build translates it (Dir 1.1).
# external_api.txt lists what panda's firmware and the tests use, so the checks see the whole system.
# misrable checks every rule; `misrable --list-rules` shows how far each check goes. Directives:
#   * 4.3, 4.10 and 4.12 are checked by misrable; 2.1, 4.1 and 4.11 by the rules they map to (1.1, 1.3, 18.1).
#   * 4.7: Rule 17.7 reports every discarded result. The (void) casts in safety.h discard errors that are handled
#     already: safety_redundancy_check sets the fault itself, and set_safety_hooks returns copy A's status.
#   * 4.14: CAN messages are the only external input; the RX checks validate them (length, checksum, counter,
#     quality flag, timing) before a safety mode uses one, and the rules treat their contents as any value.
#   * The rest are advisory or about the development process.
#
# Deviations, each documented where it is suppressed:
#   * Rule 1.2 (advisory) in can.h: CANPacket_t needs __attribute__((packed, aligned(4))) for panda's packet layout.
#     Risk: none in practice, gcc and clang are the only compilers that build the safety code and both support it.
#   * Rule 8.9 (advisory) in safety.h: safety_hook_registry stays at file scope, next to the hooks it lists.
#     Risk: none, it's a const table.
#
# Rules 2.1, 2.2 and 14.3 (unreachable code, dead code and invariant conditions) need runtime values that a static
# check can't always know. misrable reports what it can prove, and the safety tests cover the rest: they must reach
# every line and branch of the ALLOW_DEBUG build (tests/test.sh), and mutation testing (tests/mutation.py) shows
# that each operator changes what the tests observe.
# TODO: the same coverage for the release build, and mutants that delete statements and operands.
#
# Rule 1.3 (undefined behavior) assumes the execution model in declarations.h: the entry points never run
# concurrently or re-enter one another, and only they write the safety state. The entry points check the first part
# at runtime: a call that breaks it changes nothing and sets safety_redundancy_fault, see declarations.h.

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd $DIR

source ../../../../setup.sh

cd ${OPENDBC_ROOT:-$BASEDIR}

# panda's firmware flags that change the translation, see `misrable --help`
PANDA_CFLAGS="-mcpu=cortex-m7 -mthumb -mlittle-endian -mhard-float -mfpu=fpv5-d16 -std=gnu11 -fsingle-precision-constant"

misrable -I . --inline-suppr --external-api opendbc/safety/tests/misra/external_api.txt \
         --config= --config=-DALLOW_DEBUG $PANDA_CFLAGS opendbc/safety/tests/misra/main.c

# cppcheck checks only the configuration given with -D, so each build gets its own run
for config in "" -DALLOW_DEBUG; do
  cppcheck --enable=all --check-level=exhaustive --safety --inline-suppr --error-exitcode=1 --quiet \
           --platform=arm32-wchar_t4 --std=c11 -D__GNUC__=9 $config -I . --suppress=missingIncludeSystem \
           --suppress=checkersReport opendbc/safety/tests/misra/main.c
done
