#!/usr/bin/env bash
# shellcheck disable=SC1090
# SC1090 -> don't try to follow sourced files

function dump_stack() {
  local frame=0
  while caller $frame; do
    ((frame++));
  done
  echo "$*"
}

function check_bail()
{
  if [ "$1" -ne 0 ]; then
    dump_stack "Exiting due to error code '$1'"
    exit "$1"
  fi
}

function check_bail_with_popd()
{
  if [ "$1" -ne 0 ]; then
    dump_stack "Popping and exiting due to error code '$1'"
    popd > /dev/null 2>&1 || exit "$1"
    exit "$1"
  fi
}

function exit_with_popd()
{
    popd > /dev/null 2>&1 || exit "$1"
    exit "$1"
}
function exit_with_code()
{
    echo "Exiting with code '$1'..."
    exit "$1"
}
