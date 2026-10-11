#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$DIR/../../../.." && pwd)"

case "$(uname -s)" in
  Linux) OS=linux ;;
  Darwin) OS=darwin ;;
  *) echo "Unsupported operating system: $(uname -s)" >&2; exit 2 ;;
esac
case "$(uname -m)" in
  x86_64) ARCH=amd64 ;;
  arm64|aarch64) ARCH=arm64 ;;
  *) echo "Unsupported architecture: $(uname -m)" >&2; exit 2 ;;
esac

cd "$ROOT"
# libsafety is the analyzed program. Everything declared in the external-api
# files is used from Python through cffi (see libsafety/libsafety_py.py).
exec "$DIR/bin/misra-$OS-$ARCH" -I . -D ALLOW_DEBUG -D __GNUC__=9 \
  --external-api opendbc/safety/can.h \
  --external-api opendbc/safety/declarations.h \
  --external-api opendbc/safety/ignition.h \
  --external-api opendbc/safety/tests/libsafety/safety.c \
  "$@" opendbc/safety/tests/libsafety/safety.c
