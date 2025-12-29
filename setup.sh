#!/bin/bash
set -e

BASEDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

# TODO: why doesn't uv do this?
export PYTHONPATH=$BASEDIR

# *** dependencies install ***
if [ "$(uname -s)" = "Linux" ]; then
  if ! command -v "mull-runner-17" > /dev/null 2>&1; then
    sudo apt-get update && sudo apt-get install -y curl clang-17
    curl -1sLf 'https://dl.cloudsmith.io/public/mull-project/mull-stable/setup.deb.sh' | sudo -E bash
    sudo apt-get update && sudo apt-get install -y mull-17
  fi
elif [ "$(uname -s)" = "Darwin" ]; then
  if ! brew list llvm@17 &>/dev/null; then
    brew install llvm@17
  fi
  if [ ! -f "$BASEDIR/.mull/bin/mull-runner-17" ]; then
    MULL_VERSION="0.27.1"
    MULL_ZIP="Mull-17-${MULL_VERSION}-LLVM-17.0.6-macOS-aarch64-15.6.1.zip"
    curl -LO "https://github.com/mull-project/mull/releases/download/${MULL_VERSION}/${MULL_ZIP}"
    mkdir -p "$BASEDIR/.mull"
    unzip -o "$MULL_ZIP" -d "$BASEDIR/.mull"
    mv "$BASEDIR/.mull/usr/local/"* "$BASEDIR/.mull/"
    rm -rf "$BASEDIR/.mull/usr"
    rm "$MULL_ZIP"
  fi
  export PATH="$BASEDIR/.mull/bin:$PATH"
fi

if ! command -v uv &>/dev/null; then
  echo "'uv' is not installed. Installing 'uv'..."
  curl -LsSf https://astral.sh/uv/install.sh | sh
fi

export UV_PROJECT_ENVIRONMENT="$BASEDIR/.venv"
uv sync --all-extras
source "$PYTHONPATH/.venv/bin/activate"

$BASEDIR/opendbc/safety/tests/misra/install.sh
