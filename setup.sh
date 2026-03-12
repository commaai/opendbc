#!/bin/bash
set -e

BASEDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

# TODO: why doesn't uv do this?
export PYTHONPATH=$BASEDIR

# *** dependencies install ***
if ! command -v uv &>/dev/null; then
  echo "'uv' is not installed. Installing 'uv'..."
  curl -LsSf https://astral.sh/uv/install.sh | sh

  # must source this after install on some platforms
  if [ -f $HOME/.local/bin/env ]; then
    source $HOME/.local/bin/env
  fi
fi

export UV_PROJECT_ENVIRONMENT="$BASEDIR/.venv"
uv sync --all-extras --inexact
source "$PYTHONPATH/.venv/bin/activate"

# *** mypyc compilation ***
uv pip install mypy setuptools
(cd "$BASEDIR" && MYPYC=1 python setup.py build_ext --inplace)
