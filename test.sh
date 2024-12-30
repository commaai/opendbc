#!/bin/bash
set -e

# *** dependencies install ***
if ! command -v uv &>/dev/null; then
  echo "'uv' is not installed. Installing 'uv'..."
  curl -LsSf https://astral.sh/uv/install.sh | sh
fi

uv sync --all-extras
source .venv/bin/activate

# *** build ***
uv run scons -j$(nproc 2>/dev/null || sysctl -n hw.logicalcpu)

# *** lint ***
# TODO: pre-commit is slow; replace it with openpilot's "op lint"
#uv run pre-commit run --all-files
uv run ruff check .

# *** test ***
uv run pytest -n$(nproc 2>/dev/null || sysctl -n hw.logicalcpu)

# *** all done ***
GREEN='\033[0;32m'
NC='\033[0m'
printf "\n${GREEN}All good!${NC} Finished build, lint, and test in ${SECONDS}s\n"
