#!/bin/bash
set -e

# *** dependencies install ***
if ! command -v uv &>/dev/null; then
  echo "'uv' is not installed. Installing 'uv'..."
  curl -LsSf https://astral.sh/uv/install.sh | sh
fi

mkdir -p .tmp
echo '
#include <re2/re2.h>
RE2 x("");int main(void) {return 0;}
' > .tmp/re2.c
g++ -o .tmp/re2.o .tmp/re2.c -lre2 &>/dev/null || {
  echo "'re2' is not installed. Installing 're2'..."
  [[ $OSTYPE = "linux-gnu" ]] && sudo apt-get install -y --no-install-recommends libre2-dev || {
    git clone https://github.com/google/re2.git
    mv re2/re2 opendbc/can
    rm -rf re2
    git clone https://github.com/abseil/abseil-cpp.git
    mv abseil-cpp/absl opendbc/can
    rm -rf abseil-cpp
  }
}
rm -rf .tmp

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
