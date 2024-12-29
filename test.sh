#!/bin/bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

# TODO: why doesn't uv do this?
export PYTHONPATH=$DIR

# *** dependencies install ***
if ! command -v uv &>/dev/null; then
  echo "'uv' is not installed. Installing 'uv'..."
  curl -LsSf https://astral.sh/uv/install.sh | sh
fi

mkdir -p $DIR/.tmp
echo '
#include <re2/re2.h>
RE2 x("");int main(void) {return 0;}
' > $DIR/.tmp/re2.c
g++ -o $DIR/.tmp/re2.o $DIR/.tmp/re2.c -lre2 &>/dev/null || {
  echo "'re2' is not installed. Installing 're2'..."
  [[ $OSTYPE = "linux-gnu" ]] && sudo apt-get install -y --no-install-recommends libre2-dev || brew install re2
}
rm -rf $DIR/.tmp

uv sync --all-extras
source .venv/bin/activate

# *** build ***
uv run scons -j8

# *** lint ***
# TODO: pre-commit is slow; replace it with openpilot's "op lint"
#uv run pre-commit run --all-files
uv run ruff check .

# *** test ***
uv run pytest -n8

# *** all done ***
GREEN='\033[0;32m'
NC='\033[0m'
printf "\n${GREEN}All good!${NC} Finished build, lint, and test in ${SECONDS}s\n"
