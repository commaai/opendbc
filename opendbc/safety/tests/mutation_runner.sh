#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

# run with unittest first since it has way less overhead than pytest
# toyota should catch ~75% of it
./test_toyota.py -f

# last resort, run full test suite
pytest -n8 --ignore-glob=misra/*
