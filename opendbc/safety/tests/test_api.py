import subprocess
from pathlib import Path


ROOT = Path(__file__).parents[3]


def compile_header(header: str) -> subprocess.CompletedProcess:
  return subprocess.run(
    ["cc", "-Werror", "-std=gnu11", "-fsyntax-only", "-x", "c", "-I", str(ROOT), "-"],
    input=f'#include "{header}"',
    text=True,
    capture_output=True,
    check=False,
  )


def test_public_api_header():
  result = compile_header("opendbc/safety/api.h")
  assert result.returncode == 0, result.stderr


def test_internal_header_is_rejected():
  result = compile_header("opendbc/safety/declarations.h")
  assert result.returncode != 0
  assert "internal opendbc safety header" in result.stderr
