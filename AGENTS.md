# AGENTS.md
Guidance for agentic coding tools operating in this repository.

## Scope
- Applies to the full repo rooted here.
- Follow local patterns before generic defaults.
- Keep edits minimal and task-focused.

## Cursor/Copilot Rule Files
No extra rule files were found when this file was written:
- `.cursorrules`
- `.cursor/rules/`
- `.github/copilot-instructions.md`
If these are added later, treat them as higher-priority instructions and update this file.

## Environment Setup
Always bootstrap first:
```bash
source ./setup.sh
```
`setup.sh` currently:
- sets `PYTHONPATH` to repo root
- installs `uv` if missing
- runs `uv sync --all-extras --inexact` into `.venv`
- activates `.venv`
- installs MISRA/cppcheck dependencies used by safety tests
Python target from `pyproject.toml`: `>=3.11,<3.13`.

## Build Commands
Full local CI-style run:
```bash
./test.sh
```
`./test.sh` runs:
1. `uv lock --check`
2. `scons -j8`
3. `lefthook run test`
Build only:
```bash
source ./setup.sh
scons -j8
```
Safety debug build:
```bash
source ./setup.sh
scons -j$(nproc) -D
```

## Lint and Static Analysis
Run all configured checks:
```bash
source ./setup.sh
lefthook run test
```
Run individual checks:
```bash
source ./setup.sh
ruff check .
ty check
codespell $(git ls-tree -r HEAD --name-only) -L tge,stdio -S '*.dbc'
cpplint --exclude=opendbc/safety/tests/misra/cppcheck/ --exclude=opendbc/can/*_pyx.cpp --recursive --quiet --counting=detailed --linelength=240 --filter=-build,-legal,-readability,-runtime,-whitespace,+build/include_subdir,+build/forward_decl,+build/include_what_you_use,+build/deprecated,+whitespace/comma,+whitespace/line_length,+whitespace/empty_if_body,+whitespace/empty_loop_body,+whitespace/empty_conditional_body,+whitespace/forcolon,+whitespace/parens,+whitespace/semicolon,+whitespace/tab,+readability/braces opendbc/
opendbc/safety/tests/misra/test_misra.sh
```

## Test Commands
Run all tests:
```bash
source ./setup.sh
pytest .
```
Repo pytest defaults include:
- `-Werror`
- `--strict-config --strict-markers`
- `--durations=10`
- `-n auto`
Run a single test file:
```bash
source ./setup.sh
pytest opendbc/can/tests/test_packer_parser.py
```
Run a single class:
```bash
source ./setup.sh
pytest opendbc/can/tests/test_packer_parser.py::TestCanParserPacker
```
Run a single test function:
```bash
source ./setup.sh
pytest opendbc/can/tests/test_packer_parser.py::TestCanParserPacker::test_packer
```
Run by expression:
```bash
source ./setup.sh
pytest -k "test_packer and not counter" opendbc/can/tests/test_packer_parser.py
```
Run one test without xdist (debug-friendly):
```bash
source ./setup.sh
pytest -n0 opendbc/safety/tests/test_honda.py::HondaButtonEnableBase::test_set_resume_buttons
```
Run full safety suite (includes 100% line coverage gate):
```bash
./opendbc/safety/tests/test.sh
```
Generate safety HTML coverage report:
```bash
./opendbc/safety/tests/test.sh --report
```
Run mutation tests:
```bash
source ./setup.sh
cd opendbc/safety/tests && ./mutation.sh
```

## Code Style Guidelines
### Formatting
- Python uses 2-space indentation (Ruff config).
- Keep lines near 160 chars max.
- Prefer small helpers and readable control flow.
- Add comments only when logic is non-obvious.

### Imports
- Typical order: stdlib, third-party, local `opendbc`.
- Match import style in the edited file (repo is not strictly alphabetical everywhere).
- Prefer explicit imports; avoid wildcard imports.
- Use dynamic imports only where existing architecture depends on them.

### Types
- Add type hints for new/modified Python code.
- Use Python 3.11 syntax (`A | B`, `list[int]`, `dict[str, Any]`).
- Keep `ty check` clean for touched code.
- Use callback type aliases for complex signatures.

### Naming
- `snake_case` for functions/variables/modules.
- `PascalCase` for classes.
- `UPPER_SNAKE_CASE` for constants.
- Follow existing car patterns (`CAR`, `CanBus`, `...Flags`, `CarInterface`, `CarState`, `CarController`).
- Test modules should be named `test_*.py`.

### Error Handling and Logging
- Fail fast for invalid state with specific exceptions (`ValueError`, `RuntimeError`, etc.).
- Do not silently swallow exceptions.
- Prefer guard clauses over deeply nested conditionals.
- Use `carlog.warning`/`carlog.error` for runtime diagnostics in relevant paths.
- In tests, use explicit assertions; skip only for intentionally unsupported cases.

### Testing Expectations
- Add/update tests when behavior changes.
- Run the narrowest relevant tests first, then broader suites.
- For interface-wide changes, run `opendbc/car/tests/test_car_interfaces.py`.
- For safety C changes, run safety tests plus MISRA checks.

### Safety C Code
- Keep safety logic deterministic and auditable.
- Preserve hook/state patterns (`rx`, `tx`, `fwd`, and reset behavior).
- Prefer explicit casts and fixed-width types when needed.
- Do not relax safety gates, whitelists, or coverage requirements.

## Agent Validation Flow
After edits, run:
```bash
source ./setup.sh
ruff check .
ty check
pytest -n0 <targeted_test>
pytest <broader_scope>
```
Before PR / final handoff, run:
```bash
./test.sh
```

## CI Notes
- Primary workflow: `.github/workflows/tests.yml`.
- CI runs `./test.sh` and dedicated safety/mutation jobs.
- If local and CI differ, prefer CI command definitions.
