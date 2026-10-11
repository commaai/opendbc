# MISRA C:2025 check

[misrable](https://github.com/adeebshihadeh/misrable) checks the safety code
against MISRA C:2025. It needs no compiler or toolchain: the target is fixed
to the 32-bit ARM the firmware runs on, and the program it analyzes is
libsafety (`tests/libsafety/safety.c` and everything it includes), the same
code the Python safety tests exercise.

```sh
./opendbc/safety/tests/misra/test_tool.sh                 # all rules
./opendbc/safety/tests/misra/test_tool.sh --rules 10.3    # one rule
./opendbc/safety/tests/misra/test_tool.sh --list-rules    # rules and their limitations
```

Exit code 0 means no findings, 1 means findings, 2 means the analysis is
incomplete. CI requires 0. There are no suppressions: every finding is fixed
in the code. The one language extension that cannot be removed is the
`__attribute__((packed, aligned(4)))` on `CANPacket_t`, which defines the
wire format shared with panda; it is reported under Directive 1.2.

`--external-api` names the files whose declarations are used outside the C
program: `can.h`, `declarations.h`, `ignition.h` and `safety.c` are read by
`libsafety_py.py` to build the cffi interface, so their functions, objects
and macros count as used.

The checked-in binaries in `bin/` were built from misrable commit
`352ea38a44a07d799bfe751a5e711c9e2341459e` with Go 1.27.1:

```sh
CGO_ENABLED=0 GOOS=<linux|darwin> GOARCH=<amd64|arm64> \
  go build -trimpath -ldflags='-s -w' -o <opendbc>/opendbc/safety/tests/misra/bin/misra-<os>-<arch> ./cmd/misrable
```

Refresh all four binaries together when the analyzer changes. The existing
cppcheck check (`test_misra.sh`) still runs as well.
