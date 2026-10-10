# MISRA tool experiment

Run the experimental standalone MISRA C:2012 analyzer from any directory:

```sh
./opendbc/safety/tests/misra/test_tool.sh
./opendbc/safety/tests/misra/test_tool.sh --format json > misra-findings.json
./opendbc/safety/tests/misra/test_tool.sh --rules 15.6,14.4
```

The entry point includes the safety headers with `ALLOW_DEBUG` enabled, including
debug safety modes. Analysis uses the tool's `arm-none-eabi` target and fallback
standard headers; it does not reproduce every firmware compiler option.

Exit codes are 0 for no findings, 1 for findings, and 2 for incomplete analysis or
invalid configuration. CI saves JSON reports on Linux and macOS, allowing findings
while failing incomplete analysis. The existing Cppcheck check is still available
through `test_misra.sh`.

The checks are experimental and partial; a clean run is not proof of MISRA
compliance. Use `bin/misra-<os>-<arch> --list-rules` to inspect check limitations.

The checked-in binaries temporarily replace distribution. They were built from
the local misra development tree at commit
`1f85bdeeea9c8e89d364292f668db391cfff7eb9`, including uncommitted development
changes, with Go 1.27.1:

```sh
CGO_ENABLED=0 GOOS=<linux|darwin> GOARCH=<amd64|arm64> \
  go build -trimpath -ldflags='-s -w' -o <opendbc>/opendbc/safety/tests/misra/bin/misra-<os>-<arch> ./cmd/misra
```

Refresh all four binaries together as the analyzer changes.
