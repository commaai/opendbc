# Observable safety tests on hardware

`test_observable.py` runs the same CAN scenarios against host C safety hooks and
an unmodified stock Panda. Supported profiles are Toyota LKA torque (safety model
2, parameter 73) and classic Hyundai torque (model 8, parameter 0). This workflow
selects safety modes through the normal Panda API; it does not flash firmware.
The installed firmware must support the selected mode and the health fields used
by the runner. Results record the DUT firmware version and hardware type.

The shared scenarios cover engagement, disengagement, brake behavior, positive
and negative steering torque limits, steering requests, opposing measured torque,
relay, stream timeout and checksum faults, recovery, rejected buses/addresses, and CAN
forwarding. Toyota additionally checks acceleration boundaries and gas inhibition,
diagnostic payloads, blocked AEB actuation and inactive LTA commands. Hyundai
additionally checks cruise button transmit rules and the enable-button requirement.
These profiles and observable scenarios are shared; legacy platform tests have
not all been migrated. Exact microsecond boundaries, internal state manipulation
and malformed USB metadata remain host coverage. The shared realtime-window
scenario checks valid rollover after a generous wait; exact rejection boundaries
remain in the supplemental host suite.

## Equipment and dependencies

- **Runner host:** this opendbc checkout with its Python dependencies installed,
  plus SSH access to the service hosts. Fixture encoders require the normal
  opendbc Python dependencies, but the hardware path does not compile or load
  libsafety.
- **DUT service host:** Python, `hardware_service.py` and the Panda Python package, with
  USB or SPI access to the stock Panda. Use a compatible Panda package and firmware.
- **CAN station:** Linux, Python and `hardware_service.py`, with three configured
  SocketCAN interfaces physically connected to the corresponding DUT CAN buses.
  Set the interfaces up with the bitrate and termination required by the bench
  before running. The station service uses the Python standard library and
  supports classic CAN frames only (up to eight data bytes).

The service hosts can be the same machine. Use an isolated bench with the DUT
and station as its CAN participants. Each service owns its transport for the run.
No service runs on the Panda MCU; the DUT service uses its normal USB or SPI API.

## Run

First run the shared scenarios against the host hooks from the checkout root:

```sh
python -m unittest opendbc.safety.tests.test_observable
```

For hardware, install the required dependencies on each service host. The runner
starts both services using the command strings below. These module commands use
a checkout on each service host and run from a directory where `opendbc` is
importable. Replace the example host names, checkout paths and serial with your
bench values:

```sh
python -m opendbc.safety.tests.run_hardware \
  --profile toyota \
  --dut-command "ssh dut-host 'cd /path/to/opendbc && python -m opendbc.safety.tests.hardware_service dut --serial PANDA_SERIAL'" \
  --station-command "ssh station-host 'cd /path/to/opendbc && python -m opendbc.safety.tests.hardware_service station --interfaces can2,can3,can0'" \
  --json-output toyota-hardware.json
```

Alternatively, copy just `opendbc/safety/tests/hardware_service.py` to each
service host; neither remote service imports opendbc. Replace each remote module
command with its standalone script invocation:

```sh
python -m opendbc.safety.tests.run_hardware \
  --profile toyota \
  --dut-command "ssh dut-host 'python /path/to/hardware_service.py dut --serial PANDA_SERIAL'" \
  --station-command "ssh station-host 'python /path/to/hardware_service.py station --interfaces can2,can3,can0'" \
  --json-output toyota-hardware.json
```

The standalone DUT service still requires the Panda Python package and USB
permissions. The runner host still requires the checkout and its dependencies.

Omit `--profile` to run the entire shared suite, or use `--profile hyundai` for
the Hyundai scenarios. `--serial` selects a particular
Panda and can be omitted when selection is unambiguous. Commands may also start
services locally, for example
`--dut-command 'python -m opendbc.safety.tests.hardware_service dut'`.
Do not use an SSH pseudo-terminal: service stdout carries the JSON-line protocol.

`--interfaces` is an ordered physical bus map: its first interface connects to
DUT bus 0, its second to bus 1, and its third to bus 2. Thus the example maps
`can2 → 0`, `can3 → 1`, and `can0 → 2`. Adapt the order to actual wiring; interface
names alone do not imply a DUT bus number. Required vehicle RX streams in both
profiles use DUT bus 0. The three-bus mapping also supports wrong-bus rejection
checks.

The station continuously transmits every required RX stream at its configured
period, including while SSH requests are in flight. Sends are staggered to avoid
overrunning station receive FIFOs. State changes replace
payload cycles while retaining stream counter positions. The DUT service sends
heartbeats every 100 ms using the observed controls state; firmware heartbeat
checks remain enabled. Individual timeout and checksum scenarios deliberately
interrupt or corrupt a required stream.

Both backends use 25 Hz state streams and 50 Hz torque measurements. These test
cadences preserve stock RX watchdogs and settle the six-sample torque windows
without overwhelming the station receiver.

A successful TX assertion requires both the DUT's accepted receipt and a matching
frame physically observed by the station. Rejection assertions require the DUT's
rejected receipt and no matching station observation. RX injection is confirmed
in the DUT receive stream and checked against its invalid-RX counter. Missing or
ambiguous evidence fails the run. The optional JSON report contains test results,
checkout revision, DUT provenance and physical evidence counts. On cleanup the
runner stops station streams and returns the DUT to silent mode.
