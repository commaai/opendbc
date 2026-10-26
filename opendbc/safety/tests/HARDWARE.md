# Safety tests on stock hardware

The existing platform tests can use the common harness with host hooks, the
simulated observable backend, or a stock Panda and a physical CAN station.
The hardware route reuses their test IDs, fixture encoders and expected verdicts;
CAN setup, public health and observed command consequences replace private-state
access where implemented. This is not proof that the whole exact suite has passed
physically: the host supplements and unsupported requirements below remain explicit.

The services use normal Panda APIs, including safety-mode selection, alternative
experience and MCU reset for ignition tests. They do not flash or modify firmware. Installed
firmware and the Panda Python package must support the selected modes, public
health fields and configuration methods. Heartbeat checks stay enabled.

## Choose and run tests

Run commands from the checkout root with the normal opendbc dependencies installed.
The default remains the deterministic host suite. Compare it with the same test
class through the observable simulation boundary:

```sh
python -m unittest opendbc.safety.tests.test_toyota.TestToyotaSafetyTorque
SAFETY_BACKEND=simulated python -m unittest opendbc.safety.tests.test_toyota.TestToyotaSafetyTorque
```

The hardware runner defaults to the original suite; `--suite legacy` remains a
compatibility option. List its discovered IDs without opening hardware, then select a
full test ID with `--test` (repeatable). Omitting `--test` selects all discovered
tests, including explicitly unsupported host/tooling tests.

```sh
python -m opendbc.safety.tests.run_hardware --suite legacy --list
python -m opendbc.safety.tests.run_hardware \
  --suite legacy \
  --test opendbc.safety.tests.test_toyota.TestToyotaSafetyTorque.test_non_realtime_limit_up \
  --dut-command "ssh dut-host 'cd /path/to/opendbc && python -m opendbc.safety.tests.hardware_service dut --serial PANDA_SERIAL'" \
  --station-command "ssh station-host 'cd /path/to/opendbc && python -m opendbc.safety.tests.hardware_service station --interfaces can2,can3,can0'" \
  --json-output safety-hardware.json
```

Use `--failfast` to stop on the first failure. Service commands can also run
locally. Do not use an SSH pseudo-terminal: stdout carries JSON-line RPC.
`--serial` may be omitted only when exactly one Panda is available.

Required-RX profiles currently cover Toyota (including LTA/SecOC variants),
Hyundai classic/legacy and CAN FD, Honda Nidec/Bosch, Chrysler/CUSW, GM, Mazda,
MG, Rivian, Subaru/global and preglobal, Volkswagen MQB/PQ/MLB/MEB, Ford,
Tesla, Nissan, BYD and PSA. Concrete fixture parameters choose message/bus/counter
variants. Body, ELM327, default modes and ignition-only fixtures also have stream
handling, including explicitly empty profiles where appropriate. A stream profile
is an input declaration, not a guarantee that every test or firmware mode is
physically supported; undeclared profiles fail explicitly.

Continuous-traffic health, checksum recovery and zero-to-opposite-sign torque
regressions live in `TestToyotaSafetyTorque` and `TestHyundaiSafety`. Their default
host runs use the simulated periodic transport; `SAFETY_BACKEND=simulated` and the
hardware runner exercise the same methods through the selected backend. Other
original methods retain their deterministic host execution.

## Bench and CAN FD setup

Use an isolated bench with three correctly terminated CAN links. The station is
Linux with SocketCAN; the DUT service needs the Panda Python package and USB/SPI
access. Service hosts may be the same machine. `--interfaces can2,can3,can0` maps
those interfaces, in that order, to DUT buses 0, 1 and 2. Interface names alone do
not determine wiring. Configure every link's nominal bitrate before running.

For example, on the station, configure one classic link or one FD link as follows
(repeat for the mapped interfaces, using the bench's actual rates):

```sh
sudo ip link set can2 down
sudo ip link set can2 type can bitrate 500000
sudo ip link set can2 up

# Alternative for CAN FD at 500 kbit/s nominal, 2 Mbit/s data:
sudo ip link set can2 down
sudo ip link set can2 type can bitrate 500000 dbitrate 2000000 fd on
sudo ip link set can2 up
```

Match the DUT's nominal/data rates through the stock Panda API
(`set_can_speed_kbps(bus, 500)` and `set_can_data_speed_kbps(bus, 2000)` for this
example), and match FD ISO/non-ISO settings where required. Configure OBD routing
externally through the stock Panda API when the bench requires it. Nominal/data
bitrates, ISO/non-ISO mode and OBD routing are external bench prerequisites; the
service's `configure` RPC only selects alternative experience. A reset may require
reapplying bench configuration.

For FD fixtures such as Hyundai CAN FD, Ford CAN FD and Volkswagen MEB, both the
Panda hardware/firmware and station adapters must support FD. Add
`--fd-buses 0,1,2` to **both** service commands when all three links use FD; use
only the appropriate DUT bus indices for a mixed bench. This flag explicitly
selects FD even for payloads of eight bytes or fewer; it does not configure link
bitrates. The installed Panda API must support `can_send(..., fd=True)`.

For standalone remote deployment, copy **both** `hardware_service.py` and
`hardware_sequence.py` from this directory into the same remote directory on
each service host. No remote opendbc installation is needed. For example, replace
the module commands above with:

```sh
ssh dut-host 'python /path/to/hardware_service.py dut --serial PANDA_SERIAL --fd-buses 0,1,2'
ssh station-host 'python /path/to/hardware_service.py station --interfaces can2,can3,can0 --fd-buses 0,1,2'
```

Pass these strings as `--dut-command` and `--station-command`; the runner starts
and owns the services. The standalone DUT still needs the Panda package, and the
runner still needs this checkout and its dependencies.

## Evidence and execution limits

Required streams run continuously during RPC calls, preserving counter phase
when payloads change. Fixture profiles generally use bounded 25 Hz state and
50 Hz measurement cadences; individual declarations determine the actual rates.
Deliberate checksum, counter and timeout tests can suspend streams so background
traffic cannot silently repair their faults. The DUT service sends heartbeats
using observed controls state.

TX acceptance requires a stock accepted receipt plus matching station wire
observation. Rejection requires a rejected receipt without matching wire traffic.
RX is physically observed and checked against invalid-RX counters. Exhaustive
scans use paced batches, retain the address domain on physical buses 0..2, and
check exact receipt/wire counts; RX scan invalid counts are aggregate evidence,
not invented per-frame verdicts. Relay-triggering scan inputs require clean resets.
Missing, conflicting, partial or overflowed evidence fails the run.

Timing-sensitive steering sequences execute near the DUT through
`hardware_sequence.py`, avoiding an SSH round trip per command. They record
bounded host-dispatch lateness and verify receipts/wire counts with health guards.
The service observes every command receipt before recording its processing cutoff;
coarse timing assertions use that conservative bound. Accepted setup ramps and
exhaustive scans use completion-relative pacing, with no catch-up bursts, while
timing tests retain bounded dispatch deadlines and receipt-window checks.
Those timestamps are not wire or MCU timestamps and cannot establish arbitrary
microsecond edges. Scans and real-time waits can make full runs lengthy.

The JSON report records discovered, selected, executed and unexecuted IDs,
per-execution status, fixture events, revision/dirty-tree state, firmware/hardware
provenance and evidence counts. A backend being created does not itself prove
physical CAN coverage. Method-level contracts separately describe shared assertions
and host supplements; undeclared assertion coverage remains `unclassified`.
Setup traffic alone cannot establish original-assertion equivalence. Raw scan and
sequence audits retain partial evidence on failures. The runner returns failure for skips, expected failures,
collection errors or unexecuted selected tests as well as ordinary failures.
Cleanup stops streams and returns the DUT to silent mode.

Retain the default host runs as supplements for exact private sample extrema and
immediate reset state, private pedal/cruise/relay latch identity, unconsumed
longitudinal flags, exact MCU clock/tick edges and wraparound, malformed USB-only
packet metadata, logical bus 3, and the exact unsupported-mode C return code.
Observable probes test downstream behavior, not equality with those private
values. Build tests, direct-hook scenarios outside `SafetyTestBase`, and internal
runner/tooling tests are classified explicitly as unsupported by the legacy
physical runner rather than silently executed on host hooks.

Original semantic skips remain: examples include absent regen messages, steering
request bits, second speed messages or realtime limits, inapplicable Hyundai
variants, and Toyota's no-DSU diagnostic test. Shared helper/base classes are also
excluded by their original unittest class setup. These are distinct from missing
physical capability, but the strict runner still reports skips as unsuccessful.

Silent-mode tests currently fail on the two-participant bench: a silent DUT does
not ACK station traffic. They need a verified independent ACK participant plus
bounded retransmission/RX evidence. Running noOutput does not prove silent-mode
behavior. Unsupported stock firmware modes, missing public APIs, incompatible FD
links and unavailable timing guarantees likewise fail explicitly; none constitutes
an automatic skip or evidence that the entire physical suite passed.
