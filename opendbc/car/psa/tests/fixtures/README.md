# Peugeot 308 T9 RX fixtures

These are two-second excerpts from a single 2018 Peugeot 308 II T9 reference
vehicle with conventional cruise control. They are ESP32 recordings, not comma
routes. `provenance.json` records the source and exported file SHA-256 hashes,
source clock offsets and frame counts.

The export keeps only standard, non-RTR, RX frames from the `live` bus, with the
address and DLC declared in `psa_308_t9_2018.dbc`. Payload bytes and frame order
are unchanged. `time_us` is the ESP32 source timestamp minus `source_start_us`.
Only frames in `[source_start_us, source_start_us + duration_us)` are retained.
The live stream is mapped to logical bus 0. This does not establish the physical
camera/powertrain bus topology on a comma harness.

| Fixture | Frames | Final mean wheel speed | Final steering angle |
| --- | ---: | ---: | ---: |
| `stationary.csv` | 1,120 | 0 km/h | 534.7 degrees |
| `moving.csv` | 1,140 | 125.815 km/h | 5.2 degrees |

The files contain ten allowlisted messages. VIN, diagnostic traffic, GPS,
absolute wall-clock timestamps, session metadata and actuation messages are
excluded. The source journals remain local to the research repository.

Run the replay, protection, fingerprint and no-output tests from the repo root:

```sh
uv run python -m unittest opendbc.car.psa.tests.test_peugeot_308_t9
```

The test feeds frames at their recorded relative times, checks validity after
initialization, verifies final decoded values and tests timeout after reception
stops. Other tests use explicitly synthetic signal overrides and single-bit
corruptions; those are not additional vehicle observations.

Known gaps: no comma route or validated production harness, no ECU firmware
fingerprint, no EPS torque calibration, and no validated forward/Park/Neutral
gear signal. The former candidate in the low nibble of `0x348` stays zero in
both complete source captures, including the moving one, so it is omitted.
Geometry and the inactive lateral-acceleration placeholder need separate review
before any future control work. This contribution provides no actuation support.
