# BrownPanda Tesla radar adapter

This fork's `master` branch adds the NGP10 consumer for BrownPanda's optional
BYD-to-Tesla radar conversion. The adapter is deliberately named for the
hardware contract, not for an MCU. NGP10 sees one BrownPanda interface and
detects availability from the wire stream.

## Wire contract

- Input to OpenDBC is 500 kbit/s classical CAN with eight-byte frames.
- `0x401` carries Continental radar status.
- Forty object A/B pairs occupy `0x410` through `0x45F`; `0x45F` completes a
  coherent set.
- Frames are on Tesla logical party bus 0 because BrownPanda exposes only party
  bus 0 and autopilot-party bus 2 to comma.
- The signal layout is Tesla Continental ARS4-B-compatible. It is not the
  industrial Continental ARS408 object protocol.

Radar-capable BrownPanda hardware receives the source BYD MVS4 objects as ten
64-byte CAN-FD frames on a separate inline pair and converts them before they
reach comma. Hardware without that source pair simply does not advertise the
stream.

## Enablement and failure behavior

Tesla fingerprinting enables radar only when both `0x401` and `0x45F` appear as
eight-byte frames on party bus 0. The parser then accepts only that bus and
requires a fresh healthy status, all forty complete pairs, coherent pair
indexes, and the `Tracked`, `Valid`, and `Meas` flags. Missing, stale,
incomplete, or unavailable input clears all points and reports temporary radar
unavailability; an explicit sensor fault reports a radar fault.

The source port did not publish the claimed CRC8/J1850 DataID table or its
offline validator. This adapter therefore does not claim to authenticate the
original 64-byte BYD payload. BrownPanda must fail closed on source, DLC,
freshness, identity, plausibility, and transport loss, and the missing payload
validator remains a replay/HIL gate.

BYD `ALEAD` and `LAT_RELATED` are reserved until capture evidence proves their
layout and units. BrownPanda transmits neutral wire values; this adapter exposes
the corresponding optional acceleration and lateral-speed fields as NaN.
Downstream `radard` estimates lead acceleration from the measured speed stream.

## Compatibility

The BrownPanda vehicle/control protocol remains compatible with Tesla
integrations in sunnypilot and dragonpilot on logical buses 0 and 2. Their
unchanged radar parsers expect the OEM radar on logical bus 1 and therefore do
not consume this bus-0 extension. NGP10 gains it by pinning an exact public
commit from this fork's `master` branch.
