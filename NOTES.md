# Component fingerprinting notes

## Proving write compatibility

We can observe stock command messages and potentially prove that a candidate
writer reproduces their address, size, counter, checksum, frequency, and static
fields. It is still unclear what evidence proves that a generated nonzero
command has the expected meaning, authority, limits, and fault behavior on an
otherwise unknown vehicle.

Question to explore: can write compatibility be established from passive stock
traffic plus carefully chosen invariants, or is independent ECU/firmware
attestation or a safe active challenge required?

## Toyota EPS torque scale

Toyota EPS torque uses several platform-selected scale factors (66, 73, 77,
88, or 100). The factor affects both decoded EPS torque and panda's steering
safety limits. In a component-fingerprinted Toyota, this must be detected or
learned live without assuming the vehicle platform. The safe observation,
initialization, and convergence strategy remains an open problem.

## `whats_a_platform` Toyota prototype

Toyota runtime behavior is decomposed into support groups for PT decoding,
cruise decoding, ACC ownership, lateral writing, longitudinal writing, SecOC,
hybrid behavior, HUD availability, stop-and-go, EPS scaling, and wheel-speed
scaling. Existing platforms are migration manifests only; Toyota CarState,
CarController, radar, DBC selection, and safety configuration do not dispatch
on platform identity.

All Toyotas currently use one generic dynamics prior. This intentionally tests
the hypothesis that paramsd/torqued can absorb platform variation. Replay data
is still required to validate startup and convergence behavior.

Support policy is downstream and represented as explicit make/model/year
tuples. VIN currently supplies model year only. Strict tuple enforcement is
implemented for a fully decoded identity, but global Toyota VDS-to-model
decoding remains to be added; until then the live fallback can only reject a
year unsupported by every identity in the legacy manifest.
