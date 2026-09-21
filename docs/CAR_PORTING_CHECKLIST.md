# Car porting checklist

Use this checklist for a new port or when extending an existing one. Check items only after verifying them; mark unsupported or inapplicable items `N/A` with a reason. Record routes, timestamps, test results, and known limitations alongside the relevant items. Distinguish behavior observed on the car from behavior inferred from logs or tested in simulation.

Repeat applicable checks for each supported hardware variant and control mode (stock ACC and openpilot longitudinal). Some checks require the full openpilot integration, beyond opendbc alone. Use bench tests or replay for fault injection, stationary checks for doors/seatbelts/gears, and a controlled environment for actuation checks.

## Vehicle and setup

- Vehicle details

  - [ ] Record make, model, year, trim, market, powertrain, and relevant driver assistance options.
  - [ ] Record ECU firmware versions, harness, device, and software revisions used for validation.
  - [ ] Identify differences from related supported cars; verify reused signals and commands on this variant.

- Harness and CAN topology

  - [ ] Confirm harness pinout, CAN bus assignments, bitrate, and CAN/CAN FD configuration.
  - [ ] Identify which ECUs send, receive, and forward each relevant message.

- Stock behavior and power lifecycle

  - [ ] Record baseline stock LKAS, ACC, stop-and-go, and driver override behavior.
  - [ ] Check for pre-existing diagnostic trouble codes before testing.
  - [ ] Verify ignition detection, startup, shutdown, and vehicle sleep with the device connected.
  - [ ] Verify stock operation with openpilot disabled and after removing the device.

## Identification and parameters

- [ ] The car identifies correctly on repeated cold starts without a manually forced fingerprint.
- [ ] Firmware queries use the correct ECU addresses and buses without disrupting normal operation.
- [ ] Exact and fuzzy fingerprinting distinguish variants that need different control or safety behavior.
- [ ] Unknown or ambiguous firmware does not silently select an incompatible platform.
- [ ] Platform flags and runtime detection match the installed camera, radar, steering, and powertrain hardware.
- [ ] Mass, wheelbase, steering ratio, and other vehicle model parameters have a documented source.
- [ ] Minimum steering/engagement speeds and stop-and-go capability match observed behavior.
- [ ] Safety model, safety parameters, bus configuration, and longitudinal availability match each supported mode.

## CAN parsing and message construction

- Signal definitions and input validation

  - [ ] Message addresses, lengths, buses, frequencies, endianness, signedness, scales, and units are verified.
  - [ ] Checksums and counters validate against real captures, including counter wraparound.
  - [ ] Required messages have appropriate frequency and timeout checks.
  - [ ] Missing, stale, malformed, or checksum-invalid messages produce the expected invalid CAN state.
  - [ ] Startup defaults and invalid signal values cannot look like valid engagement conditions.

- Outgoing messages and forwarding

  - [ ] Copied stock messages preserve unrelated fields and use fresh source data.
  - [ ] Transmitted messages have the correct bus, timing, counters, checksums, and request bits.
  - [ ] Forwarding and replacement rules prevent duplicate actuation messages or bus loops.

- Diagnostics

  - [ ] Diagnostic sessions, ECU disabling, and keepalives work across startup, restart, and shutdown, if used.

## Vehicle state: easily missed basics

- Seatbelt and doors

  - [ ] Driver seatbelt: buckle and unbuckle both update `seatbeltUnlatched` correctly.
  - [ ] Doors: open and close each door individually; `doorOpen` remains true until all monitored doors close.
  - [ ] Document whether the trunk/hatch and hood are included in door detection.

- Gears

  - [ ] Gear selection: park, reverse, neutral, drive, and available alternate drive modes map correctly.
  - [ ] Unknown gear values are handled explicitly instead of defaulting to drive.

- Pedals and regen

  - [ ] Brake pedal: light press, firm press, hold, and release update `brakePressed` reliably.
  - [ ] Accelerator pedal: press and release update `gasPressed`, including while cruise is active.
  - [ ] Driver pedal signals are distinguished from braking or acceleration commanded by ACC/openpilot.
  - [ ] Regen paddles or other driver regen controls update `regenBraking`, if applicable.

- Parking brake and auto hold

  - [ ] Parking brake engagement and release update `parkingBrake`.
  - [ ] Auto hold engagement and release update `brakeHoldActive`, independently of the brake pedal.

- Speed and standstill

  - [ ] Vehicle speed and individual wheel speeds have correct units and plausible values.
  - [ ] `standstill` distinguishes a complete stop from slow creep and updates promptly on drive-off.
  - [ ] Cluster speed and cruise set speed agree with the car in both metric and imperial settings.

- Steering

  - [ ] Steering angle has the correct sign, scale, center offset, and behavior through its full range.
  - [ ] Steering torque has the correct sign and scale; driver torque and EPS torque are not confused.
  - [ ] `steeringPressed` detects driver input in both directions without triggering on normal commanded steering.

- Blinkers and blind spots

  - [ ] Left/right blinkers, momentary lane-change taps, and hazard lights are reported correctly.
  - [ ] Left/right blind spot indications are mapped correctly and clear when the stock indication clears.

- Faults and stock system state

  - [ ] Temporary and permanent steering faults map correctly and recover only when the fault clears.
  - [ ] ACC faults, invalid vehicle sensors, and transient not-ready states are reported correctly.
  - [ ] ESP disabled/active states and stock AEB/FCW states are parsed where available.
  - [ ] Stock LKAS settings and conventional/non-adaptive cruise modes are detected where relevant.

## Buttons and cruise state

- [ ] Set, resume, cancel, main cruise, speed up/down, gap, and LKAS buttons are mapped where present.
- [ ] Button press and release edges each occur once; held buttons do not cause unintended repeated engagement.
- [ ] Short presses, long presses, and simultaneous button activity are handled correctly.
- [ ] Cruise available, enabled, standstill, and set speed are distinct and match stock behavior.
- [ ] Cruise main on/off and ACC standby/active transitions are reflected promptly.
- [ ] Set speed changes, unit changes, and minimum/maximum set speeds behave correctly.
- [ ] Physical buttons still work when openpilot transmits button or cruise messages.
- [ ] Synthetic button messages release correctly and cannot leave a button logically held.

## Engagement, cancellation, and overrides

- Engagement conditions

  - [ ] The intended set/resume action engages once, only when the car is ready.
  - [ ] Engagement refusal and disengagement for seatbelt, doors, gear, parking brake, and faults match the intended policy.
  - [ ] Engagement at, below, and above minimum speed behaves correctly.

- Cancellation

  - [ ] The physical cancel button disengages openpilot and cancels stock cruise as intended.
  - [ ] **Our cancel message actually cancels cruise on the car:** verify the stock cruise state changes and cruise stops commanding acceleration.
  - [ ] Software cancellation works in each supported cruise state, including standstill and driver accelerator override.
  - [ ] Cancel wins over simultaneous set/resume requests; repeated cancel requests do not resume or re-enable cruise.
  - [ ] Cruise main off disengages and prevents unintended re-engagement.

- Driver overrides

  - [ ] Brake press disengages as intended while moving; standstill brake behavior matches the platform policy.
  - [ ] Accelerator override and release behave correctly for both lateral and longitudinal control.
  - [ ] Steering override works in both directions; any steering-based disengagement matches the platform policy.

- Disengagement and recovery

  - [ ] Releasing a pedal, closing a door, buckling a belt, or clearing a fault does not unexpectedly re-engage.
  - [ ] Stock ACC disengagement/fault causes the expected openpilot response when using stock longitudinal control.
  - [ ] Lateral-only and longitudinal-active states set their respective actuation request bits correctly.
  - [ ] Disengagement removes active actuation requests without leaving a held button or stale command.
  - [ ] Process restart or device reconnect does not reuse stale engagement or resume state.

## Lateral control

- Commands, limits, and driver override

  - [ ] Small left/right commands produce the expected steering direction.
  - [ ] Torque, angle, or curvature scaling matches the selected control interface.
  - [ ] Command magnitude and rate limits match the platform and safety implementation.
  - [ ] Driver override reduces or releases steering authority as intended.

- Low-speed behavior

  - [ ] Steering behaves correctly around the minimum steering speed, including crossing it in both directions.
  - [ ] Standstill steering behavior is understood and represented correctly.

- Faults and recovery

  - [ ] Sustained steering, high steering rates, and request-bit transitions do not cause unexpected EPS faults.
  - [ ] Temporary steering unavailability stops active requests and produces the expected user alert.
  - [ ] Recovery from steering unavailability does not cause a sudden command jump.

- Tuning and reported output

  - [ ] Tuning is checked across representative speeds and curves for oscillation, lag, and saturation.
  - [ ] Reported actuator output matches the command actually sent after clipping and rate limiting.

## Longitudinal control

- Supported control modes

  - [ ] Stock ACC works correctly with openpilot lateral control, including cancel, override, and stop-and-go.
  - [ ] Openpilot longitudinal is offered only on configurations where it is implemented and validated.

- Acceleration, braking, and driver override

  - [ ] Acceleration and braking command signs, scales, limits, and rate limits are verified.
  - [ ] Acceleration, coasting, and braking transitions are smooth and free of conflicting requests.
  - [ ] Driver accelerator and brake inputs retain their intended priority over commanded actuation.

- Stopping, holding, and resuming

  - [ ] Low-speed braking reaches a complete stop without unexpected creep or brake release.
  - [ ] Stop hold works for short and extended stops; timeout and parking brake behavior are understood.
  - [ ] Resume/drive-off releases hold in the correct sequence without unintended acceleration.
  - [ ] Resume is sent only when requested and supported; repeated messages do not cause repeated launches.
  - [ ] Cancel/disengagement while stopped has a verified brake-hold handoff and clear driver indication.
  - [ ] Stops and starts on grades behave correctly without unexpected rollback or parking brake/gear transitions.

- Powertrain states and faults

  - [ ] Behavior is checked across relevant powertrain states, such as engine stop/start or limited EV regen.
  - [ ] ACC faults and transient not-ready states inhibit requests appropriately and recover cleanly.

- Tuning and stock safety features

  - [ ] Longitudinal delay and tuning are evaluated across representative speeds, following, stops, and starts.
  - [ ] Stock AEB/FCW interaction and any feature loss from ECU disabling are understood and documented.

## Radar, alerts, and stock features

- Radar

  - [ ] Radar availability and configuration match the installed hardware.
  - [ ] Radar distance, lateral position, relative velocity, signs, units, and track IDs are verified, if supported.
  - [ ] Radar track creation, updates, disappearance, invalid targets, and timeouts behave correctly.

- HUD and alerts

  - [ ] HUD engaged/standby state, set speed, lane lines, lead indication, and gap bars match the requested state.
  - [ ] Takeover, steering unavailable, and other required alerts appear and clear correctly.
  - [ ] Audible alerts are delivered through the intended device or vehicle output.

- Stock feature interaction

  - [ ] Stock LKAS and ACC do not fight openpilot commands.
  - [ ] Stock safety messages are forwarded or arbitrated as intended; validate AEB interactions with replay/bench tests.
  - [ ] Normal operation does not introduce persistent dash warnings or diagnostic trouble codes.

## Safety and failure handling

- Message permissions and engagement

  - [ ] The selected safety mode permits only intended transmit addresses, buses, and message lengths.
  - [ ] Safety engagement/disengagement agrees with the car interface for cruise, buttons, pedals, and motion state.
  - [ ] Nonzero actuation or active request bits are rejected when their safety conditions are not met.
  - [ ] Any messages permitted while disengaged are limited to their intended inactive/cancel behavior.

- Actuation limits

  - [ ] Steering magnitude, rate, driver torque, measured torque, and request-bit checks are covered as applicable.
  - [ ] Longitudinal limits, inactive values, and cancel/resume restrictions are covered as applicable.
  - [ ] All actuation-capable fields in transmitted messages are constrained, including fields unused by the controller.

- Communication failures and recovery

  - [ ] Invalid counters/checksums, missing required input messages, and stale state disable control as intended.
  - [ ] Relay malfunction detection and forwarding restrictions cover the actual harness topology.
  - [ ] Loss of control messages, CAN communication, or the controlling process produces the intended ECU fallback.
  - [ ] Restart and recovery require valid state and do not unexpectedly restore actuation.

- Test coverage

  - [ ] Safety tests cover every new safety flag or hardware variant and reject disallowed commands.

## Regression tests and handoff

- Routes and automated checks

  - [ ] Add representative routes to the [car test routes](../opendbc/car/tests/routes.py), covering relevant variants and modes.
  - [ ] Add or update tests for fingerprinting, parsing, command construction, and discovered edge cases.
  - [ ] Run applicable [car interface tests](../opendbc/car/tests/) and [safety tests](../opendbc/safety/tests/).
  - [ ] Replay recorded drives to check CAN validity, safety acceptance of intended commands, and unexpected faults.
  - [ ] Run the repository checks in [`test.sh`](../test.sh) and record any unresolved failures.

- Regression validation

  - [ ] Recheck related existing cars affected by shared DBC, controller, interface, or safety changes.
  - [ ] Repeat startup, engagement, cancellation, and shutdown over multiple drives and ignition cycles.

- Documentation and evidence

  - [ ] Update car documentation with supported years/trims, required hardware, control modes, and limitations.
  - [ ] Record evidence for completed checks and explicitly list untested behavior or unsupported features.
