# Car porting checklist

Use this checklist for a new port or when extending an existing one. Check items only after verifying them; mark unsupported or inapplicable items `N/A` with a reason. Record routes, timestamps, test results, and known limitations alongside the relevant items. Distinguish behavior observed on the car from behavior inferred from logs or tested in simulation.

Repeat applicable checks for each supported hardware variant and control mode (stock ACC and openpilot longitudinal). Some checks require the full openpilot integration, beyond opendbc alone. Use bench tests or replay for fault injection, stationary checks for doors/seatbelts/gears, and a controlled environment for actuation checks.

## Vehicle and setup

- Vehicle details

  - [ ] Record make, model, year, trim, market, and relevant driver assistance options in values.py.
    - If all variants in the US market come equipped with the necessary ADAS package, no package documentation is required.
    - If a vehicle varies in features in another market, add a new CarDocs entry.
  - [ ] Record ECU firmware versions, harness, device, and software revisions used for validation.
  - [ ] Identify differences from related supported cars; verify reused signals and commands on this variant.

- Harness and CAN topology

  - [ ] Confirm harness pinout, CAN bus assignments, bitrate, and CAN/CAN FD configuration.
  - [ ] Identify which ECUs send, receive, and forward each relevant message.

- Stock behavior and power lifecycle

  - [ ] Record baseline stock LKAS, ACC, stop-and-go, and driver override behavior for reference when writing the port.
    - Stock behavior should be matched when possible, i.e. how stock ACC commands standstill and resume.
  - [ ] Verify ignition detection, startup, shutdown, and vehicle sleep with the device connected.

## Identification and parameters

- [ ] The car identifies correctly on repeated cold starts without a manually forced fingerprint or fingerprint cache.
- [ ] Firmware queries use the correct ECU addresses and buses without causing any DTCs on the dashboard.
- [ ] Exact and fuzzy fingerprinting distinguish variants that need different control or safety behavior.
- [ ] Unknown or ambiguous firmware does not silently select an incompatible platform.
  - A vehicle variant we do not support but has identical firmware (such as lacking ACC or LKAS hardware) should be detected with a CAN signal in interface.py or carstate.py depending on what we can check.
- [ ] Reliable source for wheelbase, steering ratio, and other vehicle model parameters.
- [ ] Minimum steering/engagement speeds and stop-and-go capability in `carParams` match observed behavior.

## CAN parsing and message construction

- Signal definitions and input validation

  - [ ] Message addresses, lengths, buses, frequencies, endianness, signedness, scales, and units are verified.
  - [ ] Checksums and counters validate against real captures, including counter wraparound.
  - [ ] Missing, stale, malformed, or checksum-invalid messages produce the expected invalid CAN state.
  - [ ] Startup defaults and invalid signal values cannot look like valid engagement conditions.

- Outgoing messages and forwarding

  - [ ] Copied stock messages preserve unrelated fields and use fresh source data.
  - [ ] Transmitted messages have the correct bus, timing, counters, checksums, and request bits.
  - [ ] Forwarding and replacement rules prevent duplicate actuation messages or bus loops.

- Diagnostics

  - [ ] Diagnostic sessions, ECU disabling, and keepalives work across startup, restart, and shutdown, if used.
    - This is for disabling an ECU, such as to gain longitudinal control. Some ports do not require this.

## Vehicle state: basics

- Seatbelt and doors

  - [ ] Driver seatbelt: buckle and unbuckle both update `seatbeltUnlatched` correctly.
  - [ ] Doors: open and close each door individually; `doorOpen` remains true until all monitored doors close.

- Gears

  - [ ] Gear selection: ideally we parse park, reverse, neutral, drive, and alternate drive modes such as sport, B (brake) gear on hybrids, etc.
    - If we cannot parse all gears, ensure we can disengage cruise on non-drive/sport gears. Anything that stock ACC would normally disengage for.
  - [ ] Unknown gear values are handled explicitly instead of defaulting to drive.

- Pedals and regen

  - [ ] Brake pedal: light press, firm press, hold, and release update `brakePressed` reliably.
  - [ ] Accelerator pedal: press and release update `gasPressed`, including while cruise is active.
  - [ ] Driver pedal signals are distinguished from braking or acceleration commanded by ACC/openpilot.
    - In the past, we found some accelerator signals either activate when stock ACC accelerates, or don't activate when the user's gas was under the stock ACC request. This needs to describe if the user is pressing the gas pedal any amount, in any state.
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
    - Use `CS.update_steering_pressed` if the signal is noisy.

- Blinkers and blind spots

  - [ ] Left/right blinkers are reported correctly.
    - If we only have lamp state (blinks), use `CS.update_blinker_from_lamp`.
  - [ ] Left/right blind spot indications are mapped correctly and clear when the stock indication clears.

- Faults and stock system state

  - [ ] Temporary and permanent steering faults are logged correctly and recover only when the fault clears.
  - [ ] ACC faults, invalid vehicle sensors, and transient not-ready states are reported correctly.
  - [ ] ESP disabled/active states and stock AEB/FCW states are parsed where available.
  - [ ] Stock LKAS settings and conventional/non-adaptive cruise modes are detected where relevant.

## Buttons and cruise state

- [ ] Set, resume, cancel, main cruise, speed up/down, gap, and LKAS buttons are mapped where present.
- [ ] Button press and release edges each occur once; held buttons do not cause unintended repeated engagement.
- [ ] Short presses, long presses, and simultaneous button pressed are handled correctly.
- [ ] Cruise available, enabled, standstill, and set speed are parsed properly and match stock behavior.
- [ ] Cruise main on/off and ACC standby/active transitions are parsed properly.
- [ ] Set speed changes, unit changes, and minimum/maximum set speeds behave correctly.

## Engagement, cancellation, and overrides

- Engagement conditions

  - [ ] The intended set/resume action engages once, only when the car is ready.
  - [ ] Engagement refusal and disengagement for seatbelt, doors, gear, parking brake, and faults match the intended policy.
  - [ ] Engagement at, below, and above minimum speed behaves correctly.

- Cancellation

  - [ ] The physical cancel button disengages openpilot and cancels stock cruise as intended.
  - [ ] If stock ACC is controlling longitudinal, verify our cancel message stops commanding acceleration.
  - [ ] Software cancellation works in each supported cruise state, including standstill and driver accelerator override.
  - [ ] Cancel wins over simultaneous set/resume requests; repeated cancel requests do not resume or re-enable cruise.
  - [ ] Cruise main off disengages and prevents unintended re-engagement.

- Driver overrides

  - [ ] Brake press disengages as intended while moving as well as while at a standstill.
  - [ ] Accelerator override and release behave correctly, i.e. we command 0 m/s^2 acceleration while continuing to steer.
  - [ ] Steering override works in both directions; any steering-based disengagement matches the platform policy.
    - Tesla has a locked-in, no-override stock policy, here steering override disengages steering as well as cruise.

- Disengagement and recovery

  - [ ] Releasing a pedal, closing a door, buckling a belt, or clearing a fault does not unexpectedly re-engage unless we are still in soft-disable grade period.
  - [ ] Stock ACC disengagement/fault causes the expected openpilot response when using stock longitudinal control.
  - [ ] Lateral-only and longitudinal-active states set their respective actuation request bits correctly.
    - For example, `CC.enabled` should be used when keeping the car's cruise engaged during a longitudinal override while `CC.longActive` falls.

## Lateral control

- Commands, limits, and driver override

  - [ ] Small left/right commands produce the expected steering direction.
  - [ ] Command magnitude and rate limits match the platform and safety implementation.
  - [ ] Driver override reduces or cancels openpilot steering assistance as intended.

- Low-speed behavior

  - [ ] Steering behaves correctly around the minimum steering speed, including crossing it in both directions.
  - [ ] Some cars have problems with oscillation at lower speeds, flag if this is the case and we can come up with solutions. One solution is to low pass filter the request under a certain speed.

- Faults and recovery

  - [ ] Sustained steering, high steering rates, and request-bit transitions do not cause unexpected EPS faults.
  - [ ] Temporary steering unavailability stops active requests and produces the expected user alert.

- Tuning and reported output

  - [ ] Tuning is checked across representative speeds and curves for oscillation, lag, and saturation.
  - [ ] Reported `actuatorsOutput` matches the command actually sent after clipping and rate limiting.

## Longitudinal control

- Supported control modes

  - [ ] Stock ACC works correctly with openpilot lateral control, including cancel, override, and stop-and-go.
  - There are two levels of support with openpilot-controlled longitudinal:
    - [ ] Alpha longitudinal is chosen if we cannot maintain stock AEB or other safety features, or the car does not have a radar we can parse. openpilot longitudinal is then only available on development branches.
    - [ ] openpilot longitudinal is the only mode and available in all branches if both of the above requirements are met, plus we have a good understanding of the car's longitudinal API.

- Acceleration, braking, and driver override

  - [ ] Acceleration and braking command signs, scales, limits, and rate limits are verified.
  - [ ] Acceleration, coasting, and braking transitions are smooth and free of conflicting requests. Some cars may accept gas and brake signals separately.
  - [ ] Driver accelerator and brake inputs retain their intended priority over commanded actuation.

- Stopping, holding, and resuming

  - [ ] Low-speed braking reaches a complete stop without unexpected creep or brake release.
  - [ ] Stop hold works for short and extended stops; timeout and parking brake behavior are understood.
  - [ ] Resume/drive-off releases hold in the correct sequence without unintended acceleration.
  - [ ] Resume is sent only when requested and supported.
  - [ ] Stops and starts on grades behave correctly without unexpected rollback or parking brake/gear transitions.

- Powertrain states and faults

  - [ ] Behavior is checked across relevant powertrain states, such as engine stop/start.
  - [ ] ACC faults and transient not-ready states inhibit requests appropriately and recover cleanly.

- Tuning and stock safety features

  - [ ] Longitudinal delay and tuning are evaluated across representative speeds, following, stops, and starts.
  - [ ] Stock AEB/FCW interaction and any feature loss from ECU disabling are understood and documented.

## Radar, alerts, and stock features

- Radar

  - [ ] Radar distance, lateral position, relative velocity, signs, units, and track IDs are verified, if supported.
  - [ ] Radar track creation, updates, disappearance, invalid targets, and timeouts behave correctly.

- HUD and alerts

  - [ ] HUD engaged/standby state, set speed, lane lines, lead indication, and gap bars match the requested state.
  - [ ] Takeover, steering unavailable, and other required alerts appear and clear correctly.

- Stock feature interaction

  - [ ] Stock LKAS and ACC do not fight openpilot commands.
  - [ ] Stock safety messages are forwarded or arbitrated as intended; validate AEB interactions with replay/bench tests.
  - [ ] Normal operation does not introduce persistent dash warnings or diagnostic trouble codes.
    - Some cars may display warning messages on the dash or force a disengage if they detect a conflict between what their camera is sending and our request through read-back signals from the PCM or EPS. If we need to spoof a message to the camera/ADAS ECU, we need to verify it does not affect stock safety features such as LKAS or AEB.

## Safety and failure handling

- Message permissions and engagement

  - [ ] The selected safety mode permits only intended transmit addresses, buses, and message lengths.
  - [ ] Safety engagement/disengagement agrees with the car interface for cruise, buttons, pedals, and motion state.
  - [ ] Non-zero actuation or active request bits are rejected when safety's `controls_allowed` is false.
  - [ ] Any messages permitted while disengaged are limited to their intended inactive/cancel behavior.

- Actuation limits

  - [ ] Steering magnitude, rate, driver torque, measured torque, and request-bit checks are covered as applicable.
  - [ ] Longitudinal limits, inactive values, and cancel/resume restrictions are covered as applicable.
  - [ ] All actuation-capable fields in transmitted messages are constrained, including fields unused by the controller.

- Communication failures and recovery

  - [ ] Invalid counters/checksums, missing required input messages, and stale state disable control as intended.
  - [ ] Relay malfunction detection and forwarding restrictions cover the actual harness topology.
  - [ ] Restart and recovery require valid state and do not unexpectedly restore actuation.

- Test coverage

  - [ ] Safety tests cover every new safety flag variant and reject disallowed commands.

## Regression tests

- Routes and automated checks

  - [ ] Add representative routes to the [car test routes](../opendbc/car/tests/routes.py), covering relevant variants and modes.
  - [ ] Add or update tests for fingerprinting, parsing, command construction, and discovered edge cases.
  - [ ] Run applicable [car interface tests](../opendbc/car/tests/) and [safety tests](../opendbc/safety/tests/).
  - [ ] Replay recorded drives to check CAN validity and safety acceptance of intended commands.
  - [ ] Run the repository checks in [`test.sh`](../test.sh) and fix any unresolved failures.

- Regression validation

  - [ ] Recheck related existing cars affected by shared DBC, controller, interface, or safety changes.
  - [ ] Repeat startup, engagement, cancellation, and shutdown over multiple drives and ignition cycles.

- Documentation and evidence

  - [ ] Update car documentation with supported years/trims, required hardware, control modes, and limitations.
  - [ ] Record evidence for completed checks and explicitly list untested behavior or unsupported features.
