import unittest
import csv
import itertools
from pathlib import Path
from types import SimpleNamespace

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.can_definitions import CanData
from opendbc.car.fingerprints import (
  all_legacy_fingerprint_cars,
  eliminate_incompatible_cars,
)
from opendbc.car.psa.fingerprints import FINGERPRINTS
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.psacan import psa_checksum
from opendbc.car.psa.values import CAR, DBC


# RX snapshot from learn-20260811T170039Z-e1fe3ff3.
# Overrides in the signal tests below are synthetic unless stated otherwise.
STATIONARY_CAPTURE = {
  0x208: "1AC133004C333233",
  0x2F5: "BB00007FFF0001",
  0x305: "14E30007E10020",
  0x30D: "0000000000000000",
  0x3AD: "500000018072008E",
  0x3CD: "FFFD00000B226040",
  0x412: "10000000007C0000",
  0x452: "000000030000",
  0x50E: "022600045E42FF2D",
  0x572: "98807FC088000001",
}


class TestPeugeot308T9(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
    self.CI = CarInterface(self.CP)

  def update_from_stationary_capture(self, overrides=None):
    capture = STATIONARY_CAPTURE | (overrides or {})
    frames = [
      CanData(address, bytes.fromhex(data_hex), 0)
      for address, data_hex in capture.items()
    ]
    return self.CI.update([(1_000_000_000, frames)])

  def test_platform_is_hard_read_only(self):
    self.assertTrue(self.CP.dashcamOnly)
    self.assertEqual(
      self.CP.safetyConfigs[0].safetyModel,
      structs.CarParams.SafetyModel.noOutput,
    )
    self.assertEqual(self.CP.safetyConfigs[0].safetyParam, 0)
    self.assertFalse(self.CP.openpilotLongitudinalControl)
    self.assertFalse(self.CP.alphaLongitudinalAvailable)
    self.assertTrue(self.CP.pcmCruise)
    self.assertFalse(self.CP.autoResumeSng)
    self.assertEqual(self.CP.steerControlType, structs.CarParams.SteerControlType.torque)
    self.assertEqual(self.CP.lateralTuning.which(), "pid")

  def test_uses_vehicle_specific_dbc_and_fingerprint(self):
    self.assertEqual(
      DBC[CAR.PSA_PEUGEOT_308_T9][Bus.pt],
      "psa_308_t9_2018",
    )
    fingerprint = FINGERPRINTS[CAR.PSA_PEUGEOT_308_T9][0]
    for address, data_hex in STATIONARY_CAPTURE.items():
      self.assertEqual(fingerprint[address], len(bytes.fromhex(data_hex)))

  def test_recorded_snapshot_fingerprints_as_t9(self):
    candidates = all_legacy_fingerprint_cars()
    for address, data_hex in STATIONARY_CAPTURE.items():
      candidates = eliminate_incompatible_cars(
        CanData(address, bytes.fromhex(data_hex), 0),
        candidates,
      )

    self.assertEqual(candidates, [CAR.PSA_PEUGEOT_308_T9])

  def test_decodes_recorded_stationary_carstate(self):
    state = self.update_from_stationary_capture()

    self.assertAlmostEqual(state.vEgoRaw, 0.0)
    self.assertTrue(state.standstill)
    self.assertAlmostEqual(state.steeringAngleDeg, 534.7, places=3)
    self.assertAlmostEqual(state.steeringRateDeg, 0.0)
    self.assertAlmostEqual(state.steeringTorque, 0.0)
    self.assertFalse(state.steeringPressed)
    self.assertFalse(state.gasPressed)
    self.assertFalse(state.brakePressed)
    self.assertTrue(state.parkingBrake)
    self.assertFalse(state.doorOpen)
    self.assertFalse(state.seatbeltUnlatched)
    self.assertEqual(state.gearShifter, structs.CarState.GearShifter.unknown)
    self.assertTrue(state.cruiseState.available)
    self.assertFalse(state.cruiseState.enabled)
    self.assertTrue(state.cruiseState.nonAdaptive)
    self.assertAlmostEqual(state.cruiseState.speed, 0.0)

  def test_only_reverse_gear_is_reported(self):
    reverse = self.update_from_stationary_capture({
      0x412: "14000000007C0000",
    })
    self.assertEqual(reverse.gearShifter, structs.CarState.GearShifter.reverse)
    state = self.update_from_stationary_capture()
    self.assertEqual(state.gearShifter, structs.CarState.GearShifter.unknown)

  def test_parking_brake_uses_validated_esp_state(self):
    released = self.update_from_stationary_capture({
      0x3AD: "000000007EE10054",
    })
    self.assertFalse(released.parkingBrake)

    self.CI = CarInterface(self.CP)
    applied = self.update_from_stationary_capture({
      0x3AD: "50000001805200EA",
    })
    self.assertTrue(applied.parkingBrake)

  def test_driver_body_and_rvv_signals(self):
    state = self.update_from_stationary_capture({
      # Accelerator > 0 and persistent RVV state 2.
      0x208: "1AC1332048333233",
      # Brake pedal plus driver and rear-left doors open.
      0x412: "30000000007C2800",
      # Driver seatbelt state 1 (unlatched).
      0x572: "58807FC088000001",
    })

    self.assertTrue(state.gasPressed)
    self.assertTrue(state.brakePressed)
    self.assertTrue(state.doorOpen)
    self.assertTrue(state.seatbeltUnlatched)
    self.assertTrue(state.cruiseState.available)
    self.assertTrue(state.cruiseState.enabled)
    self.assertTrue(state.cruiseState.nonAdaptive)

  def test_vehicle_validated_psa_checksums(self):
    # One real payload per protected message. The complete 72-session corpus
    # and pass counts are recorded beside the DBC in the OBD repository.
    samples = {
      # address: (payload, checksum start bit, transmitted checksum)
      0x2F5: ("9D00007FFF0001", 7, 0x9),
      0x3AD: ("50000001805200EA", 59, 0xA),
      0x3CD: ("000100000B33B17E", 63, 0x7),
    }

    for address, (payload, start_bit, expected) in samples.items():
      with self.subTest(address=hex(address)):
        signal = SimpleNamespace(start_bit=start_bit)
        self.assertEqual(
          psa_checksum(address, signal, bytearray.fromhex(payload)),
          expected,
        )

  def test_real_capture_replay(self):
    for name, speed_kph, angle_deg, parked in (
      ('stationary', 0., 534.7, True),
      ('moving', 125.815, 5.2, False),
    ):
      with self.subTest(capture=name):
        ci = CarInterface(self.CP)
        with (Path(__file__).parent / 'fixtures' / f'{name}.csv').open() as f:
          packets = [(1_000_000_000 + int(row['time_us']) * 1000,
                      [CanData(int(row['address'], 16), bytes.fromhex(row['data']), 0)]) for row in csv.DictReader(f)]
        cursor = 0
        for now in range(1_010_000_000, 3_000_000_001, 10_000_000):
          start = cursor
          while cursor < len(packets) and packets[cursor][0] <= now:
            cursor += 1
          state = ci.update(packets[start:cursor] or [(now, [])])
          if now >= 2_000_000_000:
            self.assertTrue(state.canValid)
            self.assertFalse(state.canTimeout)
          self.assertEqual(ci.apply(structs.CarControl().as_reader(), now)[1], [])
        self.assertAlmostEqual(state.vEgoRaw * 3.6, speed_kph, places=3)
        self.assertAlmostEqual(state.steeringAngleDeg, angle_deg, places=3)
        self.assertEqual(state.parkingBrake, parked)
        # All protected streams remain sequential with no counter failures.
        for message in ci.can_parsers[Bus.main].message_states.values():
          self.assertEqual(message.counter_fail, 0)
        # No new messages: the stream must eventually become invalid.
        for now in range(5_000_000_000, 5_100_000_000, 10_000_000):
          state = ci.update([(now, [])])
        self.assertFalse(state.canValid)
        self.assertTrue(state.canTimeout)

  def test_corrupt_protected_frames_do_not_refresh_signals(self):
    for address, signal in ((0x2F5, 'DriverTorqueRaw'), (0x3AD, 'ParkingBrakeState'), (0x3CD, 'YawRateDegS')):
      original = bytes.fromhex(STATIONARY_CAPTURE[address])
      # Flip each individual bit, including opaque bytes and the checksum.
      for bit in range(len(original) * 8):
        with self.subTest(address=hex(address), bit=bit):
          cp = CANParser('psa_308_t9_2018', [(address, 100)], 0)
          cp.update([(1_000_000_000, [CanData(address, original, 0)])])
          self.assertEqual(cp.ts_nanos[address][signal], 1_000_000_000)
          corrupted = bytearray(original)
          corrupted[bit // 8] ^= 1 << (bit % 8)
          cp.update([(1_010_000_000, [CanData(address, bytes(corrupted), 0)])])
          self.assertEqual(cp.ts_nanos[address][signal], 1_000_000_000)

  def test_repeated_counter_is_rejected(self):
    cp = CANParser('psa_308_t9_2018', [(0x2F5, 100)], 0)
    frame = CanData(0x2F5, bytes.fromhex(STATIONARY_CAPTURE[0x2F5]), 0)
    for i in range(20):
      cp.update([(1_000_000_000 + i * 10_000_000, [frame])])
    self.assertFalse(cp.can_valid)
    self.assertLess(cp.ts_nanos[0x2F5]['DriverTorqueRaw'], 1_100_000_000)

  def test_other_buses_do_not_update_carstate(self):
    for bus in (1, 2):
      ci = CarInterface(self.CP)
      frames = [CanData(address, bytes.fromhex(data), bus) for address, data in STATIONARY_CAPTURE.items()]
      ci.update([(1_000_000_000, frames)])
      cp = ci.can_parsers[Bus.main]
      self.assertEqual(cp.ts_nanos[0x305]['SteeringAngleDeg'], 0)
      self.assertFalse(cp.can_valid)

  def test_limiter_and_off_are_not_cruise(self):
    for mode in (0, 2, 3):
      payload = bytearray.fromhex(STATIONARY_CAPTURE[0x50E])
      payload[6] = 90
      payload[7] = mode << 5
      state = self.update_from_stationary_capture({0x50E: payload.hex(), 0x208: '1AC1332048333233'})
      self.assertFalse(state.cruiseState.enabled)
      self.assertFalse(state.cruiseState.available)
      self.assertEqual(state.cruiseState.speed, 0.)

  def test_controller_never_emits_can(self):
    for enabled, lateral, longitudinal, have_can in itertools.product((False, True), repeat=4):
      self.CI = CarInterface(self.CP)
      if have_can:
        self.update_from_stationary_capture()
      control = structs.CarControl(enabled=enabled, latActive=lateral, longActive=longitudinal)
      control.actuators.steeringAngleDeg = 90.0
      control.actuators.torque = 1.0
      control.actuators.accel = 2.0
      control = control.as_reader()
      for frame in range(100):
        applied, can_sends = self.CI.apply(control, frame * 10_000_000)
        self.assertEqual(applied.torque, 0.)
        self.assertEqual(applied.steeringAngleDeg, 0.)
        self.assertEqual(applied.accel, 0.)
        self.assertEqual(can_sends, [])


if __name__ == "__main__":
  unittest.main()
