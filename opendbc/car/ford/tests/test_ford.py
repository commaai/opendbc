import random
import unittest

from opendbc.can.dbc import DBC as CANDBC
from opendbc.car import DT_CTRL, Bus, gen_empty_fingerprint
from opendbc.car.structs import CarControl, CarParams
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, DBC, FW_QUERY_CONFIG, FW_PATTERN, FordSafetyFlags, get_platform_codes, match_fw_to_car_fuzzy
from opendbc.car.ford.fingerprints import FW_VERSIONS
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.testing import fuzzy_test, parameterized

Ecu = CarParams.Ecu


ECU_ADDRESSES = {
  Ecu.eps: 0x730,          # Power Steering Control Module (PSCM)
  Ecu.abs: 0x760,          # Anti-Lock Brake System (ABS)
  Ecu.fwdRadar: 0x764,     # Cruise Control Module (CCM)
  Ecu.fwdCamera: 0x706,    # Image Processing Module A (IPMA)
  Ecu.engine: 0x7E0,       # Powertrain Control Module (PCM)
  Ecu.shiftByWire: 0x732,  # Gear Shift Module (GSM)
  Ecu.debug: 0x7D0,        # Accessory Protocol Interface Module (APIM)
}


ECU_PART_NUMBER = {
  Ecu.eps: [
    b"14D003",
  ],
  Ecu.abs: [
    b"2D053",
  ],
  Ecu.fwdRadar: [
    b"14D049",
  ],
  Ecu.fwdCamera: [
    b"14F397",  # Ford Q3
    b"14H102",  # Ford Q4
  ],
}


class TestFordFW(unittest.TestCase):
  def test_fw_query_config(self):
    for (ecu, addr, subaddr) in FW_QUERY_CONFIG.extra_ecus:
      assert ecu in ECU_ADDRESSES, "Unknown ECU"
      assert addr == ECU_ADDRESSES[ecu], "ECU address mismatch"
      assert subaddr is None, "Unexpected ECU subaddress"

  @parameterized("car_model, fw_versions", FW_VERSIONS.items())
  def test_fw_versions(self, car_model, fw_versions):
    for (ecu, addr, subaddr), fws in fw_versions.items():
      assert ecu in ECU_PART_NUMBER, "Unexpected ECU"
      assert addr == ECU_ADDRESSES[ecu], "ECU address mismatch"
      assert subaddr is None, "Unexpected ECU subaddress"

      for fw in fws:
        assert len(fw) == 24, "Expected ECU response to be 24 bytes"

        match = FW_PATTERN.match(fw)
        assert match is not None, f"Unable to parse FW: {fw!r}"
        if match:
          part_number = match.group("part_number")
          assert part_number in ECU_PART_NUMBER[ecu], f"Unexpected part number for {fw!r}"

        codes = get_platform_codes([fw])
        assert 1 == len(codes), f"Unable to parse FW: {fw!r}"

  @fuzzy_test(max_examples=100)
  def test_platform_codes_fuzzy_fw(self, fuzzy):
    """Ensure function doesn't raise an exception"""
    get_platform_codes(fuzzy.list(fuzzy.binary))

  def test_platform_codes_spot_check(self):
    # Asserts basic platform code parsing behavior for a few cases
    results = get_platform_codes([
      b"JX6A-14C204-BPL\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      b"NZ6T-14F397-AC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      b"PJ6T-14H102-ABJ\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      b"LB5A-14C204-EAC\x00\x00\x00\x00\x00\x00\x00\x00\x00",
    ])
    assert results == {(b"X6A", b"J"), (b"Z6T", b"N"), (b"J6T", b"P"), (b"B5A", b"L")}

  def test_fuzzy_match(self):
    for platform, fw_by_addr in FW_VERSIONS.items():
      # Ensure there's no overlaps in platform codes
      for _ in range(20):
        car_fw = []
        for ecu, fw_versions in fw_by_addr.items():
          ecu_name, addr, sub_addr = ecu
          fw = random.choice(fw_versions)
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

        CP = CarParams(carFw=car_fw)
        matches = match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
        assert matches == {platform}

  def test_match_fw_fuzzy(self):
    offline_fw = {
      (Ecu.eps, 0x730, None): [
        b"L1MC-14D003-AJ\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"L1MC-14D003-AL\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      ],
      (Ecu.abs, 0x760, None): [
        b"L1MC-2D053-BA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"L1MC-2D053-BD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      ],
      (Ecu.fwdRadar, 0x764, None): [
        b"LB5T-14D049-AB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"LB5T-14D049-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      ],
      # We consider all model year hints for ECU, even with different platform codes
      (Ecu.fwdCamera, 0x706, None): [
        b"LB5T-14F397-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"NC5T-14F397-AF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      ],
    }
    expected_fingerprint = CAR.FORD_EXPLORER_MK6

    # ensure that we fuzzy match on all non-exact FW with changed revisions
    live_fw = {
      (0x730, None): {b"L1MC-14D003-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"},
      (0x760, None): {b"L1MC-2D053-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"},
      (0x764, None): {b"LB5T-14D049-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"},
      (0x706, None): {b"LB5T-14F397-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"},
    }
    candidates = match_fw_to_car_fuzzy(live_fw, '', {expected_fingerprint: offline_fw})
    assert candidates == {expected_fingerprint}

    # model year hint in between the range should match
    live_fw[(0x706, None)] = {b"MB5T-14F397-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"}
    candidates = match_fw_to_car_fuzzy(live_fw, '', {expected_fingerprint: offline_fw,})
    assert candidates == {expected_fingerprint}

    # unseen model year hint should not match
    live_fw[(0x760, None)] = {b"M1MC-2D053-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"}
    candidates = match_fw_to_car_fuzzy(live_fw, '', {expected_fingerprint: offline_fw})
    assert len(candidates) == 0, "Should not match new model year hint"


class TestFordDBC(unittest.TestCase):
  def test_tron_dbc(self):
    # Same messages and signals as the base DBC, checksums and counters included. Only the sizes differ
    base = CANDBC(DBC[CAR.FORD_F_150_MK14][Bus.pt])
    tron = CANDBC(DBC[CAR.FORD_F_150_MK14_5][Bus.pt])
    assert base.msgs.keys() == tron.msgs.keys()
    for addr, msg in base.msgs.items():
      assert tron.msgs[addr].sigs == msg.sigs, f"Signals differ for {msg.name}"
      assert tron.msgs[addr].size in (msg.size, 16)

    for name in ("LateralMotionControl2", "Lane_Assist_Data1"):
      assert tron.name_to_msg[name].size == 16


class TestFordTron(unittest.TestCase):
  # LateralMotionControl2 and ACCDATA on the camera bus: 16 bytes on the 2026 F-150, 8 bytes on the 2021-23 F-150
  CAMERA_16 = {0x3d6: 16, 0x186: 16}
  CAMERA_8 = {0x3d6: 8, 0x186: 8}

  @staticmethod
  def _get_params(platform, camera_fingerprint, alpha_long=False, is_release=False):
    fingerprint = gen_empty_fingerprint()
    fingerprint[CanBus(fingerprint=fingerprint).camera] = camera_fingerprint
    return CarInterface.get_params(platform, fingerprint, [], alpha_long, is_release, docs=False)

  def test_tron(self):
    for alpha_long in (False, True):
      CP = self._get_params(CAR.FORD_F_150_MK14_5, self.CAMERA_16, alpha_long=alpha_long)
      assert CP.safetyConfigs[-1].safetyParam == FordSafetyFlags.CANFD | FordSafetyFlags.TRON
      assert not CP.dashcamOnly
      assert not CP.openpilotLongitudinalControl
      assert not CP.alphaLongitudinalAvailable

    assert self._get_params(CAR.FORD_F_150_MK14_5, self.CAMERA_16, is_release=True).dashcamOnly

  def test_tron_dashcam_on_mismatch(self):
    # the platform and the camera's message lengths must agree
    for platform, camera_fingerprint in ((CAR.FORD_F_150_MK14_5, self.CAMERA_8), (CAR.FORD_F_150_MK14_5, {}),
                                         (CAR.FORD_F_150_MK14, self.CAMERA_16)):
      CP = self._get_params(platform, camera_fingerprint)
      assert CP.dashcamOnly, (platform, camera_fingerprint)
      assert CP.safetyConfigs[-1].safetyParam == FordSafetyFlags.CANFD

  def test_not_tron(self):
    for camera_fingerprint in (self.CAMERA_8, {}):
      CP = self._get_params(CAR.FORD_F_150_MK14, camera_fingerprint)
      assert not CP.dashcamOnly
      assert CP.safetyConfigs[-1].safetyParam == FordSafetyFlags.CANFD
      assert CP.alphaLongitudinalAvailable
    assert self._get_params(CAR.FORD_F_150_MK14, self.CAMERA_8, alpha_long=True).openpilotLongitudinalControl
    assert self._get_params(CAR.FORD_EXPLORER_MK6, {}).openpilotLongitudinalControl

  def test_car_controller(self):
    # Same messages as the 2021-23 F-150 with the lane messages zero padded to 16 bytes, except the camera's
    # ACCDATA_3 and IPMA_Data, which are not replaced. All of it is allowed by safety
    CP = self._get_params(CAR.FORD_F_150_MK14_5, self.CAMERA_16)
    safety = libsafety_py.libsafety
    safety.set_safety_hooks(CP.safetyConfigs[-1].safetyModel.raw, CP.safetyConfigs[-1].safetyParam)
    safety.init_tests()
    safety.set_controls_allowed(True)
    safety.set_cruise_engaged_prev(True)

    CI = CarInterface(CP)
    CI_canfd = CarInterface(self._get_params(CAR.FORD_F_150_MK14, self.CAMERA_8))
    addrs_sent, addrs_sent_canfd = set(), set()
    now_nanos = 0
    for kwargs in ({}, {"latActive": True}, {"cruiseControl": {"cancel": True}}, {"cruiseControl": {"resume": True}}):
      CC = CarControl(**kwargs).as_reader()
      for _ in range(200):
        CI.update([])
        CI_canfd.update([])
        _, sendcan = CI.apply(CC, now_nanos)
        _, sendcan_canfd = CI_canfd.apply(CC, now_nanos)
        assert sendcan == [(addr, dat.ljust(16 if addr in (0x3ca, 0x3d6) else 8, b'\x00'), bus)
                           for addr, dat, bus in sendcan_canfd if addr not in (0x18a, 0x3d8)]
        addrs_sent_canfd.update(addr for addr, _, _ in sendcan_canfd)

        safety.set_timer(now_nanos // 1000)
        for addr, dat, bus in sendcan:
          assert safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat)), (hex(addr), dat, bus)
          addrs_sent.add(addr)
        now_nanos += int(DT_CTRL * 1e9)
    assert addrs_sent == {0x83, 0x3ca, 0x3d6}
    assert addrs_sent_canfd == addrs_sent | {0x18a, 0x3d8}
