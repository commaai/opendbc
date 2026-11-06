import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, CanData, gen_empty_fingerprint, structs
from opendbc.car.structs import CarParams
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.radar_interface import RADAR_START_ADDR
from opendbc.car.hyundai.values import CAR, DBC, DATE_FW_ECUS, FW_QUERY_CONFIG, CANFD_FUZZY_WHITELIST, \
                                         PLATFORM_CODE_ECUS, HYUNDAI_VERSION_REQUEST_LONG, \
                                         Buttons, HyundaiFlags, get_platform_codes, HyundaiSafetyFlags
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_hyundai import checksum as hyundai_can_checksum
from opendbc.testing import fuzzy_test

ButtonType = structs.CarState.ButtonEvent.Type
Ecu = CarParams.Ecu

# Some platforms have date codes in a different format we don't yet parse (or are missing).
# For now, assert list of expected missing date cars
NO_DATES_PLATFORMS = {
  # CAN FD
  CAR.KIA_SPORTAGE_5TH_GEN,
  CAR.HYUNDAI_SANTA_CRUZ_1ST_GEN,
  CAR.HYUNDAI_TUCSON_4TH_GEN,
  # CAN
  CAR.HYUNDAI_ELANTRA,
  CAR.HYUNDAI_ELANTRA_GT_I30,
  CAR.KIA_CEED,
  CAR.KIA_CEED_PHEV,
  CAR.KIA_FORTE,
  CAR.KIA_OPTIMA_G4,
  CAR.KIA_OPTIMA_G4_FL,
  CAR.KIA_SORENTO,
  CAR.HYUNDAI_KONA,
  CAR.HYUNDAI_KONA_EV,
  CAR.HYUNDAI_KONA_EV_2022,
  CAR.HYUNDAI_KONA_HEV,
  CAR.HYUNDAI_SONATA_LF,
  CAR.HYUNDAI_VELOSTER,
  CAR.HYUNDAI_KONA_2022,
}

CANFD_EXPECTED_ECUS = {Ecu.fwdCamera, Ecu.fwdRadar}

# CAN-only feature flags that should not appear on CAN FD platforms
CAN_FEATURE_FLAGS = (HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.TCU_GEARS | HyundaiFlags.CHECKSUM_CRC8 |
                     HyundaiFlags.CHECKSUM_6B | HyundaiFlags.LEGACY | HyundaiFlags.UNSUPPORTED_LONGITUDINAL |
                     HyundaiFlags.CAMERA_SCC)


def cars_with(flags):
  return {c for c in CAR if c.config.flags & flags}


class TestHyundaiFingerprint(unittest.TestCase):
  def test_feature_detection(self):
    # LKA steering
    for lka_steering in (True, False):
      fingerprint = gen_empty_fingerprint()
      if lka_steering:
        cam_can = CanBus(None, fingerprint).CAM
        fingerprint[cam_can] = [0x50, 0x110]  # LKA steering messages
      CP = CarInterface.get_params(CAR.KIA_EV6, fingerprint, [], False, False, False)
      assert bool(CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG) == lka_steering

    # radar available
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_alternate_limits(self):
    # Alternate lateral control limits, for high torque cars, verify Panda safety mode flag is set
    fingerprint = gen_empty_fingerprint()
    for car_model in CAR:
      CP = CarInterface.get_params(car_model, fingerprint, [], False, False, False)
      assert bool(CP.flags & HyundaiFlags.ALT_LIMITS) == bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.ALT_LIMITS)

  def test_pause_resume_flag(self):
    fingerprint = gen_empty_fingerprint()
    for car_model in CAR:
      CP = CarInterface.get_params(car_model, fingerprint, [], True, False, False)
      assert bool(CP.flags & HyundaiFlags.PAUSE_RESUME) == bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.PAUSE_RESUME)
      if CP.flags & HyundaiFlags.CANFD:
        assert bool(CP.flags & HyundaiFlags.PAUSE_RESUME)

  def test_can_features(self):
    for car_model in CAR:
      flags = car_model.config.flags
      # Test no EV/HEV with cluster/TCU gears (should all use ELECT_GEAR)
      if flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
        assert not (flags & (HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.TCU_GEARS))

      # Test CAN FD car not in CAN feature lists
      if flags & HyundaiFlags.CANFD:
        assert not (flags & CAN_FEATURE_FLAGS), "CAN FD car unexpectedly has a CAN feature flag"

  def test_hybrid_ev_flags(self):
    for car_model in CAR:
      flags = car_model.config.flags
      assert not (flags & HyundaiFlags.HYBRID and flags & HyundaiFlags.EV), "Shared cars between hybrid and EV"
      assert not (flags & HyundaiFlags.CANFD and flags & HyundaiFlags.HYBRID), \
        "Hard coding CAN FD cars as hybrid is no longer supported"

  def test_canfd_ecu_whitelist(self):
    # Asserts only expected Ecus can exist in database for CAN-FD cars
    for car_model in CAR:
      if not (car_model.config.flags & HyundaiFlags.CANFD):
        continue
      ecus = {fw[0] for fw in FW_VERSIONS[car_model].keys()}
      ecus_not_in_whitelist = ecus - CANFD_EXPECTED_ECUS
      ecu_strings = ", ".join([f"Ecu.{ecu}" for ecu in ecus_not_in_whitelist])
      assert len(ecus_not_in_whitelist) == 0, \
                       f"{car_model}: Car model has unexpected ECUs: {ecu_strings}"

  def test_blacklisted_parts(self):
    # Asserts no ECUs known to be shared across platforms exist in the database.
    # Tucson having Santa Cruz camera and EPS for example
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        if car_model == CAR.HYUNDAI_SANTA_CRUZ_1ST_GEN:
          raise unittest.SkipTest("Skip checking Santa Cruz for its parts")

        for code, _ in get_platform_codes(ecus[(Ecu.fwdCamera, 0x7c4, None)]):
          if b"-" not in code:
            continue
          part = code.split(b"-")[1]
          assert not part.startswith(b'CW'), "Car has bad part number"

  def test_correct_ecu_response_database(self):
    """
    Assert standard responses for certain ECUs, since they can
    respond to multiple queries with different data
    """
    expected_fw_prefix = HYUNDAI_VERSION_REQUEST_LONG[1:]
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          assert all(fw.startswith(expected_fw_prefix) for fw in fws), \
                          f"FW from unexpected request in database: {(ecu, fws)}"

  @fuzzy_test(max_examples=100)
  def test_platform_codes_fuzzy_fw(self, fuzzy):
    """Ensure function doesn't raise an exception"""
    get_platform_codes(fuzzy.list(fuzzy.binary))

  def test_expected_platform_codes(self):
    # Ensures we don't accidentally add multiple platform codes for a car unless it is intentional
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          if ecu[0] not in PLATFORM_CODE_ECUS:
            continue

          # Third and fourth character are usually EV/hybrid identifiers
          codes = {code.split(b"-")[0][:2] for code, _ in get_platform_codes(fws)}
          if car_model == CAR.HYUNDAI_PALISADE:
            assert codes == {b"LX", b"ON"}, f"Car has unexpected platform codes: {car_model} {codes}"
          elif car_model == CAR.HYUNDAI_KONA_EV and ecu[0] == Ecu.fwdCamera:
            assert codes == {b"OE", b"OS"}, f"Car has unexpected platform codes: {car_model} {codes}"
          else:
            assert len(codes) == 1, f"Car has multiple platform codes: {car_model} {codes}"

  # Tests for platform codes, part numbers, and FW dates which Hyundai will use to fuzzy
  # fingerprint in the absence of full FW matches:
  def test_platform_code_ecus_available(self):
    # TODO: add queries for these non-CAN FD cars to get EPS
    no_eps_platforms = cars_with(HyundaiFlags.CANFD) | {CAR.KIA_SORENTO, CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL, CAR.KIA_OPTIMA_H, CAR.KIA_K7_2017,
                                    CAR.KIA_OPTIMA_H_G4_FL, CAR.HYUNDAI_SONATA_LF, CAR.HYUNDAI_TUCSON, CAR.GENESIS_G90, CAR.GENESIS_G80, CAR.HYUNDAI_ELANTRA}

    # Asserts ECU keys essential for fuzzy fingerprinting are available on all platforms
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for platform_code_ecu in PLATFORM_CODE_ECUS:
          if platform_code_ecu in (Ecu.fwdRadar, Ecu.eps) and car_model == CAR.HYUNDAI_GENESIS:
            continue
          if platform_code_ecu == Ecu.eps and car_model in no_eps_platforms:
            continue
          assert platform_code_ecu in [e[0] for e in ecus]

  def test_fw_format(self):
    # Asserts:
    # - every supported ECU FW version returns one platform code
    # - every supported ECU FW version has a part number
    # - expected parsing of ECU FW dates

    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          if ecu[0] not in PLATFORM_CODE_ECUS:
            continue

          codes = set()
          for fw in fws:
            result = get_platform_codes([fw])
            assert 1 == len(result), f"Unable to parse FW: {fw}"
            codes |= result

          if ecu[0] not in DATE_FW_ECUS or car_model in NO_DATES_PLATFORMS:
            assert all(date is None for _, date in codes)
          else:
            assert all(date is not None for _, date in codes)

          if car_model == CAR.HYUNDAI_GENESIS:
            raise unittest.SkipTest("No part numbers for car model")

          # Hyundai places the ECU part number in their FW versions, assert all parsable
          # Some examples of valid formats: b"56310-L0010", b"56310L0010", b"56310/M6300"
          assert all(b"-" in code for code, _ in codes), \
                          f"FW does not have part number: {fw}"

  def test_platform_codes_spot_check(self):
    # Asserts basic platform code parsing behavior for a few cases
    results = get_platform_codes([b"\xf1\x00DH LKAS 1.1 -150210"])
    assert results == {(b"DH", b"150210")}

    # Some cameras and all radars do not have dates
    results = get_platform_codes([b"\xf1\x00AEhe SCC H-CUP      1.01 1.01 96400-G2000         "])
    assert results == {(b"AEhe-G2000", None)}

    results = get_platform_codes([b"\xf1\x00CV1_ RDR -----      1.00 1.01 99110-CV000         "])
    assert results == {(b"CV1-CV000", None)}

    results = get_platform_codes([
      b"\xf1\x00DH LKAS 1.1 -150210",
      b"\xf1\x00AEhe SCC H-CUP      1.01 1.01 96400-G2000         ",
      b"\xf1\x00CV1_ RDR -----      1.00 1.01 99110-CV000         ",
    ])
    assert results == {(b"DH", b"150210"), (b"AEhe-G2000", None), (b"CV1-CV000", None)}

    results = get_platform_codes([
      b"\xf1\x00LX2 MFC  AT USA LHD 1.00 1.07 99211-S8100 220222",
      b"\xf1\x00LX2 MFC  AT USA LHD 1.00 1.08 99211-S8100 211103",
      b"\xf1\x00ON  MFC  AT USA LHD 1.00 1.01 99211-S9100 190405",
      b"\xf1\x00ON  MFC  AT USA LHD 1.00 1.03 99211-S9100 190720",
    ])
    assert results == {(b"LX2-S8100", b"220222"), (b"LX2-S8100", b"211103"),
                               (b"ON-S9100", b"190405"), (b"ON-S9100", b"190720")}

  def test_fuzzy_excluded_platforms(self):
    # Asserts a list of platforms that will not fuzzy fingerprint with platform codes due to them being shared.
    # This list can be shrunk as we combine platforms and detect features
    excluded_platforms = {
      CAR.GENESIS_G70,            # shared platform code, part number, and date
      CAR.GENESIS_G70_2020,
    }
    excluded_platforms |= cars_with(HyundaiFlags.CANFD) - cars_with(HyundaiFlags.EV) - CANFD_FUZZY_WHITELIST  # shared platform codes
    excluded_platforms |= NO_DATES_PLATFORMS  # date codes are required to match

    platforms_with_shared_codes = set()
    for platform, fw_by_addr in FW_VERSIONS.items():
      car_fw = []
      for ecu, fw_versions in fw_by_addr.items():
        ecu_name, addr, sub_addr = ecu
        for fw in fw_versions:
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

      CP = CarParams(carFw=car_fw)
      matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
      if len(matches) == 1:
        assert list(matches)[0] == platform
      else:
        platforms_with_shared_codes.add(platform)

    assert platforms_with_shared_codes == excluded_platforms


class _ButtonHarness:
  def __init__(self, car_model: CAR, use_cc: bool = False):
    fingerprint = gen_empty_fingerprint()
    car_fw = []
    if car_model.config.flags & HyundaiFlags.CANFD:
      fingerprint[0][0x1cf] = 8
      fingerprint[2][0x50] = 16
      car_fw.append(CarParams.CarFw(ecu=Ecu.adas, fwVersion=b"\xf1\x00test", address=0x730))

    self.CP = CarInterface.get_params(car_model, fingerprint, car_fw, alpha_long=True, is_release=False, docs=False)
    assert self.CP.openpilotLongitudinalControl
    self.CI = CarInterface(self.CP)
    self.canfd = bool(self.CP.flags & HyundaiFlags.CANFD)
    self.pt_bus = CanBus(self.CP).ECAN if self.canfd else 0
    self.packer = CANPacker(DBC[car_model][Bus.pt])

    self.safety = libsafety_py.libsafety
    cfg = self.CP.safetyConfigs[-1]
    self.safety.set_safety_hooks(cfg.safetyModel.raw, cfg.safetyParam)
    self.safety.init_tests()

    self.use_cc = use_cc
    self.cc_enabled = False
    self.cnt = 0
    self.nanos = 0

  def step(self, samples: list[tuple[int, int]], brake: bool = False, speed_kph: float = 20.0) -> structs.CarState:
    can_msgs: list[CanData] = []
    if self.canfd:
      whl_msg = self.packer.make_can_msg("WHEEL_SPEEDS", self.pt_bus, {f"WHL_Spd{p}Val": speed_kph for p in ("FL", "FR", "RL", "RR")})
      tcs_msg = self.packer.make_can_msg("TCS", self.pt_bus, {"DriverBraking": int(brake)})
      can_msgs.extend([CanData(*whl_msg), CanData(*tcs_msg)])
      for cruise_btn, main_btn in samples:
        btn_msg = self.packer.make_can_msg("CRUISE_BUTTONS", self.pt_bus, {
          "CRUISE_BUTTONS": cruise_btn,
          "ADAPTIVE_CRUISE_MAIN_BTN": main_btn,
        })
        can_msgs.append(CanData(*btn_msg))
    else:
      whl_vals: dict[str, float] = {f"WHL_SPD_{p}": speed_kph for p in ("FL", "FR", "RL", "RR")}
      whl_vals["WHL_SPD_AliveCounter_LSB"] = (self.cnt % 16) & 0x3
      whl_vals["WHL_SPD_AliveCounter_MSB"] = (self.cnt % 16) >> 2
      whl_msg = hyundai_can_checksum(self.packer.make_can_msg("WHL_SPD11", 0, whl_vals))
      tcs_msg = hyundai_can_checksum(self.packer.make_can_msg("TCS13", 0, {
        "DriverOverride": 2 if brake else 0,
        "AliveCounterTCS": self.cnt % 8,
      }))
      can_msgs.extend([CanData(*whl_msg), CanData(*tcs_msg)])
      for cruise_btn, main_btn in samples:
        btn_msg = self.packer.make_can_msg("CLU11", 0, {
          "CF_Clu_CruiseSwState": cruise_btn,
          "CF_Clu_CruiseSwMain": main_btn,
          "CF_Clu_AliveCnt1": self.cnt % 16,
        })
        self.cnt += 1
        can_msgs.append(CanData(*btn_msg))
      if not samples:
        self.cnt += 1

    for m in can_msgs:
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(m.address, m.src, m.dat))

    self.nanos += 10_000_000
    cs = self.CI.update([(self.nanos, can_msgs)])

    if self.use_cc:
      if not cs.cruiseState.available or (cs.brakePressed and not cs.standstill):
        self.cc_enabled = False
      if any(b.type == ButtonType.cancel for b in cs.buttonEvents):
        self.cc_enabled = False
      elif cs.buttonEnable:
        self.cc_enabled = True
      CC = structs.CarControl(enabled=self.cc_enabled)
      self.CI.apply(CC.as_reader(), self.nanos)

    assert cs.cruiseState.available == bool(self.safety.get_acc_main_on())
    return cs


class TestHyundaiButtonLogic(unittest.TestCase):
  def test_classic_button_logic(self):
    h = _ButtonHarness(CAR.HYUNDAI_SONATA)
    assert not (h.CP.flags & HyundaiFlags.PAUSE_RESUME)

    # buttons ignored while main is off
    for btn in (Buttons.SET_DECEL, Buttons.RES_ACCEL):
      assert not h.step([(btn, 0)]).buttonEnable
      assert not h.step([(Buttons.NONE, 0)]).buttonEnable
      assert not h.safety.get_controls_allowed()

    # main rising edge enables availability only; RESUME blocked before first SET
    cs = h.step([(Buttons.NONE, 1)])
    assert cs.cruiseState.available and not cs.buttonEnable and not h.safety.get_controls_allowed()
    h.step([(Buttons.NONE, 0)])
    assert not h.step([(Buttons.RES_ACCEL, 0)]).buttonEnable
    assert not h.step([(Buttons.NONE, 0)]).buttonEnable
    assert not h.safety.get_controls_allowed()

    # SET falling edge sets cruise speed and enables
    assert not h.step([(Buttons.SET_DECEL, 0)]).buttonEnable
    assert not h.safety.get_controls_allowed()
    cs = h.step([(Buttons.NONE, 0)])
    assert cs.buttonEnable and h.safety.get_controls_allowed()

    # CANCEL disables controls while preserving availability and stored cruise speed
    cs = h.step([(Buttons.CANCEL, 0)])
    assert not cs.buttonEnable and not h.safety.get_controls_allowed() and cs.cruiseState.available
    assert any(b.type == ButtonType.cancel and b.pressed for b in cs.buttonEvents)
    h.step([(Buttons.NONE, 0)])

    # RESUME falling edge now enables
    assert not h.step([(Buttons.RES_ACCEL, 0)]).buttonEnable
    cs = h.step([(Buttons.NONE, 0)])
    assert cs.buttonEnable and h.safety.get_controls_allowed()

    # toggling main off disables controls and clears stored cruise speed
    cs = h.step([(Buttons.NONE, 1)])
    assert not cs.cruiseState.available and not cs.buttonEnable and not h.safety.get_controls_allowed()
    h.step([(Buttons.NONE, 0)])
    h.step([(Buttons.NONE, 1)])
    h.step([(Buttons.NONE, 0)])
    assert not h.step([(Buttons.RES_ACCEL, 0)]).buttonEnable
    assert not h.step([(Buttons.NONE, 0)]).buttonEnable
    assert not h.safety.get_controls_allowed()

  def test_pause_resume_button_logic(self):
    for car_model in (CAR.HYUNDAI_ELANTRA_2021, CAR.KIA_EV6):
      for use_cc in (False, True):
        with self.subTest(car_model=car_model, use_cc=use_cc):
          h = _ButtonHarness(car_model, use_cc=use_cc)
          assert bool(h.CP.flags & HyundaiFlags.PAUSE_RESUME)

          # buttons ignored while main is off
          for btn in (Buttons.SET_DECEL, Buttons.RES_ACCEL, Buttons.CANCEL):
            assert not h.step([(btn, 0)]).buttonEnable
            assert not h.step([(Buttons.NONE, 0)]).buttonEnable
            assert not h.safety.get_controls_allowed()

          # main rising edge immediately enables cruise
          cs = h.step([(Buttons.NONE, 1)])
          assert cs.cruiseState.available and cs.buttonEnable and h.safety.get_controls_allowed()
          assert any(b.type == ButtonType.mainCruise and b.pressed for b in cs.buttonEvents)
          cs = h.step([(Buttons.NONE, 0)])
          assert cs.cruiseState.available and not cs.buttonEnable and h.safety.get_controls_allowed()

          # first pause/resume press disengages with cancel event on both press and release
          cs = h.step([(Buttons.CANCEL, 0)])
          assert not cs.buttonEnable and not h.safety.get_controls_allowed()
          assert [(b.type, b.pressed) for b in cs.buttonEvents] == [(ButtonType.cancel, True)]
          cs = h.step([(Buttons.NONE, 0)])
          assert not cs.buttonEnable and not h.safety.get_controls_allowed()
          assert [(b.type, b.pressed) for b in cs.buttonEvents] == [(ButtonType.cancel, False)]

          # second pause/resume press resumes on rising edge and latches resumeCruise through release
          cs = h.step([(Buttons.CANCEL, 0)])
          assert cs.buttonEnable and h.safety.get_controls_allowed()
          assert [(b.type, b.pressed) for b in cs.buttonEvents] == [(ButtonType.resumeCruise, True)]
          cs = h.step([(Buttons.NONE, 0)])
          assert not cs.buttonEnable and h.safety.get_controls_allowed()
          assert [(b.type, b.pressed) for b in cs.buttonEvents] == [(ButtonType.resumeCruise, False)]

          # brake disengage followed by pause/resume press resumes instead of canceling
          h.step([(Buttons.NONE, 0)], brake=True)
          assert not h.safety.get_controls_allowed()
          h.step([(Buttons.NONE, 0)], brake=False)
          cs = h.step([(Buttons.CANCEL, 0)])
          assert cs.buttonEnable and h.safety.get_controls_allowed()
          assert [(b.type, b.pressed) for b in cs.buttonEvents] == [(ButtonType.resumeCruise, True)]
          h.step([(Buttons.NONE, 0)])

          # pause then multi-sample + (RES_ACCEL) press+release in a single 10ms step enables
          h.step([(Buttons.CANCEL, 0)])
          h.step([(Buttons.NONE, 0)])
          assert not h.safety.get_controls_allowed()
          cs = h.step([(Buttons.RES_ACCEL, 0), (Buttons.NONE, 0)])
          assert cs.buttonEnable and h.safety.get_controls_allowed()

          # toggling main off disables availability and controls
          cs = h.step([(Buttons.NONE, 1)])
          assert not cs.cruiseState.available and not cs.buttonEnable and not h.safety.get_controls_allowed()
