from collections import defaultdict
import random
import unittest

from opendbc.car.fw_versions import build_fw_dict, match_fw_to_car
from opendbc.car.structs import CarParams
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.values import CAR, FW_QUERY_CONFIG, SubaruFlags

Ecu = CarParams.Ecu
CarFw = CarParams.CarFw


class TestSubaruFingerprint(unittest.TestCase):
  def test_fw_version_format(self):
    for platform, fws_per_ecu in FW_VERSIONS.items():
      for (ecu, _, _), fws in fws_per_ecu.items():
        fw_size = len(fws[0])
        for fw in fws:
          assert len(fw) == fw_size, f"{platform} {ecu}: {len(fw)} {fw_size}"
          if ecu == Ecu.fwdCamera:
            assert fw_size in (8, 10), f"{platform} fwdCamera unexpected size: {fw_size}"
            half = fw_size // 2
            assert fw[:half] != b"\x00" * half, f"{platform} fwdCamera has all-zero prefix: {fw!r}"

  def test_fwd_camera_prefix_bijection(self):
    # Settled EyeSight 0xf182 responses must map 1-to-1 from (length, first_half) to full version
    prefix_to_settled: dict[tuple[int, bytes], set[bytes]] = defaultdict(set)
    for fws_per_ecu in FW_VERSIONS.values():
      for fw in fws_per_ecu[(Ecu.fwdCamera, 0x787, None)]:
        half = len(fw) // 2
        if fw[half:] != b"\x00" * half:
          prefix_to_settled[(len(fw), fw[:half])].add(fw)

    for prefix, full_fws in prefix_to_settled.items():
      assert len(full_fws) == 1, f"Multiple settled fwdCamera FWs share prefix {prefix!r}: {full_fws}"

  def test_startup_incomplete_fwd_camera_fuzzy_match(self):
    # Strip existing zero-suffixed fwdCamera workarounds (keeping any prefix-only entry) to verify
    # startup zero-padded EyeSight responses match every Subaru platform cleanly via match_fw_to_car_fuzzy
    settled_offline = {}
    for platform, fws_per_ecu in FW_VERSIONS.items():
      settled_offline[platform] = {}
      for ecu, fws in fws_per_ecu.items():
        if ecu[0] == Ecu.fwdCamera:
          half = len(fws[0]) // 2
          non_zero = [fw for fw in fws if fw[half:] != b"\x00" * half]
          non_zero_prefixes = {fw[:half] for fw in non_zero}
          orphan_zeros = [fw for fw in fws if fw[half:] == b"\x00" * half and fw[:half] not in non_zero_prefixes]
          settled_offline[platform][ecu] = non_zero + orphan_zeros
        else:
          settled_offline[platform][ecu] = list(fws)

    for platform, fws_per_ecu in FW_VERSIONS.items():
      is_gen2 = bool(platform.config.flags & SubaruFlags.GLOBAL_GEN2)
      for drop_eps in ([False, True] if is_gen2 else [False]):
        for _ in range(10):
          car_fw = []
          for (ecu_type, addr, sub_addr), fws in fws_per_ecu.items():
            if drop_eps and ecu_type == Ecu.eps:
              continue
            chosen = random.choice(fws)
            if ecu_type == Ecu.fwdCamera:
              half = len(chosen) // 2
              chosen = chosen[:half] + b"\x00" * half
            car_fw.append(CarFw(ecu=ecu_type, fwVersion=chosen, brand="subaru",
                                address=addr, subAddress=0 if sub_addr is None else sub_addr))

          live_dict = build_fw_dict(car_fw)
          matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_dict, "", settled_offline)
          assert matches == {platform}, f"{platform} (drop_eps={drop_eps}): got {matches}"

  def test_startup_camera_with_updated_powertrain_ecu(self):
    # Regression test for #1153: 2023 Ascent returning 5-byte prefix + 5 zero bytes on startup
    # when engine/transmission also have minor calibration updates not yet in FW_VERSIONS
    car_fw = [
      CarFw(ecu=Ecu.abs, fwVersion=b"\xa5 #\x03\x00", brand="subaru", address=0x7b0, subAddress=0),
      CarFw(ecu=Ecu.fwdCamera, fwVersion=b"\x05!\x08\x1dK\x00\x00\x00\x00\x00", brand="subaru", address=0x787, subAddress=0),
      CarFw(ecu=Ecu.engine, fwVersion=b"\xe5,\xa0Q\x07", brand="subaru", address=0x7a2, subAddress=0),
      CarFw(ecu=Ecu.transmission, fwVersion=b"\x04\xfe\xf4\x00\x00", brand="subaru", address=0x7a3, subAddress=0),
    ]
    exact, matches = match_fw_to_car(car_fw, "", log=False)
    assert not exact
    assert matches == {CAR.SUBARU_ASCENT_2023}

    # Shared camera + engine prefix across Outback and Legacy without distinguishing ABS/TCM must not guess a single model
    ambiguous_live = {
      (0x7b0, None): {b"\xa1  \x99\x99"},
      (0x787, None): {b"\x00\x00eJ\x00\x00\x00\x00\x00\x00"},
      (0x7e0, None): {b"\xde,\xa0@\x07"},
      (0x7e1, None): {b"\xa5\xfe\x99@\x00"},
    }
    candidates = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(ambiguous_live, "", FW_VERSIONS)
    assert candidates == {CAR.SUBARU_OUTBACK, CAR.SUBARU_LEGACY}

  def test_non_eyesight_or_corrupt_camera_rejected(self):
    base_live = {
      (0x7b0, None): {b"\xa5 #\x03\x00"},
      (0x746, None): {b"%\xc0\xd0\x11"},
      (0x7a2, None): {b"\xe5,\xa0P\x07"},
      (0x7a3, None): {b"\x04\xfe\xf3\x00\x00"},
    }
    # Missing fwdCamera (non-EyeSight vehicle without LKAS/ACC API)
    assert FW_QUERY_CONFIG.match_fw_to_car_fuzzy(base_live, "", FW_VERSIONS) == set()

    # All-zero fwdCamera response
    assert FW_QUERY_CONFIG.match_fw_to_car_fuzzy({**base_live, (0x787, None): {b"\x00" * 10}}, "", FW_VERSIONS) == set()
    assert FW_QUERY_CONFIG.match_fw_to_car_fuzzy({**base_live, (0x787, None): {b"\x00" * 8}}, "", FW_VERSIONS) == set()

    # Unknown non-zero settled suffix must not fuzzy-match as a startup response
    assert FW_QUERY_CONFIG.match_fw_to_car_fuzzy({**base_live, (0x787, None): {b"\x05!\x08\x1dK\xff\xff\xff\xff\xff"}}, "", FW_VERSIONS) == set()
