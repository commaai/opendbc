import re
import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.radar_interface import RADAR_START_ADDR
from opendbc.car.tesla.values import CAR, CANBUS, DBC

Ecu = CarParams.Ecu

# Fields prefixed unknown_* we observe structurally but don't know the meaning of.
# Only `platform` has evidence-backed semantic meaning (matches car_model in FW_VERSIONS).
#
# unknown_prefix is everything before the comma; we don't split it because we don't know what its
# parts mean, but observed shape is: <family>_<package>_<triplet> (<build>), e.g.
#   TeMYG4 _ Main     _ 0.0.0 (78)     or     TeM3 _ SP_XP002p2 _ 0.0.0 (23)
#   family   package    triplet build           family  package    triplet build
#
# After the comma, the version string decomposes into:
#   platform             : E/Y/X = car model (Model 3 / Y / X). The only field with known meaning.
#   variant_code         : differentiator WITHIN a platform — hardware/trim/calibration bits packed
#                          into <digit?><letters?><3-digit series>, e.g. '4HP015', '4003', 'L014',
#                          'PR003'. We don't fully know what the parts mean individually, but the
#                          whole string identifies a specific variant within the car model.
#   software_major/minor : numeric components after the first '.' — conventional release numbers.
#                          minor is optional (e.g. 'E4S014.27' has no minor).
#
# Suspected (not confirmed): for M3/MY, `TeM3_*` outer + no-leading-digit variant_code == HW3, and
# `TeMYG4_*` outer + leading-'4' variant_code == HW4 (the 'G4' in TeMYG4 likely denotes Gen 4).
#
# Example full parse of 'TeMYG4_Main_0.0.0 (78),E4HP015.05.0':
#   unknown_prefix='TeMYG4_Main_0.0.0 (78)'
#   platform=E  variant_code=4HP015  software_major=05  software_minor=0
#
# variant_code and software_major don't split the DAS_steeringControlType width: HW3 'E014.20.2' and HW4 'Y4S002.27.0'
# are 3-bit, while 'Y4OC003.04.3' is 2-bit. The (build) number isn't monotonic across updates: '(36) XPR003.10.0' (2-bit)
# -> '(34) XPR003.9.2' (3-bit).
FW_RE = re.compile(
  rb'^(?P<unknown_prefix>.+),' +
  rb'(?P<platform>[EYX])' +
  rb'(?P<variant_code>\d?[A-Z]*\d{3})' +
  rb'\.(?P<software_major>\d+)' +
  rb'(?:\.(?P<software_minor>\d+))?$'
)

PLATFORM_TO_CAR = {
  b'E': CAR.TESLA_MODEL_3,
  b'Y': CAR.TESLA_MODEL_Y,
  b'X': CAR.TESLA_MODEL_X,
}


class TestTeslaFingerprint(unittest.TestCase):
  def test_fw_platform_code(self):
    # Every EPS FW must parse and its platform letter must match the car it's filed under.
    for car_model, ecus in FW_VERSIONS.items():
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        m = FW_RE.match(fw)

        assert m is not None, f"Unparsable FW: {fw}"
        assert PLATFORM_TO_CAR[m['platform']] == car_model, f"Platform letter {m['platform']!r} != {car_model.value}: {fw}"

  def test_radar_detection(self):
    # Test radar availability detection for cars with radar DBC defined
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_no_radar_car(self):
    # Model X doesn't have radar DBC defined, should always be unavailable
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_X, fingerprint, [], False, False, False)
      assert CP.radarUnavailable  # Always unavailable since no radar DBC

  def test_2_bit_das_steering_dashcam(self):
    # Firmware with the 2-bit DAS_steeringControlType isn't supported. Only 3-bit firmware sends 0x489 (HW3/HW4 Autopilot computers)
    # and 0x054 (the car, covers HW2.5)
    for bus, addr in ((None, None), (CANBUS.autopilot_party, 0x489), (CANBUS.party, 0x054)):
      fingerprint = gen_empty_fingerprint()
      if addr is not None:
        fingerprint[bus][addr] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_Y, fingerprint, [], False, False, False)
      assert CP.dashcamOnly == (addr is None)

    assert not CarInterface.get_params(CAR.TESLA_MODEL_Y, gen_empty_fingerprint(), [], False, False, True).dashcamOnly

  def test_eps_angle_control_mismatch(self):
    # The EPS must be in ANGLE_CONTROL while it receives ANGLE_CONTROL, otherwise fault after 0.5 s
    packer = CANPacker(DBC[CAR.TESLA_MODEL_Y][Bus.party])
    fingerprint = gen_empty_fingerprint()
    fingerprint[CANBUS.party][0x054] = 8
    CP = CarInterface.get_params(CAR.TESLA_MODEL_Y, fingerprint, [], False, False, False)
    for eac_status, mismatch in ((2, False), (5, True)):  # ACTIVE, LKA_ACTIVE
      CI = CarInterface(CP)
      for frame in range(50):
        msgs = [
          packer.make_can_msg("DAS_steeringControl", 128, {"DAS_steeringControlType": 1, "DAS_steeringControlCounter": frame % 16}),
          packer.make_can_msg("EPAS3S_sysStatus", CANBUS.party, {"EPAS3S_eacStatus": eac_status, "EPAS3S_sysStatusCounter": frame % 16}),
        ]
        fault = CI.update([(frame * 20_000_000, msgs)]).steerFaultPermanent
        if frame < 20:
          assert not fault
      assert fault == mismatch
