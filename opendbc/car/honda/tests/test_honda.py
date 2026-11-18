import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.radar_interface import RadarInterface
from opendbc.car.honda.values import CAR, DBC, HondaFlags


class TestHondaFingerprint(unittest.TestCase):
  def test_tja_bosch_only(self):
    for car_model in CAR:
      if car_model.config.flags & HondaFlags.BOSCH_TJA_CONTROL:
        assert car_model.config.flags & HondaFlags.BOSCH, "Nidec car found with TJA control"


class TestHondaRadarInterface(unittest.TestCase):
  def test_nidec_radar_state(self):
    CP = CarInterface.get_non_essential_params(CAR.HONDA_ODYSSEY)
    RI = RadarInterface(CP)
    packer = CANPacker(DBC[CP.carFingerprint][Bus.radar])
    track_msgs = [packer.make_can_msg(addr, 1, {'LONG_DIST': 255.5}) for addr in list(range(0x430, 0x43A)) + list(range(0x440, 0x446))]

    cases = (
      # (RADAR_STATE, radarFault, wrongConfig, radarUnavailableTemporary)
      (0x79, False, False, False),
      (0x68, False, False, True),
      (0x6E, False, False, True),
      (0x79, False, False, False),
      (0x69, True, True, False),
      (0x00, True, False, False),
      (0x79, False, False, False),
    )
    for radar_state, fault, wrong_config, temp_unavailable in cases:
      diag_msg = packer.make_can_msg('RADAR_DIAGNOSTIC', 1, {'RADAR_STATE': radar_state})
      ret = RI.update([(0, [diag_msg, *track_msgs])])
      assert ret is not None
      assert not ret.errors.canError
      assert ret.errors.radarFault == fault, hex(radar_state)
      assert ret.errors.wrongConfig == wrong_config, hex(radar_state)
      assert ret.errors.radarUnavailableTemporary == temp_unavailable, hex(radar_state)
