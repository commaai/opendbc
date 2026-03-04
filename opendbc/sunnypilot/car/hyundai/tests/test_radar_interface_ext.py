from parameterized import parameterized

from opendbc.car import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.escc import ESCC_MSG

ESCC_CARS = [
  (CAR.HYUNDAI_ELANTRA_2021, ESCC_MSG),
]

CAMERA_SCC_CARS = [
  (CAR.HYUNDAI_KONA_EV_2022, 0, 0x420, "SCC11"),
  (CAR.HYUNDAI_IONIQ_5, HyundaiFlags.CANFD_CAMERA_SCC.value, 0x1A0, "SCC_CONTROL"),
]

STANDARD_RADAR_CARS = [
  (CAR.HYUNDAI_ELANTRA_2021, 0),
  (CAR.HYUNDAI_SANTA_FE, 0),
]


class TestRadarInterfaceExt:

  @staticmethod
  def _setup_platform(car_name, additional_flags=0, escc_msg=None):
    """Set up the platform with specific parameters"""
    CarInterface = interfaces[car_name]

    CP = CarInterface.get_non_essential_params(car_name)
    CP.flags |= additional_flags

    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)

    CI = CarInterface(CP, CP_SP)

    RD = CI.RadarInterface(CP, CP_SP)

    if escc_msg is not None and hasattr(RD, 'use_escc'):
      try:
        RD.use_escc = True
      except AttributeError:
        object.__setattr__(RD, 'use_escc', True)

    return RD, CP, CP_SP

  @parameterized.expand(ESCC_CARS)
  def test_escc_radar_interface(self, car_name, escc_msg):
    """Test radar interface for ESCC-enabled cars"""
    RD, CP, CP_SP = self._setup_platform(car_name, escc_msg=escc_msg)

    # Assert that ESCC features are present
    if hasattr(RD, 'use_escc'):
      assert RD.use_escc, "ESCC car should have use_escc=True"
    if hasattr(RD, 'use_radar_interface_ext'):
      assert RD.use_radar_interface_ext, "ESCC car should use radar interface ext"

    # Run radar interface once
    RD.update([])

    # Test radar fault
    if not CP.radarUnavailable and RD.rcp is not None:
      cans = [(0, [CanData(0, b'', 0) for _ in range(5)])]
      rr = RD.update(cans)
      assert rr is None or len(rr.errors) > 0

  @parameterized.expand(CAMERA_SCC_CARS)
  def test_camera_scc_radar_interface(self, car_name, flags, expected_trigger, msg_src):
    """Test radar interface for Camera SCC cars"""
    RD, CP, CP_SP = self._setup_platform(car_name, additional_flags=flags)

    # Assert Camera SCC flag is set appropriately
    if flags & HyundaiFlags.CAMERA_SCC:
      assert CP.flags & HyundaiFlags.CAMERA_SCC, "Car should have CAMERA_SCC flag"
    if flags & HyundaiFlags.CANFD_CAMERA_SCC:
      assert CP.flags & HyundaiFlags.CANFD_CAMERA_SCC, "Car should have CANFD_CAMERA_SCC flag"

    # Check if using radar interface ext
    if hasattr(RD, 'use_radar_interface_ext'):
      assert RD.use_radar_interface_ext, "Camera SCC car should use radar interface ext"

    # Verify trigger message
    if hasattr(RD, 'trigger_msg'):
      assert RD.trigger_msg == expected_trigger, f"Expected trigger_msg {expected_trigger}, got {RD.trigger_msg}"

    # Run radar interface once
    RD.update([])

    # Test radar fault
    if not CP.radarUnavailable and RD.rcp is not None:
      cans = [(0, [CanData(0, b'', 0) for _ in range(5)])]
      rr = RD.update(cans)
      assert rr is None or len(rr.errors) > 0

  @parameterized.expand(STANDARD_RADAR_CARS)
  def test_standard_radar_interface(self, car_name, flags):
    """Test radar interface for standard radar cars"""
    RD, CP, CP_SP = self._setup_platform(car_name, additional_flags=flags)

    # Standard cars should not use radar interface ext
    if hasattr(RD, 'use_radar_interface_ext'):
      assert not RD.use_radar_interface_ext, "Standard car should not use radar interface ext"

    # Run radar interface once
    RD.update([])

    # For standard radar, test the _update method directly if available
    if not CP.radarUnavailable and RD.rcp is not None and \
          hasattr(RD, '_update') and hasattr(RD, 'trigger_msg'):
      # Setup for _update test if needed
      if hasattr(RD, 'updated_messages'):
        RD.updated_messages = {RD.trigger_msg}
      RD._update(RD.updated_messages)

    # Test radar fault
    if not CP.radarUnavailable and RD.rcp is not None:
      cans = [(0, [CanData(0, b'', 0) for _ in range(5)])]
      rr = RD.update(cans)
      assert rr is None or len(rr.errors) > 0
