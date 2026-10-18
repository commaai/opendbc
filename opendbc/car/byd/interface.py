from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase, PandaSafetyTxTestCase
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.radar_interface import RadarInterface
from opendbc.car.byd.values import BydSafetyFlags, DBC, SAFETY_PLATFORM_PARAMS


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @classmethod
  def panda_safety_tx_test_cases(cls, CP, safety):
    # Only TX 0x1E2 while engaged; seed angle/speed so VM limits pass. No resume button TX.
    from opendbc.can.packer import CANPacker
    from opendbc.safety.tests.libsafety import libsafety_py

    packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])
    for _ in range(6):
      for name, values in (
        ("STEER_MODULE_2", {"STEER_ANGLE_2": 0.0}),
        ("WHEELSPEED_CLEAN", {"WHEELSPEED_CLEAN": 36.0}),
      ):
        addr, dat, bus = packer.make_can_msg(name, 0, values)
        assert safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, dat)) == 1
    safety.set_desired_angle_last(0)
    safety.set_controls_allowed(True)

    def on_frame(s, i):
      # RT angle limits and byd_fwd timestamps need a non-zero clock.
      s.set_timer(int((i + 1) * 1e6 / 50))

    def setup_cancel(s):
      s.set_cruise_engaged_prev(True)

    return [
      PandaSafetyTxTestCase(structs.CarControl(enabled=True, latActive=True), on_frame=on_frame),
      PandaSafetyTxTestCase(structs.CarControl(cruiseControl=structs.CarControl.CruiseControl(cancel=True)),
                            setup=setup_cancel, on_frame=on_frame),
    ]

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "byd"

    # LKS_ON matches first-boot persist. card.py clears/sets it from BydLksEnabled.
    # The platform bit selects this platform's steering/long limits in byd_init.
    safety_param = int(BydSafetyFlags.LKS_ON) | SAFETY_PLATFORM_PARAMS[candidate].safety_param
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd, safety_param)]

    ret.dashcamOnly = False
    ret.hudLaneFromModel = True
    ret.hudLeadShading = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.25
    ret.steerLimitTimer = 0.4
    ret.longitudinalActuatorDelay = 0.5

    ret.radarUnavailable = False

    ret.alphaLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = alpha_long and ret.alphaLongitudinalAvailable
    ret.pcmCruise = not ret.openpilotLongitudinalControl
    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= BydSafetyFlags.LONG_CONTROL.value

    return ret