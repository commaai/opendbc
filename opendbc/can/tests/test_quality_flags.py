import os
import unittest

from opendbc import DBC_PATH, get_generated_dbcs
from opendbc.can import CANPacker, CANParser
from opendbc.can.dbc import DBC

TOYOTA_QUALITY_FLAGS = {"WHEEL_SPEEDS": {f"WHEEL_SPEED_{whl}_FAULT": (0,) for whl in ("FR", "FL", "RR", "RL")}}

# the quality flags the safety checks, {dbc: {msg: {signal: valid raw values}}}
QUALITY_FLAGS = {
  "ford_lincoln_base_pt": {
    "BrakeSysFeatures": {"VehVActlBrk_D_Qf": (3,)},
    "EngVehicleSpThrottle2": {"VehVActlEng_D_Qf": (3,)},
    "Yaw_Data_FD1": {"VehYawWActl_D_Qf": (3,)},
  },
  "rivian_primary_actuator": {
    "ESP_Status": {"ESP_Vehicle_Speed_Q": (1,)},
    "VDM_PropStatus": {"VDM_VehicleSpeedQ": (1,)},
    "iBESP2": {"iBESP2_BrakePedalApplied_Q": (1,)},
  },
  "tesla_model3_party": {
    "ESP_B": {"ESP_wheelSpeedsQF": (1,)},
    "ESP_status": {"ESP_driverBrakeApply": (1, 2)},
  },
  "toyota_nodsu_pt_generated": TOYOTA_QUALITY_FLAGS,
  "toyota_new_mc_pt_generated": TOYOTA_QUALITY_FLAGS,
  "toyota_tnga_k_pt_generated": TOYOTA_QUALITY_FLAGS,
  "toyota_secoc_pt_generated": TOYOTA_QUALITY_FLAGS,
}


def quality_flags(dbc_name: str) -> dict[str, dict[str, tuple[int, ...]]]:
  flags: dict[str, dict[str, tuple[int, ...]]] = {}
  for msg in DBC(dbc_name).msgs.values():
    for sig in msg.sigs.values():
      if sig.quality_flag_values is not None:
        flags.setdefault(msg.name, {})[sig.name] = sig.quality_flag_values
  return flags


class TestQualityFlags(unittest.TestCase):
  def test_quality_flags(self):
    dbc_names = {f.removesuffix(".dbc") for f in os.listdir(DBC_PATH) if f.endswith(".dbc")} | set(get_generated_dbcs())
    for dbc_name in sorted(dbc_names):
      with self.subTest(dbc=dbc_name):
        self.assertEqual(quality_flags(dbc_name), QUALITY_FLAGS.get(dbc_name, {}))

  def test_parser_rejects_invalid_flags(self):
    for dbc_name, msgs in QUALITY_FLAGS.items():
      dbc = DBC(dbc_name)
      for msg_name, flags in msgs.items():
        valid = {sig_name: valid_values[0] for sig_name, valid_values in flags.items()}
        for sig_name, valid_values in flags.items():
          with self.subTest(dbc=dbc_name, msg=msg_name, sig=sig_name):
            msg = dbc.name_to_msg[msg_name]
            packer = CANPacker(dbc_name)
            parser = CANParser(dbc_name, [(msg_name, 0)], 0)
            last_valid = valid_values[0]
            parser.update([0, [packer.make_can_msg(msg_name, 0, valid)]])

            for raw in range(1 << msg.sigs[sig_name].size):
              updated = parser.update([0, [packer.make_can_msg(msg_name, 0, valid | {sig_name: raw})]])
              self.assertEqual(raw in valid_values, msg.address in updated, f"{sig_name}={raw}")
              if raw in valid_values:
                last_valid = raw
              # a rejected frame keeps the last valid value
              self.assertEqual(parser.vl[msg_name][sig_name], last_valid)

  def test_invalid_flags_time_out(self):
    packer = CANPacker("tesla_model3_party")
    parser = CANParser("tesla_model3_party", [("ESP_B", 50)], 0)

    def send(frames: range, quality_flag: int) -> bool:
      for i in frames:
        t = int(0.02 * i * 1e9)
        parser.update([t, [packer.make_can_msg("ESP_B", 0, {"ESP_wheelSpeedsQF": quality_flag})]])
        can_valid = parser.can_valid  # read every frame, like openpilot does
      return can_valid

    self.assertTrue(send(range(1, 100), 1))

    # same as a bad checksum: the message stops updating and times out
    self.assertFalse(send(range(100, 200), 0))

    self.assertTrue(send(range(200, 300), 1))


if __name__ == "__main__":
  unittest.main()
