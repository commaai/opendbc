from opendbc.car.landrover.values import CanBus

# crc8 poly=0x1d, xor=0xcc , 32bit
def defender_crc(data):
   crc = 0
   poly = 0x1d

   for byte in data:
      crc = crc ^ byte
      for _i in range(8):
          if crc & 0x80:
              crc = (crc << 1) ^ poly
          else:
              crc = (crc << 1)
      crc &= 0xFF

   return crc ^ 0xcc


def create_lkas_command_defender(packer, enable, latActive, apply_angle, cnt):

  values = {
    "Lkas_checksum": 0,
    "counter": cnt,
    "ReqAngleTorque": apply_angle,  # todo check
    "EnAngle": latActive,
    "Engaged": enable,
  }

  dat = packer.make_can_msg("LKAS_OP_TO_FLEXRAY", CanBus.CAN2FLEXRAY, values)[1]
  values["Lkas_checksum"] = defender_crc(dat[1:5])

  return packer.make_can_msg("LKAS_OP_TO_FLEXRAY", CanBus.CAN2FLEXRAY, values)

def create_hud_command_defender(packer, enable, latActive, cnt, left_lane, right_lane):

  values = {
    "Lkas_checksum": 0,
    "counter": cnt,
    "EnAngle": latActive,
    "lane_left": left_lane,
    "lane_right": right_lane,
    "Engaged": enable,
  }

  dat = packer.make_can_msg("HUD_OP_TO_FLEXRAY", CanBus.CAN2FLEXRAY, values)[1]
  values["Lkas_checksum"] = defender_crc(dat[1:5])

  return packer.make_can_msg("HUD_OP_TO_FLEXRAY", CanBus.CAN2FLEXRAY, values)
