
# crc8 poly=0x1d, xor=0xcc , 32bit
def defender_crc(data):
   crc = 0
   poly = 0x1d

   for byte in data:
      crc = crc ^ byte
      for i in range(0, 8):
          if crc & 0x80:
              crc = (crc << 1) ^ poly
          else:
              crc = (crc << 1)
      crc &= 0xFF

   return crc ^ 0xcc



def create_lkas_command_defender(packer, enable, latActive, apply_angle, cnt, left_lane, right_lane):

  values = {
    "Lkas_checksum" : 0,
    "counter"       : cnt,
    "ReqAngleTorque": apply_angle,  # todo check
    "EnAngle"       : latActive,
    "lane_left"     : left_lane,
    "lane_right"    : right_lane,
    "Engaged"       : enable,
  }

  dat = paccker.make_can_msg("LKAS_OP_TO_FLEXRAY", 1, values)[1]
  values["Lkas_checksum"] =  defender_crc(dat[1:5])

  return paccker.make_can_msg("LKAS_OP_TO_FLEXRAY", 1, values)

