"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import uds
from opendbc.car.carlog import carlog
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery

DEVELOPER_DIAGNOSTIC = 0x07
CUSTOM_DIAGNOSTIC_REQUEST = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, DEVELOPER_DIAGNOSTIC])
CUSTOM_DIAGNOSTIC_RESPONSE = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, DEVELOPER_DIAGNOSTIC])

READ_DATA_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER])
READ_DATA_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40])

WRITE_DATA_REQUEST = bytes([uds.SERVICE_TYPE.WRITE_DATA_BY_IDENTIFIER])
WRITE_DATA_RESPONSE = bytes([uds.SERVICE_TYPE.WRITE_DATA_BY_IDENTIFIER + 0x40])

CONFIG_DATA_ID = bytes([0x01, 0x42])
DEFAULT_CONFIG = bytes([0x00, 0x00, 0x00, 0x01, 0x00, 0x00])
TRACKS_ENABLED_CONFIG = bytes([0x00, 0x00, 0x00, 0x01, 0x00, 0x01])
TRACKS_ENABLED_CONFIG_BYTES = b"\x00\x00\x01\x00\x01"


def enable_radar_tracks(logcan, sendcan, bus=0, addr=0x7d0, timeout=0.1, retry=2):
  carlog.error("radar_tracks: enabling ...")

  for i in range(retry):
    try:
      query = IsoTpParallelQuery(sendcan, logcan, bus, [addr], [CUSTOM_DIAGNOSTIC_REQUEST], [CUSTOM_DIAGNOSTIC_RESPONSE])

      for _, _ in query.get_data(timeout).items():
        carlog.error("radar_tracks: check current config ...")

        request = READ_DATA_REQUEST + CONFIG_DATA_ID
        query = IsoTpParallelQuery(sendcan, logcan, bus, [addr], [request], [READ_DATA_RESPONSE])

        for _, data in query.get_data(timeout).items():
          current_config = data[3:]

          carlog.error(f"radar_tracks: current config: {current_config.hex()}")

          if current_config == TRACKS_ENABLED_CONFIG_BYTES:
            carlog.error("radar_tracks: already enabled, skipping ...")
          else:
            carlog.error("radar_tracks: reconfigure radar to output radar points ...")
            request = WRITE_DATA_REQUEST + CONFIG_DATA_ID + TRACKS_ENABLED_CONFIG
            query = IsoTpParallelQuery(sendcan, logcan, bus, [addr], [request], [WRITE_DATA_RESPONSE])
            query.get_data(0)

            carlog.error("radar_tracks: successfully enabled")

          return True

    except Exception as e:
      carlog.exception(f"radar_tracks exception: {e}")

    carlog.error(f"radar_tracks retry ({i + 1}) ...")
  carlog.error("radar_tracks: failed")
  return False


if __name__ == "__main__":
  import time
  import cereal.messaging as messaging
  sendcan = messaging.pub_sock('sendcan')
  logcan = messaging.sub_sock('can')
  time.sleep(1)

  enabled = enable_radar_tracks(logcan, sendcan, bus=0, addr=0x7d0, timeout=0.1)
  print(f"enabled: {enabled}")
