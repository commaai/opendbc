"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import carlog, uds
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery

DEVELOPER_DIAGNOSTIC = 0x07
CUSTOM_DIAGNOSTIC_REQUEST = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, DEVELOPER_DIAGNOSTIC])
CUSTOM_DIAGNOSTIC_RESPONSE = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, DEVELOPER_DIAGNOSTIC])

WRITE_DATA_REQUEST = bytes([uds.SERVICE_TYPE.WRITE_DATA_BY_IDENTIFIER])
WRITE_DATA_RESPONSE = bytes([uds.SERVICE_TYPE.WRITE_DATA_BY_IDENTIFIER + 0x40])

CONFIG_DATA_ID = bytes([0x01, 0x42])
RADAR_TRACKS_CONFIG = bytes([0x00, 0x00, 0x00, 0x01, 0x00, 0x01])


def enable_radar_tracks(logcan, sendcan, bus=0, addr=0x7d0, timeout=0.1, retry=2, debug=False):
  carlog.warning("radar_tracks: enabling ...")

  for i in range(retry):
    try:
      query = IsoTpParallelQuery(sendcan, logcan, bus, [addr], [CUSTOM_DIAGNOSTIC_REQUEST], [CUSTOM_DIAGNOSTIC_RESPONSE], debug=debug)

      for _, _ in query.get_data(timeout).items():
        carlog.warning("radar_tracks: reconfigure radar to output radar points ...")

        request = WRITE_DATA_REQUEST + CONFIG_DATA_ID + RADAR_TRACKS_CONFIG
        query = IsoTpParallelQuery(sendcan, logcan, bus, [addr], [request], [WRITE_DATA_RESPONSE], debug=debug)
        query.get_data(0)

        carlog.warning("radar_tracks: successfully enabled")
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

  enabled = enable_radar_tracks(logcan, sendcan, bus=0, addr=0x7d0, timeout=0.1, debug=False)
  print(f"enabled: {enabled}")
