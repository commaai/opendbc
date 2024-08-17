import re

from panda.python.uds import get_rx_addr_for_tx_addr, FUNCTIONAL_ADDRS
from openpilot.selfdrive.car import carlog
from openpilot.selfdrive.car.isotp_parallel_query import IsoTpParallelQuery
from openpilot.selfdrive.car.fw_query_definitions import STANDARD_VIN_ADDRS, StdQueries

VIN_UNKNOWN = "0" * 17
VIN_RE = "[A-HJ-NPR-Z0-9]{17}"


def is_valid_vin(vin: str):
  return re.fullmatch(VIN_RE, vin) is not None


def get_vin(can_recv, can_send, buses, timeout=0.1, retry=2, debug=False):
  for i in range(retry):
    for bus in buses:
      for request, response, valid_buses, vin_addrs, functional_addrs, rx_offset in (
        (StdQueries.UDS_VIN_REQUEST, StdQueries.UDS_VIN_RESPONSE, (0, 1), STANDARD_VIN_ADDRS, FUNCTIONAL_ADDRS, 0x8),
        (StdQueries.OBD_VIN_REQUEST, StdQueries.OBD_VIN_RESPONSE, (0, 1), STANDARD_VIN_ADDRS, FUNCTIONAL_ADDRS, 0x8),
        (StdQueries.GM_VIN_REQUEST, StdQueries.GM_VIN_RESPONSE, (0,), [0x24b], None, 0x400),  # Bolt fwdCamera
        (StdQueries.KWP_VIN_REQUEST, StdQueries.KWP_VIN_RESPONSE, (0,), [0x797], None, 0x3),  # Nissan Leaf VCM
        (StdQueries.UDS_VIN_REQUEST, StdQueries.UDS_VIN_RESPONSE, (0,), [0x74f], None, 0x6a),  # Volkswagen fwdCamera
      ):
        if bus not in valid_buses:
          continue

        # When querying functional addresses, ideally we respond to everything that sends a first frame to avoid leaving the
        # ECU in a temporary bad state. Note that we may not cover all ECUs and response offsets. TODO: query physical addrs
        tx_addrs = vin_addrs
        if functional_addrs is not None:
          tx_addrs = [a for a in range(0x700, 0x800) if a != 0x7DF] + list(range(0x18DA00F1, 0x18DB00F1, 0x100))

        try:
          query = IsoTpParallelQuery(can_send, can_recv, bus, tx_addrs, [request, ], [response, ], response_offset=rx_offset,
                                     functional_addrs=functional_addrs, debug=debug)
          results = query.get_data(timeout)

          for addr in vin_addrs:
            vin = results.get((addr, None))
            if vin is not None:
              # Ford and Nissan pads with null bytes
              if len(vin) in (19, 24):
                vin = re.sub(b'\x00*$', b'', vin)

              # Honda Bosch response starts with a length, trim to correct length
              if vin.startswith(b'\x11'):
                vin = vin[1:18]

              carlog.error(f"got vin with {request=}")
              return get_rx_addr_for_tx_addr(addr, rx_offset=rx_offset), bus, vin.decode()
        except Exception:
          carlog.exception("VIN query exception")

    carlog.error(f"vin query retry ({i+1}) ...")

  return -1, -1, VIN_UNKNOWN
