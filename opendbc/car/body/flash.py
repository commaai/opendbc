import os
import time
import struct
import subprocess
from itertools import accumulate

from uds import UdsClient, CanClient, IsoTpMessage, DATA_IDENTIFIER_TYPE, MessageTimeoutError

REQUEST_IN = 0xC0
REQUEST_OUT = 0x40

F4Config = {
  "sector_sizes": [0x4000 for _ in range(4)] + [0x10000] + [0x20000 for _ in range(11)],
}

class CanHandle():
  def __init__(self, can_send, can_recv, bus):
    self.client = CanClient(can_send, can_recv, tx_addr=1, rx_addr=2, bus=bus)

  def transact(self, dat):
    try:
      msg = IsoTpMessage(self.client, timeout=2)
      msg.send(dat)
      ret, _ = msg.recv
      return ret
    except (MessageTimeoutError): raise

  def controlWrite(self, request_type, request, value, index, timeout=0):
    return self.controlRead(request_type, request, value, index, 0, timeout)

  def controlRead(self, request_type, request, value, index, length):
    dat = struct.pack("HHBBHHH", 0, 0, request_type, request, value, index, length)
    return self.transact(dat)

  def bulkWrite(self, endpoint, data):
    dat = struct.pack("HH", endpoint, len(data)) + data
    return self.transact(dat)

  def bulkRead(self, endpoint):
    dat = struct.pack("HH", endpoint, 0)
    return self.transact(dat)

def flush_recv_buffer(can_recv):
  while(1):
    if len(can_recv()) == 0:
      break


def fetch_bin(bin_path, update_url):
  result = subprocess.run(
    ["curl", "-L", "-o", bin_path, "-w", "%{http_code}", "--silent", update_url],
    capture_output=True, text=True
  )
  status = result.stdout.strip()
  if status == "200":
    print("downloaded latest body firmware binary")
  else:
    raise RuntimeError(f"download failed with HTTP {status}")


def flash_can(handle, code, mcu_config):
    assert mcu_config is not None, "must set valid mcu_type to flash"

    # confirm flasher is present
    fr = handle.controlRead(REQUEST_IN, 0xb0, 0, 0, 0xc)
    assert fr[4:8] == b"\xde\xad\xd0\x0d"

    apps_sectors_cumsum = accumulate(mcu_config["sector_sizes"][1:])
    last_sector = next((i + 1 for i, v in enumerate(apps_sectors_cumsum) if v > len(code)), -1)
    assert last_sector >= 1, "Binary too small? No sector to erase."
    assert last_sector < 7, "Binary too large! Risk of overwriting provisioning chunk."

    print("flash: unlocking")
    handle.controlWrite(REQUEST_IN, 0xb1, 0, 0, b'')

    print(f"flash: erasing sectors 1 - {last_sector}")
    for i in range(1, last_sector + 1):
      handle.controlWrite(REQUEST_IN, 0xb2, i, 0, b'')

    STEP = 0x10
    print("flash: flashing")
    for i in range(0, len(code), STEP):
      handle.bulkWrite(2, code[i:i + STEP])

    print("flash: resetting")
    try:
      handle.controlWrite(REQUEST_IN, 0xd8, 0, 0, b'', expect_disconnect=True)
    except Exception:
      pass


def update(bootloader_addr, file, update_url, can_send, can_recv, uds_base, uds_signature_offset):
  if not os.path.exists(file):
    print("local bin is not up-to-date, fetching latest")
    fetch_bin(file, update_url)

  if uds_base and uds_signature_offset:
    print("checking body firmware signature")
    uds_client = UdsClient({can_send: can_send, can_recv: can_recv}, uds_base, uds_signature_offset, bus=bus, timeout=2)

    try:
      current_signature = uds_client.read_data_by_identifier(DATA_IDENTIFIER_TYPE.APPLICATION_SOFTWARE_FINGERPRINT)
    except MessageTimeoutError:
      current_signature = b""

    with open(file, "rb") as f:
      expected_signature = f.read()[-128:]

    print(f"expected body signature: {expected_signature.hex()}")
    print(f"current body signature: {current_signature.hex()}")

  if not current_signature or current_signature != expected_signature:
    print("flashing motherboard")
    can_send(bootloader_addr, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x0a", 0)
    time.sleep(0.1)
    msgs = can_recv()
    for ids, _, bus in msgs:
      if ids == bootloader_addr and bus == 128:
        raise RuntimeError("flash failed: device likely did not enter bootloader")

    print("flashing", file)
    flush_recv_buffer(can_recv)
    code = open(file, "rb").read()
    handle = CanHandle(can_send, can_recv, 0)
    retries = 3
    for i in range(retries):
      try:
        flash_can(handle, code, F4Config)
      except (TimeoutError):
        print(f"flash failed (attempt {i + 1}/{retries}), trying again...")
      else:
        print("successfully flashed")
        return

    # on fail: attempt to exit bootloader
    handle.controlWrite(REQUEST_IN, 0xd8, 0, 0, b'', expect_disconnect=True)
    raise RuntimeError(f"flash failed after {retries} attempts")
  else:
    print("body firmware is up to date")
