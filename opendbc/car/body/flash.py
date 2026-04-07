import os
import time
import struct
import subprocess
from itertools import accumulate

from opendbc.car.uds import CanClient, IsoTpMessage, MessageTimeoutError

REQUEST_IN = 0xC0
REQUEST_OUT = 0x40
DEFAULT_ISOTP_TIMEOUT = 2

F4Config = {
  "sector_sizes": [0x4000 for _ in range(4)] + [0x10000] + [0x20000 for _ in range(11)],
}

class CanHandle:
  def __init__(self, can_send, can_recv, bus):
    self.client = CanClient(can_send, can_recv, tx_addr=1, rx_addr=2, bus=bus)

  def transact(self, dat, timeout=DEFAULT_ISOTP_TIMEOUT, expect_disconnect=False):
    try:
      msg = IsoTpMessage(self.client, timeout=timeout)
      msg.send(dat)
      if expect_disconnect:
        deadline = time.monotonic() + timeout
        while not msg.tx_done:
          msg.recv(timeout=0)
          if not msg.tx_done and time.monotonic() > deadline:
            raise MessageTimeoutError("timeout waiting for flow control")
          time.sleep(0.01)
        return b""
      ret, _ = msg.recv()
      return ret
    except MessageTimeoutError as e:
      raise TimeoutError from e

  def controlWrite(self, request_type, request, value, index, data, timeout=DEFAULT_ISOTP_TIMEOUT, expect_disconnect=False):
    dat = struct.pack("HHBBHHH", 0, 0, request_type, request, value, index, 0)
    return self.transact(dat, timeout=timeout, expect_disconnect=expect_disconnect)

  def controlRead(self, request_type, request, value, index, length, timeout=DEFAULT_ISOTP_TIMEOUT):
    dat = struct.pack("HHBBHHH", 0, 0, request_type, request, value, index, length)
    return self.transact(dat, timeout=timeout)

  def bulkWrite(self, endpoint, data, timeout=DEFAULT_ISOTP_TIMEOUT):
    dat = struct.pack("HH", endpoint, len(data)) + data
    return self.transact(dat, timeout=timeout)

  def bulkRead(self, endpoint, timeout=DEFAULT_ISOTP_TIMEOUT):
    dat = struct.pack("HH", endpoint, 0)
    return self.transact(dat, timeout=timeout)


def flush_recv_buffer(can_recv):
  while (1):
    if len(can_recv()) == 0:
      break


def fetch_bin(bin_path, update_url):
  os.makedirs(os.path.dirname(bin_path), exist_ok=True)
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


def reset_body(handle):
  print("flash: resetting")
  handle.controlWrite(REQUEST_IN, 0xd8, 0, 0, b'', expect_disconnect=True)


def update(can_send, can_recv, addr, bus, file, update_url, current_signature=None):
  if not os.path.exists(file):
    print("local bin is not up-to-date, fetching latest")
    fetch_bin(file, update_url)

  if current_signature is not None:
    print("checking body firmware signature")
    with open(file, "rb") as f:
      expected_signature = f.read()[-128:]

    print(f"expected body signature: {expected_signature.hex()}")
    print(f"current body signature: {current_signature.hex()}")

  if current_signature is None or current_signature != expected_signature:
    print("flashing motherboard")
    can_send(addr, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x0a", bus)
    time.sleep(0.1)

    print("flashing", file)
    flush_recv_buffer(can_recv)
    with open(file, "rb") as f:
      code = f.read()
    handle = CanHandle(can_send, can_recv, bus)
    retries = 3
    for i in range(retries):
      try:
        flash_can(handle, code, F4Config)
        reset_body(handle)
      except (TimeoutError, RuntimeError) as e:
        print(f"flash failed (attempt {i + 1}/{retries}): {e}, trying again...")
      else:
        print("successfully flashed")

        # Clear cached CarParams so FW queries run fresh after flash
        car_params_cache = "/data/params/d/CarParamsCache"
        if os.path.isfile(car_params_cache):
          os.remove(car_params_cache)
          print("cleared CarParamsCache")
        return

    # on fail: attempt to exit bootloader
    reset_body(handle)
    raise RuntimeError(f"flash failed after {retries} attempts")
  else:
    print("body firmware is up to date")
