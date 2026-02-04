#!/usr/bin/env python3
import sys
import csv


def gwm_basic_chksum_for_0x12B(address: int, sig, d: bytearray) -> int:
  invert_direction = (d[12] >> 7) & 0x1
  counter = d[15] & 0xF
  steer_requested = (d[15] >> 5) & 0x1
  return (28 - (steer_requested * 8) - counter - invert_direction) & 0x1F


def extract_basic_checksum_from_frame(d: bytes) -> int:
  # BASIC_CHECKSUM: byte 9, bits 6..2 (LSB=0)
  return (d[9] >> 2) & 0x1F


def main():
  if len(sys.argv) < 5:
    print("Usage: my_test_basic_checksum_0x12B.py <can-id> <input_csv_file.csv> <init_timestamp> <end_timestamp>")
    print("Example: ./my_test_basic_checksum_0x12B.py 0x12B log.csv 0.061 72.923\n")
    sys.exit(1)

  can_id = int(sys.argv[1], 16)
  input_csv_file = sys.argv[2]
  init_timestamp = float(sys.argv[3])
  end_timestamp = float(sys.argv[4])

  print(f"Reading from {input_csv_file} can-ID 0x{can_id:03X} between {init_timestamp} and {end_timestamp}")
  print("-" * 120 + "\n")

  with open(input_csv_file) as csvfile:
    reader = csv.DictReader(csvfile)
    skipone = True

    for row in reader:
      timestamp = float(row['time'])
      if timestamp < init_timestamp or timestamp > end_timestamp:
        continue

      if int(row['addr'], 16) != can_id:
        continue

      if skipone:
        skipone = False
        continue  # skip first line to avoid partial data

      hex_data = row['data']
      if hex_data.startswith(('0x', '0X')):
        hex_data = hex_data[2:]

      data_bytes = bytes.fromhex(hex_data)

      frame_chk = extract_basic_checksum_from_frame(data_bytes)
      computed_chk = gwm_basic_chksum_for_0x12B(0, 0, bytearray(data_bytes))

      match = "✓" if frame_chk == computed_chk else "✗"

      print(
        f"Timestamp: {timestamp:.3f} | "
        f"Data: {data_bytes.hex()} | "
        f"Frame BASIC_CHECKSUM: 0x{frame_chk:02X} | "
        f"Computed: 0x{computed_chk:02X} | "
        f"Match: {match}"
      )

      skipone = True


if __name__ == "__main__":
  main()
