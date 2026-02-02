#!/usr/bin/env python3
import sys
import csv


def gwm_crc_for_0x12B(address: int, sig, d: bytearray) -> int:
  crc = 0x00
  poly = 0x1D
  xor_out = 0x9B
  for byte in d[9:16]:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ poly) if (crc & 0x80) else (crc << 1)
      crc &= 0xFF
  return crc ^ xor_out


def main():
  if len(sys.argv) < 6:
    print("Usage: my_test_crc_0x12B.py <input_csv_file.csv> <can-id> <crc-byte> [init_timestamp] [end_timestamp]")
    print("Example: ./my_test_crc_0x12B.py 0x12B 8 075b133b6181e058--000002ca--3d33eeaf8a.csv 0.061 72.923\n")
    sys.exit(1)
  can_id = int(sys.argv[1], 16)
  crc_byte_index = int(sys.argv[2])
  input_csv_file = sys.argv[-3]
  init_timestamp = float(sys.argv[-2])
  end_timestamp = float(sys.argv[-1])
  print(f"Reading from {input_csv_file} can-ID 0x{can_id:03X} with CRC byte index {crc_byte_index} between {init_timestamp} and {end_timestamp}")
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
      crc = data_bytes[crc_byte_index]
      computed_crc = gwm_crc_for_0x12B(0, 0, bytearray(data_bytes))
      match = "✓" if crc == computed_crc else "✗"
      print(f"Timestamp: {timestamp:.3f} | Data: {data_bytes.hex()} | Original CRC: 0x{crc:02X} | Computed CRC: 0x{computed_crc:02X} | Match: {match}")
      skipone = True


if __name__ == "__main__":
  main()