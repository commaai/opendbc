#!/usr/bin/env python3
import os

from opendbc.car.common.basedir import BASEDIR
from opendbc.car.interfaces import get_interface_attr
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu

CARS = get_interface_attr('CAR')
FW_VERSIONS = get_interface_attr('FW_VERSIONS')
FINGERPRINTS = get_interface_attr('FINGERPRINTS')
ECU_NAME = {v: k for k, v in Ecu.schema.enumerants.items()}


def _format_fingerprint_dict(fingerprint: dict) -> str:
  items = [f"{key}: {value}" for key, value in fingerprint.items()]
  return ", ".join(items)


def _format_ecu_key(key: tuple) -> str:
  ecu, addr, subaddr = key
  addr_s = f"0x{int(addr):x}"
  subaddr_s = f"0x{int(subaddr):x}" if subaddr else str(subaddr)
  return f"(Ecu.{ECU_NAME[ecu]}, {addr_s}, {subaddr_s})"


def render_fingerprints(brand: str, comments: list[str],
                        extra_fw_versions: dict[str, dict[tuple, list[bytes]]]) -> str:
  brand_fingerprints = FINGERPRINTS[brand]
  brand_fw = FW_VERSIONS[brand]
  lines: list[str] = []

  if brand_fingerprints and brand != 'body':
    lines.append("# ruff: noqa: E501")

  lines.append('""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""')

  if brand_fw:
    lines.append("from opendbc.car.structs import CarParams")
  lines.append(f"from opendbc.car.{brand}.values import CAR")
  if brand_fw:
    lines.append("")
    lines.append("Ecu = CarParams.Ecu")

  if comments:
    lines.append("")
    # comments already include trailing newlines from the source file lines;
    # strip them and re-add as separate lines for consistency
    for comment in comments:
      lines.append(comment.rstrip("\n"))

  if brand_fingerprints:
    lines.append("")
    lines.append("FINGERPRINTS = {")
    for car, fingerprints in brand_fingerprints.items():
      for i, fingerprint in enumerate(fingerprints):
        if i == 0:
          lines.append(f"  CAR.{car.name}: [{{")
        else:
          lines.append("  {")
        lines.append(f"    {_format_fingerprint_dict(fingerprint)}")
        if i == len(fingerprints) - 1:
          lines.append("  }],")
        else:
          lines.append("  },")
    lines.append("}")

  lines.append("")
  if brand_fw:
    lines.append("FW_VERSIONS = {")
  else:
    lines.append("FW_VERSIONS: dict[str, dict[tuple, list[bytes]]] = {")

  for car, _ in brand_fw.items():
    lines.append(f"  CAR.{car.name}: {{")
    for key, fw_versions in brand_fw[car].items():
      extra = extra_fw_versions.get(car, {}).get(key, [])
      # dedupe preserving first-seen order, then sort
      seen: set[bytes] = set()
      unique_fw: list[bytes] = []
      for fw in fw_versions + extra:
        if fw not in seen:
          seen.add(fw)
          unique_fw.append(fw)
      unique_fw.sort()

      lines.append(f"    {_format_ecu_key(key)}: [")
      for fw_version in unique_fw:
        lines.append(f"    {fw_version},")
      lines.append("  ],")
    lines.append("  },")
  lines.append("}")
  lines.append("")  # trailing newline at EOF
  return "\n".join(lines)


def format_brand_fw_versions(brand, extra_fw_versions: None | dict[str, dict[tuple, list[bytes]]] = None):
  extra_fw_versions = extra_fw_versions or {}

  fingerprints_file = os.path.join(BASEDIR, f"{brand}/fingerprints.py")
  with open(fingerprints_file) as f:
    comments = [line for line in f.readlines() if line.startswith("#") and "noqa" not in line]

  with open(fingerprints_file, "w") as f:
    f.write(render_fingerprints(brand, comments, extra_fw_versions))


if __name__ == "__main__":
  for brand in FW_VERSIONS.keys():
    format_brand_fw_versions(brand)
