import argparse
import json
import os

from selfdrive.car.car_helpers import get_interface_attr
from opendbc import DBC_PATH


DBC_JSON_OUT = os.path.join(DBC_PATH, "scripts", "dbc.json")


def generate_dbc_json() -> str:
  all_car_dbc = get_interface_attr("DBC", combine_brands=True)
  dbc_map = {car: d['pt'] for car, d in all_car_dbc.items()}
  return json.dumps(dbc_map, indent=2)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Generate mapping for all cars to DBC names and outputs json file",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)

  parser.add_argument("--out", default=DBC_JSON_OUT, help="Override default generated filename")
  args = parser.parse_args()

  with open(args.out, 'w') as f:
    f.write(generate_dbc_json())
  print(f"Generated and written to {args.out}")
