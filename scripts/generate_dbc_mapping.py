import argparse
import json
import os
from typing import Dict

from common.basedir import BASEDIR
from selfdrive.car import dbc_dict
from selfdrive.car.car_helpers import get_interface_attr


DBC_JSON_OUT = os.path.join(BASEDIR, "opendbc", "scripts", "dbc.json")


def generate_dbc_json() -> str:
  all_car_dbc: Dict[str, dbc_dict] = get_interface_attr("DBC", combine_brands=True)
  dbc_map = {car: d['pt'] for car, d in all_car_dbc.items()}
  return json.dumps(dbc_map, indent=2)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Auto generates car DBC mapping",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)

  parser.add_argument("--out", default=DBC_JSON_OUT, help="Override default generated filename")
  args = parser.parse_args()

  with open(args.out, 'w') as f:
    f.write(generate_dbc_json())
  print(f"Generated and written to {args.out}")
