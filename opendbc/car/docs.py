#!/usr/bin/env python3
import argparse
from collections import defaultdict
import jinja2
from enum import Enum
from natsort import natsorted
import os

from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, Column, CommonFootnote, PartType
from opendbc.car.car_helpers import interfaces, get_interface_attr
from opendbc.car.values import DOC_PLATFORMS
from opendbc.car.mock.values import CAR as MOCK
from opendbc.car.other_cars import Footnote as OtherFootnotes


# TODO: does opendbc need its own version of openpilot.common.basedir?
BASEDIR = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../"))
CARS_MD_OUT = os.path.join(BASEDIR, "opendbc", "car", "docs", "CARS.md")
CARS_MD_TEMPLATE = os.path.join(BASEDIR, "opendbc", "car", "CARS_template.md")


def get_all_footnotes() -> dict[Enum, int]:
  all_footnotes = list(CommonFootnote)
  all_footnotes.extend(list(OtherFootnotes))
  for footnotes in get_interface_attr("Footnote", ignore_none=True).values():
    all_footnotes.extend(footnotes)
  return {fn: idx + 1 for idx, fn in enumerate(all_footnotes)}


def get_all_car_docs() -> list[CarDocs]:
  all_car_docs: list[CarDocs] = []
  footnotes = get_all_footnotes()
  for model, platform in DOC_PLATFORMS.items():
    car_docs = platform.config.car_docs
    if not len(car_docs):
      continue
    cp_model, cp_platform = (model, platform) if model in interfaces else ("MOCK", MOCK.MOCK)
    # If available, uses experimental longitudinal limits for the docs
    CP = interfaces[cp_model][0].get_params(cp_platform, fingerprint=gen_empty_fingerprint(),
                                             car_fw=[CarParams.CarFw(ecu=CarParams.Ecu.unknown)], experimental_long=True, docs=True)

    # A platform can include multiple car models
    for _car_docs in car_docs:
      if not hasattr(_car_docs, "row"):
        _car_docs.init_make(CP)
        _car_docs.init(CP, footnotes)
      all_car_docs.append(_car_docs)

  # Sort cars by make and model + year
  sorted_cars: list[CarDocs] = natsorted(all_car_docs, key=lambda car: car.name.lower())
  return sorted_cars


def group_by_make(all_car_docs: list[CarDocs]) -> dict[str, list[CarDocs]]:
  sorted_car_docs = defaultdict(list)
  for car_docs in all_car_docs:
    sorted_car_docs[car_docs.make].append(car_docs)
  return dict(sorted_car_docs)


def generate_cars_md(all_car_docs: list[CarDocs], template_fn: str) -> str:
  with open(template_fn) as f:
    template = jinja2.Template(f.read(), trim_blocks=True, lstrip_blocks=True)

  footnotes = [fn.value.text for fn in get_all_footnotes()]
  cars_md: str = template.render(all_car_docs=all_car_docs, PartType=PartType,
                                 group_by_make=group_by_make, footnotes=footnotes,
                                 Column=Column)
  return cars_md


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Auto generates supported cars documentation",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)

  parser.add_argument("--template", default=CARS_MD_TEMPLATE, help="Override default template filename")
  parser.add_argument("--out", default=CARS_MD_OUT, help="Override default generated filename")
  args = parser.parse_args()

  with open(args.out, 'w') as f:
    f.write(generate_cars_md(get_all_car_docs(), args.template))
  print(f"Generated and written to {args.out}")
