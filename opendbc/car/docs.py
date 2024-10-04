#!/usr/bin/env python3
import argparse
import os

from collections import defaultdict
import jinja2
from enum import Enum
from natsort import natsorted

from opendbc.car.common.basedir import BASEDIR
from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, ExtraCarDocs, Column, ExtraCarsColumn, CommonFootnote, PartType
from opendbc.car.car_helpers import interfaces, get_interface_attr
from opendbc.car.values import PLATFORMS, EXTRA_PLATFORMS
from opendbc.car.mock.values import CAR as MOCK


EXTRA_CARS_MD_OUT = os.path.join(BASEDIR, "../", "../", "docs", "CARS.md")
EXTRA_CARS_MD_TEMPLATE = os.path.join(BASEDIR, "CARS_template.md")


def get_all_footnotes() -> dict[Enum, int]:
  all_footnotes = list(CommonFootnote)
  for footnotes in get_interface_attr("Footnote", ignore_none=True).values():
    all_footnotes.extend(footnotes)
  return {fn: idx + 1 for idx, fn in enumerate(all_footnotes)}


def get_all_car_docs() -> list[CarDocs]:
  all_car_docs: list[CarDocs] = []
  footnotes = get_all_footnotes()
  for model, platform in PLATFORMS.items():
    car_docs = platform.config.car_docs
    # If available, uses experimental longitudinal limits for the docs
    CP = interfaces[model][0].get_params(platform, fingerprint=gen_empty_fingerprint(),
                                         car_fw=[CarParams.CarFw(ecu=CarParams.Ecu.unknown)], experimental_long=True, docs=True)

    if CP.dashcamOnly or not len(car_docs):
      continue

    # A platform can include multiple car models
    for _car_docs in car_docs:
      if not hasattr(_car_docs, "row"):
        _car_docs.init_make(CP)
        _car_docs.init(CP, footnotes)
      all_car_docs.append(_car_docs)

  # Sort cars by make and model + year
  sorted_cars: list[CarDocs] = natsorted(all_car_docs, key=lambda car: car.name.lower())
  return sorted_cars


def get_car_docs_with_extras() -> list[CarDocs | ExtraCarDocs]:
  car_docs_with_extras: list[CarDocs | ExtraCarDocs] = []
  for model, platform in EXTRA_PLATFORMS.items():
    car_docs = platform.config.car_docs
    cp_model, cp_platform = (model, platform) if model in interfaces else ("MOCK", MOCK.MOCK)
    CP = interfaces[cp_model][0].get_params(cp_platform, fingerprint=gen_empty_fingerprint(),
                                            car_fw=[CarParams.CarFw(ecu=CarParams.Ecu.unknown)], experimental_long=True, docs=True)

    # A platform can include multiple car models
    for _car_docs in car_docs:
      if not hasattr(_car_docs, "row"):
        _car_docs.init_make(CP)
        _car_docs.init(CP)
      car_docs_with_extras.append(_car_docs)

  # Sort cars by make and model + year
  sorted_cars: list[CarDocs | ExtraCarDocs] = natsorted(car_docs_with_extras, key=lambda car: car.name.lower())
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


def generate_cars_md_with_extras(car_docs_with_extras: list[CarDocs | ExtraCarDocs], template_fn: str) -> str:
  with open(template_fn) as f:
    template = jinja2.Template(f.read(), trim_blocks=True, lstrip_blocks=True)

  cars_md: str = template.render(car_docs_with_extras=car_docs_with_extras, PartType=PartType,
                                 group_by_make=group_by_make, ExtraCarsColumn=ExtraCarsColumn)
  return cars_md


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Auto generates supportability info docs for all known cars",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)

  parser.add_argument("--template", default=EXTRA_CARS_MD_TEMPLATE, help="Override default template filename")
  parser.add_argument("--out", default=EXTRA_CARS_MD_OUT, help="Override default generated filename")
  args = parser.parse_args()

  with open(args.out, 'w') as f:
    f.write(generate_cars_md_with_extras(get_car_docs_with_extras(), args.template))
  print(f"Generated and written to {args.out}")
