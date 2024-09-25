from dataclasses import dataclass, field
from enum import Enum

from opendbc.car import dbc_dict, structs, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column, Device


@dataclass
class OtherCarDocs(CarDocs):
  package: str = "Unknown"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.unknown]))

  def init_make(self, CP: structs.CarParams):
    pass


@dataclass
class OtherPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('unknown', None))


@dataclass(frozen=True, kw_only=True)
class OtherCarSpecs(CarSpecs):
  # FIXME: Need to be able to print these as N/A or Unknown or whatever, try to eliminate entirely
  steerRatio: float = 0.


class Footnote(Enum):
  HYUNDAI_WIP = CarFootnote(
    "Official support is under review.",
    Column.MODEL)


class CAR(Platforms):
  config: OtherPlatformConfig

  HYUNDAI_PALISADE_FACELIFT = OtherPlatformConfig(
    [
      OtherCarDocs("Hyundai Palisade 2023-24", package="All"),
      OtherCarDocs("Kia Telluride 2023-24", package="All"),
    ],
    OtherCarSpecs(mass=9, wheelbase=0.406),  # TODO: Don't require CarSpecs for unsupported cars
  )
