from dataclasses import dataclass, field
from enum import Enum

from opendbc.car import dbc_dict, structs, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column, SupportType


@dataclass
class OtherPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('unknown', None))


@dataclass(frozen=True, kw_only=True)
class OtherCarSpecs(CarSpecs):
  # FIXME: Need to be able to print these as N/A or Unknown or whatever, try to eliminate entirely
  steerRatio: float = 0.


@dataclass
class OtherCarDocs(CarDocs):
  package: str = "Unknown"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.unknown]))


@dataclass
class CustomForkCarDocs(OtherCarDocs):
  # TODO: attach a footnote here for the more common forks, plus allow footnotes at the car level for special cases
  support_type = SupportType.CUSTOM


@dataclass
class ToyotaSecurityCarDocs(CustomForkCarDocs):
  def init_make(self, CP: structs.CarParams):
    self.footnotes.append(Footnote.TOYOTA_SECOC)


@dataclass
class IncompatibleCarDocs(OtherCarDocs):
  support_type = SupportType.INCOMPATIBLE


@dataclass
class FlexRayCarDocs(IncompatibleCarDocs):
  def init_make(self, CP: structs.CarParams):
    self.footnotes.append(Footnote.FLEXRAY)


class Footnote(Enum):
  TOYOTA_SECOC = CarFootnote("Uses cryptographic message authentication, for which openpilot support is under review.", Column.SUPPORT_TYPE)
  FLEXRAY = CarFootnote("Uses a proprietary network topology incompatible with openpilot.", Column.SUPPORT_TYPE)
  HYUNDAI_WIP = CarFootnote("Official support is under review.", Column.SUPPORT_TYPE)


class CAR(Platforms):
  config: OtherPlatformConfig

  HYUNDAI_PALISADE_FACELIFT = OtherPlatformConfig(
    [
      CustomForkCarDocs("Hyundai Palisade 2023-24", package="All"),
      CustomForkCarDocs("Kia Telluride 2023-24", package="All"),
    ],
    OtherCarSpecs(mass=0., wheelbase=0.),  # TODO: Don't require CarSpecs for unsupported cars
  )

  TOYOTA_SECURITY_CARS = OtherPlatformConfig(
    [
      CustomForkCarDocs("Toyota RAV4 Prime 2021-24", package="All"),
    ],
    OtherCarSpecs(mass=0., wheelbase=0.),  # TODO: Don't require CarSpecs for unsupported cars
  )

  AUDI_FLEXRAY = OtherPlatformConfig(
    [
      FlexRayCarDocs("Audi A4 2016-24", package="All"),
      FlexRayCarDocs("Audi A5 2016-24", package="All"),
      FlexRayCarDocs("Audi Q5 2017-24", package="All"),
    ],
    OtherCarSpecs(mass=0., wheelbase=0.),  # TODO: Don't require CarSpecs for unsupported cars
  )
