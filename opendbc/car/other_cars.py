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
  package: str = "N/A"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.unknown]))


@dataclass
class CustomForkCarDocs(OtherCarDocs):
  # TODO: attach a footnote here for the more common forks, plus allow footnotes at the car level for special cases
  # TODO: maybe these custom fork pointers want to be intra-document #links rather than footnotes
  support_type = SupportType.CUSTOM


@dataclass
class ToyotaSecurityCarDocs(CustomForkCarDocs):
  def init_make(self, CP: structs.CarParams):
    # TODO: Make this an intra-document #link to more detailed info, rather than a footnote
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
  UNDER_REVIEW = CarFootnote("Official support is under review.", Column.SUPPORT_TYPE)


class CAR(Platforms):
  config: OtherPlatformConfig

  HYUNDAI_PALISADE_FACELIFT = OtherPlatformConfig(
    [
      CustomForkCarDocs("Hyundai Palisade 2023-24", package="All", footnotes=[Footnote.UNDER_REVIEW]),
      CustomForkCarDocs("Kia Telluride 2023-24", package="All", footnotes=[Footnote.UNDER_REVIEW]),
    ],
    OtherCarSpecs(mass=0., wheelbase=0.),  # TODO: Don't require CarSpecs for unsupported cars
  )

  TOYOTA_SECURITY_CARS = OtherPlatformConfig(
    [
      ToyotaSecurityCarDocs("Subaru Solterra 2023-25"),
      ToyotaSecurityCarDocs("Lexus NS 2022-25"),
      ToyotaSecurityCarDocs("Toyota bZ4x 2023-25"),
      ToyotaSecurityCarDocs("Toyota Camry 2025"),
      ToyotaSecurityCarDocs("Toyota Corolla Cross 2022-25"),
      ToyotaSecurityCarDocs("Toyota Highlander 2025"),
      ToyotaSecurityCarDocs("Toyota RAV4 Prime 2021-25"),
      ToyotaSecurityCarDocs("Toyota Sequoia 2023-25"),
      ToyotaSecurityCarDocs("Toyota Sienna 2021-25"),
      ToyotaSecurityCarDocs("Toyota Tundra 2022-25"),
      ToyotaSecurityCarDocs("Toyota Venza 2021-25"),
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
