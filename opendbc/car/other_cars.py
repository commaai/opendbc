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
class CommunityCarDocs(OtherCarDocs):
  def init_make(self, CP: structs.CarParams):
    self.support_type = SupportType.COMMUNITY


@dataclass
class IncompatibleCarDocs(OtherCarDocs):
  def init_make(self, CP: structs.CarParams):
    self.support_type = SupportType.INCOMPATIBLE


@dataclass
class ToyotaSecurityCarDocs(IncompatibleCarDocs):
  def init_make(self, CP: structs.CarParams):
    super().init_make(CP)
    self.footnotes.append(Footnote.TOYOTA_SECOC)


@dataclass
class FlexRayCarDocs(IncompatibleCarDocs):
  def init_make(self, CP: structs.CarParams):
    super().init_make(CP)
    self.footnotes.append(Footnote.FLEXRAY)

# TODO: Convert most or all of these footnotes to intra-document #links to a longer explanation
class Footnote(Enum):
  TOYOTA_SECOC = CarFootnote("Uses cryptographic message authentication, for which openpilot support is under review.", Column.SUPPORT_TYPE)
  FLEXRAY = CarFootnote("Uses a proprietary network topology incompatible with openpilot.", Column.SUPPORT_TYPE)
  UNDER_REVIEW = CarFootnote("Official support is under review.", Column.SUPPORT_TYPE)


class CAR(Platforms):
  config: OtherPlatformConfig

  HYUNDAI_PALISADE_FACELIFT = OtherPlatformConfig(
    [
      CommunityCarDocs("Hyundai Palisade 2023-24", package="All", footnotes=[Footnote.UNDER_REVIEW]),
      CommunityCarDocs("Kia Telluride 2023-24", package="All", footnotes=[Footnote.UNDER_REVIEW]),
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
      CommunityCarDocs("Toyota RAV4 Prime 2021-23"),
      ToyotaSecurityCarDocs("Toyota RAV4 Prime 2024-25"),
      ToyotaSecurityCarDocs("Toyota Sequoia 2023-25"),
      CommunityCarDocs("Toyota Sienna 2021-23"),
      ToyotaSecurityCarDocs("Toyota Sienna 2024-25"),
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
