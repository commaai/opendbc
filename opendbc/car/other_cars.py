from dataclasses import dataclass
from enum import Enum

from opendbc.car import structs, Platforms, OtherPlatformConfig
from opendbc.car.docs_definitions import CarFootnote, OtherCarDocs, Column, SupportType


@dataclass
class CommunityCarDocs(OtherCarDocs):
  def init_make(self, CP: structs.CarParams):
    self.support_type = SupportType.COMMUNITY


@dataclass
class ToyotaSecurityCarDocs(OtherCarDocs):
  def init_make(self, CP: structs.CarParams):
    self.support_type = SupportType.INCOMPATIBLE
    self.footnotes.append(Footnote.TOYOTA_SECOC)


@dataclass
class FlexRayCarDocs(OtherCarDocs):
  def init_make(self, CP: structs.CarParams):
    self.support_type = SupportType.INCOMPATIBLE
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
  )

  AUDI_FLEXRAY = OtherPlatformConfig(
    [
      FlexRayCarDocs("Audi A4 2016-24", package="All"),
      FlexRayCarDocs("Audi A5 2016-24", package="All"),
      FlexRayCarDocs("Audi Q5 2017-24", package="All"),
    ],
  )
