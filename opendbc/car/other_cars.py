from dataclasses import dataclass

from opendbc.car import structs, Platforms, OtherPlatformConfig
from opendbc.car.docs_definitions import OtherCarDocs, SupportType


@dataclass
class CommunityCarDocs(OtherCarDocs):
  def init_make(self, CP: structs.CarParams):
    self.support_type = SupportType.COMMUNITY
    self.support_link = "#community-maintained-cars"


@dataclass
class ToyotaSecurityCarDocs(OtherCarDocs):
  def init_make(self, CP: structs.CarParams):
    self.support_type = SupportType.INCOMPATIBLE
    self.support_link = "#toyota-security"


@dataclass
class FlexRayCarDocs(OtherCarDocs):
  def init_make(self, CP: structs.CarParams):
    self.support_type = SupportType.INCOMPATIBLE
    self.support_link = "#flexray"


class CAR(Platforms):
  config: OtherPlatformConfig

  HYUNDAI_PALISADE_FACELIFT = OtherPlatformConfig(
    [
      CommunityCarDocs("Hyundai Palisade 2023-24", package="HDA2"),
      CommunityCarDocs("Kia Telluride 2023-24", package="HDA2"),
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
