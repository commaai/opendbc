from dataclasses import dataclass, field
from enum import Enum
from opendbc.car.docs_definitions import CarDocs, CarFootnote, CarHarness, CarParts, Column


class Footnote(Enum):
  HW_TYPE = CarFootnote(
    "Some 2023 model years have HW4. To check which hardware type your vehicle has, look for " +
    "<b>Autopilot computer</b> under <b>Software -> Additional Vehicle Information</b> on your vehicle's touchscreen. </br></br>" +
    "See <a href=\"https://www.notateslaapp.com/news/2173/how-to-check-if-your-tesla-has-hardware-4-ai4-or-hardware-3\">this page</a> for more information.",
    Column.MODEL)

  SETUP = CarFootnote(
    "See more setup details for <a href=\"https://github.com/commaai/openpilot/wiki/tesla\" target=\"_blank\">Tesla</a>.",
    Column.MAKE, setup_note=True)


@dataclass
class TeslaCarDocsHW3(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.tesla_a]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.HW_TYPE, Footnote.SETUP])


@dataclass
class TeslaCarDocsHW4(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.tesla_b]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.HW_TYPE, Footnote.SETUP])


METADATA = {
  "TESLA_MODEL_3": [
    # TODO: do we support 2017? It's HW3
    TeslaCarDocsHW3(
      name="Tesla Model 3 (with HW3) 2019-23",
    ),
    TeslaCarDocsHW4(
      name="Tesla Model 3 (with HW4) 2024-25",
    ),
  ],
  "TESLA_MODEL_Y": [
    TeslaCarDocsHW3(
      name="Tesla Model Y (with HW3) 2020-23",
    ),
    TeslaCarDocsHW4(
      name="Tesla Model Y (with HW4) 2024",
    ),
  ],
}