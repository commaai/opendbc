from dataclasses import dataclass, field
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts, Device


@dataclass
class RivianCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts([Device.threex_angled_mount, CarHarness.rivian]))
  setup_video: str = "https://youtu.be/uaISd1j7Z4U"


METADATA = {
  # TODO: verify this
  "RIVIAN_R1_GEN1": [
    RivianCarDocs(
      name="Rivian R1S 2022-24",
    ),
    RivianCarDocs(
      name="Rivian R1T 2022-24",
    ),
  ],
}