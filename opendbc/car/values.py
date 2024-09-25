from typing import get_args
from opendbc.car.body.values import CAR as BODY
from opendbc.car.chrysler.values import CAR as CHRYSLER
from opendbc.car.ford.values import CAR as FORD
from opendbc.car.gm.values import CAR as GM
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.hyundai.values import CAR as HYUNDAI
from opendbc.car.mazda.values import CAR as MAZDA
from opendbc.car.mock.values import CAR as MOCK
from opendbc.car.nissan.values import CAR as NISSAN
from opendbc.car.subaru.values import CAR as SUBARU
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.volkswagen.values import CAR as VOLKSWAGEN
from opendbc.car.other_cars import CAR as OTHER

Platform = BODY | CHRYSLER | FORD | GM | HONDA | HYUNDAI | MAZDA | MOCK | NISSAN | SUBARU | TOYOTA | VOLKSWAGEN
Doc_Platform = Platform | OTHER

BRANDS = get_args(Platform)
DOC_BRANDS = get_args(Doc_Platform)

PLATFORMS: dict[str, Platform] = {str(platform): platform for brand in BRANDS for platform in brand}
DOC_PLATFORMS: dict[str, Doc_Platform] = {str(platform): platform for brand in DOC_BRANDS for platform in brand}
